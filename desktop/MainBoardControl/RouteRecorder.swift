import AppKit
import Combine
import Foundation
import UniformTypeIdentifiers

// Route = dead reckoning. An MPU6050 alone cannot give position: double
// integration of acceleration drifts by metres within seconds. Distance
// therefore comes from Aura's leg odometry (planted feet + joint encoders),
// heading from the gyro through a Kalman filter, and height from the ground
// slope under the feet. Ground frame: X forward at start, Z left, Y up, mm.

struct RoutePoint: Codable, Equatable {
    var t: Double          // s since recording started
    var x: Double, y: Double, z: Double
    var heading: Double    // rad, + = left turn
    var roll: Double, pitch: Double  // torso, deg
}

struct RouteRecord: Codable, Identifiable {
    var id = UUID()
    var name: String
    var created: Date
    var points: [RoutePoint]

    var distance: Double {
        zip(points, points.dropFirst()).reduce(0) { sum, pair in
            sum + ((pair.1.x - pair.0.x) * (pair.1.x - pair.0.x) + (pair.1.y - pair.0.y) * (pair.1.y - pair.0.y) +
                   (pair.1.z - pair.0.z) * (pair.1.z - pair.0.z)).squareRoot()
        }
    }
    var duration: Double { (points.last?.t ?? 0) - (points.first?.t ?? 0) }
    var climb: Double { (points.map(\.y).max() ?? 0) - (points.map(\.y).min() ?? 0) }
}

@MainActor
final class RouteRecorder: ObservableObject {
    @Published private(set) var revision = 0
    @Published private(set) var recording = false
    @Published private(set) var saved: [RouteRecord] = []
    @Published var comparison: RouteRecord?

    // Live pose, also while not recording, so the 3D view always shows Aura.
    private(set) var position = SIMD3<Double>(repeating: 0)
    private(set) var roll = 0.0, pitch = 0.0
    private(set) var points: [RoutePoint] = []
    private(set) var filter = HeadingFilter()
    private(set) var still = false
    private(set) var odometryValid = false

    private var lastTick: UInt32?
    private var lastOdometry: (x: Double, z: Double, yaw: Double)?
    private var stillTime = 0.0
    private var recordStart = 0.0
    private var clock = 0.0
    private var dirty = false
    private var timer: AnyCancellable?

    static var folder: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("Aura/Trasy", isDirectory: true)
    }

    init() {
        timer = Timer.publish(every: 0.2, on: .main, in: .common).autoconnect().sink { [weak self] _ in
            guard let self, self.dirty else { return }
            self.dirty = false
            self.revision &+= 1
        }
        loadSaved()
    }

    var heading: Double { filter.heading }
    var distance: Double { RouteRecord(name: "", created: Date(), points: points).distance }
    var duration: Double { (points.last?.t ?? 0) - (points.first?.t ?? 0) }

    // 50 Hz, from the network path.
    func ingest(state: AuraRobotState, imu: AuraIMU, attitude: AuraAttitude) {
        // Aura's gait tick is the clock: 20 ms per tick, immune to Wi-Fi jitter.
        guard let previous = lastTick else { lastTick = state.tickCount; return }
        let ticks = state.tickCount &- previous
        lastTick = state.tickCount
        guard ticks > 0, ticks < 50 else { return }
        let dt = Double(ticks) * 0.02
        clock += dt

        let yawRate = imu.gyroscope[1] * .pi / 180     // Aura Y is up; + = left turn
        filter.predict(gyroRate: yawRate, dt: dt)

        // Still: no walking command, gravity alone on the accelerometer and a
        // quiet gyro for half a second. Then the true yaw rate is zero.
        let accelNorm = (imu.acceleration[0] * imu.acceleration[0] + imu.acceleration[1] * imu.acceleration[1] +
                         imu.acceleration[2] * imu.acceleration[2]).squareRoot()
        let commanded = abs(state.forward) + abs(state.lateral) + abs(state.turn) > 0.02
        let quiet = !commanded && abs(accelNorm - 1) < 0.04 &&
            imu.gyroscope.allSatisfy { abs($0) < 4 }
        stillTime = quiet ? stillTime + dt : 0
        still = stillTime > 0.5
        if still { filter.observeStill(gyroRate: yawRate) }

        // Ground slope under the feet: torso attitude minus the posture the
        // IMU loop commands (same sign convention as Aura's posture loop).
        roll = attitude.rollDegrees - (attitude.referenceCaptured ? attitude.referenceRollDegrees : 0)
        pitch = attitude.pitchDegrees - (attitude.referenceCaptured ? attitude.referencePitchDegrees : 0)
        let slopePitch = (attitude.pitchDegrees - attitude.referencePitchDegrees - attitude.correctionPitchDegrees) * .pi / 180
        let slopeRoll = (attitude.rollDegrees - attitude.referenceRollDegrees - attitude.correctionRollDegrees) * .pi / 180

        odometryValid = state.odometryValid
        if state.odometryValid {
            let current = (x: state.odometryX, z: state.odometryZ, yaw: state.odometryYaw)
            if let last = lastOdometry {
                // Step in Aura's own body frame, then turned by the filtered heading.
                let dx = current.x - last.x, dz = current.z - last.z
                let c = cos(last.yaw), s = sin(last.yaw)
                let forward = c * dx + s * dz, left = -s * dx + c * dz
                if (forward * forward + left * left).squareRoot() < 60 {
                    let h = filter.heading, ch = cos(h), sh = sin(h)
                    position.x += ch * forward - sh * left
                    position.z += sh * forward + ch * left
                    position.y += forward * sin(slopePitch) - left * sin(slopeRoll)
                }
            }
            lastOdometry = current
        } else {
            lastOdometry = nil      // Aura restarts its odometry at the next arm
        }

        if recording {
            let point = RoutePoint(t: clock - recordStart, x: position.x, y: position.y, z: position.z,
                                   heading: filter.heading, roll: roll, pitch: pitch)
            if let last = points.last {
                let moved = ((point.x - last.x) * (point.x - last.x) + (point.z - last.z) * (point.z - last.z)).squareRoot()
                if moved >= 10 || point.t - last.t >= 0.5 { points.append(point) }
            } else {
                points.append(point)
            }
        }
        dirty = true
    }

    func start() {
        points.removeAll()
        position = .zero
        filter.reset(keepBias: true)
        recordStart = clock
        recording = true
        dirty = true
    }

    func stop() {
        recording = false
        dirty = true
    }

    func clear() {
        points.removeAll()
        position = .zero
        filter.reset(keepBias: true)
        dirty = true
    }

    func save() {
        guard points.count > 1 else { return }
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.dateFormat = "yyyy-MM-dd HH-mm-ss"
        let record = RouteRecord(name: formatter.string(from: Date()), created: Date(), points: points)
        do {
            try FileManager.default.createDirectory(at: Self.folder, withIntermediateDirectories: true)
            let encoder = JSONEncoder()
            encoder.outputFormatting = [.prettyPrinted]
            encoder.dateEncodingStrategy = .iso8601
            try encoder.encode(record).write(to: Self.folder.appendingPathComponent(record.name + ".json"))
            loadSaved()
        } catch {
            NSSound.beep()
        }
    }

    func delete(_ record: RouteRecord) {
        try? FileManager.default.removeItem(at: Self.folder.appendingPathComponent(record.name + ".json"))
        if comparison?.id == record.id { comparison = nil }
        loadSaved()
    }

    func loadSaved() {
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        let files = (try? FileManager.default.contentsOfDirectory(at: Self.folder, includingPropertiesForKeys: nil)) ?? []
        saved = files.filter { $0.pathExtension == "json" }
            .compactMap { try? decoder.decode(RouteRecord.self, from: Data(contentsOf: $0)) }
            .sorted { $0.created > $1.created }
    }

    func exportCSV(_ record: RouteRecord?) {
        let source = record?.points ?? points
        guard !source.isEmpty else { return }
        let panel = NSSavePanel()
        panel.allowedContentTypes = [.commaSeparatedText]
        panel.nameFieldStringValue = (record?.name ?? "trasa") + ".csv"
        guard panel.runModal() == .OK, let url = panel.url else { return }
        var lines = ["t_s,x_mm,y_mm,z_mm,heading_deg,roll_deg,pitch_deg"]
        lines += source.map {
            String(format: "%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f",
                   $0.t, $0.x, $0.y, $0.z, $0.heading * 180 / .pi, $0.roll, $0.pitch)
        }
        try? (lines.joined(separator: "\n") + "\n").write(to: url, atomically: true, encoding: .utf8)
    }
}
