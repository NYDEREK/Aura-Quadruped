import AppKit
import Combine
import Foundation
import UniformTypeIdentifiers

// The route comes from Aura's on-board Kalman filter (frame 0x32): IMU
// acceleration and gyro fused with leg kinematics, heading with gyro bias
// estimation. Firmware without that frame falls back to the older desktop
// estimate: leg odometry plus a gyro heading filter.
// Ground frame: X forward, Y up, Z left, mm.

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
    enum Source { case none, aura, desktop }

    @Published private(set) var revision = 0
    @Published private(set) var recording = false
    @Published private(set) var saved: [RouteRecord] = []
    @Published var comparison: RouteRecord?

    // Live pose, also while not recording, so the 3D view always shows Aura.
    private(set) var position = SIMD3<Double>(repeating: 0)
    private(set) var heading = 0.0
    private(set) var roll = 0.0, pitch = 0.0
    private(set) var speed = 0.0              // mm/s, 1 s average
    private(set) var maxSpeed = 0.0           // of the recording
    private(set) var gyroBias = 0.0           // deg/s
    private(set) var still = false
    private(set) var valid = false
    private(set) var source = Source.none
    private(set) var points: [RoutePoint] = []

    // Desktop fallback state.
    private var filter = HeadingFilter()
    private var lastOdometry: (x: Double, z: Double, yaw: Double)?
    private var stillTime = 0.0

    private var lastTick: UInt32?
    private var origin = SIMD3<Double>(repeating: 0)
    private var raw = SIMD3<Double>(repeating: 0)
    private var speedWindow: [(t: Double, p: SIMD3<Double>)] = []
    private var recordStart = 0.0
    private var clock = 0.0
    private var dirty = false
    private var timer: AnyCancellable?

    static var folder: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("Aura/Routes", isDirectory: true)
    }
    private static var legacyFolder: URL {
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
        roll = attitude.rollDegrees - (attitude.referenceCaptured ? attitude.referenceRollDegrees : 0)
        pitch = attitude.pitchDegrees - (attitude.referenceCaptured ? attitude.referencePitchDegrees : 0)

        if state.estimate.available {
            source = .aura
            let e = state.estimate
            raw = SIMD3(e.x, e.y, e.z)
            heading = e.heading
            gyroBias = e.gyroBias
            still = e.still
            valid = e.valid
        } else {
            source = .desktop
            desktopEstimate(state: state, imu: imu, attitude: attitude, dt: dt)
        }
        position = raw - origin

        // Speed over the last second of positions: the gait's own sway
        // and step-to-step jitter do not count as speed.
        speedWindow.append((clock, position))
        while let first = speedWindow.first, clock - first.t > 1.0 { speedWindow.removeFirst() }
        if let first = speedWindow.first, clock - first.t > 0.5 {
            let d = position - first.p
            speed = (d.x * d.x + d.z * d.z).squareRoot() / (clock - first.t)
        }

        if recording {
            maxSpeed = max(maxSpeed, speed)
            let point = RoutePoint(t: clock - recordStart, x: position.x, y: position.y, z: position.z,
                                   heading: heading, roll: roll, pitch: pitch)
            if let last = points.last {
                let moved = ((point.x - last.x) * (point.x - last.x) + (point.z - last.z) * (point.z - last.z)).squareRoot()
                if moved >= 10 || point.t - last.t >= 0.5 { points.append(point) }
            } else {
                points.append(point)
            }
        }
        dirty = true
    }

    // Older firmware: leg odometry turned by a desktop gyro heading filter.
    private func desktopEstimate(state: AuraRobotState, imu: AuraIMU, attitude: AuraAttitude, dt: Double) {
        let yawRate = imu.gyroscope[1] * .pi / 180
        filter.predict(gyroRate: yawRate, dt: dt)
        let norm = (imu.acceleration[0] * imu.acceleration[0] + imu.acceleration[1] * imu.acceleration[1] +
                    imu.acceleration[2] * imu.acceleration[2]).squareRoot()
        let commanded = abs(state.forward) + abs(state.lateral) + abs(state.turn) > 0.02
        let quiet = !commanded && abs(norm - 1) < 0.04 && imu.gyroscope.allSatisfy { abs($0) < 4 }
        stillTime = quiet ? stillTime + dt : 0
        still = stillTime > 0.5
        if still { filter.observeStill(gyroRate: yawRate) }
        heading = filter.heading
        gyroBias = filter.bias * 180 / .pi
        valid = state.odometryValid
        if state.odometryValid {
            let current = (x: state.odometryX, z: state.odometryZ, yaw: state.odometryYaw)
            if let last = lastOdometry {
                let dx = current.x - last.x, dz = current.z - last.z
                let c = cos(last.yaw), s = sin(last.yaw)
                let forward = c * dx + s * dz, left = -s * dx + c * dz
                if (forward * forward + left * left).squareRoot() < 60 {
                    let ch = cos(heading), sh = sin(heading)
                    raw.x += ch * forward - sh * left
                    raw.z += sh * forward + ch * left
                }
            }
            lastOdometry = current
        } else {
            lastOdometry = nil
        }
    }

    func start() {
        points.removeAll()
        origin = raw
        position = .zero
        maxSpeed = 0
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
        origin = raw
        position = .zero
        maxSpeed = 0
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
        for folder in [Self.folder, Self.legacyFolder] {
            try? FileManager.default.removeItem(at: folder.appendingPathComponent(record.name + ".json"))
        }
        if comparison?.id == record.id { comparison = nil }
        loadSaved()
    }

    func loadSaved() {
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        let files = [Self.folder, Self.legacyFolder].flatMap {
            (try? FileManager.default.contentsOfDirectory(at: $0, includingPropertiesForKeys: nil)) ?? []
        }
        saved = files.filter { $0.pathExtension == "json" }
            .compactMap { try? decoder.decode(RouteRecord.self, from: Data(contentsOf: $0)) }
            .sorted { $0.created > $1.created }
    }

    func exportCSV(_ record: RouteRecord?) {
        let source = record?.points ?? points
        guard !source.isEmpty else { return }
        let panel = NSSavePanel()
        panel.allowedContentTypes = [.commaSeparatedText]
        panel.nameFieldStringValue = (record?.name ?? "route") + ".csv"
        guard panel.runModal() == .OK, let url = panel.url else { return }
        var lines = ["t_s,x_mm,y_mm,z_mm,heading_deg,roll_deg,pitch_deg"]
        lines += source.map {
            String(format: "%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f",
                   $0.t, $0.x, $0.y, $0.z, $0.heading * 180 / .pi, $0.roll, $0.pitch)
        }
        try? (lines.joined(separator: "\n") + "\n").write(to: url, atomically: true, encoding: .utf8)
    }
}
