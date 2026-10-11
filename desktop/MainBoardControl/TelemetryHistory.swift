import Combine
import Foundation

// One 40 ms snapshot of everything Aura reports, kept for the charts.
struct TelemetrySample {
    var time: Double                 // s since the app started recording
    var busVoltage: Double
    var current: Double
    var power: Double
    var roll: Double, pitch: Double  // measured, deg
    var referenceRoll: Double, referencePitch: Double
    var correctionRoll: Double, correctionPitch: Double
    var gyro: SIMD3<Double>          // deg/s, Aura frame (X fwd, Y up, Z left)
    var accel: SIMD3<Double>         // g
    var imuTemperature: Double
    var servoTemperature: [Double]   // 12, LF RF LR RR x ab/ad hip knee
    var trackingError: [Double]      // 12, target - measured, deg
    var bodyHeight: Double
    var phase: Double
    var forward: Double, lateral: Double, turn: Double
    var bodyShiftX: Double, bodyShiftZ: Double
    var odometryX: Double, odometryZ: Double, odometryYaw: Double
    var armed: Bool
    var mode: Int
    var contacts: UInt8
}

// Ring buffer of TelemetrySample at 25 Hz, 10 minutes deep. Appends come from
// the 50 Hz network path; observers are told about new data at most 4 times
// a second, so charts never redraw at the transport rate.
@MainActor
final class TelemetryHistory: ObservableObject {
    static let capacity = 25 * 600

    @Published private(set) var revision = 0
    @Published var paused = false

    private var buffer: [TelemetrySample] = []
    private var head = 0
    private var frameParity = false
    private let start = ProcessInfo.processInfo.systemUptime
    private var dirty = false
    private var timer: AnyCancellable?

    init() {
        buffer.reserveCapacity(Self.capacity)
        timer = Timer.publish(every: 0.25, on: .main, in: .common).autoconnect().sink { [weak self] _ in
            guard let self, self.dirty, !self.paused else { return }
            self.dirty = false
            self.revision &+= 1
        }
    }

    var now: Double { ProcessInfo.processInfo.systemUptime - start }
    var latest: TelemetrySample? { buffer.isEmpty ? nil : buffer[(head - 1 + buffer.count) % buffer.count] }
    var count: Int { buffer.count }

    func append(_ make: (Double) -> TelemetrySample) {
        // Pause freezes the charts only; recording continues underneath.
        frameParity.toggle()
        guard frameParity else { return }
        let sample = make(now)
        if buffer.count < Self.capacity {
            buffer.append(sample)
            head = buffer.count % Self.capacity
        } else {
            buffer[head] = sample
            head = (head + 1) % Self.capacity
        }
        dirty = true
    }

    func clear() {
        buffer.removeAll(keepingCapacity: true)
        head = 0
        revision &+= 1
    }

    // Chronological samples of the last `seconds`, thinned to about
    // `maximumPoints` so a chart stays cheap whatever the window.
    func window(seconds: Double, maximumPoints: Int = 400) -> [TelemetrySample] {
        guard !buffer.isEmpty else { return [] }
        let ordered = buffer.count < Self.capacity ? buffer : Array(buffer[head...] + buffer[..<head])
        let cutoff = (ordered.last?.time ?? 0) - seconds
        let first = ordered.firstIndex { $0.time >= cutoff } ?? ordered.startIndex
        let slice = ordered[first...]
        let stride = max(1, slice.count / maximumPoints)
        return stride == 1 ? Array(slice) : slice.enumerated().compactMap { $0.offset % stride == 0 ? $0.element : nil }
    }

    func csv() -> String {
        let ordered = buffer.count < Self.capacity ? buffer : Array(buffer[head...] + buffer[..<head])
        let legs = ["LF", "RF", "LR", "RR"], axes = ["abad", "hip", "knee"]
        let joints = legs.flatMap { leg in axes.map { "\(leg)_\($0)" } }
        var header = ["t_s", "bus_V", "current_A", "power_W", "roll_deg", "pitch_deg",
                      "ref_roll_deg", "ref_pitch_deg", "corr_roll_deg", "corr_pitch_deg",
                      "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "acc_x_g", "acc_y_g", "acc_z_g",
                      "imu_temp_C", "body_height_mm", "phase", "forward", "lateral", "turn",
                      "body_shift_x_mm", "body_shift_z_mm", "odo_x_mm", "odo_z_mm", "odo_yaw_deg",
                      "armed", "mode", "contacts"]
        header += joints.map { "temp_\($0)_C" } + joints.map { "err_\($0)_deg" }
        var lines = [header.joined(separator: ",")]
        for s in ordered {
            var fields: [String] = [s.time, s.busVoltage, s.current, s.power, s.roll, s.pitch,
                                    s.referenceRoll, s.referencePitch, s.correctionRoll, s.correctionPitch,
                                    s.gyro.x, s.gyro.y, s.gyro.z, s.accel.x, s.accel.y, s.accel.z,
                                    s.imuTemperature, s.bodyHeight, s.phase, s.forward, s.lateral, s.turn,
                                    s.bodyShiftX, s.bodyShiftZ, s.odometryX, s.odometryZ,
                                    s.odometryYaw * 180 / .pi].map { String(format: "%.4g", $0) }
            fields += [s.armed ? "1" : "0", "\(s.mode)", "\(s.contacts)"]
            fields += s.servoTemperature.map { String(format: "%.0f", $0) }
            fields += s.trackingError.map { String(format: "%.2f", $0) }
            lines.append(fields.joined(separator: ","))
        }
        return lines.joined(separator: "\n") + "\n"
    }
}
