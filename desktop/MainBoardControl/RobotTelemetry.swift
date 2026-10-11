import Foundation

struct AuraRobotSafetyFault: Equatable {
    var latched = false
    // -1 means that no emergency stop is currently latched.
    var leg = -1
    var axis = -1
    var servoID = 0
    var currentRaw = 0
    var loadRaw = 0
    var trackingErrorDegrees = 0.0
    var confirmations = 0

    var currentAmperes: Double { Double(abs(currentRaw)) * 0.0065 }
    var loadPercent: Double { Double(abs(loadRaw)) / 10.0 }
    var legTitle: String {
        ["LF · left front", "RF · right front", "LR · left rear", "RR · right rear"]
            .indices.contains(leg) ? ["LF · left front", "RF · right front", "LR · left rear", "RR · right rear"][leg] : "?"
    }
    var axisTitle: String { ["Ab/ad", "Hip", "Knee"].indices.contains(axis) ? ["Ab/ad", "Hip", "Knee"][axis] : "?" }
}


struct AuraRobotState: Equatable {
    var armed = false
    // Aura sends the selected single-leg test slot as 0...3, or nil in
    // normal operation. The desktop uses it only for rendering.
    var calibrationLeg: Int? = nil
    var configurationComplete = false
    var controllerConnected = false
    var controllerHasInput = false
    var spinMode = false
    var jumping = false
    var trackingLimited = false
    var phaseRate = 1.0
    var bodyReferenceValid = false
    var bodyPreviewActive = false
    var swingBalanceActive = false
    var constrainedLegMask = 0
    var effectiveFrequencyHz: Double?
    var tripodWalkEnabled = false
    var tripodExcludedLeg = 3
    var balanceExtensionAvailable = false
    var bodyShiftX = 0.0
    var bodyShiftZ = 0.0
    var bodyReferenceRoll = 0.0
    var bodyReferencePitch = 0.0
    var bodyReferenceTick: UInt32 = 0
    var selectedGait = 0
    var activeGait = 0
    var phase = 0.0
    var forward = 0.0
    var lateral = 0.0
    var turn = 0.0
    var bodyHeight = 0
    var tickCount: UInt32 = 0
    var virtualInput = false
    var odometryValid = false
    var odometryX = 0.0
    var odometryZ = 0.0
    var odometryYaw = 0.0
    var safetyFault = AuraRobotSafetyFault()
    var estimate = AuraEstimate()
}

// Aura's on-board Kalman estimate (frame 0x32): IK frame, X forward at the
// last re-zero, Y up, Z left; heading > 0 is a left turn.
// Frame 0x33: why the board last restarted and how fast each of its tasks
// is looping. A task at 0 Hz while telemetry still arrives has stopped.
struct AuraHealth: Equatable {
    static let tasks = ["Gait", "Servo control", "Servo feedback", "Pad", "Commands",
                        "Network", "Telemetry", "Estimator", "Sensors"]
    var available = false
    var resetReason = 0
    var beats = [Int](repeating: 0, count: tasks.count)
    var rates = [Double](repeating: -1, count: tasks.count)   // loops per second, -1 = not measured yet
    var stackFree = [Int](repeating: 0, count: tasks.count)   // least free stack since boot, bytes (0 = unknown)
    var gaitTickMaxMs = 0                                      // longest gait tick in the last 100 ms (budget 20)
    var stackAvailable = false

    // Frame 0x35: longest time of each gait tick part in the last 100 ms and
    // the board's free RAM.
    static let sections = ["input", "posture", "planning", "joints", "estimator"]
    var timingAvailable = false
    var sectionMs = [Double](repeating: 0, count: sections.count)
    var heapFree = 0, heapMinimum = 0
    var slowestSection: (name: String, ms: Double) {
        let index = sectionMs.indices.max { sectionMs[$0] < sectionMs[$1] } ?? 0
        return (Self.sections[index], sectionMs[index])
    }
    static let lowHeapBytes = 10_000

    static let lowStackBytes = 512
    var lowStackTasks: [String] {
        Self.tasks.indices.filter { stackFree[$0] > 0 && stackFree[$0] < Self.lowStackBytes }.map { Self.tasks[$0] }
    }

    var resetText: String {
        switch resetReason {
        case 1: "power on"
        case 2: "reset pin"
        case 3: "software"
        case 4: "crash (panic)"
        case 5: "interrupt watchdog"
        case 6: "task watchdog"
        case 7: "watchdog"
        case 8: "deep sleep"
        case 9: "brownout"
        case 10: "SDIO"
        case 11: "USB"
        case 12: "JTAG"
        case 13: "eFuse"
        case 14: "power glitch"
        case 15: "CPU lockup"
        default: "unknown"
        }
    }
    var resetAbnormal: Bool { [4, 5, 6, 7, 9, 14, 15].contains(resetReason) }
    func stalled(_ task: Int) -> Bool { rates.indices.contains(task) && rates[task] == 0 }
    var stalledTasks: [String] { Self.tasks.indices.filter(stalled).map { Self.tasks[$0] } }
}

struct AuraEstimate: Equatable {
    var available = false          // the firmware sends frame 0x32
    var valid = false
    var still = false
    var x = 0.0, y = 0.0, z = 0.0  // mm; y is the ground height under Aura, not the torso
    var heading = 0.0              // rad
    var forward = 0.0, left = 0.0  // mm/s in the heading frame
    var gyroBias = 0.0             // deg/s
    var speed: Double { (forward * forward + left * left).squareRoot() }
}

extension AuraRobotState {
    mutating func applyCoreFrame(_ data: Data) {
        guard data.count == 20, data[0] == 0x27 else { return }
        let flags = data[2]
        // This packet updates the core state only. Other packet types
        // own body reference, odometry and fault data; resetting the
        // whole struct here made those values flicker every 20 ms.
        armed = flags & 1 != 0
        calibrationLeg = data[1] == 0 ? nil : Int(data[1] - 1)
        configurationComplete = flags & 2 != 0
        controllerConnected = flags & 4 != 0
        controllerHasInput = flags & 8 != 0
        spinMode = flags & 16 != 0
        jumping = flags & 32 != 0
        trackingLimited = flags & 64 != 0
        phaseRate = Double(data[19]) / 100.0
        selectedGait = Int(data[3])
        activeGait = Int(data[4])
        phase = Double(u16(data, 5)) / 1000.0
        forward = Double(i16(data, 7)) / 1000.0
        lateral = Double(i16(data, 9)) / 1000.0
        turn = Double(i16(data, 11)) / 1000.0
        bodyHeight = Int(i16(data, 13))
        tickCount = u32(data, 15)
        virtualInput = flags & 128 != 0
    }
    mutating func applyEstimateFrame(_ data: Data) {
        guard data.count == 20, data[0] == 0x32 else { return }
        estimate = AuraEstimate(
            available: true, valid: data[1] & 1 != 0, still: data[1] & 2 != 0,
            x: Double(Int32(bitPattern: u32(data, 2))) / 10, y: Double(i16(data, 10)),
            z: Double(Int32(bitPattern: u32(data, 6))) / 10,
            heading: Double(i16(data, 12)) * .pi / 18000,
            forward: Double(i16(data, 14)), left: Double(i16(data, 16)),
            gyroBias: Double(i16(data, 18)) / 1000)
    }
    mutating func applyBodyFrame(_ data: Data) {
        guard data.count == 20, data[0] == 0x31 else { return }
        bodyReferenceValid = data[1] & 1 != 0
        bodyPreviewActive = data[1] & 2 != 0
        bodyShiftX = Double(i16(data, 2)) / 10
        bodyShiftZ = Double(i16(data, 4)) / 10
        bodyReferenceRoll = Double(i16(data, 6)) * .pi / 18000
        bodyReferencePitch = Double(i16(data, 8)) * .pi / 18000
        bodyReferenceTick = u32(data, 10)
        balanceExtensionAvailable = data[18] >= 1
        effectiveFrequencyHz = data[18] >= 2 ? Double(data[19]) / 50 : nil
        tripodWalkEnabled = data[14] != 0
        tripodExcludedLeg = Int(data[15] & 3)
        swingBalanceActive = data[16] != 0
        constrainedLegMask = Int(data[17])
    }

    private func u16(_ data: Data, _ offset: Int) -> UInt16 {
        UInt16(data[offset]) | (UInt16(data[offset + 1]) << 8)
    }
    private func i16(_ data: Data, _ offset: Int) -> Int16 {
        Int16(bitPattern: u16(data, offset))
    }
    private func u32(_ data: Data, _ offset: Int) -> UInt32 {
        UInt32(data[offset]) | (UInt32(data[offset + 1]) << 8) |
        (UInt32(data[offset + 2]) << 16) | (UInt32(data[offset + 3]) << 24)
    }
}

// Pad modes chosen on Aura with the D-pad; telemetry byte 3.
enum AuraPadMode {
    static func title(_ mode: Int) -> String {
        switch mode {
        case 1: "1 · BALANCE"
        case 2: "2 · TORSO POSE"
        case 3: "3 · WALK"
        case 4: "4 · TROT (TEST)"
        case 5: "5 · STAND"
        default: "—"
        }
    }
}
