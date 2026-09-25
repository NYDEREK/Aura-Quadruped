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
        ["LF · lewy przód", "RF · prawy przód", "LR · lewy tył", "RR · prawy tył"]
            .indices.contains(leg) ? ["LF · lewy przód", "RF · prawy przód", "LR · lewy tył", "RR · prawy tył"][leg] : "?"
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
