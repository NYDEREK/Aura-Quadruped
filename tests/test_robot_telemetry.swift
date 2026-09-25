import Foundation
@main struct TestRobotTelemetry {
    static func main() {
        var state = AuraRobotState()
        var body = Data(repeating: 0, count: 20)
        body[0]=0x31; body[1]=3; body[2]=120; body[4]=80
        body[10]=100; body[14]=1; body[15]=2; body[16]=1; body[18]=2; body[19]=35
        state.applyBodyFrame(body)
        state.odometryValid=true; state.odometryX=123
        state.safetyFault.latched=true
        var core = Data(repeating: 0, count: 20)
        core[0]=0x27; core[2]=15; core[3]=5; core[4]=5; core[19]=50
        for i in 0..<100 {
            core[15]=UInt8(i)
            state.applyCoreFrame(core)
            precondition(state.selectedGait==5 && state.armed && state.tickCount==UInt32(i))
            precondition(state.bodyReferenceValid && state.bodyPreviewActive)
            precondition(state.bodyShiftX==12 && state.bodyShiftZ==8)
            precondition(state.tripodWalkEnabled && state.tripodExcludedLeg==2)
            precondition(state.balanceExtensionAvailable && state.swingBalanceActive)
            precondition(state.effectiveFrequencyHz==0.70)
            precondition(state.odometryValid && state.odometryX==123 && state.safetyFault.latched)
        }
        body[18]=1; state.applyBodyFrame(body)
        precondition(state.balanceExtensionAvailable && state.effectiveFrequencyHz==nil)
        let previous=state
        state.applyCoreFrame(Data([0x27])); state.applyBodyFrame(Data([0x31]))
        precondition(state==previous)
        print("telemetry packet merge, legacy schema and malformed frames: passed")
    }
}
