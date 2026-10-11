import AppKit
import Foundation
import SceneKit
import SwiftUI
import simd

enum CalibrationStep: Int, CaseIterable, Identifiable {
    case zero, direction, limits, test

    var id: Int { rawValue }
    var title: String {
        switch self {
        case .zero: "1. Zero"
        case .direction: "2. Direction"
        case .limits: "3. Limits"
        case .test: "4. Leg test"
        }
    }
    var instruction: String {
        switch self {
        case .zero:
            "Set the leg exactly as in the model. Torque is off. Aura then reads the three real positions and stores them as this leg's 0°."
        case .direction:
            "Pick an axis. Aura moves only that axis +12° from the reference. The green arrow shows the expected direction; saving a reversed direction makes the same +12° test move the other way."
        case .limits:
            "Release one axis, move it by hand to its safe negative end and save. Then move it to the other end and save the positive limit. Limits are stored on Aura."
        case .test:
            "Aura drives only the selected leg, without the Mac. Left stick up/down runs a small step cycle, left/right checks ab/ad. Stop the test before moving on."
        }
    }
}

@MainActor
final class CalibrationStore: ObservableObject {
    // Mechanical order and ID groups: LF 1-3, LR 4-6, RF 7-9, RR 10-12.
    let order: [RobotLegID] = RobotLegID.physicalOrder
    @Published var legIndex = 0
    @Published var step: CalibrationStep = .zero
    @Published var axisIndex = 0
    // 0 means the reference pose is shown. The direction probe is always the
    // same logical +12° movement; a saved direction flip reflects it physically.
    @Published var directionTestOffset = 0.0
    @Published var zeroRecorded = [false, false, false]
    @Published var testRunning = false
    @Published var previewMode: CalibrationPreviewMode = .still
    @Published var finishedLegs: Set<RobotLegID> = []

    var leg: RobotLegID { order[legIndex] }

    func startLeg(_ index: Int) {
        legIndex = max(0, min(order.count - 1, index))
        step = .zero
        axisIndex = 0
        directionTestOffset = 0
        zeroRecorded = [false, false, false]
        testRunning = false
    }

    func prepareZeroCapture() {
        zeroRecorded = [false, false, false]
    }

    func markZero(axis: Int) {
        guard zeroRecorded.indices.contains(axis) else { return }
        zeroRecorded[axis] = true
    }

    func beginDirection() {
        guard zeroRecorded.allSatisfy({ $0 }) else { return }
        axisIndex = 0
        directionTestOffset = 0
        step = .direction
    }

    func acceptDirection() {
        directionTestOffset = 0
        if axisIndex < 2 { axisIndex += 1 }
        else { axisIndex = 0; step = .limits }
    }

    func beginTest() {
        axisIndex = 0
        step = .test
    }

    func finishLeg() {
        testRunning = false
        finishedLegs.insert(leg)
        if legIndex < order.count - 1 { startLeg(legIndex + 1) }
    }

    func previousStep() {
        testRunning = false
        switch step {
        case .zero: break
        case .direction: step = .zero
        case .limits: step = .direction
        case .test: step = .limits
        }
    }
}

private struct CalibrationVector {
    var x: Float
    var y: Float
    var z: Float

    static func + (lhs: Self, rhs: Self) -> Self {
        .init(x: lhs.x + rhs.x, y: lhs.y + rhs.y, z: lhs.z + rhs.z)
    }

    var simd: SIMD3<Float> { SIMD3(x, y, z) }
}

private final class CalibrationLegRenderer: NSObject {
    let scene = SCNScene()
    let camera = SCNNode()
    private let root = SCNNode()
    private var rods: [SCNNode] = []
    private var joints: [SCNNode] = []
    private let foot = SCNNode(geometry: SCNSphere(radius: 5))
    private let chassis = SCNNode()
    private var highlightedAxis = -1

    override init() {
        super.init()
        scene.background.contents = NSColor(calibratedWhite: 0.075, alpha: 1)
        scene.rootNode.addChildNode(root)

        camera.camera = SCNCamera()
        camera.camera?.zFar = 1500
        camera.position = SCNVector3(280, 200, 330)
        let lookAt = SCNLookAtConstraint(target: root)
        lookAt.isGimbalLockEnabled = true
        camera.constraints = [lookAt]
        scene.rootNode.addChildNode(camera)

        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 650
        scene.rootNode.addChildNode(ambient)
        let key = SCNNode()
        key.light = SCNLight()
        key.light?.type = .omni
        key.light?.intensity = 1100
        key.position = SCNVector3(180, 250, 220)
        scene.rootNode.addChildNode(key)

        let floor = SCNFloor()
        floor.reflectivity = 0
        let floorMaterial = SCNMaterial()
        floorMaterial.diffuse.contents = NSColor(calibratedWhite: 0.16, alpha: 1)
        floor.firstMaterial = floorMaterial
        scene.rootNode.addChildNode(SCNNode(geometry: floor))

        // Keep the calibrated leg attached to a small body reference. A free
        // leg looked identical for left and right sides, which made the
        // calibration preview misleading even when it targeted the right
        // physical servo records.
        let chassisGeometry = SCNBox(width: 360, height: 12, length: 130, chamferRadius: 9)
        chassisGeometry.firstMaterial = material(NSColor(calibratedWhite: 0.28, alpha: 1))
        chassis.geometry = chassisGeometry
        chassis.position = SCNVector3(0, 118, 0)
        root.addChildNode(chassis)

        rods = (0..<3).map { _ in
            let node = SCNNode(geometry: SCNCylinder(radius: 3, height: 1))
            root.addChildNode(node)
            return node
        }
        joints = (0..<3).map { _ in
            let node = SCNNode(geometry: SCNSphere(radius: 5.5))
            root.addChildNode(node)
            return node
        }
        root.addChildNode(foot)
        update(angles: [0, 0, -90], leg: .rf, highlightedAxis: -1)
    }

    private func material(_ color: NSColor) -> SCNMaterial {
        let material = SCNMaterial()
        material.diffuse.contents = color
        material.metalness.contents = 0.25
        material.roughness.contents = 0.35
        return material
    }

    private func setRod(_ node: SCNNode, _ start: CalibrationVector, _ end: CalibrationVector, color: NSColor) {
        let a = start.simd
        let b = end.simd
        let vector = b - a
        let length = simd_length(vector)
        guard length > 0.01 else { return }
        if let cylinder = node.geometry as? SCNCylinder {
            if abs(cylinder.height - CGFloat(length)) > 0.01 { cylinder.height = CGFloat(length) }
            cylinder.firstMaterial = material(color)
        }
        node.position = SCNVector3((a + b) / 2)
        node.simdOrientation = simd_quatf(from: SIMD3<Float>(0, 1, 0), to: simd_normalize(vector))
    }

    func update(angles degrees: [Double], leg: RobotLegID, highlightedAxis: Int) {
        let q = (0..<3).map { Float((degrees.indices.contains($0) ? degrees[$0] : 0) * .pi / 180.0) }
        let abduction = q[0]
        let hip = q[1]
        let knee = q[2]
        // The assembly view is opposite SceneKit's mathematical frame in
        // both horizontal directions. Mirror X and Z together so FL is shown
        // as the physical front-left corner, never as rear-right. This is
        // display-only: it does not change a leg number or any servo record.
        let front: Float = (leg == .lf || leg == .rf) ? -1 : 1
        let left: Float = leg.isLeft ? 1 : -1
        let frameAngle: Float = left > 0 ? .pi / 2 : -.pi / 2
        let anchor = CalibrationVector(x: front * 180, y: 118, z: left * 65)
        let local0 = CalibrationVector(x: 0, y: 0, z: 0)
        let local1 = local0 + CalibrationVector(x: -35 * cos(abduction), y: -35 * sin(abduction), z: 0)
        let local2 = local1 + CalibrationVector(x: 130 * sin(abduction) * cos(hip),
                                                  y: -130 * cos(abduction) * cos(hip), z: 130 * sin(hip))
        let sum = hip + knee
        let local3 = local2 + CalibrationVector(x: 205 * sin(abduction) * cos(sum),
                                                  y: -205 * cos(abduction) * cos(sum), z: 205 * sin(sum))
        func toBody(_ point: CalibrationVector) -> CalibrationVector {
            .init(x: anchor.x + cos(frameAngle) * point.x + sin(frameAngle) * point.z,
                  y: anchor.y + point.y,
                  z: anchor.z - sin(frameAngle) * point.x + cos(frameAngle) * point.z)
        }
        let points = [local0, local1, local2, local3].map(toBody)
        let nominal = NSColor(red: 0.12, green: 0.62, blue: 0.92, alpha: 1)
        let active = NSColor.systemYellow
        for index in 0..<3 {
            setRod(rods[index], points[index], points[index + 1], color: index == highlightedAxis ? active : nominal)
            joints[index].position = SCNVector3(points[index].simd)
            joints[index].geometry?.firstMaterial = material(index == highlightedAxis ? active : NSColor.white)
        }
        foot.position = SCNVector3(points[3].simd)
        foot.geometry?.firstMaterial = material(nominal)
        self.highlightedAxis = highlightedAxis
    }
}

private struct CalibrationLegScene: NSViewRepresentable {
    let angles: [Double]
    let leg: RobotLegID
    let highlightedAxis: Int

    func makeCoordinator() -> CalibrationLegRenderer { CalibrationLegRenderer() }

    func makeNSView(context: Context) -> SCNView {
        let view = SCNView()
        view.scene = context.coordinator.scene
        view.pointOfView = context.coordinator.camera
        view.backgroundColor = .clear
        view.allowsCameraControl = true
        view.autoenablesDefaultLighting = false
        view.antialiasingMode = .none
        view.rendersContinuously = false
        return view
    }

    func updateNSView(_ view: SCNView, context: Context) {
        context.coordinator.update(angles: angles, leg: leg, highlightedAxis: highlightedAxis)
    }
}

private struct CalibrationStepMarker: View {
    let step: CalibrationStep
    let active: Bool

    var body: some View {
        Text(step.title)
            .font(.caption.weight(active ? .semibold : .regular))
            .foregroundStyle(active ? .primary : .secondary)
            .padding(.horizontal, 9)
            .padding(.vertical, 6)
            .background(active ? Color.primary.opacity(0.14) : Color.clear, in: Capsule())
    }
}

struct CalibrationView: View {
    @EnvironmentObject private var aura: AuraConnection
    @EnvironmentObject private var robot: RobotControlStore
    @EnvironmentObject private var calibration: CalibrationStore

    private var currentLeg: RobotLegID { calibration.leg }
    private var currentLegIndex: Int { currentLeg.wireIndex }

    private var currentAxis: RobotAxisConfiguration? {
        robot.axes.first { $0.leg == currentLeg && RobotAxisID.allCases.firstIndex(of: $0.axis) == calibration.axisIndex }
    }

    private var legAxes: [RobotAxisConfiguration] {
        robot.axes.filter { $0.leg == currentLeg }.sorted {
            (RobotAxisID.allCases.firstIndex(of: $0.axis) ?? 0) < (RobotAxisID.allCases.firstIndex(of: $1.axis) ?? 0)
        }
    }

    private var legReady: Bool {
        legAxes.count == 3 && legAxes.allSatisfy(\.assigned)
    }

    private var kneeReferenceDegrees: Double { currentLeg.isLeft ? 90 : -90 }

    private var calibrationReferenceText: String {
        String(format: "0° / 0° / %+.0f°", kneeReferenceDegrees)
    }

    private var previewReferenceAngles: [Double] {
        // The static selected leg is the physical assembly reference. The
        // walking preview itself comes from the shared Quadruped planner.
        var angles = [0.0, 0.0, kneeReferenceDegrees]
        if calibration.step == .direction { angles[calibration.axisIndex] += calibration.directionTestOffset }
        return angles
    }

    private func referenceDegrees(for axis: Int) -> Double {
        axis == 2 ? kneeReferenceDegrees : 0
    }

    private var legTitle: String {
        switch currentLeg {
        case .lf: "FL • front left"
        case .lr: "RL • rear left"
        case .rf: "FR • front right"
        case .rr: "RR • rear right"
        }
    }

    private func selectAxis(_ index: Int) {
        calibration.axisIndex = index
        calibration.directionTestOffset = 0
        aura.setCalibrationLegTest(leg: currentLegIndex, enabled: false)
        calibration.testRunning = false
    }

    private let directionProbeDegrees = 12.0

    private var directionTargetDegrees: Double {
        referenceDegrees(for: calibration.axisIndex) + calibration.directionTestOffset
    }

    private var directionProbeWasSent: Bool { calibration.directionTestOffset != 0 }

    private func sendDirectionProbe() {
        calibration.directionTestOffset = directionProbeDegrees
        aura.nudgeCalibrationAxis(leg: currentLegIndex, axis: calibration.axisIndex,
                                  degrees: directionTargetDegrees)
    }

    private func releaseDirectionAxis() {
        aura.releaseCalibrationAxis(leg: currentLegIndex, axis: calibration.axisIndex)
        // A new probe is permitted only after torque has been explicitly
        // released. The firmware enforces the same rule independently.
        calibration.directionTestOffset = 0
    }

    private func flipDirectionAtManualReference() {
        // Direction changes must never move a mounted joint.  The operator
        // first releases torque and places the axis back at its reference;
        // Aura then only stores the reflected map from the current encoder.
        aura.flipCalibrationDirectionAtReference(leg: currentLegIndex,
                                                 axis: calibration.axisIndex,
                                                 referenceDegrees: referenceDegrees(for: calibration.axisIndex))
        calibration.directionTestOffset = 0
    }

    private func recaptureCurrentAxisReference() {
        aura.recaptureCalibrationAxisReference(leg: currentLegIndex,
                                               axis: calibration.axisIndex,
                                               referenceDegrees: referenceDegrees(for: calibration.axisIndex))
        calibration.directionTestOffset = 0
    }

    private func repairLegacyDirectionFlipAtReference() {
        // The earlier app build flipped the sign but left the knee's ±90°
        // reference centre unchanged. Keep the now-selected sign and rebuild
        // the mapping only after the user has placed the joint by hand.
        aura.repairLegacyDirectionFlipAtReference(leg: currentLegIndex,
                                                  axis: calibration.axisIndex,
                                                  referenceDegrees: referenceDegrees(for: calibration.axisIndex))
        calibration.directionTestOffset = 0
    }

    private func applyAuraConfiguration(_ configuration: AuraCalibrationAxisConfiguration?) {
        guard let configuration else { return }
        robot.applyCalibration(configuration)
    }

    private func applyZeroProgress(_ progress: AuraCalibrationZeroProgress?) {
        guard let progress, progress.leg == currentLegIndex else { return }
        for axis in progress.completedAxes { calibration.markZero(axis: axis) }
    }

    private func axisLabel(_ index: Int) -> String {
        RobotAxisID.allCases.indices.contains(index) ? RobotAxisID.allCases[index].rawValue : "Axis"
    }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                HStack(alignment: .center) {
                    StatusDot(color: aura.robotState.armed ? Theme.orange : Theme.green,
                              text: aura.robotState.armed ? "Disarm the servos first" : "Safe calibration · one leg at a time")
                    Spacer()
                }

                HStack(spacing: 6) {
                    ForEach(Array(calibration.order.enumerated()), id: \.element) { index, leg in
                        Button {
                            aura.setCalibrationLegTest(leg: currentLegIndex, enabled: false)
                            aura.setCalibrationLegTorque(leg: currentLegIndex, enabled: false)
                            calibration.startLeg(index)
                        } label: {
                            let done = calibration.finishedLegs.contains(leg)
                            let selected = index == calibration.legIndex
                            VStack(spacing: 5) {
                                Text(leg.displayCode + (done ? " ✓" : ""))
                                    .font(.system(size: 12, weight: selected ? .medium : .regular))
                                    .foregroundStyle(selected ? Theme.text : done ? Theme.green : Theme.muted)
                                Rectangle().fill(selected ? Theme.accent : .clear).frame(height: 2)
                            }
                            .padding(.horizontal, 8)
                            .fixedSize()
                        }
                        .buttonStyle(.plain)
                    }
                }

                HStack(spacing: 4) {
                    ForEach(CalibrationStep.allCases) { item in
                        CalibrationStepMarker(step: item, active: item == calibration.step)
                    }
                }

                if aura.robotState.armed {
                    ContentUnavailableView("Disarm Aura before calibrating", systemImage: "bolt.slash",
                                           description: Text("Zeros and limits cannot change while all 12 servos are under control."))
                } else if !legReady {
                    ContentUnavailableView("Assign this leg's three IDs", systemImage: "point.3.connected.trianglepath.dotted",
                                           description: Text("In Configuration, assign unique servos to Ab/ad, Hip and Knee, then come back."))
                } else {
                    HStack(alignment: .top, spacing: 16) {
                        let liveReadPreview = aura.calibrationLivePreviewActive
                        TimelineView(.animation(minimumInterval: 1.0 / 50.0,
                                                 paused: !(calibration.previewMode.isAnimated || calibration.testRunning || liveReadPreview))) { timeline in
                            CalibrationQuadrupedScene(leg: currentLeg,
                                                      referenceAngles: previewReferenceAngles,
                                                      directionReferenceAngles: calibration.step == .direction
                                                        ? [0, 0, kneeReferenceDegrees] : nil,
                                                      previewMode: calibration.previewMode,
                                                      liveTestTelemetry: calibration.testRunning || liveReadPreview,
                                                      allLegLivePreview: liveReadPreview,
                                                      highlightedAxis: calibration.step == .direction ? calibration.axisIndex : nil,
                                                      time: timeline.date.timeIntervalSinceReferenceDate,
                                                      channel: aura.robotRenderChannel)
                                .frame(minWidth: 380, minHeight: 420)
                                .clipShape(RoundedRectangle(cornerRadius: 6))
                                .overlay(alignment: .bottomLeading) {
                                    Text(liveReadPreview
                                         ? "TTL preview · all 12 servos, torque off"
                                         : calibration.testRunning
                                         ? "Pad test · \(currentLeg.displayCode) follows Aura's 50 Hz TTL reading"
                                         : calibration.previewMode == .trot
                                         ? "Forward walk as in Robot 3D · highlighted: \(currentLeg.displayCode)"
                                         : "Standing · highlighted leg at reference \(calibrationReferenceText)")
                                        .font(.caption)
                                        .foregroundStyle(.secondary)
                                        .padding(12)
                                }
                                .overlay(alignment: .topLeading) {
                                    if calibration.step == .direction {
                                        Label("Green arrow: expected +12° move",
                                              systemImage: "arrow.turn.up.right")
                                            .font(.caption.weight(.semibold))
                                            .foregroundStyle(.green)
                                            .padding(10)
                                            .background(.black.opacity(0.42), in: Capsule())
                                            .padding(12)
                                    }
                                }
                        }

                        VStack(alignment: .leading, spacing: 13) {
                            Text(legTitle).font(Theme.heading)
                            Text("ID " + legAxes.map { String($0.servoID) }.joined(separator: ", "))
                                .font(Theme.monoSmall).foregroundStyle(Theme.muted)
                            Text(calibration.step.instruction).foregroundStyle(.secondary)
                            Picker("Model preview", selection: $calibration.previewMode) {
                                ForEach(CalibrationPreviewMode.allCases) { mode in
                                    Text(mode.title).tag(mode)
                                }
                            }
                            .pickerStyle(.segmented)
                            .labelsHidden()
                            Button(liveReadPreview ? "Stop TTL preview" : "TTL preview, all legs") {
                                aura.setCalibrationLivePreview(enabled: !liveReadPreview)
                            }
                            .buttonStyle(AuraButtonStyle())
                            .disabled(aura.busy || aura.robotState.armed)
                            .help("Reads all 12 servo positions without arming or sending motion.")
                            Divider()
                            Button(aura.calibrationArmedLeg == currentLegIndex
                                   ? "Disarm this leg" : "Arm this leg only") {
                                aura.setCalibrationLegTorque(leg: currentLegIndex,
                                                             enabled: aura.calibrationArmedLeg != currentLegIndex)
                            }
                            .buttonStyle(AuraButtonStyle(prominent: true))
                            .disabled(aura.busy)
                            .help("Only IDs \(legAxes.map { String($0.servoID) }.joined(separator: ", ")); all other servos stay disarmed.")
                            Divider()
                            stepControls
                        }
                        .frame(width: 290, alignment: .leading)
                    }

                    GroupBox("Stored · \(legTitle)") {
                        Grid(alignment: .leading, horizontalSpacing: 16, verticalSpacing: 8) {
                            GridRow {
                                Text("Axis").foregroundStyle(.secondary)
                                Text("ID").foregroundStyle(.secondary)
                                Text("Zero").foregroundStyle(.secondary)
                                Text("Direction").foregroundStyle(.secondary)
                                Text("Min / Max").foregroundStyle(.secondary)
                            }
                            ForEach(legAxes) { axis in
                                GridRow {
                                    Text(axis.axis.rawValue).bold()
                                    Text("\(axis.servoID)").monospacedDigit()
                                    Text("\(axis.centerTick) tick").monospacedDigit()
                                    Text(axis.reversed ? "−" : "+").monospacedDigit()
                                    Text(String(format: "%+.1f° / %+.1f°", axis.minimumDegrees, axis.maximumDegrees)).monospacedDigit()
                                }
                            }
                        }
                        .font(.caption)
                        .padding(.vertical, 4)
                    }
                }
            }
            .padding()
        }
        .onChange(of: aura.calibrationAxisConfiguration) { _, configuration in
            applyAuraConfiguration(configuration)
        }
        .onChange(of: aura.calibrationZeroProgress) { _, progress in
            applyZeroProgress(progress)
        }
        .onDisappear {
            if calibration.testRunning {
                aura.setCalibrationLegTest(leg: currentLegIndex, enabled: false)
                calibration.testRunning = false
            }
            if aura.calibrationArmedLeg == currentLegIndex {
                aura.setCalibrationLegTorque(leg: currentLegIndex, enabled: false)
            }
            if aura.calibrationLivePreviewActive {
                aura.setCalibrationLivePreview(enabled: false)
            }
        }
    }

    @ViewBuilder
    private var stepControls: some View {
        switch calibration.step {
        case .zero:
            Text("Set all three axes exactly to the model pose: Ab/ad 0°, Hip 0°, Knee \(String(format: "%+.0f°", kneeReferenceDegrees)).")
                .font(.callout)
            Text("Positioning alone saves nothing. Click below and Aura reads the three servos in turn.")
                .font(.caption).foregroundStyle(.secondary)
            HStack(spacing: 8) {
                ForEach(0..<3, id: \.self) { axis in
                    Label(axisLabel(axis), systemImage: calibration.zeroRecorded[axis] ? "checkmark.circle.fill" : "circle")
                        .font(.caption)
                        .foregroundStyle(calibration.zeroRecorded[axis] ? .green : .secondary)
                }
            }
            Button("Save leg reference") {
                calibration.prepareZeroCapture()
                aura.captureCalibrationZeros(leg: currentLegIndex)
            }
            .buttonStyle(AuraButtonStyle(prominent: true))
            .disabled(aura.busy)
            if calibration.zeroRecorded.allSatisfy({ $0 }) {
                Text("All three references are stored on Aura.")
                    .font(.caption).foregroundStyle(.green)
                Button("Continue to direction check") { calibration.beginDirection() }
                    .buttonStyle(AuraButtonStyle(prominent: true))
            } else {
                Text("Keep the leg still during the three reads. Continue appears after three green marks.")
                    .font(.caption).foregroundStyle(.secondary)
            }

        case .direction:
            axisPicker
            if let axis = currentAxis {
                Text("Testing \(axis.axis.rawValue), servo ID \(axis.servoID). Only this axis is yellow in the model.")
                    .font(.callout)
                Text(directionProbeWasSent
                     ? "Compare the real move with the model. Aura released this axis after the test. If it moved the wrong way: set the axis by hand exactly to the model reference, then save the reversed direction. Saving sends no motion. Run the next +12° test with a separate click."
                     : "The green arrow shows the expected direction. Run one +12° move from the reference and compare it with the model.")
                    .font(.caption).foregroundStyle(.secondary)
                HStack {
                    Button("Move +12°") { sendDirectionProbe() }
                }
                .disabled(aura.busy || directionProbeWasSent)
                Button("Release this axis") { releaseDirectionAxis() }
                .buttonStyle(AuraButtonStyle())
                .disabled(aura.busy)
                Text("The test checks position mode, the 0…4095 range and that the axis sits at its stored reference. It always releases torque at +12° or on a wrong move. If the move was wrong, set the axis to the reference by hand before saving the reversed direction.")
                    .font(.caption).foregroundStyle(.secondary)
                Button("Save reversed direction · no motion") { flipDirectionAtManualReference() }
                    .buttonStyle(AuraButtonStyle(prominent: true))
                    .disabled(!directionProbeWasSent || aura.busy)
                Text("After saving, start the +12° test separately. Aura makes no automatic return or second test; a single test stops and releases the axis at its target.")
                    .font(.caption).foregroundStyle(.secondary)
                Divider()
                Text("Recovery from an older app version")
                    .font(.caption.weight(.semibold))
                Text("If the previous button was clicked with the axis already at +12°: release torque, set the exact model reference by hand, then save it without changing the sign. This removes the 12° offset.")
                    .font(.caption).foregroundStyle(.orange)
                Button("Recapture axis reference · no motion") { recaptureCurrentAxisReference() }
                    .buttonStyle(AuraButtonStyle())
                    .disabled(aura.busy)
                Text("Keeps direction and limits; only reads the current tick as the reference. Use this first after an old change made at +12°.")
                    .font(.caption).foregroundStyle(.secondary)
                Text("For an even older version that flipped the sign without recomputing limits, use the option below after setting the reference by hand.")
                    .font(.caption).foregroundStyle(.secondary)
                Button("Repair older flip · no motion") { repairLegacyDirectionFlipAtReference() }
                    .buttonStyle(AuraButtonStyle())
                    .disabled(aura.busy)
                Text("Keeps the current sign but recomputes zero and limits so the next +12° test moves the right way without reaching the end stop.")
                    .font(.caption).foregroundStyle(.secondary)
                Button(calibration.axisIndex == 2 ? "Directions done → limits" : "Direction OK → next axis") {
                    releaseDirectionAxis()
                    calibration.acceptDirection()
                }
                .buttonStyle(AuraButtonStyle(prominent: true))
                .disabled(!directionProbeWasSent || aura.busy)
            }

        case .limits:
            axisPicker
            if let axis = currentAxis {
                Text("Axis \(axis.axis.rawValue), ID \(axis.servoID). Release it, move it by hand, then save the limits.")
                    .font(.callout)
                Button("Release this axis") {
                    aura.releaseCalibrationAxis(leg: currentLegIndex, axis: calibration.axisIndex)
                }
                .buttonStyle(AuraButtonStyle())
                HStack {
                    Button("Save − limit") {
                        aura.captureCalibrationLimit(leg: currentLegIndex, axis: calibration.axisIndex, maximum: false)
                    }
                    Button("Save + limit") {
                        aura.captureCalibrationLimit(leg: currentLegIndex, axis: calibration.axisIndex, maximum: true)
                    }
                }
                .buttonStyle(AuraButtonStyle(prominent: true))
                .disabled(aura.busy)
                Text(String(format: "Current: %+.1f° … %+.1f°", axis.minimumDegrees, axis.maximumDegrees))
                    .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                Button(calibration.axisIndex == 2 ? "Limits done → leg test" : "Next axis") {
                    if calibration.axisIndex < 2 { calibration.axisIndex += 1 }
                    else { calibration.beginTest() }
                }
                .buttonStyle(AuraButtonStyle(prominent: true))
            }

        case .test:
            Text("Aura reads the DualSense and drives only this leg in its own 50 Hz loop. The model shows the whole virtual robot on the same pad; the selected leg uses its real TTL angles.")
                .font(.callout)
            let telemetry = robot.telemetry[currentLegIndex]
            let angles = telemetry.present.allSatisfy { $0 } ? telemetry.measuredDegrees : telemetry.targetDegrees
            Grid(alignment: .leading, horizontalSpacing: 10, verticalSpacing: 4) {
                GridRow { Text("TTL").foregroundStyle(.secondary); Text("Ab/ad"); Text("Hip"); Text("Knee") }
                GridRow { Text(telemetry.present.allSatisfy { $0 } ? "read" : "target").foregroundStyle(.secondary); ForEach(0..<3, id: \.self) { index in Text(String(format: "%+.1f°", angles[index])).monospacedDigit() } }
            }
            .font(.caption)
            Button(calibration.testRunning ? "Stop one-leg trot" : "Start one-leg trot with pad") {
                let enabled = !calibration.testRunning
                aura.setCalibrationLegTest(leg: currentLegIndex, enabled: enabled)
                calibration.testRunning = enabled
            }
            .buttonStyle(AuraButtonStyle(prominent: true))
            .disabled(aura.busy)
            Button("Save leg and continue") {
                aura.setCalibrationLegTest(leg: currentLegIndex, enabled: false)
                calibration.finishLeg()
            }
            .disabled(calibration.testRunning)
        }

        if calibration.step != .zero {
            Button("Back") { calibration.previousStep() }
                .buttonStyle(.plain)
                .foregroundStyle(.secondary)
        }
    }

    private var axisPicker: some View {
        HStack(spacing: 5) {
            ForEach(0..<3, id: \.self) { index in
                if index == calibration.axisIndex {
                    Button(axisLabel(index)) { selectAxis(index) }
                        .buttonStyle(AuraButtonStyle(prominent: true))
                } else {
                    Button(axisLabel(index)) { selectAxis(index) }
                        .buttonStyle(AuraButtonStyle())
                }
            }
        }
    }
}
