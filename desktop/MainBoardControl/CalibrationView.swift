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
        case .direction: "2. Kierunek"
        case .limits: "3. Krańce"
        case .test: "4. Test nogi"
        }
    }
    var instruction: String {
        switch self {
        case .zero:
            "Ustaw nogę dokładnie jak w modelu po lewej. Moment jest wyłączony. Następnie Aura odczyta trzy rzeczywiste pozycje i zapisze je jako 0° tej nogi."
        case .direction:
            "Wybierz oś. Aura porusza wyłącznie nią o +12° od pozycji referencyjnej. Zielona strzałka pokazuje oczekiwany zwrot ruchu; zapis odwróconego kierunku sprawia, że ten sam test +12° idzie fizycznie w drugą stronę."
        case .limits:
            "Zwolnij moment jednej osi, ustaw ją ręką na bezpiecznym krańcu ujemnym, zapisz. Następnie ustaw drugi kraniec i zapisz dodatni. Zapis trafia do limitów Aury."
        case .test:
            "Aura steruje tylko wybraną nogą bez komputera. Lewy drążek góra/dół prowadzi mały cykl kroku, lewo/prawo sprawdza ab/ad. Zatrzymaj test przed przejściem dalej."
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
        case .lf: "FL • lewy przód"
        case .lr: "RL • lewy tył"
        case .rf: "FR • prawy przód"
        case .rr: "RR • prawy tył"
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
        RobotAxisID.allCases.indices.contains(index) ? RobotAxisID.allCases[index].rawValue : "Oś"
    }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                HStack(alignment: .firstTextBaseline) {
                    VStack(alignment: .leading, spacing: 3) {
                        Text("Calibration").font(.title2.bold())
                        Text("Jedna noga naraz • ustawienia zapisują się bezpośrednio w Aurze")
                            .foregroundStyle(.secondary)
                    }
                    Spacer()
                    Label(aura.robotState.armed ? "Najpierw rozbrój serwa" : "Tryb bezpiecznej kalibracji",
                          systemImage: aura.robotState.armed ? "bolt.slash.fill" : "checkmark.shield.fill")
                        .foregroundStyle(aura.robotState.armed ? .orange : .green)
                }

                HStack(spacing: 6) {
                    ForEach(Array(calibration.order.enumerated()), id: \.element) { index, leg in
                        Button {
                            aura.setCalibrationLegTest(leg: currentLegIndex, enabled: false)
                            aura.setCalibrationLegTorque(leg: currentLegIndex, enabled: false)
                            calibration.startLeg(index)
                        } label: {
                            let done = calibration.finishedLegs.contains(leg)
                            Label(leg.displayCode, systemImage: done ? "checkmark.circle.fill" : "circle")
                                .font(.caption.weight(index == calibration.legIndex ? .semibold : .regular))
                                .foregroundStyle(index == calibration.legIndex ? .primary : .secondary)
                                .padding(.horizontal, 9)
                                .padding(.vertical, 7)
                                .background(index == calibration.legIndex ? Color.primary.opacity(0.14) : Color.clear,
                                            in: Capsule())
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
                    ContentUnavailableView("Rozbrój robota przed kalibracją", systemImage: "bolt.slash",
                                           description: Text("Kalibrator nie może zmieniać zer ani krańców, gdy działa sterowanie 12 serw."))
                } else if !legReady {
                    ContentUnavailableView("Przypisz trzy ID tej nogi", systemImage: "point.3.connected.trianglepath.dotted",
                                           description: Text("W zakładce Robot przypisz unikalne serwa do Ab/ad, Hip i Knee, a następnie wróć tutaj."))
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
                                .frame(minWidth: 490, minHeight: 420)
                                .clipShape(RoundedRectangle(cornerRadius: 14))
                                .overlay(alignment: .bottomLeading) {
                                    Text(liveReadPreview
                                         ? "Podgląd pozycji TTL całego robota • 12 odczytów, moment wyłączony"
                                         : calibration.testRunning
                                         ? "Test na padzie • \(currentLeg.displayCode) śledzi odczyt TTL Aury 50 Hz"
                                         : calibration.previewMode == .trot
                                         ? "Ten sam chód do przodu co Quadruped • podświetlona noga: \(currentLeg.displayCode)"
                                         : "Normalna pozycja stania • podświetlona noga: referencja \(calibrationReferenceText)")
                                        .font(.caption)
                                        .foregroundStyle(.secondary)
                                        .padding(12)
                                }
                                .overlay(alignment: .topLeading) {
                                    if calibration.step == .direction {
                                        Label("Zielona strzałka: oczekiwany ruch +12°",
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
                            Text(legTitle).font(.title3.bold())
                            Text("Sterujesz zapisanymi osiami: \(legAxes.map { String($0.servoID) }.joined(separator: ", "))")
                                .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                            Text(calibration.step.instruction).foregroundStyle(.secondary)
                            Picker("Podgląd modelu", selection: $calibration.previewMode) {
                                ForEach(CalibrationPreviewMode.allCases) { mode in
                                    Text(mode.title).tag(mode)
                                }
                            }
                            .pickerStyle(.segmented)
                            Text("Podgląd Trot jest wyłącznie wizualny. Pełny test z padem poniżej napędza tylko tę nogę i pokazuje jej odczyt TTL na tle kompletnej wirtualnej postawy robota.")
                                .font(.caption).foregroundStyle(.secondary)
                            Button(liveReadPreview ? "Wyłącz podgląd pozycji TTL" : "Włącz podgląd pozycji TTL całego robota") {
                                aura.setCalibrationLivePreview(enabled: !liveReadPreview)
                            }
                            .buttonStyle(.bordered)
                            .disabled(aura.busy || aura.robotState.armed)
                            Text("Odczytuje stale pozycje wszystkich 12 serw i pokazuje je na tym samym modelu co Quadruped. Nie uzbraja serw i nie wysyła komend ruchu.")
                                .font(.caption).foregroundStyle(.secondary)
                            Divider()
                            Button(aura.calibrationArmedLeg == currentLegIndex
                                   ? "Rozbrój tę nogę" : "Uzbrój tylko tę nogę") {
                                aura.setCalibrationLegTorque(leg: currentLegIndex,
                                                             enabled: aura.calibrationArmedLeg != currentLegIndex)
                            }
                            .buttonStyle(.borderedProminent)
                            .disabled(aura.busy)
                            Text("Działa wyłącznie na ID \(legAxes.map { String($0.servoID) }.joined(separator: ", ")). Pozostałe dziewięć serw pozostaje rozbrojone.")
                                .font(.caption).foregroundStyle(.secondary)
                            Divider()
                            stepControls
                        }
                        .frame(minWidth: 275, maxWidth: 360, alignment: .leading)
                    }

                    GroupBox("Zapisane parametry \(legTitle)") {
                        Grid(alignment: .leading, horizontalSpacing: 16, verticalSpacing: 8) {
                            GridRow {
                                Text("Oś").foregroundStyle(.secondary)
                                Text("ID").foregroundStyle(.secondary)
                                Text("Zero").foregroundStyle(.secondary)
                                Text("Kierunek").foregroundStyle(.secondary)
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
            Text("Ustaw wszystkie trzy osie dokładnie na pozycję z modelu: Ab/ad 0°, Hip 0°, Knee \(String(format: "%+.0f°", kneeReferenceDegrees)).")
                .font(.callout)
            Text("Samo ustawienie mechaniczne nie zapisuje kalibracji. Kliknij poniżej, aby Aura odczytała kolejno wszystkie trzy serwa.")
                .font(.caption).foregroundStyle(.secondary)
            HStack(spacing: 8) {
                ForEach(0..<3, id: \.self) { axis in
                    Label(axisLabel(axis), systemImage: calibration.zeroRecorded[axis] ? "checkmark.circle.fill" : "circle")
                        .font(.caption)
                        .foregroundStyle(calibration.zeroRecorded[axis] ? .green : .secondary)
                }
            }
            Button("Zapisz pozycje referencyjne nogi") {
                calibration.prepareZeroCapture()
                aura.captureCalibrationZeros(leg: currentLegIndex)
            }
            .buttonStyle(.borderedProminent)
            .disabled(aura.busy)
            if calibration.zeroRecorded.allSatisfy({ $0 }) {
                Text("Trzy pozycje referencyjne są zapisane w Aurze.")
                    .font(.caption).foregroundStyle(.green)
                Button("Przejdź do kontroli kierunku") { calibration.beginDirection() }
                    .buttonStyle(.borderedProminent)
            } else {
                Text("Nie ruszaj nogi podczas trzech odczytów. Po trzech zielonych znacznikach pojawi się przycisk przejścia dalej.")
                    .font(.caption).foregroundStyle(.secondary)
            }

        case .direction:
            axisPicker
            if let axis = currentAxis {
                Text("Testujesz \(axis.axis.rawValue), serwo ID \(axis.servoID). Model pokazuje wyłącznie tę oś na żółto.")
                    .font(.callout)
                Text(directionProbeWasSent
                     ? "Porównaj fizyczny ruch z modelem. Aura po teście sama wyłączyła moment tej osi. Jeśli ruch był przeciwny: ustaw oś ręcznie dokładnie na referencji z modelu, a potem zapisz odwrócony kierunek. Ten zapis nie wysyła ruchu. Kolejny test +12° uruchomisz osobnym kliknięciem."
                     : "Model pokazuje oczekiwany zwrot zieloną strzałką. Wykonaj pojedynczy ruch +12° od pozycji referencyjnej i porównaj go z modelem.")
                    .font(.caption).foregroundStyle(.secondary)
                HStack {
                    Button("Wykonaj ruch +12°") { sendDirectionProbe() }
                }
                .disabled(aura.busy || directionProbeWasSent)
                Button("Zwolnij moment tej osi") { releaseDirectionAxis() }
                .buttonStyle(.bordered)
                .disabled(aura.busy)
                Text("Test sprawdza tryb pozycyjny, fizyczny zakres 0…4095 i to, że oś stoi przy zapisanej referencji. Po osiągnięciu +12° albo wykryciu złego ruchu zawsze wyłącza moment. Gdy ruch był zły, ustaw oś ręcznie na referencji i dopiero wtedy zapisz odwrócony kierunek.")
                    .font(.caption).foregroundStyle(.secondary)
                Button("Zapisz odwrócony kierunek — bez ruchu") { flipDirectionAtManualReference() }
                    .buttonStyle(.borderedProminent)
                    .disabled(!directionProbeWasSent || aura.busy)
                Text("Po zapisie kliknij osobno test +12°. Aura nie wykona automatycznego powrotu ani drugiego testu; pojedynczy test zatrzyma i rozbroi oś po dojściu do celu.")
                    .font(.caption).foregroundStyle(.secondary)
                Divider()
                Text("Odzyskanie po poprzedniej wersji")
                    .font(.caption.weight(.semibold))
                Text("Jeżeli poprzedni przycisk został kliknięty, gdy oś była już na +12°: zwolnij moment, ustaw ręcznie dokładnie referencję z modelu, a następnie zapisz ją bez zmiany znaku. To usuwa błąd przesunięcia o 12°.")
                    .font(.caption).foregroundStyle(.orange)
                Button("Przywróć referencję tej osi — bez ruchu") { recaptureCurrentAxisReference() }
                    .buttonStyle(.bordered)
                    .disabled(aura.busy)
                Text("Zachowuje obecny kierunek i krańce; tylko odczytuje aktualny tick jako referencję. Użyj tego najpierw po starej zmianie wykonanej w pozycji +12°.")
                    .font(.caption).foregroundStyle(.secondary)
                Text("Dla jeszcze starszej wersji, która odwracała znak bez przeliczenia krańców, użyj drugiej opcji po ręcznym ustawieniu referencji.")
                    .font(.caption).foregroundStyle(.secondary)
                Button("Napraw poprzednią zmianę — bez ruchu") { repairLegacyDirectionFlipAtReference() }
                    .buttonStyle(.bordered)
                    .disabled(aura.busy)
                Text("Ta opcja zachowuje obecny znak, ale przelicza zero i krańce tak, aby nowy test +12° wrócił do właściwej strony bez dojazdu do końca.")
                    .font(.caption).foregroundStyle(.secondary)
                Button(calibration.axisIndex == 2 ? "Kierunki gotowe → krańce" : "Kierunek poprawny → następna oś") {
                    releaseDirectionAxis()
                    calibration.acceptDirection()
                }
                .buttonStyle(.borderedProminent)
                .disabled(!directionProbeWasSent || aura.busy)
            }

        case .limits:
            axisPicker
            if let axis = currentAxis {
                Text("Oś \(axis.axis.rawValue), ID \(axis.servoID). Najpierw zwolnij moment, przesuń ją ręką, potem zapisz krańce.")
                    .font(.callout)
                Button("Zwolnij moment tej osi") {
                    aura.releaseCalibrationAxis(leg: currentLegIndex, axis: calibration.axisIndex)
                }
                .buttonStyle(.bordered)
                HStack {
                    Button("Zapisz kraniec −") {
                        aura.captureCalibrationLimit(leg: currentLegIndex, axis: calibration.axisIndex, maximum: false)
                    }
                    Button("Zapisz kraniec +") {
                        aura.captureCalibrationLimit(leg: currentLegIndex, axis: calibration.axisIndex, maximum: true)
                    }
                }
                .buttonStyle(.borderedProminent)
                .disabled(aura.busy)
                Text(String(format: "Aktualnie: %+.1f° … %+.1f°", axis.minimumDegrees, axis.maximumDegrees))
                    .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                Button(calibration.axisIndex == 2 ? "Krańce gotowe → test nogi" : "Następna oś") {
                    if calibration.axisIndex < 2 { calibration.axisIndex += 1 }
                    else { calibration.beginTest() }
                }
                .buttonStyle(.borderedProminent)
            }

        case .test:
            Text("Aura odczytuje DualSense i steruje tylko tą jedną nogą we własnej pętli 50 Hz. Model po lewej pokazuje cały wirtualny robot sterowany tym samym padem; wybrana noga jest zastąpiona jej rzeczywistymi kątami TTL.")
                .font(.callout)
            let telemetry = robot.telemetry[currentLegIndex]
            let angles = telemetry.present.allSatisfy { $0 } ? telemetry.measuredDegrees : telemetry.targetDegrees
            Grid(alignment: .leading, horizontalSpacing: 10, verticalSpacing: 4) {
                GridRow { Text("TTL").foregroundStyle(.secondary); Text("Ab/ad"); Text("Hip"); Text("Knee") }
                GridRow { Text(telemetry.present.allSatisfy { $0 } ? "odczyt" : "cel").foregroundStyle(.secondary); ForEach(0..<3, id: \.self) { index in Text(String(format: "%+.1f°", angles[index])).monospacedDigit() } }
            }
            .font(.caption)
            Button(calibration.testRunning ? "Zatrzymaj Trot jednej nogi" : "Włącz Trot jednej nogi z padem") {
                let enabled = !calibration.testRunning
                aura.setCalibrationLegTest(leg: currentLegIndex, enabled: enabled)
                calibration.testRunning = enabled
            }
            .buttonStyle(.borderedProminent)
            .disabled(aura.busy)
            Button("Zapisz nogę i przejdź dalej") {
                aura.setCalibrationLegTest(leg: currentLegIndex, enabled: false)
                calibration.finishLeg()
            }
            .disabled(calibration.testRunning)
        }

        if calibration.step != .zero {
            Button("Wstecz") { calibration.previousStep() }
                .buttonStyle(.plain)
                .foregroundStyle(.secondary)
        }
    }

    private var axisPicker: some View {
        HStack(spacing: 5) {
            ForEach(0..<3, id: \.self) { index in
                if index == calibration.axisIndex {
                    Button(axisLabel(index)) { selectAxis(index) }
                        .buttonStyle(.borderedProminent)
                } else {
                    Button(axisLabel(index)) { selectAxis(index) }
                        .buttonStyle(.bordered)
                }
            }
        }
    }
}
