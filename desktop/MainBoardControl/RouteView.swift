import SceneKit
import SwiftUI

enum RouteCameraMode: String, Hashable { case orbit, top, follow }

// Aura's ground frame is X forward, Y up, Z left; SceneKit is right-handed
// with Z to the right of +X, so Z is mirrored on the way in.
private func scenePoint(_ x: Double, _ y: Double, _ z: Double) -> SCNVector3 {
    SCNVector3(Float(x), Float(y), Float(-z))
}

private func lineGeometry(_ vertices: [SCNVector3], strip: Bool, color: NSColor) -> SCNGeometry {
    var indices: [Int32] = []
    if strip {
        guard vertices.count > 1 else { return SCNGeometry() }
        for i in 1..<vertices.count { indices += [Int32(i - 1), Int32(i)] }
    } else {
        indices = (0..<vertices.count).map { Int32($0) }
    }
    let geometry = SCNGeometry(sources: [SCNGeometrySource(vertices: vertices)],
                               elements: [SCNGeometryElement(indices: indices, primitiveType: .line)])
    let material = SCNMaterial()
    material.diffuse.contents = color
    material.emission.contents = color
    material.lightingModel = .constant
    geometry.firstMaterial = material
    return geometry
}

@MainActor
final class RouteScene: NSObject {
    let scene = SCNScene()
    let camera = SCNNode()
    private let path = SCNNode()
    private let comparisonPath = SCNNode()
    private let robot = SCNNode()
    private let startMarker = SCNNode(geometry: SCNSphere(radius: 18))
    private var drawnCount = -1
    private var drawnComparison: UUID?
    private var mode: RouteCameraMode?

    override init() {
        super.init()
        scene.background.contents = Theme.canvasNS
        camera.camera = SCNCamera()
        camera.camera?.zNear = 10
        camera.camera?.zFar = 200_000
        camera.camera?.fieldOfView = 45
        scene.rootNode.addChildNode(camera)

        // Floor grid: 0.25 m minor, 1 m major, 20 x 20 m.
        var minor: [SCNVector3] = [], major: [SCNVector3] = []
        for step in stride(from: -10_000, through: 10_000, by: 250) {
            let segment = [SCNVector3(Float(step), 0, -10_000), SCNVector3(Float(step), 0, 10_000),
                           SCNVector3(-10_000, 0, Float(step)), SCNVector3(10_000, 0, Float(step))]
            if step % 1000 == 0 { major += segment } else { minor += segment }
        }
        scene.rootNode.addChildNode(SCNNode(geometry: lineGeometry(minor, strip: false, color: NSColor(rgb: 0x2E3034))))
        scene.rootNode.addChildNode(SCNNode(geometry: lineGeometry(major, strip: false, color: NSColor(rgb: 0x43454A))))
        let axes = SCNNode(geometry: lineGeometry([SCNVector3(0, 1, 0), SCNVector3(500, 1, 0)], strip: false,
                                                  color: NSColor(rgb: 0xE55765)))
        scene.rootNode.addChildNode(axes)

        scene.rootNode.addChildNode(comparisonPath)
        scene.rootNode.addChildNode(path)
        startMarker.geometry?.firstMaterial?.diffuse.contents = Theme.accent2LineNS
        startMarker.geometry?.firstMaterial?.lightingModel = .constant
        scene.rootNode.addChildNode(startMarker)

        // Robot: frame-sized slab with a nose cone pointing along +X.
        let body = SCNNode(geometry: SCNBox(width: 360, height: 50, length: 130, chamferRadius: 8))
        body.geometry?.firstMaterial?.diffuse.contents = Theme.accent2NS
        let nose = SCNNode(geometry: SCNCone(topRadius: 0, bottomRadius: 40, height: 90))
        nose.geometry?.firstMaterial?.diffuse.contents = Theme.accentNS
        nose.eulerAngles.z = -.pi / 2
        nose.position = SCNVector3(225, 0, 0)
        robot.addChildNode(body)
        robot.addChildNode(nose)
        scene.rootNode.addChildNode(robot)

        let light = SCNNode()
        light.light = SCNLight()
        light.light?.type = .directional
        light.light?.intensity = 900
        light.eulerAngles = SCNVector3(-1.0, 0.6, 0)
        scene.rootNode.addChildNode(light)
        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 450
        scene.rootNode.addChildNode(ambient)
    }

    func update(_ recorder: RouteRecorder, bodyHeight: Double, mode: RouteCameraMode, view: SCNView) {
        SCNTransaction.begin()
        SCNTransaction.animationDuration = 0
        if recorder.points.count != drawnCount {
            drawnCount = recorder.points.count
            let vertices = recorder.points.map { scenePoint($0.x, $0.y + 4, $0.z) }
            path.geometry = lineGeometry(vertices, strip: true, color: Theme.accentNS)
            startMarker.isHidden = recorder.points.isEmpty
            if let first = recorder.points.first { startMarker.position = scenePoint(first.x, first.y + 18, first.z) }
        }
        if recorder.comparison?.id != drawnComparison {
            drawnComparison = recorder.comparison?.id
            let vertices = (recorder.comparison?.points ?? []).map { scenePoint($0.x, $0.y + 2, $0.z) }
            comparisonPath.geometry = lineGeometry(vertices, strip: true, color: NSColor(rgb: 0x8C8F96))
        }
        let p = recorder.position
        robot.position = scenePoint(p.x, p.y + max(80, bodyHeight), p.z)
        // Heading about +Y. Roll > 0 lowers the left side (-Z in the scene),
        // which is a negative turn about +X; pitch > 0 raises the nose (+X).
        robot.eulerAngles = SCNVector3(Float(-recorder.roll * .pi / 180), Float(recorder.heading),
                                       Float(recorder.pitch * .pi / 180))

        let target = scenePoint(p.x, p.y, p.z)
        switch mode {
        case .orbit:
            if self.mode != .orbit {
                camera.camera?.usesOrthographicProjection = false
                camera.position = SCNVector3(target.x - 1600, 1400, target.z + 1600)
                camera.look(at: target)
                view.allowsCameraControl = true
                view.defaultCameraController.interactionMode = .orbitTurntable
                view.defaultCameraController.target = target
            }
        case .top:
            view.allowsCameraControl = false
            camera.camera?.usesOrthographicProjection = true
            let extent = recorder.points.reduce(1500.0) { max($0, abs($1.x - p.x), abs($1.z - p.z)) }
            camera.camera?.orthographicScale = min(12_000, extent * 1.3)
            camera.position = SCNVector3(target.x, 8000, target.z)
            camera.eulerAngles = SCNVector3(-Float.pi / 2, 0, 0)
        case .follow:
            view.allowsCameraControl = false
            camera.camera?.usesOrthographicProjection = false
            let h = recorder.heading
            let back = CGFloat(cos(h) * 1300), side = CGFloat(sin(h) * 1300)
            camera.position = SCNVector3(target.x - back, 900, target.z + side)
            camera.look(at: SCNVector3(target.x, 150, target.z))
        }
        if self.mode != mode { view.pointOfView = camera }
        self.mode = mode
        SCNTransaction.commit()
    }
}

struct RouteSceneView: NSViewRepresentable {
    @ObservedObject var recorder: RouteRecorder
    let bodyHeight: Double
    let mode: RouteCameraMode

    func makeCoordinator() -> RouteScene { RouteScene() }

    func makeNSView(context: Context) -> SCNView {
        let view = SCNView()
        view.scene = context.coordinator.scene
        view.pointOfView = context.coordinator.camera
        view.backgroundColor = Theme.canvasNS
        view.antialiasingMode = .multisampling4X
        view.preferredFramesPerSecond = 30
        context.coordinator.update(recorder, bodyHeight: bodyHeight, mode: mode, view: view)
        return view
    }

    func updateNSView(_ view: SCNView, context: Context) {
        context.coordinator.update(recorder, bodyHeight: bodyHeight, mode: mode, view: view)
    }
}

struct RouteView: View {
    @EnvironmentObject var aura: AuraConnection
    @EnvironmentObject var recorder: RouteRecorder
    @AppStorage("aura.route.camera") private var mode: RouteCameraMode = .orbit

    var body: some View {
        let _ = recorder.revision
        HStack(spacing: 0) {
            VStack(spacing: 0) {
                RouteToolbar(recorder: recorder, mode: $mode)
                Divider().overlay(Theme.line)
                RouteSceneView(recorder: recorder, bodyHeight: Double(aura.robotState.bodyHeight), mode: mode)
                    .overlay(alignment: .topLeading) { RecordingBadge(recorder: recorder) }
            }
            Divider().overlay(Theme.line)
            RouteInspector(recorder: recorder)
        }
    }
}

private func metres(_ mm: Double) -> String { String(format: "%.2f m", mm / 1000) }

private struct RouteToolbar: View {
    @ObservedObject var recorder: RouteRecorder
    @Binding var mode: RouteCameraMode

    var body: some View {
        HStack(spacing: 10) {
            TextTabs(tabs: [(.orbit, "Perspektywa"), (.top, "Z góry"), (.follow, "Za robotem")], selection: $mode)
            Spacer(minLength: 0)
        }
        .padding(.horizontal, 8).padding(.top, 8)
        .background(Theme.panel)
    }
}

private struct RecordingControls: View {
    @ObservedObject var recorder: RouteRecorder

    var body: some View {
        Panel("Nagrywanie") {
            Grid(horizontalSpacing: 6, verticalSpacing: 6) {
                GridRow {
                    if recorder.recording {
                        Button("Zatrzymaj") { recorder.stop() }.frame(maxWidth: .infinity)
                    } else {
                        Button("Nagrywaj") { recorder.start() }.keyboardShortcut("r", modifiers: [.command])
                            .frame(maxWidth: .infinity)
                    }
                    Button("Zapisz") { recorder.save() }.disabled(recorder.recording || recorder.points.count < 2)
                        .frame(maxWidth: .infinity)
                }
                GridRow {
                    Button("Eksport CSV") { recorder.exportCSV(nil) }.disabled(recorder.points.isEmpty)
                        .frame(maxWidth: .infinity)
                    Button("Wyczyść") { recorder.clear() }.disabled(recorder.recording)
                        .frame(maxWidth: .infinity)
                }
            }
            .controlSize(.small)
        }
    }
}

private struct RecordingBadge: View {
    @ObservedObject var recorder: RouteRecorder

    var body: some View {
        if recorder.recording {
            StatusDot(color: Theme.red, text: "REC " + String(format: "%.0f s", recorder.duration))
                .padding(8)
                .background(Theme.panel.opacity(0.85), in: RoundedRectangle(cornerRadius: 4))
                .padding(10)
        }
    }
}

private struct RouteInspector: View {
    @ObservedObject var recorder: RouteRecorder

    var body: some View {
        let current = RouteRecord(name: "", created: Date(), points: recorder.points)
        let average = recorder.duration > 0 ? String(format: "%.0f mm/s", current.distance / recorder.duration) : "—"
        ScrollView {
            VStack(alignment: .leading, spacing: 10) {
                RecordingControls(recorder: recorder)
                Panel("Trasa") {
                    ValueRow(label: "Dystans", value: metres(current.distance))
                    ValueRow(label: "Czas", value: String(format: "%.0f s", recorder.duration))
                    ValueRow(label: "Prędkość śr.", value: average)
                    ValueRow(label: "Przewyższenie", value: metres(current.climb))
                    ValueRow(label: "Punkty", value: "\(recorder.points.count)")
                }
                Panel("Pozycja") {
                    ValueRow(label: "X", value: metres(recorder.position.x))
                    ValueRow(label: "Z", value: metres(recorder.position.z))
                    ValueRow(label: "Y", value: metres(recorder.position.y))
                    ValueRow(label: "Kurs", value: String(format: "%+.1f°", recorder.heading * 180 / .pi))
                }
                FilterPanel(recorder: recorder)
                SavedRoutes(recorder: recorder)
            }
            .padding(10)
        }
        .frame(width: 250)
        .background(Theme.canvas)
    }
}

private struct FilterPanel: View {
    @ObservedObject var recorder: RouteRecorder

    var body: some View {
        Panel("Filtr kursu") {
            ValueRow(label: "Niepewność", value: String(format: "±%.1f°", recorder.filter.headingSigma * 180 / .pi))
            ValueRow(label: "Dryf żyroskopu", value: String(format: "%+.2f°/s", recorder.filter.bias * 180 / .pi))
            ValueRow(label: "Postój", value: recorder.still ? "tak" : "nie",
                     tint: recorder.still ? Theme.green : Theme.muted)
            ValueRow(label: "Odometria", value: recorder.odometryValid ? "aktywna" : "brak",
                     tint: recorder.odometryValid ? Theme.green : Theme.yellow)
        }
        .help("Droga z odometrii nóg, kurs z żyroskopu (filtr Kalmana z estymacją dryfu na postoju), wysokość z nachylenia podłoża.")
    }
}

private struct SavedRoutes: View {
    @ObservedObject var recorder: RouteRecorder

    var body: some View {
        Panel("Zapisane") {
            if recorder.saved.isEmpty {
                Text("Brak").font(Theme.label).foregroundStyle(Theme.muted)
            }
            ForEach(recorder.saved) { record in
                SavedRouteRow(recorder: recorder, record: record)
            }
        }
    }
}

private struct SavedRouteRow: View {
    @ObservedObject var recorder: RouteRecorder
    let record: RouteRecord

    var body: some View {
        let selected = recorder.comparison?.id == record.id
        HStack(spacing: 6) {
            Button {
                recorder.comparison = selected ? nil : record
            } label: {
                VStack(alignment: .leading, spacing: 2) {
                    Text(record.name).font(Theme.monoSmall).foregroundStyle(selected ? Theme.accent : Theme.text)
                    Text(metres(record.distance) + String(format: " · %.0f s", record.duration))
                        .font(Theme.label).foregroundStyle(Theme.muted)
                }
                .frame(maxWidth: .infinity, alignment: .leading)
                .contentShape(Rectangle())
            }
            .buttonStyle(.plain)
            .help("Pokaż na scenie do porównania")
            Button("CSV") { recorder.exportCSV(record) }
                .buttonStyle(.plain).font(Theme.label).foregroundStyle(Theme.muted)
            Button("Usuń") { recorder.delete(record) }
                .buttonStyle(.plain).font(Theme.label).foregroundStyle(Theme.muted)
        }
        .padding(.vertical, 2)
    }
}
