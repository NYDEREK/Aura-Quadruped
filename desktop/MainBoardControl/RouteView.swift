import Combine
import SceneKit
import SwiftUI

enum RouteCameraMode: String, Hashable { case orbit, top, follow }

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

private func material(_ color: NSColor) -> SCNMaterial {
    let result = SCNMaterial()
    result.diffuse.contents = color
    result.roughness.contents = 0.35
    result.isDoubleSided = true     // the mirrored world flips face winding
    return result
}

// The Robot 3D leg model: body frame, three rods, joints and a foot per leg,
// posed from Aura's 50 Hz joint telemetry through the same QuadrupedModel.
@MainActor
private final class RobotRig {
    let yawNode = SCNNode()          // heading about Y
    private let body = SCNNode()     // tilt about the torso centre
    private var frame: [SCNNode] = []
    private var rods: [QuadLeg: [SCNNode]] = [:]
    private var joints: [QuadLeg: [SCNNode]] = [:]
    private var feet: [QuadLeg: SCNNode] = [:]

    init() {
        yawNode.addChildNode(body)
        let frameMaterial = material(NSColor(rgb: 0xDFE1E5))
        frame = (0..<4).map { _ in
            let node = SCNNode(geometry: SCNCylinder(radius: 2.4, height: 1))
            node.geometry?.firstMaterial = frameMaterial
            body.addChildNode(node)
            return node
        }
        for leg in QuadLeg.allCases {
            let color = leg.left > 0 ? Theme.accentNS : Theme.accent2LineNS
            rods[leg] = (0..<3).map { _ in
                let node = SCNNode(geometry: SCNCylinder(radius: 3, height: 1))
                node.geometry?.firstMaterial = material(color)
                body.addChildNode(node)
                return node
            }
            joints[leg] = (0..<3).map { _ in
                let node = SCNNode(geometry: SCNSphere(radius: 5))
                node.geometry?.firstMaterial = frameMaterial
                body.addChildNode(node)
                return node
            }
            let foot = SCNNode(geometry: SCNSphere(radius: 6))
            foot.geometry?.firstMaterial = material(color)
            body.addChildNode(foot)
            feet[leg] = foot
        }
    }

    private func rod(_ node: SCNNode, _ a: QuadVector, _ b: QuadVector) {
        let start = SIMD3<Float>(Float(a.x), Float(a.y), Float(a.z)), end = SIMD3<Float>(Float(b.x), Float(b.y), Float(b.z))
        let length = simd_length(end - start)
        guard length.isFinite, length > 0.01 else { node.isHidden = true; return }
        node.isHidden = false
        (node.geometry as? SCNCylinder)?.height = CGFloat(length)
        node.simdPosition = (start + end) / 2
        node.simdOrientation = simd_quatf(from: SIMD3<Float>(0, 1, 0), to: simd_normalize(end - start))
    }

    // Leg points are in the torso frame with the ground at Y = 0; the torso
    // rotates about its own centre (as the IK does).
    func apply(_ snapshot: QuadrupedSnapshot, dimensions: (length: Double, width: Double),
               position: SIMD3<Double>, heading: Double, roll: Double, pitch: Double) {
        let h = snapshot.bodyHeight.isFinite && (80...350).contains(snapshot.bodyHeight) ? snapshot.bodyHeight : 220
        let halfX = dimensions.length / 2, halfZ = dimensions.width / 2
        let corners = [QuadVector(x: halfX, y: h, z: halfZ), QuadVector(x: halfX, y: h, z: -halfZ),
                       QuadVector(x: -halfX, y: h, z: -halfZ), QuadVector(x: -halfX, y: h, z: halfZ)]
        for i in 0..<4 { rod(frame[i], corners[i], corners[(i + 1) % 4]) }
        for pose in snapshot.legs {
            guard let legRods = rods[pose.leg], let legJoints = joints[pose.leg], let foot = feet[pose.leg] else { continue }
            for i in 0..<3 {
                rod(legRods[i], pose.points[i], pose.points[i + 1])
                legJoints[i].position = pose.points[i].scn
            }
            foot.position = pose.points[3].scn
        }
        body.pivot = SCNMatrix4MakeTranslation(0, CGFloat(h), 0)
        body.position = SCNVector3(0, CGFloat(h), 0)
        // IK tilt convention: Rz(pitch) * Rx(roll); roll > 0 lowers the left
        // side, pitch > 0 raises the nose.
        body.simdOrientation = simd_quatf(angle: Float(pitch * .pi / 180), axis: SIMD3(0, 0, 1)) *
            simd_quatf(angle: Float(roll * .pi / 180), axis: SIMD3(1, 0, 0))
        yawNode.position = SCNVector3(CGFloat(position.x), CGFloat(position.y), CGFloat(position.z))
        // A left turn (+heading) carries X towards +Z in Aura's frame, which
        // is a negative turn about Y in these coordinates.
        yawNode.eulerAngles = SCNVector3(0, CGFloat(-heading), 0)
    }
}

@MainActor
final class RouteScene: NSObject {
    let scene = SCNScene()
    let camera = SCNNode()
    // Aura's frame (X forward, Y up, Z left) is the mirror of SceneKit's;
    // all content sits in this Z-mirrored node, the camera outside it.
    private let world: SCNNode = { let node = SCNNode(); node.scale = SCNVector3(1, 1, -1); return node }()
    private let path = SCNNode()
    private let comparisonPath = SCNNode()
    private let startMarker = SCNNode(geometry: SCNSphere(radius: 16))
    private let rig = RobotRig()
    private let model = QuadrupedModel()
    private var snapshot: QuadrupedSnapshot?
    private var subscription: AnyCancellable?
    private var drawnCount = -1
    private var drawnComparison: UUID?
    private var mode: RouteCameraMode?

    override init() {
        super.init()
        scene.background.contents = Theme.canvasNS
        scene.rootNode.addChildNode(world)
        camera.camera = SCNCamera()
        camera.camera?.zNear = 10
        camera.camera?.zFar = 200_000
        camera.camera?.fieldOfView = 45
        scene.rootNode.addChildNode(camera)

        // Floor grid: 0.25 m minor, 1 m major, 20 x 20 m, plus the start axes.
        var minor: [SCNVector3] = [], major: [SCNVector3] = []
        for step in stride(from: -10_000, through: 10_000, by: 250) {
            let segment = [SCNVector3(Float(step), 0, -10_000), SCNVector3(Float(step), 0, 10_000),
                           SCNVector3(-10_000, 0, Float(step)), SCNVector3(10_000, 0, Float(step))]
            if step % 1000 == 0 { major += segment } else { minor += segment }
        }
        world.addChildNode(SCNNode(geometry: lineGeometry(minor, strip: false, color: NSColor(rgb: 0x2E3034))))
        world.addChildNode(SCNNode(geometry: lineGeometry(major, strip: false, color: NSColor(rgb: 0x43454A))))
        world.addChildNode(SCNNode(geometry: lineGeometry([SCNVector3(0, 1, 0), SCNVector3(500, 1, 0)], strip: false,
                                                          color: Theme.accentNS)))
        world.addChildNode(SCNNode(geometry: lineGeometry([SCNVector3(0, 1, 0), SCNVector3(0, 1, 500)], strip: false,
                                                          color: Theme.accent2LineNS)))
        world.addChildNode(comparisonPath)
        world.addChildNode(path)
        startMarker.geometry?.firstMaterial = material(Theme.accent2LineNS)
        world.addChildNode(startMarker)
        world.addChildNode(rig.yawNode)

        let light = SCNNode()
        light.light = SCNLight()
        light.light?.type = .directional
        light.light?.intensity = 900
        light.eulerAngles = SCNVector3(-1.0, 0.6, 0)
        scene.rootNode.addChildNode(light)
        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 500
        scene.rootNode.addChildNode(ambient)
    }

    func bind(_ channel: AuraRobotRenderChannel) {
        guard subscription == nil else { return }
        subscription = channel.frames.receive(on: RunLoop.main).sink { [weak self] frame in
            guard let self else { return }
            self.snapshot = self.model.poseFromAura(frame.legs, state: frame.state, controller: frame.controller)
        }
    }

    func update(_ recorder: RouteRecorder, mode: RouteCameraMode, view: SCNView) {
        SCNTransaction.begin()
        SCNTransaction.disableActions = true
        if recorder.points.count != drawnCount {
            drawnCount = recorder.points.count
            path.geometry = lineGeometry(recorder.points.map { SCNVector3(Float($0.x), Float($0.y + 3), Float($0.z)) },
                                         strip: true, color: Theme.accentNS)
            startMarker.isHidden = recorder.points.isEmpty
            if let first = recorder.points.first { startMarker.position = SCNVector3(Float(first.x), Float(first.y), Float(first.z)) }
        }
        if recorder.comparison?.id != drawnComparison {
            drawnComparison = recorder.comparison?.id
            comparisonPath.geometry = lineGeometry(
                (recorder.comparison?.points ?? []).map { SCNVector3(Float($0.x), Float($0.y + 2), Float($0.z)) },
                strip: true, color: NSColor(rgb: 0x8C8F96))
        }
        let p = recorder.position
        let pose = snapshot ?? model.pose(at: ProcessInfo.processInfo.systemUptime, pad: AuraDualSense())
        rig.apply(pose, dimensions: model.visualFrameDimensions(), position: p, heading: recorder.heading,
                  roll: recorder.roll, pitch: recorder.pitch)

        // Camera lives outside the mirror: Aura (x, y, z) is (x, y, -z) here.
        let target = SCNVector3(CGFloat(p.x), CGFloat(p.y) + 120, CGFloat(-p.z))
        switch mode {
        case .orbit:
            if self.mode != .orbit {
                camera.camera?.usesOrthographicProjection = false
                camera.position = SCNVector3(target.x - 900, 700, target.z + 900)
                camera.look(at: target)
                view.allowsCameraControl = true
                view.defaultCameraController.interactionMode = .orbitTurntable
                view.defaultCameraController.target = target
            }
        case .top:
            // Map view: Aura forward to the right, its left up, as seen from above.
            view.allowsCameraControl = false
            camera.camera?.usesOrthographicProjection = true
            let extent = recorder.points.reduce(900.0) { max($0, abs($1.x - p.x), abs($1.z - p.z)) }
            camera.camera?.orthographicScale = min(12_000, extent * 1.3)
            camera.position = SCNVector3(target.x, 8000, target.z)
            camera.eulerAngles = SCNVector3(-CGFloat.pi / 2, 0, 0)
        case .follow:
            view.allowsCameraControl = false
            camera.camera?.usesOrthographicProjection = false
            let h = recorder.heading
            let back = CGFloat(cos(h) * 900), side = CGFloat(sin(h) * 900)
            camera.position = SCNVector3(target.x - back, 520, target.z + side)
            camera.look(at: target)
        }
        if self.mode != mode { view.pointOfView = camera }
        self.mode = mode
        SCNTransaction.commit()
    }
}

struct RouteSceneView: NSViewRepresentable {
    @ObservedObject var recorder: RouteRecorder
    let channel: AuraRobotRenderChannel
    let mode: RouteCameraMode

    func makeCoordinator() -> RouteScene { RouteScene() }

    func makeNSView(context: Context) -> SCNView {
        let view = SCNView()
        view.scene = context.coordinator.scene
        view.pointOfView = context.coordinator.camera
        view.backgroundColor = Theme.canvasNS
        view.antialiasingMode = .multisampling4X
        view.preferredFramesPerSecond = 30
        view.rendersContinuously = true
        context.coordinator.bind(channel)
        context.coordinator.update(recorder, mode: mode, view: view)
        return view
    }

    func updateNSView(_ view: SCNView, context: Context) {
        context.coordinator.update(recorder, mode: mode, view: view)
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
                HStack {
                    TextTabs(tabs: [(.orbit, "Orbit"), (.top, "Top"), (.follow, "Follow")], selection: $mode)
                    Spacer(minLength: 0)
                }
                .padding(.horizontal, 8).padding(.top, 8)
                .background(Theme.panel)
                Rectangle().fill(Theme.line).frame(height: 1)
                RouteSceneView(recorder: recorder, channel: aura.robotRenderChannel, mode: mode)
                    .overlay(alignment: .topLeading) { RecordingBadge(recorder: recorder) }
            }
            Rectangle().fill(Theme.line).frame(width: 1)
            RouteInspector(recorder: recorder, aura: aura)
        }
    }
}

private func metres(_ mm: Double) -> String { String(format: "%.2f m", mm / 1000) }

private struct RecordingBadge: View {
    @ObservedObject var recorder: RouteRecorder

    var body: some View {
        if recorder.recording {
            StatusDot(color: Theme.red, text: String(format: "REC %.0f s", recorder.duration))
                .padding(8)
                .background(Theme.panel.opacity(0.85), in: RoundedRectangle(cornerRadius: 4))
                .padding(10)
        }
    }
}

private struct RouteInspector: View {
    @ObservedObject var recorder: RouteRecorder
    @ObservedObject var aura: AuraConnection

    var body: some View {
        let current = RouteRecord(name: "", created: Date(), points: recorder.points)
        ScrollView {
            VStack(alignment: .leading, spacing: 10) {
                Panel("Recording") {
                    Grid(horizontalSpacing: 6, verticalSpacing: 6) {
                        GridRow {
                            if recorder.recording {
                                Button("Stop") { recorder.stop() }
                            } else {
                                Button("Record") { recorder.start() }.keyboardShortcut("r", modifiers: [.command])
                            }
                            Button("Save") { recorder.save() }.disabled(recorder.recording || recorder.points.count < 2)
                        }
                        GridRow {
                            Button("CSV") { recorder.exportCSV(nil) }.disabled(recorder.points.isEmpty)
                            Button("Clear") { recorder.clear() }.disabled(recorder.recording)
                        }
                    }
                    .buttonStyle(AuraButtonStyle(fill: true))
                }
                Panel("Route") {
                    ValueRow(label: "Distance", value: metres(current.distance))
                    ValueRow(label: "Time", value: String(format: "%.0f s", recorder.duration))
                    ValueRow(label: "Average", value: recorder.duration > 0
                             ? String(format: "%.0f mm/s", current.distance / recorder.duration) : "—")
                    ValueRow(label: "Max", value: String(format: "%.0f mm/s", recorder.maxSpeed))
                    ValueRow(label: "Climb", value: metres(current.climb))
                }
                Panel("Pose") {
                    ValueRow(label: "X / Z", value: String(format: "%.2f / %.2f m", recorder.position.x / 1000,
                                                           recorder.position.z / 1000))
                    ValueRow(label: "Height", value: metres(recorder.position.y))
                    ValueRow(label: "Heading", value: String(format: "%+.1f°", recorder.heading * 180 / .pi))
                    ValueRow(label: "Speed", value: String(format: "%.0f mm/s", recorder.speed))
                }
                Panel("Estimator", trailing: {
                    Button("Re-zero") { aura.rezeroEstimator(); recorder.clear() }
                        .buttonStyle(AuraButtonStyle(compact: true))
                        .disabled(!aura.isConnected || recorder.source != .aura)
                }) {
                    ValueRow(label: "Source", value: recorder.source == .aura ? "Aura Kalman"
                             : recorder.source == .desktop ? "Desktop" : "—",
                             tint: recorder.source == .aura ? Theme.green : Theme.yellow)
                    ValueRow(label: "Still", value: recorder.still ? "yes" : "no",
                             tint: recorder.still ? Theme.green : Theme.muted)
                    ValueRow(label: "Gyro bias", value: String(format: "%+.3f°/s", recorder.gyroBias))
                }
                .help("Kalman filter on Aura: IMU acceleration and gyro with leg kinematics, heading with gyro bias learned while still.")
                Panel("Saved") {
                    if recorder.saved.isEmpty {
                        Text("None").font(Theme.label).foregroundStyle(Theme.muted)
                    }
                    ForEach(recorder.saved) { record in
                        SavedRouteRow(recorder: recorder, record: record)
                    }
                }
            }
            .padding(10)
        }
        .frame(width: 250)
        .background(Theme.canvas)
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
            .help("Overlay in the scene")
            Button("CSV") { recorder.exportCSV(record) }
                .buttonStyle(.plain).font(Theme.label).foregroundStyle(Theme.muted)
            Button("Delete") { recorder.delete(record) }
                .buttonStyle(.plain).font(Theme.label).foregroundStyle(Theme.muted)
        }
        .padding(.vertical, 2)
    }
}
