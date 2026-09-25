import AppKit
import SceneKit
import SwiftUI
import simd

// A fixed-camera orientation check. The coordinate legend matches the frame
// used by firmware: X forward, Y up, Z left. The chassis rotates only from
// the filtered MPU6050 sample; it never sends a robot command.
struct IMUAttitudePreview: NSViewRepresentable {
    let rollDegrees: Double
    let pitchDegrees: Double
    let valid: Bool

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeNSView(context: Context) -> SCNView {
        let view = SCNView(frame: .zero)
        view.scene = context.coordinator.scene
        view.backgroundColor = .clear
        view.allowsCameraControl = false
        view.autoenablesDefaultLighting = false
        view.antialiasingMode = .multisampling4X
        return view
    }

    func updateNSView(_ view: SCNView, context: Context) {
        context.coordinator.update(rollDegrees: rollDegrees, pitchDegrees: pitchDegrees,
                                   valid: valid)
    }

    final class Coordinator {
        let scene = SCNScene()
        private let attitudeNode = SCNNode()
        private let statusRing = SCNNode()
        private var hasMeasurement = false

        init() {
            scene.rootNode.addChildNode(makeFloor())
            scene.rootNode.addChildNode(attitudeNode)
            attitudeNode.addChildNode(makeChassis())
            attitudeNode.addChildNode(makeAxisLegend())
            scene.rootNode.addChildNode(statusRing)
            configureCameraAndLights()
        }

        func update(rollDegrees: Double, pitchDegrees: Double, valid: Bool) {
            guard valid, rollDegrees.isFinite, pitchDegrees.isFinite else {
                statusRing.geometry?.firstMaterial?.diffuse.contents = NSColor.systemOrange
                return
            }
            hasMeasurement = true
            statusRing.geometry?.firstMaterial?.diffuse.contents = NSColor.systemGreen

            let roll = Float(rollDegrees * .pi / 180.0)
            let pitch = Float(pitchDegrees * .pi / 180.0)
            // SceneKit local axes: -Z = Aura forward, +Y = Aura up,
            // -X = Aura left. The observer's roll is around Aura +X and its
            // pitch is around Aura +Z, hence the two mapped scene axes below.
            let rollRotation = simd_quatf(angle: -roll, axis: SIMD3<Float>(0, 0, 1))
            let pitchRotation = simd_quatf(angle: -pitch, axis: SIMD3<Float>(1, 0, 0))
            SCNTransaction.begin()
            SCNTransaction.animationDuration = 0.08
            SCNTransaction.animationTimingFunction = CAMediaTimingFunction(name: .easeOut)
            attitudeNode.simdOrientation = simd_normalize(pitchRotation * rollRotation)
            SCNTransaction.commit()
        }

        private func configureCameraAndLights() {
            let camera = SCNNode()
            camera.camera = SCNCamera()
            camera.camera?.fieldOfView = 43
            camera.position = SCNVector3(4.2, 3.1, 5.2)
            camera.look(at: SCNVector3(0, 0, 0), up: SCNVector3(0, 1, 0), localFront: SCNVector3(0, 0, -1))
            scene.rootNode.addChildNode(camera)

            let key = SCNNode()
            key.light = SCNLight()
            key.light?.type = .directional
            key.light?.intensity = 900
            key.eulerAngles = SCNVector3(-0.8, 0.7, 0)
            scene.rootNode.addChildNode(key)

            let fill = SCNNode()
            fill.light = SCNLight()
            fill.light?.type = .ambient
            fill.light?.intensity = 520
            scene.rootNode.addChildNode(fill)
        }

        private func makeFloor() -> SCNNode {
            let floor = SCNFloor()
            floor.reflectivity = 0.08
            floor.firstMaterial?.diffuse.contents = NSColor(calibratedWhite: 0.12, alpha: 1)
            floor.firstMaterial?.roughness.contents = 0.95
            let node = SCNNode(geometry: floor)
            node.position.y = -0.86
            return node
        }

        private func material(_ colour: NSColor, metalness: CGFloat = 0.15) -> SCNMaterial {
            let material = SCNMaterial()
            material.diffuse.contents = colour
            material.metalness.contents = metalness
            material.roughness.contents = 0.42
            return material
        }

        private func makeChassis() -> SCNNode {
            let root = SCNNode()
            let chassis = SCNBox(width: 1.55, height: 0.16, length: 3.55, chamferRadius: 0.06)
            chassis.materials = [material(NSColor(calibratedWhite: 0.16, alpha: 1))]
            root.addChildNode(SCNNode(geometry: chassis))

            // Narrow coloured plate at the physical front of the frame. It
            // makes a wrong pitch sign immediately obvious to the operator.
            let front = SCNBox(width: 1.36, height: 0.03, length: 0.34, chamferRadius: 0.02)
            front.materials = [material(.systemRed, metalness: 0.05)]
            let frontNode = SCNNode(geometry: front)
            frontNode.position = SCNVector3(0, 0.10, -1.57)
            root.addChildNode(frontNode)

            let imu = SCNCylinder(radius: 0.19, height: 0.06)
            imu.materials = [material(.systemTeal, metalness: 0.08)]
            let imuNode = SCNNode(geometry: imu)
            imuNode.position = SCNVector3(0, 0.12, 0)
            root.addChildNode(imuNode)
            return root
        }

        private func makeAxisLegend() -> SCNNode {
            let root = SCNNode()
            // Every arrow begins at the assumed MPU position in the centre of
            // the chassis: X/front red, Y/up green, Z/left blue.
            root.addChildNode(axisArrow(direction: SCNVector3(0, 0, -1), colour: .systemRed))
            root.addChildNode(axisArrow(direction: SCNVector3(0, 1, 0), colour: .systemGreen))
            root.addChildNode(axisArrow(direction: SCNVector3(-1, 0, 0), colour: .systemBlue))
            return root
        }

        private func axisArrow(direction: SCNVector3, colour: NSColor) -> SCNNode {
            let root = SCNNode()
            let length: CGFloat = 0.7
            let shaft = SCNCylinder(radius: 0.025, height: CGFloat(length))
            shaft.materials = [material(colour, metalness: 0.0)]
            let shaftNode = SCNNode(geometry: shaft)
            shaftNode.position = direction * (length * 0.5)
            shaftNode.eulerAngles = rotationFromY(to: direction)
            root.addChildNode(shaftNode)

            let tip = SCNCone(topRadius: 0, bottomRadius: 0.075, height: 0.18)
            tip.materials = [material(colour, metalness: 0.0)]
            let tipNode = SCNNode(geometry: tip)
            tipNode.position = direction * (length + 0.08)
            tipNode.eulerAngles = rotationFromY(to: direction)
            root.addChildNode(tipNode)
            return root
        }

        private func rotationFromY(to direction: SCNVector3) -> SCNVector3 {
            // Node geometry points along +Y by default. This fixed mapping is
            // sufficient for the three cardinal arrows and avoids an arbitrary
            // screen-space convention in the calibration view.
            if direction.y > 0.5 { return SCNVector3(0, 0, 0) }
            if direction.z < -0.5 { return SCNVector3(-CGFloat.pi / 2, 0, 0) }
            return SCNVector3(0, 0, -CGFloat.pi / 2)
        }
    }
}

private func * (vector: SCNVector3, scalar: CGFloat) -> SCNVector3 {
    SCNVector3(vector.x * scalar, vector.y * scalar, vector.z * scalar)
}
