import Combine
import Darwin
import Foundation
import SceneKit
import SwiftUI
import simd

// Motion follows the same model used by ElectroPup: stance is a controlled
// ground stroke and swing is a Bezier-like lifting arc. All units are mm.
enum CalibrationPreviewMode: String, CaseIterable, Identifiable {
    case still
    case trot

    var id: Self { self }

    var title: String {
        switch self {
        case .still: "Pozycja referencyjna"
        case .trot: "Chód (Trot)"
        }
    }

    var isAnimated: Bool { self == .trot }
}

private enum QuadGait: String, CaseIterable {
    case stand = "Stanie", crawl = "Krok 1×", trot = "Trot", run = "Run", climb = "Climb", tripod = "3 łapy"
    var boardID: Int {
        switch self {
        case .stand: 0
        case .trot: 1
        case .crawl: 2
        case .run: 3
        case .climb: 4
        case .tripod: 5
        }
    }
    var isSingleFoot: Bool { self == .crawl || self == .climb }
    var next: QuadGait {
        Self.auraMode(Int(robot_locomotion_next_mode(UInt8(boardID))))
    }
    static func auraMode(_ value: Int) -> QuadGait {
        switch value {
        case 1: .trot
        case 2: .crawl
        case 3: .run
        case 4: .climb
        case 5: .tripod
        default: .stand
        }
    }
}

private enum QuadLeg: String, CaseIterable, Identifiable {
    case frontLeft = "FL", frontRight = "FR", rearLeft = "RL", rearRight = "RR"
    var id: String { rawValue }
    // Firmware telemetry slots are LF, RF, LR, RR.
    var wireIndex: Int {
        switch self {
        case .frontLeft: 0
        case .frontRight: 1
        case .rearLeft: 2
        case .rearRight: 3
        }
    }
    var front: Double { self == .frontLeft || self == .frontRight ? 1 : -1 }
    var left: Double { self == .frontLeft || self == .rearLeft ? 1 : -1 }
    var trotPhase: Double { (self == .frontLeft || self == .rearRight) ? 0 : 0.5 }
    var crawlPhase: Double { switch self { case .frontRight: 0; case .rearLeft: 0.25; case .frontLeft: 0.5; case .rearRight: 0.75 } }
    // Matches the firmware's quasistatic order: RR → LF → LR → RF.
    var staticCrawlOffset: Double {
        switch self {
        case .rearRight: 0
        case .frontLeft: 0.25
        case .rearLeft: 0.5
        case .frontRight: 0.75
        }
    }
    var title: String {
        switch self {
        case .frontLeft: "Lewy przód (FL)"
        case .frontRight: "Prawy przód (FR)"
        case .rearLeft: "Lewy tył (RL)"
        case .rearRight: "Prawy tył (RR)"
        }
    }
}

private struct QuadVector {
    var x: Double; var y: Double; var z: Double
    static func + (a: Self, b: Self) -> Self { .init(x: a.x + b.x, y: a.y + b.y, z: a.z + b.z) }
    static func - (a: Self, b: Self) -> Self { .init(x: a.x - b.x, y: a.y - b.y, z: a.z - b.z) }
    static func * (a: Self, b: Double) -> Self { .init(x: a.x * b, y: a.y * b, z: a.z * b) }
    var scn: SCNVector3 { .init(Float(x), Float(y), Float(z)) }
    var isFinite: Bool { x.isFinite && y.isFinite && z.isFinite }
}

private struct LegPose: Identifiable {
    let leg: QuadLeg; let yaw: Double; let hip: Double; let knee: Double; let points: [QuadVector]
    let grounded: Bool
    let footVelocity: QuadVector
    var id: String { leg.id }
}

// Result of an entirely local inverse-kinematics calculation.  The Test IK
// tab uses it before a trajectory is ever sent to Aura.
private struct IKProbeSolution {
    let reachable: Bool
    let abduction: Double
    let hip: Double
    let knee: Double
    let radialError: Double
}

private struct StaticCrawlStep {
    let bodyShift: Double
    let stroke: Double
    let lift: Double
    let airborne: Bool
}

private struct StaticCrawlPlan {
    let swingLeg: QuadLeg
    let bodyShift: QuadVector
}

// The direction-calibration screen overlays this on the shared 3D model.
// `reference` is the known mechanical pose and `target` differs by +12° in
// exactly one joint, so the arrow describes an actual model rotation rather
// than an arbitrary screen direction.
private struct CalibrationDirectionGuide {
    let leg: QuadLeg
    let axis: Int
    let reference: LegPose
    let target: LegPose
}

// Equivalent to ElectroPup's visual "rings": every foot follows its own
// circle around one instantaneous centre of rotation (CoR).
private struct TurnGuide {
    let center: QuadVector
    let radii: [QuadLeg: Double]
}

// The contact model intentionally distinguishes a polygon (three or four
// feet) from a line (the diagonal pair in trot).  A line has no static area:
// it can only be held by dynamic balancing with an IMU, not by a geometric
// foot path alone.
private enum GravitySupportMode: String {
    case stable
    case edge
    case dynamicLine
    case falling

    var title: String {
        switch self {
        case .stable: "stabilny wielokąt podparcia"
        case .edge: "mały margines podparcia"
        case .dynamicLine: "dwie stopy — wymagany balans dynamiczny"
        case .falling: "rzut środka masy poza podparciem"
        }
    }
}

private struct GravitySupport {
    let mode: GravitySupportMode
    let contacts: Int
    // Positive means inside a support polygon.  For a two-foot support it is
    // the signed distance from the support line: zero is the only neutral
    // static point.
    let margin: Double
    let projectedCOM: QuadVector
}

private struct QuadrupedSnapshot {
    let gait: QuadGait
    // One global [0, 1) gait cycle. It lets the renderer retain exactly one
    // trajectory cycle instead of connecting samples from several laps.
    let phase: Double
    let input: (forward: Double, lateral: Double, turn: Double, height: Double)
    let legs: [LegPose]
    let position: QuadVector
    // Planned torso translation in the level ground frame. Leg poses are
    // expressed in the torso frame, therefore the renderer adds this once to
    // expose the actual body-over-feet motion while keeping the camera centred.
    let bodyShift: QuadVector
    let yaw: Double
    let roll: Double
    let pitch: Double
    let bodyHeight: Double
    let isJumping: Bool
    let turnGuide: TurnGuide?
    let gravity: GravitySupport
}

private final class QuadrupedModel: ObservableObject {
    private var bodyTrajectory = robot_body_trajectory_t()
    @Published var tripodExcluded = 3
    private struct SavedGeometry: Codable, Equatable {
        var frameLength: Double
        var frameWidth: Double
        var bodyHeight: Double
        var cornerInset: Double
        var yawLink: Double
        var upperLeg: Double
        var lowerLeg: Double
        // Extra endpoint offset after the actual ab/ad link. This is the
        // number that widens the physical support footprint on Aura.
        var lateralStance: Double

        init(frameLength: Double = 360, frameWidth: Double = 130,
             bodyHeight: Double = 180, cornerInset: Double = 0,
             yawLink: Double = 35, upperLeg: Double = 130,
             lowerLeg: Double = 205, lateralStance: Double = 25) {
            self.frameLength = frameLength
            self.frameWidth = frameWidth
            self.bodyHeight = bodyHeight
            self.cornerInset = cornerInset
            self.yawLink = yawLink
            self.upperLeg = upperLeg
            self.lowerLeg = lowerLeg
            self.lateralStance = lateralStance
        }

        enum CodingKeys: String, CodingKey {
            case frameLength, frameWidth, bodyHeight, cornerInset, yawLink
            case upperLeg, lowerLeg, lateralStance
        }

        // Geometry v2 did not contain lateralStance. Decode it explicitly so
        // existing measured dimensions are retained instead of being reset.
        init(from decoder: Decoder) throws {
            let values = try decoder.container(keyedBy: CodingKeys.self)
            self.init(
                frameLength: try values.decodeIfPresent(Double.self, forKey: .frameLength) ?? 360,
                frameWidth: try values.decodeIfPresent(Double.self, forKey: .frameWidth) ?? 130,
                bodyHeight: try values.decodeIfPresent(Double.self, forKey: .bodyHeight) ?? 180,
                cornerInset: try values.decodeIfPresent(Double.self, forKey: .cornerInset) ?? 0,
                yawLink: try values.decodeIfPresent(Double.self, forKey: .yawLink) ?? 35,
                upperLeg: try values.decodeIfPresent(Double.self, forKey: .upperLeg) ?? 130,
                lowerLeg: try values.decodeIfPresent(Double.self, forKey: .lowerLeg) ?? 205,
                lateralStance: try values.decodeIfPresent(Double.self, forKey: .lateralStance) ?? 25)
        }
    }
    private static let geometryKey = "aura.robot.geometry.v2"
    private struct SavedStaticBalance: Codable {
        var comForward: Double = 0
        var comLeft: Double = 0
        var supportMargin: Double = 15
    }
    private static let staticBalanceKey = "aura.robot.static-balance.v1"
    private let persistsGeometry: Bool

    // A partially edited TextField may briefly publish zero, and older builds
    // could persist that value.  Zero link/frame lengths collapse every rod
    // into the same SceneKit point.  Rendering always uses a physically valid
    // geometry; valid custom dimensions are left untouched.
    private static func sane(_ value: Double, range: ClosedRange<Double>, fallback: Double) -> Double {
        value.isFinite && range.contains(value) ? value : fallback
    }

    private static func sanitised(_ source: SavedGeometry) -> SavedGeometry {
        var result = source
        result.frameLength = sane(result.frameLength, range: 150...700, fallback: 360)
        result.frameWidth = sane(result.frameWidth, range: 70...400, fallback: 130)
        result.bodyHeight = sane(result.bodyHeight, range: 80...350, fallback: 180)
        result.cornerInset = sane(result.cornerInset, range: 0...120, fallback: 0)
        result.yawLink = sane(result.yawLink, range: 10...120, fallback: 35)
        result.upperLeg = sane(result.upperLeg, range: 40...350, fallback: 130)
        result.lowerLeg = sane(result.lowerLeg, range: 40...400, fallback: 205)
        result.lateralStance = sane(result.lateralStance, range: 0...80, fallback: 25)
        return result
    }

    private func renderGeometry() -> SavedGeometry {
        Self.sanitised(SavedGeometry(frameLength: frameLength, frameWidth: frameWidth,
                                     bodyHeight: bodyHeight, cornerInset: cornerInset,
                                     yawLink: yawLink, upperLeg: upperLeg, lowerLeg: lowerLeg,
                                     lateralStance: lateralStance))
    }

    @Published var frameLength = 360.0 { didSet { saveGeometry() } }
    @Published var frameWidth = 130.0 { didSet { saveGeometry() } }
    @Published var bodyHeight = 180.0 { didSet { saveGeometry() } }
    @Published var cornerInset = 0.0 { didSet { saveGeometry() } }
    @Published var yawLink = 35.0 { didSet { saveGeometry() } }
    @Published var upperLeg = 130.0 { didSet { saveGeometry() } }
    @Published var lowerLeg = 205.0 { didSet { saveGeometry() } }
    // Stored as the extra distance per side; `finalFootSpreadMm` exposes the
    // endpoint-to-endpoint number that is meaningful while setting the stance.
    @Published var lateralStance = 25.0 { didSet { saveGeometry() } }
    // A local mirror of Aura's profiles solely for rendering. The real-time
    // planner and all physical motion stay on the ESP32.
    @Published private var locomotionProfiles = AuraGaitProfile.defaults
    @Published var configuredGait = 2
    // The mechanical centre of mass can differ from the geometric centre of
    // the frame because of the battery and the main board. These values are
    // setup data only; the board rejects them while torque is enabled.
    @Published var staticComForward = 0.0 { didSet { saveStaticBalance() } }
    @Published var staticComLeft = 0.0 { didSet { saveStaticBalance() } }
    @Published var staticSupportMargin = 15.0 { didSet { saveStaticBalance() } }
    // Published slowly (rather than every rendered frame) so the inspector
    // does not compete with the 50 Hz SceneKit path.
    @Published private(set) var gravityStatusText = "Grawitacja: oczekiwanie na kontakt stóp"
    @Published private(set) var gravityMarginMm = 0.0
    @Published private(set) var gravityRollDegrees = 0.0
    @Published private(set) var gravityPitchDegrees = 0.0
    @Published private(set) var cameraResetToken = 0
    private var worldPosition = QuadVector(x: 0, y: 0, z: 0)
    private var worldYaw = 0.0
    private var worldRoll = 0.0
    private var worldPitch = 0.0
    private var rollVelocity = 0.0
    private var pitchVelocity = 0.0
    private var gravityPublishElapsed = 0.0
    private var resetWasPressed = false
    private var r1WasPressed = false
    private var l1WasPressed = false
    private var crossWasPressed = false
    private var jumpStartedAt: TimeInterval?
    private(set) var selectedGait: QuadGait = .crawl
    private(set) var spinMode = false
    private var lastPoseTime: TimeInterval?
    private var previewPhase = 0.0
    private var filteredDrive = (forward: 0.0, lateral: 0.0, turn: 0.0, height: 0.0)
    private var lastAuraTick: UInt32?
    private var cachedAuraSnapshot: QuadrupedSnapshot?
    private var calibrationTick: UInt32 = 0
    // A stance foot owns one fixed point in the world.  The body's planar pose
    // is solved from these points every frame; it is never advanced directly
    // from a joystick value.
    private var plantedFeet: [QuadLeg: QuadVector] = [:]

    init(persistGeometry: Bool = true) {
        persistsGeometry = persistGeometry
        if let data = UserDefaults.standard.data(forKey: Self.geometryKey),
           let saved = try? JSONDecoder().decode(SavedGeometry.self, from: data) {
            let valid = Self.sanitised(saved)
            frameLength = valid.frameLength
            frameWidth = valid.frameWidth
            bodyHeight = valid.bodyHeight
            cornerInset = valid.cornerInset
            yawLink = valid.yawLink
            upperLeg = valid.upperLeg
            lowerLeg = valid.lowerLeg
            lateralStance = valid.lateralStance
            if persistsGeometry, valid != saved, let data = try? JSONEncoder().encode(valid) {
                UserDefaults.standard.set(data, forKey: Self.geometryKey)
            }
        }
        if let data = UserDefaults.standard.data(forKey: Self.staticBalanceKey),
           let saved = try? JSONDecoder().decode(SavedStaticBalance.self, from: data) {
            staticComForward = Self.sane(saved.comForward, range: -120...120, fallback: 0)
            staticComLeft = Self.sane(saved.comLeft, range: -120...120, fallback: 0)
            staticSupportMargin = Self.sane(saved.supportMargin, range: 5...45, fallback: 15)
        }
    }

    private func saveGeometry() {
        guard persistsGeometry else { return }
        let saved = renderGeometry()
        guard let data = try? JSONEncoder().encode(saved) else { return }
        UserDefaults.standard.set(data, forKey: Self.geometryKey)
    }

    private func saveStaticBalance() {
        guard persistsGeometry else { return }
        let saved = SavedStaticBalance(
            comForward: Self.sane(staticComForward, range: -120...120, fallback: 0),
            comLeft: Self.sane(staticComLeft, range: -120...120, fallback: 0),
            supportMargin: Self.sane(staticSupportMargin, range: 5...45, fallback: 15))
        guard let data = try? JSONEncoder().encode(saved) else { return }
        UserDefaults.standard.set(data, forKey: Self.staticBalanceKey)
    }

    // Final lateral distance from the left foot endpoint to the right foot
    // endpoint in the neutral stance. It is derived from actual frame/link
    // dimensions, rather than treating servo housing width as kinematics.
    var finalFootSpreadMm: Double {
        get {
            let geometry = renderGeometry()
            return geometry.frameWidth - 2 * geometry.cornerInset +
                   2 * (geometry.yawLink + geometry.lateralStance)
        }
        set {
            let base = frameWidth - 2 * cornerInset + 2 * yawLink
            lateralStance = min(80, max(0, (newValue - base) / 2))
        }
    }

    func resetCamera() { cameraResetToken &+= 1 }

    func setLocomotionProfiles(_ profiles: [AuraGaitProfile]) {
        let valid = profiles.filter { (1...5).contains($0.gait) }
        guard (1...4).allSatisfy({ gait in valid.contains(where: { $0.gait == gait }) }) else { return }
        locomotionProfiles = AuraGaitProfile.defaults.map { fallback in
            valid.first(where: { $0.gait == fallback.gait }) ?? fallback
        }
    }

    private func locomotionProfile(for gait: QuadGait) -> AuraGaitProfile {
        locomotionProfiles.first(where: { $0.gait == gait.boardID }) ??
            AuraGaitProfile.defaults.first(where: { $0.gait == gait.boardID }) ??
            AuraGaitProfile.defaults[0]
    }

    func visualFrameDimensions() -> (length: Double, width: Double) {
        let geometry = renderGeometry()
        return (geometry.frameLength, geometry.frameWidth)
    }

    // Offline-only virtual pad.  It shares the desktop kinematic model but
    // has no AuraConnection and therefore cannot produce a radio command.
    func ikLabSnapshot(at time: TimeInterval, gait: QuadGait,
                       forward: Double, lateral: Double, turn: Double,
                       running: Bool) -> QuadrupedSnapshot {
        selectedGait = gait
        spinMode = false
        jumpStartedAt = nil
        var pad = AuraDualSense()
        func raw(_ value: Double) -> Int {
            Int(max(1, min(255, (128.0 + value * 127.0).rounded())))
        }
        // Stand is a true stationary reference in Test IK.  The main robot
        // intentionally turns Stand plus stick input into Trot, but carrying
        // over a virtual stick while this picker says “Stanie” is confusing.
        if running && gait != .stand {
            // `input` maps left-Y with an inverted sign.
            pad.leftY = raw(-forward)
            pad.leftX = raw(lateral)
            pad.rightX = raw(turn)
        }
        return pose(at: time, pad: pad)
    }

    private func smootherstep(_ value: Double) -> Double {
        let u = min(1, max(0, value))
        return u * u * u * (u * (u * 6 - 15) + 10)
    }

    private func staticCrawlPreload(for profile: AuraGaitProfile) -> Double {
        // Four serial leg slots share one whole gait cycle. Match the firmware:
        // `Kontakt` sets each foot's airborne fraction and the remainder of
        // its slot becomes the CoM pre-load before liftoff.
        let duty = Double(profile.dutyPercent) / 100.0
        return min(0.60, max(0.02, 1.0 - 4.0 * (1.0 - duty)))
    }

    private func staticCrawlStep(at phase: Double, preloadEnd: Double) -> StaticCrawlStep {
        let phase = phase - floor(phase)
        let preloadEnd = min(0.60, max(0.02, preloadEnd))
        if phase < preloadEnd {
            return StaticCrawlStep(bodyShift: smootherstep(phase / preloadEnd),
                                   stroke: -0.5, lift: 0, airborne: false)
        }
        let progress = smootherstep((phase - preloadEnd) / (1 - preloadEnd))
        return StaticCrawlStep(bodyShift: 1,
                               stroke: -0.5 + progress,
                               lift: 4 * progress * (1 - progress),
                               airborne: phase < 0.9999)
    }

    private func staticCrawlStanceStroke(for leg: QuadLeg, globalPhase: Double,
                                         preloadEnd: Double) -> Double {
        var cycle = globalPhase - leg.staticCrawlOffset
        cycle -= floor(cycle)
        if cycle < 0.25 { return staticCrawlStep(at: cycle * 4, preloadEnd: preloadEnd).stroke }
        let touchdownCycle = 0.25
        return 0.5 - smootherstep((cycle - touchdownCycle) / (1 - touchdownCycle))
    }

    private func staticCrawlPlan(at globalPhase: Double, geometry: SavedGeometry,
                                 forward: Double, lateral: Double, stride: Double,
                                 profile: AuraGaitProfile) -> StaticCrawlPlan {
        let preloadEnd = staticCrawlPreload(for: profile)
        let scaled = globalPhase * 4
        let slot = Int(floor(scaled)) & 3
        let local = scaled - floor(scaled)
        let order: [QuadLeg] = [.rearRight, .frontLeft, .rearLeft, .frontRight]
        let swingLeg = order[slot]
        let previousLeg = order[(slot + 3) & 3]

        func footBase(_ leg: QuadLeg) -> QuadVector {
            QuadVector(x: leg.front * (geometry.frameLength / 2 - geometry.cornerInset), y: 0,
                       z: leg.left * (geometry.frameWidth / 2 - geometry.cornerInset + geometry.yawLink + geometry.lateralStance))
        }
        func unshiftedFoot(_ leg: QuadLeg) -> QuadVector {
            let stroke = staticCrawlStanceStroke(for: leg, globalPhase: globalPhase,
                                                  preloadEnd: preloadEnd) * stride
            return footBase(leg) + QuadVector(x: stroke * forward, y: 0, z: stroke * lateral)
        }
        func safeBodyTranslation(excluding excluded: QuadLeg) -> QuadVector {
            let support = QuadLeg.allCases.filter { $0 != excluded }.map(unshiftedFoot)
            guard support.count == 3 else { return .init(x: 0, y: 0, z: 0) }
            let area2 = (support[1].x - support[0].x) * (support[2].z - support[0].z) -
                        (support[1].z - support[0].z) * (support[2].x - support[0].x)
            guard abs(area2) > 0.001 else { return .init(x: 0, y: 0, z: 0) }
            let winding = area2 > 0 ? 1.0 : -1.0
            let lengths = (0..<3).map { index -> Double in
                let end = support[(index + 1) % 3]
                return hypot(end.x - support[index].x, end.z - support[index].z)
            }
            let perimeter = lengths.reduce(0, +)
            let inradius = abs(area2) / max(0.001, perimeter)
            let requiredMargin = staticSupportMargin
            guard inradius + 0.02 >= requiredMargin else { return .init(x: 0, y: 0, z: 0) }

            // Project the *real* centre of mass into the support triangle,
            // inset by the configured margin.  The old desktop planner used
            // the geometrical centre of the frame here, which is wrong as
            // soon as the battery shifts the mass rearward or sideways.
            var target = QuadVector(x: staticComForward, y: 0, z: staticComLeft)
            for _ in 0..<12 {
                var adjusted = false
                for index in 0..<3 {
                    let start = support[index]
                    let end = support[(index + 1) % 3]
                    let dx = end.x - start.x, dz = end.z - start.z
                    let length = lengths[index]
                    guard length > 0.001 else { return .init(x: 0, y: 0, z: 0) }
                    let nx = winding * -dz / length, nz = winding * dx / length
                    let distance = (target.x - start.x) * nx + (target.z - start.z) * nz
                    if distance < requiredMargin {
                        target.x += nx * (requiredMargin - distance)
                        target.z += nz * (requiredMargin - distance)
                        adjusted = true
                    }
                }
                if !adjusted { break }
            }
            return target - QuadVector(x: staticComForward, y: 0, z: staticComLeft)
        }

        let from = safeBodyTranslation(excluding: previousLeg)
        let to = safeBodyTranslation(excluding: swingLeg)
        let fraction = staticCrawlStep(at: local, preloadEnd: preloadEnd).bodyShift
        return StaticCrawlPlan(swingLeg: swingLeg,
                               bodyShift: from + (to + (from * -1)) * fraction)
    }

    func ikLabProbe(leg: QuadLeg, target: QuadVector) -> IKProbeSolution {
        let geometry = renderGeometry()
        let anchor = QuadVector(x: leg.front * (geometry.frameLength / 2 - geometry.cornerInset),
                                y: geometry.bodyHeight,
                                z: leg.left * (geometry.frameWidth / 2 - geometry.cornerInset))
        let frameAngle: Double = leg.left > 0 ? .pi / 2 : -.pi / 2
        let dx = target.x - anchor.x, dy = target.y - anchor.y, dz = target.z - anchor.z
        let localX = cos(frameAngle) * dx - sin(frameAngle) * dz
        let localY = dy
        let localZ = sin(frameAngle) * dx + cos(frameAngle) * dz
        let planarSquared = localX * localX + localY * localY - geometry.yawLink * geometry.yawLink
        guard planarSquared >= -0.01 else {
            return IKProbeSolution(reachable: false, abduction: 0, hip: 0, knee: 0,
                                   radialError: sqrt(abs(planarSquared)))
        }
        let planar = sqrt(max(0, planarSquared))
        let rawCosine = (localX * localX + localY * localY + localZ * localZ -
                         geometry.yawLink * geometry.yawLink - geometry.upperLeg * geometry.upperLeg -
                         geometry.lowerLeg * geometry.lowerLeg) / (2 * geometry.upperLeg * geometry.lowerLeg)
        guard rawCosine >= -1.0001, rawCosine <= 1.0001 else {
            return IKProbeSolution(reachable: false, abduction: 0, hip: 0, knee: 0,
                                   radialError: abs(rawCosine) - 1)
        }
        let cosine = min(1, max(-1, rawCosine))
        let knee = leg.left > 0 ? atan2(sqrt(max(0, 1 - cosine * cosine)), cosine)
                                : atan2(-sqrt(max(0, 1 - cosine * cosine)), cosine)
        let hip = atan2(localZ, planar) - atan2(geometry.lowerLeg * sin(knee),
                                                geometry.upperLeg + geometry.lowerLeg * cos(knee))
        let abduction = atan2(localY, localX) + atan2(planar, -geometry.yawLink)
        return IKProbeSolution(reachable: true, abduction: abduction * 180 / .pi,
                               hip: hip * 180 / .pi, knee: knee * 180 / .pi,
                               radialError: 0)
    }

    func ikLabNeutralTarget(for leg: QuadLeg) -> QuadVector {
        let geometry = renderGeometry()
        return QuadVector(
            x: leg.front * (geometry.frameLength / 2 - geometry.cornerInset),
            y: 0,
            z: leg.left * (geometry.frameWidth / 2 - geometry.cornerInset + geometry.yawLink + geometry.lateralStance)
        )
    }

    private func updateButtonEvents(_ pad: AuraDualSense, at time: TimeInterval) {
        let r1Pressed = pad.buttons[1] & 0x02 != 0
        if r1Pressed && !r1WasPressed { selectedGait = selectedGait.next }
        r1WasPressed = r1Pressed

        let l1Pressed = pad.buttons[1] & 0x01 != 0
        if l1Pressed && !l1WasPressed { spinMode.toggle() }
        l1WasPressed = l1Pressed

        // × is deliberately an edge-triggered event. Holding it must not make
        // the robot start a new jump 50 times per second.
        let crossPressed = pad.buttons[0] & 0x20 != 0
        if crossPressed && !crossWasPressed { jumpStartedAt = time }
        crossWasPressed = crossPressed
    }

    func input(_ pad: AuraDualSense) -> (forward: Double, lateral: Double, turn: Double, height: Double) {
        func axis(_ value: Int) -> Double { let raw = max(-1, min(1, Double(value - 128) / 127.0)); return abs(raw) < 0.07 ? 0 : raw }
        return (-axis(pad.leftY), axis(pad.leftX), axis(pad.rightX), -axis(pad.rightY))
    }

    // Exact MIT FootSwingTrajectory profile shared with Aura: cubic Bézier
    // in horizontal motion and two equal half-swing vertical Bézier sections.
    private func cubicBezier(_ from: Double, _ to: Double, _ phase: Double) -> Double {
        let t = min(1, max(0, phase))
        let progress = t * t * (3 - 2 * t)
        return from + (to - from) * progress
    }

    private func gaitFootPath(phase: Double, duty: Double,
                              stride: Double, height: Double,
                              active: Bool) -> (stroke: Double, lift: Double, stance: Bool) {
        let path = robot_gait_foot_path(Float(phase), Float(duty), Float(stride), Float(height), active)
        return (Double(path.stroke_mm), Double(path.lift_mm), path.stance)
    }

    private func footBase(for leg: QuadLeg, geometry: SavedGeometry) -> QuadVector {
        QuadVector(x: leg.front * (geometry.frameLength / 2 - geometry.cornerInset), y: 0,
                   z: leg.left * (geometry.frameWidth / 2 - geometry.cornerInset +
                                  geometry.yawLink + geometry.lateralStance))
    }

    // The same portable body/foot planner is compiled into ESP and the preview.
    private func portableRequest(profile: AuraGaitProfile, geometry: SavedGeometry,
                                 drive: (forward: Double, lateral: Double, turn: Double, height: Double),
                                 stride: Double, steppingInPlace: Bool,
                                 turnCentre: QuadVector?, excludedLeg: Int? = nil, spin: Bool = false) -> robot_body_trajectory_request_t {
        var request = robot_body_trajectory_request_t()
        request.gait = UInt8(profile.gait)
        request.profile = robot_locomotion_profile_t(
            stride_mm: UInt16(profile.strideMm), step_height_mm: UInt16(profile.stepHeightMm),
            frequency_centi_hz: UInt16(profile.frequencyCentiHz), duty_percent: UInt8(profile.dutyPercent))
        request.geometry = robot_geometry_t(frame_length_mm: Float(geometry.frameLength),
            frame_width_mm: Float(geometry.frameWidth), corner_inset_mm: Float(geometry.cornerInset),
            abduction_link_mm: Float(geometry.yawLink), upper_leg_mm: Float(geometry.upperLeg),
            lower_leg_mm: Float(geometry.lowerLeg))
        request.body_height_mm = Float(geometry.bodyHeight + drive.height * 35)
        request.com_height_mm = request.body_height_mm - 25
        request.com_offset_x_mm = Float(staticComForward); request.com_offset_z_mm = Float(staticComLeft)
        request.joint_velocity_limit = 2 * Float.pi * 3400 / 4095
        request.joint_acceleration_limit = 2 * Float.pi * 25400 / 4095
        request.lateral_stance_mm = Float(geometry.lateralStance)
        request.forward = Float(drive.forward); request.lateral = Float(drive.lateral)
        request.turn = Float(drive.turn)
        request.spin = spin
        request.excluded_leg = robot_leg_t(rawValue: UInt32(excludedLeg ?? tripodExcluded))
        request.stride_mm = Float(stride); request.stepping_in_place = steppingInPlace
        return request
    }

    private func dynamicBodyShift(phaseTime: Double, gait: QuadGait,
                                  profile: AuraGaitProfile, geometry: SavedGeometry,
                                  drive: (forward: Double, lateral: Double, turn: Double, height: Double),
                                  stride: Double, steppingInPlace: Bool,
                                  turnCentre: QuadVector?, excludedLeg: Int? = nil, spin: Bool = false) -> QuadVector {
        guard gait != .stand else { return .init(x: 0, y: 0, z: 0) }
        var request = portableRequest(profile: profile, geometry: geometry, drive: drive,
                                      stride: stride, steppingInPlace: steppingInPlace, turnCentre: turnCentre,
                                      excludedLeg: excludedLeg, spin: spin)
        if robot_body_trajectory_build(&bodyTrajectory, &request) {
            let sample = robot_body_trajectory_sample(&bodyTrajectory, Float(phaseTime))
            return QuadVector(x: Double(sample.position_mm.x) - staticComForward, y: 0,
                              z: Double(sample.position_mm.z) - staticComLeft)
        }
        return .init(x: 0, y: 0, z: 0)
    }

    // Matches body_pose_foot_target() in robot_gait.c. The gait creates a
    // foot path in the level world frame; this single inverse transform gives
    // the path to IK from the translated and tilted torso frame.
    private func bodyPoseFootTarget(_ worldTarget: QuadVector, bodyHeight: Double,
                                    shift: QuadVector, roll: Double, pitch: Double) -> QuadVector {
        let relative = QuadVector(x: worldTarget.x - shift.x,
                                  y: worldTarget.y - bodyHeight,
                                  z: worldTarget.z - shift.z)
        let cPitch = cos(pitch), sPitch = sin(pitch)
        let cRoll = cos(roll), sRoll = sin(roll)
        let x1 = cPitch * relative.x + sPitch * relative.y
        let y1 = -sPitch * relative.x + cPitch * relative.y
        return QuadVector(x: x1,
                          y: cRoll * y1 + sRoll * relative.z + bodyHeight,
                          z: -sRoll * y1 + cRoll * relative.z)
    }

    private func jumpLift(at time: TimeInterval) -> Double {
        guard let jumpStartedAt else { return 0 }
        let phase = (time - jumpStartedAt) / 0.66
        guard phase < 1 else { self.jumpStartedAt = nil; return 0 }
        // Feet remain on the floor while the body rises and returns. The IK
        // therefore extends and retracts the three real joint axes.
        return 52 * sin(.pi * max(0, phase))
    }

    private func turningCentre(for drive: (forward: Double, lateral: Double, turn: Double)) -> QuadVector? {
        guard abs(drive.turn) > 0.02 else { return nil }
        // v = omega × CoR in Aura's X-forward / Z-left body frame.
        // 220 mm gives a useful visual scale while preserving the exact ratio.
        let scale = 220.0 / drive.turn
        return QuadVector(x: -drive.lateral * scale, y: 0, z: drive.forward * scale)
    }

    // Coordinate convention shared by the model and renderer:
    // X is forward, Y is up, Z is left.  Pitch is rotation about Z and roll
    // about X.  A positive roll lowers the left side; a positive pitch lowers
    // the rear.  This is useful because a rear-mounted battery naturally
    // produces a positive pitch fall when it lies outside the support area.
    private func rotatedBodyPoint(_ local: QuadVector) -> QuadVector {
        let cp = cos(worldPitch), sp = sin(worldPitch)
        let pitched = QuadVector(x: cp * local.x - sp * local.y,
                                 y: sp * local.x + cp * local.y,
                                 z: local.z)
        let cr = cos(worldRoll), sr = sin(worldRoll)
        return QuadVector(x: pitched.x,
                          y: cr * pitched.y - sr * pitched.z,
                          z: sr * pitched.y + cr * pitched.z)
    }

    private func inverseRotatedBodyPoint(_ point: QuadVector) -> QuadVector {
        let cr = cos(worldRoll), sr = sin(worldRoll)
        let unrolled = QuadVector(x: point.x,
                                  y: cr * point.y + sr * point.z,
                                  z: -sr * point.y + cr * point.z)
        let cp = cos(worldPitch), sp = sin(worldPitch)
        return QuadVector(x: cp * unrolled.x + sp * unrolled.y,
                          y: -sp * unrolled.x + cp * unrolled.y,
                          z: unrolled.z)
    }

    private func worldPoint(_ local: QuadVector) -> QuadVector {
        let rotated = rotatedBodyPoint(local)
        let c = cos(worldYaw), s = sin(worldYaw)
        return QuadVector(x: worldPosition.x + c * rotated.x - s * rotated.z,
                          y: worldPosition.y + rotated.y,
                          z: worldPosition.z + s * rotated.x + c * rotated.z)
    }

    private func localPoint(_ world: QuadVector, yaw: Double) -> QuadVector {
        let dx = world.x - worldPosition.x, dz = world.z - worldPosition.z
        let c = cos(yaw), s = sin(yaw)
        return inverseRotatedBodyPoint(QuadVector(x: c * dx + s * dz,
                                                  y: world.y - worldPosition.y,
                                                  z: -s * dx + c * dz))
    }

    private func resetGravityState() {
        worldRoll = 0; worldPitch = 0
        rollVelocity = 0; pitchVelocity = 0
    }

    func centreOfMassLocal(bodyHeight: Double) -> QuadVector {
        QuadVector(x: staticComForward, y: max(55, bodyHeight - 25), z: staticComLeft)
    }

    private func closestPoint(on segmentStart: QuadVector, _ segmentEnd: QuadVector,
                              to point: QuadVector) -> QuadVector {
        let dx = segmentEnd.x - segmentStart.x, dz = segmentEnd.z - segmentStart.z
        let lengthSquared = dx * dx + dz * dz
        guard lengthSquared > 0.0001 else { return segmentStart }
        let t = min(1, max(0, ((point.x - segmentStart.x) * dx +
                               (point.z - segmentStart.z) * dz) / lengthSquared))
        return QuadVector(x: segmentStart.x + dx * t, y: 0, z: segmentStart.z + dz * t)
    }

    private func xzDistance(_ a: QuadVector, _ b: QuadVector) -> Double {
        hypot(a.x - b.x, a.z - b.z)
    }

    private func convexHull(_ source: [QuadVector]) -> [QuadVector] {
        let sorted = source.sorted { lhs, rhs in
            abs(lhs.x - rhs.x) > 0.001 ? lhs.x < rhs.x : lhs.z < rhs.z
        }
        guard sorted.count > 2 else { return sorted }
        func turn(_ origin: QuadVector, _ a: QuadVector, _ b: QuadVector) -> Double {
            (a.x - origin.x) * (b.z - origin.z) - (a.z - origin.z) * (b.x - origin.x)
        }
        var lower: [QuadVector] = []
        for point in sorted {
            while lower.count >= 2 && turn(lower[lower.count - 2], lower[lower.count - 1], point) <= 0 {
                lower.removeLast()
            }
            lower.append(point)
        }
        var upper: [QuadVector] = []
        for point in sorted.reversed() {
            while upper.count >= 2 && turn(upper[upper.count - 2], upper[upper.count - 1], point) <= 0 {
                upper.removeLast()
            }
            upper.append(point)
        }
        lower.removeLast(); upper.removeLast()
        return lower + upper
    }

    private func gravitySupport(for contacts: [QuadVector], projectedCOM: QuadVector)
        -> (GravitySupport, QuadVector?) {
        switch contacts.count {
        case 0:
            return (GravitySupport(mode: .falling, contacts: 0, margin: -.infinity,
                                   projectedCOM: projectedCOM), nil)
        case 1:
            let escape = projectedCOM - contacts[0]
            return (GravitySupport(mode: .falling, contacts: 1,
                                   margin: -xzDistance(projectedCOM, contacts[0]),
                                   projectedCOM: projectedCOM), escape)
        case 2:
            let nearest = closestPoint(on: contacts[0], contacts[1], to: projectedCOM)
            let escape = projectedCOM - nearest
            // A pair of contacts is a line, not a support polygon.  Even a
            // very small non-zero lever arm creates a gravity moment.
            return (GravitySupport(mode: .dynamicLine, contacts: 2,
                                   margin: -xzDistance(projectedCOM, nearest),
                                   projectedCOM: projectedCOM), escape)
        default:
            let hull = convexHull(contacts)
            guard hull.count >= 3 else {
                return gravitySupport(for: Array(hull.prefix(2)), projectedCOM: projectedCOM)
            }
            var signs: [Double] = []
            var nearest = hull[0]
            var nearestDistance = Double.infinity
            for index in hull.indices {
                let start = hull[index], end = hull[(index + 1) % hull.count]
                let side = (end.x - start.x) * (projectedCOM.z - start.z) -
                           (end.z - start.z) * (projectedCOM.x - start.x)
                signs.append(side)
                let candidate = closestPoint(on: start, end, to: projectedCOM)
                let distance = xzDistance(projectedCOM, candidate)
                if distance < nearestDistance { nearestDistance = distance; nearest = candidate }
            }
            let hasPositive = signs.contains { $0 > 0.001 }
            let hasNegative = signs.contains { $0 < -0.001 }
            let inside = !(hasPositive && hasNegative)
            if inside {
                let mode: GravitySupportMode = nearestDistance >= staticSupportMargin ? .stable : .edge
                return (GravitySupport(mode: mode, contacts: contacts.count,
                                       margin: nearestDistance, projectedCOM: projectedCOM), nil)
            }
            return (GravitySupport(mode: .falling, contacts: contacts.count,
                                   margin: -nearestDistance, projectedCOM: projectedCOM),
                    projectedCOM - nearest)
        }
    }

    // The desktop scene evaluates gravity against the active support geometry,
    // but it does not invent a roll or pitch from commanded leg angles. Those
    // angles are not a body-attitude measurement: without a working IMU,
    // integrating a free fall here made a visual model tip over even when the
    // real robot's stance and compliance were unknown. The board planner
    // still receives the support/CoM correction; a future IMU loop will own
    // measured body attitude.
    private func applyGravity(from poses: [LegPose], bodyHeight: Double, dt: Double) -> GravitySupport {
        _ = dt
        resetGravityState()
        let massCentre = worldPoint(centreOfMassLocal(bodyHeight: bodyHeight))
        let projection = QuadVector(x: massCentre.x, y: 0, z: massCentre.z)
        let contacts = poses.filter(\.grounded).compactMap { plantedFeet[$0.leg] }
        let (support, _) = gravitySupport(for: contacts, projectedCOM: projection)

        gravityPublishElapsed += dt
        if gravityPublishElapsed >= 0.10 || gravityStatusText.hasSuffix("oczekiwanie na kontakt stóp") {
            gravityPublishElapsed = 0
            gravityStatusText = "Podparcie: \(support.mode.title) • \(support.contacts) stopy • margines \(String(format: "%+.0f", support.margin)) mm"
            gravityMarginMm = support.margin.isFinite ? support.margin : 0
            gravityRollDegrees = worldRoll * 180 / .pi
            gravityPitchDegrees = worldPitch * 180 / .pi
        }
        return support
    }

    private func solveBodyPose(from poses: [LegPose], jumpLift: Double) {
        guard jumpLift < 0.01 else {
            plantedFeet.removeAll()
            let floorOffset = max(0, -(poses.map { rotatedBodyPoint($0.points[3]).y }.min() ?? 0))
            worldPosition.y = floorOffset + jumpLift
            return
        }
        let contacts = poses.filter { $0.grounded }
        let contactLegs = Set(contacts.map(\.leg))
        plantedFeet = plantedFeet.filter { contactLegs.contains($0.key) }
        for pose in contacts where plantedFeet[pose.leg] == nil {
            let point = worldPoint(pose.points[3])
            plantedFeet[pose.leg] = QuadVector(x: point.x, y: 0, z: point.z)
        }
        guard !contacts.isEmpty else { return }

        if contacts.count == 1, let pose = contacts.first, let anchor = plantedFeet[pose.leg] {
            let foot = rotatedBodyPoint(pose.points[3])
            let c = cos(worldYaw), s = sin(worldYaw)
            worldPosition.x = anchor.x - c * foot.x + s * foot.z
            worldPosition.z = anchor.z - s * foot.x - c * foot.z
        } else {
            let count = Double(contacts.count)
            let local = contacts.map { rotatedBodyPoint($0.points[3]) }
            let localMeanX = local.map(\.x).reduce(0, +) / count
            let localMeanZ = local.map(\.z).reduce(0, +) / count
            let anchors = contacts.compactMap { plantedFeet[$0.leg] }
            let anchorMeanX = anchors.map(\.x).reduce(0, +) / count
            let anchorMeanZ = anchors.map(\.z).reduce(0, +) / count
            var dot = 0.0, cross = 0.0
            for (index, pose) in contacts.enumerated() {
                guard let anchor = plantedFeet[pose.leg] else { continue }
                let px = local[index].x - localMeanX
                let pz = local[index].z - localMeanZ
                let ax = anchor.x - anchorMeanX
                let az = anchor.z - anchorMeanZ
                dot += px * ax + pz * az
                cross += px * az - pz * ax
            }
            worldYaw = atan2(cross, dot)
            let c = cos(worldYaw), s = sin(worldYaw)
            worldPosition.x = anchorMeanX - c * localMeanX + s * localMeanZ
            worldPosition.z = anchorMeanZ - s * localMeanX - c * localMeanZ
        }
        // Preserve the floor contact after any gravity-driven body attitude.
        let floorOffset = max(0, -(contacts.map { rotatedBodyPoint($0.points[3]).y }.min() ?? 0))
        worldPosition.y = floorOffset
    }

    func pose(at time: TimeInterval, pad: AuraDualSense) -> QuadrupedSnapshot {
        let geometry = renderGeometry()
        lastAuraTick = nil
        cachedAuraSnapshot = nil
        updateButtonEvents(pad, at: time)
        let dt = min(0.05, max(0, time - (lastPoseTime ?? time)))
        let rawDrive = input(pad)
        let rawMagnitude = min(1, sqrt(rawDrive.forward * rawDrive.forward + rawDrive.lateral * rawDrive.lateral + rawDrive.turn * rawDrive.turn))
        let currentJumpLift = jumpLift(at: time)
        let jumping = currentJumpLift > 0
        let steppingInPlace = (selectedGait == .trot || selectedGait == .tripod) && rawMagnitude < 0.02
        func filtered(_ before: Double, _ after: Double) -> Double {
            Double(robot_locomotion_filter_command(Float(before), Float(after), Float(dt)))
        }
        filteredDrive = (filtered(filteredDrive.forward, rawDrive.forward),
                         filtered(filteredDrive.lateral, rawDrive.lateral),
                         filtered(filteredDrive.turn, rawDrive.turn),
                         filtered(filteredDrive.height, rawDrive.height))
        let drive = filteredDrive
        let magnitude = min(1, sqrt(drive.forward * drive.forward + drive.lateral * drive.lateral + drive.turn * drive.turn))
        // In Stanie the robot is still ready: the first stick command starts a
        // trot, while a neutral pad keeps every foot planted.
        let gait: QuadGait = selectedGait == .stand && rawMagnitude > 0.02 ? .trot : selectedGait
        let spinRequested = spinMode && !jumping && abs(rawDrive.turn) > 0.02
        let moving = !jumping && gait != .stand && (steppingInPlace || magnitude > 0.02 || spinRequested)
        let optionsPressed = pad.buttons[1] & 0x20 != 0
        if optionsPressed && !resetWasPressed {
            worldPosition = QuadVector(x: 0, y: 0, z: 0); worldYaw = 0
            resetGravityState()
        }
        resetWasPressed = optionsPressed
        let profile = locomotionProfile(for: gait)
        // Prepare the same bounded period as ESP before advancing its clock.
        let previewStatic = gait.isSingleFoot && moving && !spinRequested && abs(drive.turn) <= 0.02
        var cadence = profile.frequencyHz
        if moving && !previewStatic {
            var request = portableRequest(profile: profile, geometry: geometry, drive: drive,
                stride: Double(profile.strideMm) * max(0.25, steppingInPlace ? 0.35 : magnitude),
                steppingInPlace: steppingInPlace, turnCentre: nil, spin: spinRequested)
            cadence = robot_body_trajectory_build(&bodyTrajectory, &request)
                ? Double(bodyTrajectory.effective_frequency_hz) : 0
        }
        if moving { previewPhase = (previewPhase + dt * cadence).truncatingRemainder(dividingBy: 1) }
        let phaseTime = previewPhase
        let motionScale = steppingInPlace ? 0.35 : magnitude
        let stride = Double(profile.strideMm) * max(0.25, motionScale)
        let effectiveBodyHeight = geometry.bodyHeight + rawDrive.height * 35
        // The local Test IK scene mirrors the firmware's quasistatic crawl:
        // preload the support triangle, lift vertically, translate at a fixed
        // clearance, lower vertically, then settle. Turning still uses the
        // regular arc planner because it has a different support geometry.
        let staticCrawl = gait.isSingleFoot && moving && !spinRequested && abs(drive.turn) <= 0.02
        let staticPlan = staticCrawl ? staticCrawlPlan(at: phaseTime, geometry: geometry,
                                                        forward: drive.forward, lateral: drive.lateral,
                                                        stride: stride, profile: profile) : nil
        let centre = turningCentre(for: (forward: drive.forward, lateral: drive.lateral, turn: drive.turn))
        // Exactly one torso plan per phase. One-foot gaits use the static
        // three-support transfer; Trot, Run and turning Crawl/Climb use VPSP.
        // This is a planned CoM reference, independent of contact estimates
        // and distinct from IMU attitude correction.
        let plannedBodyShift = staticPlan?.bodyShift ??
            (moving && !jumping
                ? dynamicBodyShift(phaseTime: phaseTime, gait: gait, profile: profile,
                                   geometry: geometry, drive: drive, stride: stride,
                                   steppingInPlace: steppingInPlace, turnCentre: centre, spin: spinRequested)
                : QuadVector(x: 0, y: 0, z: 0))
        let plannedPosture = (roll: 0.0, pitch: 0.0)
        var guideRadii: [QuadLeg: Double] = [:]
        let poses: [LegPose] = QuadLeg.allCases.map { leg -> LegPose in
            let offset = gait.isSingleFoot ? leg.crawlPhase : leg.trotPhase
            let phase = (phaseTime + offset).truncatingRemainder(dividingBy: 1)
            var staticCycle = phaseTime - leg.staticCrawlOffset
            staticCycle -= floor(staticCycle)
            let staticStep = staticCrawlStep(at: staticCycle * 4,
                                             preloadEnd: staticCrawlPreload(for: profile))
            let staticAirborne = staticCrawl && staticCycle < 0.25 && staticStep.airborne
            let duty = Double(profile.dutyPercent) / 100.0
            let dynamicPath = gaitFootPath(phase: phase, duty: duty,
                                           stride: steppingInPlace ? 0 : stride,
                                           height: Double(profile.stepHeightMm), active: moving)
            var stance = staticCrawl ? !staticAirborne : dynamicPath.stance
            let stroke: Double, lift: Double
            if staticCrawl {
                stroke = staticCrawlStanceStroke(for: leg, globalPhase: phaseTime,
                                                  preloadEnd: staticCrawlPreload(for: profile)) * stride
                lift = staticAirborne ? Double(profile.stepHeightMm) * staticStep.lift : 0
            } else {
                stroke = dynamicPath.stroke
                lift = dynamicPath.lift
            }

            // This is the three-DOF chain from ElectroPup's kinematics.py:
            // q1 is ab/ad, then q2 hip pitch and q3 knee pitch. q1 rotates a
            // lateral hip link; it is deliberately not a steering/yaw joint.
            let anchor = QuadVector(x: leg.front * (geometry.frameLength / 2 - geometry.cornerInset), y: effectiveBodyHeight, z: leg.left * (geometry.frameWidth / 2 - geometry.cornerInset))
            // Neutral foot is directly below the ab/ad link. That makes q1 = 0
            // in the standing pose instead of splaying every leg outward.
            let footBase = QuadVector(x: anchor.x, y: 0, z: anchor.z + leg.left * (geometry.yawLink + geometry.lateralStance))
            var target: QuadVector
            let footVelocity: QuadVector
            if staticPlan != nil {
                target = footBase + QuadVector(x: stroke * drive.forward, y: lift,
                                                z: stroke * drive.lateral)
                footVelocity = QuadVector(x: 0, y: 0, z: 0)
            } else if moving {
                // The very same compiled C planner as ESP: no separate arc
                // formula, artificial body yaw, or special three-leg animation.
                var request = portableRequest(profile: profile, geometry: geometry, drive: drive,
                    stride: stride, steppingInPlace: steppingInPlace, turnCentre: centre, spin: spinRequested)
                let index = robot_leg_t(rawValue: UInt32(leg.wireIndex))
                let p = robot_body_trajectory_foot(&request, index, Float(phaseTime))
                target = QuadVector(x: Double(p.x), y: Double(p.y), z: Double(p.z))
                stance = gait == .tripod
                    ? robot_locomotion_tripod_phase(index, request.excluded_leg,
                        Float(phaseTime), &request.profile).support_contact
                    : robot_locomotion_leg_phase(UInt8(gait.boardID), index,
                        Float(phaseTime), &request.profile).support_contact
                request.profile.frequency_centi_hz = UInt16(max(1, cadence * 100))
                let twist = robot_body_trajectory_twist(&request)
                footVelocity = stance
                    ? QuadVector(x: -Double(twist.x_mm_s) + Double(twist.yaw_rad_s) * target.z,
                                 y: 0, z: -Double(twist.z_mm_s) - Double(twist.yaw_rad_s) * target.x)
                    : QuadVector(x: 0, y: 0, z: 0)
                if let centre { guideRadii[leg] = hypot(footBase.x-centre.x, footBase.z-centre.z) }
            } else {
                target = footBase
                stance = true
                footVelocity = QuadVector(x: 0, y: 0, z: 0)
            }
            target = bodyPoseFootTarget(target, bodyHeight: effectiveBodyHeight,
                                         shift: plannedBodyShift,
                                         roll: plannedPosture.roll,
                                         pitch: plannedPosture.pitch)

            // The body-to-leg transforms are the +/- 90 degree Y transforms in
            // ElectroPup's t_front_* / t_back_* functions.
            let frameAngle: Double = leg.left > 0 ? .pi / 2 : -.pi / 2
            let dx = target.x - anchor.x, dy = target.y - anchor.y, dz = target.z - anchor.z
            let localX = cos(frameAngle) * dx - sin(frameAngle) * dz
            let localY = dy
            let localZ = sin(frameAngle) * dx + cos(frameAngle) * dz
            let planarSquared = max(0, localX * localX + localY * localY - geometry.yawLink * geometry.yawLink)
            let planar = sqrt(planarSquared)
            let d = max(-1, min(1, (localX * localX + localY * localY + localZ * localZ - geometry.yawLink * geometry.yawLink - geometry.upperLeg * geometry.upperLeg - geometry.lowerLeg * geometry.lowerLeg) / (2 * geometry.upperLeg * geometry.lowerLeg)))
            let kneeRadians = leg.left > 0 ? atan2(sqrt(max(0, 1 - d * d)), d) : atan2(-sqrt(max(0, 1 - d * d)), d)
            let hipRadians = atan2(localZ, planar) - atan2(geometry.lowerLeg * sin(kneeRadians), geometry.upperLeg + geometry.lowerLeg * cos(kneeRadians))
            let abduction = atan2(localY, localX) + atan2(planar, -geometry.yawLink)

            // Exact forward chain: t01(q1,l1) * t12 * t23(q2,l2) * t34(q3,l3).
            let p1 = QuadVector(x: 0, y: 0, z: 0)
            let p2 = QuadVector(x: -geometry.yawLink * cos(abduction), y: -geometry.yawLink * sin(abduction), z: 0)
            let p3 = p2 + QuadVector(x: geometry.upperLeg * sin(abduction) * cos(hipRadians), y: -geometry.upperLeg * cos(abduction) * cos(hipRadians), z: geometry.upperLeg * sin(hipRadians))
            let sum = hipRadians + kneeRadians
            let p4 = p3 + QuadVector(x: geometry.lowerLeg * sin(abduction) * cos(sum), y: -geometry.lowerLeg * cos(abduction) * cos(sum), z: geometry.lowerLeg * sin(sum))
            func toBody(_ point: QuadVector) -> QuadVector {
                QuadVector(x: anchor.x + cos(frameAngle) * point.x + sin(frameAngle) * point.z,
                           y: anchor.y + point.y,
                           z: anchor.z - sin(frameAngle) * point.x + cos(frameAngle) * point.z)
            }
            return LegPose(leg: leg, yaw: abduction * 180 / .pi, hip: hipRadians * 180 / .pi,
                           knee: kneeRadians * 180 / .pi, points: [toBody(p1), toBody(p2), toBody(p3), toBody(p4)],
                           grounded: !jumping && (moving ? stance : true), footVelocity: footVelocity)
        }
        solveBodyPose(from: poses, jumpLift: currentJumpLift)
        let gravity: GravitySupport
        if jumping {
            gravity = GravitySupport(mode: .falling, contacts: 0, margin: -.infinity,
                                     projectedCOM: QuadVector(x: worldPosition.x, y: 0, z: worldPosition.z))
        } else {
            gravity = applyGravity(from: poses, bodyHeight: effectiveBodyHeight, dt: dt)
            // The attitude changes the world location of every foot.  Solve
            // the contact pose once more so the grounded feet stay tied to
            // their planted locations instead of visually sliding through the
            // floor during a gravity-driven lean.
            solveBodyPose(from: poses, jumpLift: 0)
        }
        lastPoseTime = time
        let turnGuide = centre.flatMap { hypot($0.x, $0.z) < 650 && moving ? TurnGuide(center: $0, radii: guideRadii) : nil }
        return .init(gait: gait, phase: phaseTime - floor(phaseTime), input: drive, legs: poses,
                     position: worldPosition, bodyShift: plannedBodyShift, yaw: worldYaw,
                     roll: worldRoll + plannedPosture.roll,
                     pitch: worldPitch + plannedPosture.pitch, bodyHeight: effectiveBodyHeight,
                     isJumping: jumping, turnGuide: turnGuide, gravity: gravity)
    }

    // This is the one forward-kinematic chain used by the live Aura renderer
    // and by Calibration. Keeping it here prevents the calibration drawing
    // from ever growing a second, visually different leg model.
    private func forwardPose(for leg: QuadLeg, degrees: [Double], height: Double,
                             grounded: Bool, footVelocity: QuadVector = QuadVector(x: 0, y: 0, z: 0)) -> LegPose {
        let geometry = renderGeometry()
        let angles = (0..<3).map { degrees.indices.contains($0) ? degrees[$0] : 0 }
        let abduction = angles[0] * .pi / 180.0
        let hip = angles[1] * .pi / 180.0
        let knee = angles[2] * .pi / 180.0
        let anchor = QuadVector(x: leg.front * (geometry.frameLength / 2 - geometry.cornerInset),
                                y: height,
                                z: leg.left * (geometry.frameWidth / 2 - geometry.cornerInset))
        let frameAngle: Double = leg.left > 0 ? .pi / 2 : -.pi / 2
        let p1 = QuadVector(x: 0, y: 0, z: 0)
        let p2 = QuadVector(x: -geometry.yawLink * cos(abduction), y: -geometry.yawLink * sin(abduction), z: 0)
        let p3 = p2 + QuadVector(x: geometry.upperLeg * sin(abduction) * cos(hip),
                                 y: -geometry.upperLeg * cos(abduction) * cos(hip),
                                 z: geometry.upperLeg * sin(hip))
        let total = hip + knee
        let p4 = p3 + QuadVector(x: geometry.lowerLeg * sin(abduction) * cos(total),
                                 y: -geometry.lowerLeg * cos(abduction) * cos(total),
                                 z: geometry.lowerLeg * sin(total))
        func toBody(_ point: QuadVector) -> QuadVector {
            QuadVector(x: anchor.x + cos(frameAngle) * point.x + sin(frameAngle) * point.z,
                       y: anchor.y + point.y,
                       z: anchor.z - sin(frameAngle) * point.x + cos(frameAngle) * point.z)
        }
        return LegPose(leg: leg, yaw: angles[0], hip: angles[1], knee: angles[2],
                       points: [toBody(p1), toBody(p2), toBody(p3), toBody(p4)],
                       grounded: grounded, footVelocity: footVelocity)
    }

    func poseFromAura(_ telemetry: [AuraRobotLegTelemetry], state: AuraRobotState,
                      controller: AuraDualSense, forceAllFeedback: Bool = false) -> QuadrupedSnapshot {
        if !forceAllFeedback, let calibrationLeg = state.calibrationLeg,
           let selected = RobotLegID(wireIndex: calibrationLeg) {
            return calibrationHybridSnapshot(selected: selected, telemetry: telemetry,
                                              state: state, controller: controller)
        }
        // Aura emits a complete robot sample every 20 ms.  TimelineView may
        // render the same sample several times at 60 Hz; integrating world
        // pose repeatedly from it was the source of the apparent looping and
        // falling-through-floor behaviour.
        if lastAuraTick == state.tickCount, let cachedAuraSnapshot { return cachedAuraSnapshot }
        let geometry = renderGeometry()
        let reportedHeight = Double(state.bodyHeight)
        let height = reportedHeight.isFinite && (80...350).contains(reportedHeight)
            ? reportedHeight : geometry.bodyHeight
        let selectedGait = QuadGait.auraMode(state.selectedGait)
        let activeGait = QuadGait.auraMode(state.activeGait)
        let activeProfile = locomotionProfile(for: activeGait)
        let input = (forward: state.forward, lateral: state.lateral,
                     turn: state.turn, height: (height - geometry.bodyHeight) / 35)
        let magnitude = min(1, sqrt(input.forward * input.forward +
                                     input.lateral * input.lateral + input.turn * input.turn))
        let steppingInPlace = (selectedGait == .trot || selectedGait == .tripod) && magnitude < 0.02
        let moving = !state.jumping && !state.spinMode && activeGait != .stand &&
                     (steppingInPlace || magnitude > 0.02)
        let staticCrawl = activeGait.isSingleFoot && moving && abs(input.turn) <= 0.02
        let centre = turningCentre(for: (input.forward, input.lateral, input.turn))
        let phaseStride = Double(activeProfile.strideMm) * (steppingInPlace ? 1 : max(0.25, magnitude))
        let liveStaticPlan = staticCrawl
            ? staticCrawlPlan(at: state.phase, geometry: geometry, forward: input.forward,
                              lateral: input.lateral, stride: phaseStride, profile: activeProfile)
            : nil
        // The ESP32 owns physical targets. This copy is presentation-only and
        // gives the fixed-frame viewer the same planned torso path as Aura.
        let hasBodyReference = state.bodyReferenceValid &&
            (state.tickCount &- state.bodyReferenceTick <= 2 || state.bodyReferenceTick &- state.tickCount <= 2)
        let plannedBodyShift = hasBodyReference
            ? QuadVector(x: state.bodyShiftX, y: 0, z: state.bodyShiftZ)
            : (liveStaticPlan?.bodyShift ??
                (moving ? dynamicBodyShift(phaseTime: state.phase, gait: activeGait,
                                           profile: activeProfile, geometry: geometry, drive: input,
                                           stride: phaseStride, steppingInPlace: steppingInPlace,
                                           turnCentre: centre, excludedLeg: state.tripodExcludedLeg)
                        : QuadVector(x: 0, y: 0, z: 0)))
        let plannedPosture = (roll: hasBodyReference ? state.bodyReferenceRoll : 0,
                              pitch: hasBodyReference ? state.bodyReferencePitch : 0)
        var guideRadii: [QuadLeg: Double] = [:]
        let legs = QuadLeg.allCases.map { leg -> LegPose in
            let index = leg.wireIndex
            let feedback = telemetry.indices.contains(index) ? telemetry[index] : AuraRobotLegTelemetry(id: index)
            // Use the physical encoder position whenever it is available. The
            // rendered robot then follows the real servos rather than the
            // command they are still travelling towards.
            let angles = (0..<3).map {
                let measured = feedback.measuredDegrees[$0]
                let target = feedback.targetDegrees[$0]
                if feedback.present[$0], measured.isFinite, abs(measured) <= 720 { return measured }
                if target.isFinite, abs(target) <= 720 { return target }
                return 0
            }
            let phaseOffset = activeGait.isSingleFoot ? leg.crawlPhase : leg.trotPhase
            let legPhase = (state.phase + phaseOffset).truncatingRemainder(dividingBy: 1)
            let crawlCycle = (state.phase - leg.staticCrawlOffset).truncatingRemainder(dividingBy: 1)
            let staticAirborne = staticCrawl && crawlCycle < 0.25 &&
                                staticCrawlStep(at: crawlCycle * 4,
                                                preloadEnd: staticCrawlPreload(for: activeProfile)).airborne
            var scheduledContact = staticCrawl ? !staticAirborne
                : legPhase < Double(activeProfile.dutyPercent) / 100.0
            if activeGait == .tripod {
                var profile = robot_locomotion_profile_t(
                    stride_mm: UInt16(activeProfile.strideMm), step_height_mm: UInt16(activeProfile.stepHeightMm),
                    frequency_centi_hz: UInt16(activeProfile.frequencyCentiHz), duty_percent: UInt8(activeProfile.dutyPercent))
                scheduledContact = robot_locomotion_tripod_phase(
                    robot_leg_t(rawValue: UInt32(index)),
                    robot_leg_t(rawValue: UInt32(state.tripodExcludedLeg)), Float(state.phase), &profile).support_contact
            }
            let grounded = !state.jumping && (moving ? scheduledContact : true)
            let pose = forwardPose(for: leg, degrees: angles, height: height, grounded: grounded)
            if let centre, moving && abs(input.turn) > 0.02 {
                guideRadii[leg] = hypot(pose.points[3].x - centre.x, pose.points[3].z - centre.z)
            }
            return pose
        }
        // Aura is now the authority for body odometry. It binds stance feet
        // and solves this planar transform at the control rate from real
        // encoders, so the desktop only renders that board-owned state.
        let odometryIsSane = state.odometryX.isFinite && state.odometryZ.isFinite &&
            state.odometryYaw.isFinite && abs(state.odometryX) <= 5000 &&
            abs(state.odometryZ) <= 5000 && abs(state.odometryYaw) <= 32 * .pi
        if state.odometryValid && odometryIsSane {
            worldPosition.x = state.odometryX
            worldPosition.z = state.odometryZ
            worldYaw = state.odometryYaw
            // Odometry owns horizontal placement.  Refresh the displayed
            // support contacts from the live leg geometry without changing
            // the board-owned X/Z pose.
            let contacts = legs.filter(\.grounded)
            let contactLegs = Set(contacts.map(\.leg))
            plantedFeet = plantedFeet.filter { contactLegs.contains($0.key) }
            for pose in contacts {
                let foot = worldPoint(pose.points[3])
                plantedFeet[pose.leg] = QuadVector(x: foot.x, y: 0, z: foot.z)
            }
            worldPosition.y = max(0, -(contacts.map { rotatedBodyPoint($0.points[3]).y }.min() ?? 0))
        } else {
            if !worldPosition.isFinite || !worldYaw.isFinite {
                worldPosition = QuadVector(x: 0, y: 0, z: 0)
                worldYaw = 0; resetGravityState()
            }
            solveBodyPose(from: legs, jumpLift: 0)
        }
        let tickDelta = lastAuraTick.map { state.tickCount &- $0 } ?? 1
        let gravityDt = min(0.05, max(0.001, Double(tickDelta) * 0.02))
        let gravity: GravitySupport
        if state.jumping {
            gravity = GravitySupport(mode: .falling, contacts: 0, margin: -.infinity,
                                     projectedCOM: QuadVector(x: worldPosition.x, y: 0, z: worldPosition.z))
        } else {
            gravity = applyGravity(from: legs, bodyHeight: height, dt: gravityDt)
            if odometryIsSane {
                let contacts = legs.filter(\.grounded)
                worldPosition.y = max(0, -(contacts.map { rotatedBodyPoint($0.points[3]).y }.min() ?? 0))
            } else {
                solveBodyPose(from: legs, jumpLift: 0)
            }
        }
        let turnGuide = centre.flatMap {
            moving && abs(input.turn) > 0.02 ? TurnGuide(center: $0, radii: guideRadii) : nil
        }
        let snapshot = QuadrupedSnapshot(gait: activeGait, phase: state.phase, input: input, legs: legs,
                                         position: worldPosition, bodyShift: plannedBodyShift,
                                         yaw: worldYaw,
                                         roll: worldRoll + plannedPosture.roll,
                                         pitch: worldPitch + plannedPosture.pitch, bodyHeight: height,
                                         isJumping: state.jumping, turnGuide: turnGuide,
                                         gravity: gravity)
        lastAuraTick = state.tickCount
        cachedAuraSnapshot = snapshot
        return snapshot
    }

    private func standingPreview(at time: TimeInterval) -> QuadrupedSnapshot {
        // The calibration screen is an isolated commissioning scene, so it
        // always starts from the same normal standing state.
        plantedFeet.removeAll()
        worldPosition = QuadVector(x: 0, y: 0, z: 0)
        worldYaw = 0
        resetGravityState()
        lastPoseTime = nil
        return pose(at: time, pad: AuraDualSense())
    }

    private func replacing(_ selectedLeg: QuadLeg, in standing: QuadrupedSnapshot,
                           with angles: [Double]) -> QuadrupedSnapshot {
        var legs = standing.legs
        if let index = legs.firstIndex(where: { $0.leg == selectedLeg }) {
            legs[index] = forwardPose(for: selectedLeg, degrees: angles,
                                      height: standing.bodyHeight, grounded: true)
        }
        // The calibration reference has a folded +/-90 degree knee, which is
        // intentionally different from the ordinary standing pose. Lift the
        // entire presentation by the actual lowest foot so that this view
        // never draws a calibrated leg through the floor. This is visual
        // grounding only; it does not modify IK, a target, or a servo record.
        let lowestFoot = legs.map { $0.points[3].y }.min() ?? 0
        var position = standing.position
        position.y = max(position.y, -lowestFoot)
        return QuadrupedSnapshot(gait: standing.gait, phase: standing.phase, input: standing.input, legs: legs,
                                 position: position, bodyShift: standing.bodyShift,
                                 yaw: standing.yaw, roll: standing.roll,
                                 pitch: standing.pitch,
                                 bodyHeight: standing.bodyHeight, isJumping: false,
                                 turnGuide: nil, gravity: standing.gravity)
    }

    // Calibration is a presentation of the same planner as Quadruped:
    // stationary shows its normal stance, while Animate sends the model the
    // exact virtual pad input used for a forward walk. Only the static
    // selected leg is replaced with its physical 0° / 0° / ±90° reference.
    func calibrationSnapshot(selected leg: RobotLegID, referenceAngles: [Double],
                             at time: TimeInterval, mode: CalibrationPreviewMode) -> QuadrupedSnapshot {
        let selectedLeg = QuadLeg(robotLeg: leg)
        switch mode {
        case .still:
            return replacing(selectedLeg, in: standingPreview(at: time), with: referenceAngles)
        case .trot:
            var forwardPad = AuraDualSense()
            // The planner maps a low left-Y value to forward, exactly as a
            // physical DualSense does.
            forwardPad.leftY = 0
            return pose(at: time, pad: forwardPad)
        }
    }

    // During a one-leg pad test the selected physical leg comes straight
    // from Aura's refreshed TTL feedback. The other three use the same phase
    // and left-stick command only as virtual legs, so the test can prove the
    // whole gait before the robot is assembled without commanding them.
    private func calibrationHybridSnapshot(selected leg: RobotLegID,
                                           telemetry: [AuraRobotLegTelemetry],
                                           state: AuraRobotState,
                                           controller: AuraDualSense) -> QuadrupedSnapshot {
        let selectedLeg = QuadLeg(robotLeg: leg)
        plantedFeet.removeAll()
        worldPosition = QuadVector(x: 0, y: 0, z: 0)
        worldYaw = 0
        resetGravityState()
        lastPoseTime = nil
        let phaseTime = state.phase / max(0.01, locomotionProfile(for: .trot).frequencyHz)
        var calibrationPad = controller
        // The physical commissioning path receives forward/back and lateral
        // only. Excluding body turn and height keeps screen and hardware on
        // exactly the same command set.
        calibrationPad.rightX = 128
        calibrationPad.rightY = 128
        previewPhase = state.phase
        filteredDrive = input(calibrationPad)
        let virtualRobot = pose(at: phaseTime, pad: calibrationPad)
        let feedback = telemetry.indices.contains(selectedLeg.wireIndex)
            ? telemetry[selectedLeg.wireIndex]
            : AuraRobotLegTelemetry(id: selectedLeg.wireIndex)
        // Feedback becomes available one addressed read at a time. Use every
        // fresh axis immediately and retain the model value only for an axis
        // that has not replied yet, instead of waiting for all three.
        let angles = (0..<3).map {
            feedback.present[$0] ? feedback.measuredDegrees[$0] : feedback.targetDegrees[$0]
        }
        return replacing(selectedLeg, in: virtualRobot, with: angles)
    }

    func calibrationLiveSnapshot(selected leg: RobotLegID, frame: AuraRobotFrame,
                                 at time: TimeInterval) -> QuadrupedSnapshot {
        calibrationHybridSnapshot(selected: leg, telemetry: frame.legs,
                                  state: frame.state, controller: frame.controller)
    }

    func calibrationAllLiveSnapshot(frame: AuraRobotFrame) -> QuadrupedSnapshot {
        // The calibration viewer uses the very same full-robot forward model
        // as the Quadruped tab. `forceAllFeedback` prevents a stale one-leg
        // test flag from replacing the other three physical legs virtually.
        poseFromAura(frame.legs, state: frame.state, controller: frame.controller,
                     forceAllFeedback: true)
    }

    func calibrationDirectionGuide(leg: RobotLegID, axis: Int,
                                   referenceAngles: [Double], targetAngles: [Double])
        -> CalibrationDirectionGuide? {
        guard (0..<3).contains(axis) else { return nil }
        let selected = QuadLeg(robotLeg: leg)
        let geometry = renderGeometry()
        return CalibrationDirectionGuide(
            leg: selected,
            axis: axis,
            reference: forwardPose(for: selected, degrees: referenceAngles,
                                   height: geometry.bodyHeight, grounded: true),
            target: forwardPose(for: selected, degrees: targetAngles,
                                height: geometry.bodyHeight, grounded: true)
        )
    }

}

private extension QuadLeg {
    init(robotLeg: RobotLegID) {
        switch robotLeg {
        case .lf: self = .frontLeft
        case .lr: self = .rearLeft
        case .rf: self = .frontRight
        case .rr: self = .rearRight
        }
    }
}

// This store has one job: refresh textual controls at a human-visible rate.
// It deliberately does not participate in the 50 Hz rendering path below.
private final class QuadrupedHUDStore: ObservableObject {
    @Published private(set) var frame = AuraRobotFrame(
        state: AuraRobotState(),
        legs: (0..<4).map { AuraRobotLegTelemetry(id: $0) },
        controller: AuraDualSense(),
        contacts: AuraFootContacts()
    )
    private var subscription: AnyCancellable?
    private var lastPublish = 0.0

    func bind(_ channel: AuraRobotRenderChannel) {
        guard subscription == nil else { return }
        subscription = channel.frames
            .receive(on: RunLoop.main)
            .sink { [weak self] frame in
                guard let self else { return }
                let now = ProcessInfo.processInfo.systemUptime
                guard now - self.lastPublish >= 0.10 else { return }
                self.lastPublish = now
                self.frame = frame
            }
    }
}

private final class QuadrupedScene: NSObject {
    let scene = SCNScene()
    let robot = SCNNode()
    // Fixed point for object-centred inspection. The robot itself is allowed
    // to follow the planned CoM trajectory around it.
    private let presentationOrigin = SCNNode()
    let cameraRig = SCNNode()
    let camera = SCNNode()
    private var bodyFrame: [SCNNode] = []
    private var rods: [QuadLeg: [SCNNode]] = [:], joints: [QuadLeg: [SCNNode]] = [:], feet: [QuadLeg: SCNNode] = [:]
    private var turnRings: [QuadLeg: [SCNNode]] = [:]
    // Three rods form the movable shaft and head of the green calibration
    // arrow. They are created lazily because the normal Quadruped scene never
    // needs them.
    private var directionArrowRods: [SCNNode] = []
    private let directionArrowPivot = SCNNode(geometry: SCNSphere(radius: 4.8))
    private var turnGuideSignature: String?
    private var calibrationHighlight: QuadLeg?
    private var calibrationHighlightAxis: Int?
    private var appliedCameraResetToken = 0
    private var frameSubscription: AnyCancellable?
    private var lastSnapshot: QuadrupedSnapshot?
    // Deliberately opposite to the initial normal scene mode so init installs
    // the first LookAt constraint as well.
    private var cameraTracksPresentationOrigin = true
    // The full Quadruped tab is a fixed-frame gait viewer.  It shows the leg
    // cycle rather than flying the camera across an imagined room.
    private var lockPresentationToOrigin = false
    private var showWorld = true
    private var showFootTrails = false
    private var floorNode: SCNNode?
    private var gridNodes: [SCNNode] = []
    private var footTrails: [QuadLeg: [QuadVector]] = [:]
    private var completedFootTrails: [QuadLeg: [QuadVector]] = [:]
    private var footTrailNodes: [QuadLeg: SCNNode] = [:]
    private var footTrailSignature = ""
    private var footTrailSampleCounter = 0
    private var footTrailPreviousPhase: Double?
    private var footTrailPreviousSamplePhase: Double?
    private let turnCentre = SCNNode(geometry: SCNSphere(radius: 5))
    private let centreOfMassMarker = SCNNode(geometry: SCNSphere(radius: 5.4))
    private let centreOfMassProjection = SCNNode(geometry: SCNCylinder(radius: 5.5, height: 1.4))
    private let frameColor = NSColor(calibratedWhite: 0.86, alpha: 1)
    private let leftColor = NSColor(red: 0.05, green: 0.75, blue: 0.62, alpha: 1)
    private let rightColor = NSColor(red: 1, green: 0.46, blue: 0.17, alpha: 1)

    override init() {
        super.init(); scene.background.contents = NSColor(calibratedWhite: 0.075, alpha: 1)
        scene.rootNode.addChildNode(robot)
        scene.rootNode.addChildNode(presentationOrigin)
        scene.rootNode.addChildNode(cameraRig)
        camera.camera = SCNCamera(); camera.camera?.zFar = 3000
        camera.position = SCNVector3(400, 285, 440)
        setCameraTarget(lockPresentationToOrigin: false)
        cameraRig.addChildNode(camera)
        configureWorld(); createBodyFrame(); QuadLeg.allCases.forEach(createLeg)
        centreOfMassMarker.geometry?.firstMaterial = material(.systemOrange, metal: 0.2)
        centreOfMassProjection.geometry?.firstMaterial = material(.systemOrange, metal: 0.1)
        robot.addChildNode(centreOfMassMarker)
        scene.rootNode.addChildNode(centreOfMassProjection)
    }
    private func material(_ color: NSColor, metal: CGFloat = 0) -> SCNMaterial {
        let result = SCNMaterial(); result.diffuse.contents = color; result.metalness.contents = metal; result.roughness.contents = 0.32; return result
    }
    private func configureWorld() {
        let floor = SCNFloor(); floor.reflectivity = 0.03; floor.firstMaterial = material(NSColor(calibratedWhite: 0.16, alpha: 1), metal: 0.15)
        let floorNode = SCNNode(geometry: floor)
        self.floorNode = floorNode
        scene.rootNode.addChildNode(floorNode)
        // A sparse grid gives scale without spending a render node on every
        // 60 mm division. The robot itself is the moving information here.
        for distance in stride(from: -480, through: 480, by: 120) {
            let lineA = SCNNode(geometry: SCNBox(width: 0.7, height: 0.3, length: 1200, chamferRadius: 0)); lineA.geometry?.firstMaterial = material(NSColor(calibratedWhite: 0.28, alpha: 0.35)); lineA.position = SCNVector3(Float(distance), 0.2, 0); scene.rootNode.addChildNode(lineA); gridNodes.append(lineA)
            let lineB = SCNNode(geometry: SCNBox(width: 1200, height: 0.3, length: 0.7, chamferRadius: 0)); lineB.geometry?.firstMaterial = material(NSColor(calibratedWhite: 0.28, alpha: 0.35)); lineB.position = SCNVector3(0, 0.2, Float(distance)); scene.rootNode.addChildNode(lineB); gridNodes.append(lineB)
        }
        let ambient = SCNNode(); ambient.light = SCNLight(); ambient.light?.type = .ambient; ambient.light?.intensity = 600; scene.rootNode.addChildNode(ambient)
        let key = SCNNode(); key.light = SCNLight(); key.light?.type = .omni; key.light?.intensity = 1350; key.position = SCNVector3(260, 420, 320); scene.rootNode.addChildNode(key)
    }
    private func setWorldVisible(_ visible: Bool) {
        floorNode?.isHidden = !visible
        gridNodes.forEach { $0.isHidden = !visible }
    }
    private func createLeg(_ leg: QuadLeg) {
        let color = leg.left > 0 ? leftColor : rightColor
        let legRods = (0..<3).map { _ -> SCNNode in let node = SCNNode(geometry: SCNCylinder(radius: 2.2, height: 1)); node.geometry?.firstMaterial = material(color, metal: 0.35); robot.addChildNode(node); return node }
        let legJoints = (0..<3).map { _ -> SCNNode in let node = SCNNode(geometry: SCNSphere(radius: 4)); node.geometry?.firstMaterial = material(frameColor, metal: 0.25); robot.addChildNode(node); return node }
        let foot = SCNNode(geometry: SCNSphere(radius: 4.5)); foot.geometry?.firstMaterial = material(color, metal: 0.25); robot.addChildNode(foot)
        rods[leg] = legRods; joints[leg] = legJoints; feet[leg] = foot
    }
    private func createBodyFrame() {
        bodyFrame = (0..<4).map { _ -> SCNNode in
            let node = SCNNode(geometry: SCNCylinder(radius: 1.8, height: 1))
            node.geometry?.firstMaterial = material(frameColor, metal: 0.45)
            robot.addChildNode(node)
            return node
        }
    }
    private func createTurnGuides() {
        let colours: [QuadLeg: NSColor] = [
            .frontLeft: leftColor, .rearLeft: leftColor,
            .frontRight: rightColor, .rearRight: rightColor,
        ]
        for leg in QuadLeg.allCases {
            let ring = (0..<12).map { _ -> SCNNode in
                let node = SCNNode(geometry: SCNCylinder(radius: 0.8, height: 1))
                node.geometry?.firstMaterial = material((colours[leg] ?? frameColor).withAlphaComponent(0.42), metal: 0.15)
                node.isHidden = true
                robot.addChildNode(node)
                return node
            }
            turnRings[leg] = ring
        }
        turnCentre.geometry?.firstMaterial = material(NSColor.systemYellow.withAlphaComponent(0.75), metal: 0.25)
        turnCentre.isHidden = true
        robot.addChildNode(turnCentre)
    }
    private func createFootTrails() {
        guard footTrailNodes.isEmpty else { return }
        for leg in QuadLeg.allCases {
            let node = SCNNode()
            let colour = (leg.left > 0 ? leftColor : rightColor).withAlphaComponent(0.72)
            node.geometry?.firstMaterial = material(colour, metal: 0.05)
            node.isHidden = true
            robot.addChildNode(node)
            footTrailNodes[leg] = node
        }
    }
    private func hideFootTrails() {
        footTrailNodes.values.forEach { $0.isHidden = true }
        footTrailSignature = ""
        footTrails.removeAll()
        completedFootTrails.removeAll()
        footTrailSampleCounter = 0
        footTrailPreviousPhase = nil
        footTrailPreviousSamplePhase = nil
    }
    private func smoothedFootTrail(_ samples: [QuadVector], closed: Bool) -> [QuadVector] {
        guard samples.count >= 3 else { return samples }
        let count = samples.count
        let segments = closed ? count : count - 1
        var result: [QuadVector] = []
        result.reserveCapacity(segments * 5 + 1)
        func point(_ index: Int) -> QuadVector {
            if closed { return samples[(index % count + count) % count] }
            return samples[min(count - 1, max(0, index))]
        }
        for index in 0..<segments {
            let p0 = point(index - 1), p1 = point(index)
            let p2 = point(index + 1), p3 = point(index + 2)
            for step in 0..<5 {
                let t = Double(step) / 5
                let t2 = t * t, t3 = t2 * t
                let a = p1 * 2
                let b = p2 - p0
                let c = p0 * 2 - p1 * 5 + p2 * 4 - p3
                let d = p1 * 3 - p2 * 3 + p3 - p0
                result.append((a + b * t + c * t2 + d * t3) * 0.5)
            }
        }
        if closed {
            result.append(result.first ?? samples[0])
        } else {
            result.append(samples[count - 1])
        }
        return result
    }
    private func setFootTrailGeometry(_ samples: [QuadVector], closed: Bool, for leg: QuadLeg) {
        guard let node = footTrailNodes[leg], samples.count >= 2 else {
            footTrailNodes[leg]?.isHidden = true
            return
        }
        let smooth = smoothedFootTrail(samples, closed: closed)
        var indices: [Int32] = []
        for index in 0..<(smooth.count - 1) {
            indices.append(Int32(index)); indices.append(Int32(index + 1))
        }
        let vertices = smooth.map(\.scn)
        let source = SCNGeometrySource(vertices: vertices)
        let element = SCNGeometryElement(indices: indices, primitiveType: .line)
        let geometry = SCNGeometry(sources: [source], elements: [element])
        geometry.firstMaterial = material((leg.left > 0 ? leftColor : rightColor).withAlphaComponent(0.72), metal: 0.05)
        node.geometry = geometry
        node.isHidden = false
    }
    private func updateFootTrails(_ snapshot: QuadrupedSnapshot, bodyShift: QuadVector,
                                  enabled: Bool) {
        guard enabled else { hideFootTrails(); return }
        createFootTrails()
        let signature = "\(snapshot.gait.rawValue):\(String(format: "%.1f", snapshot.input.forward)):\(String(format: "%.1f", snapshot.input.lateral)):\(String(format: "%.1f", snapshot.input.turn))"
        if signature != footTrailSignature {
            footTrailSignature = signature
            footTrails.removeAll()
            completedFootTrails.removeAll()
            footTrailSampleCounter = 0
            footTrailPreviousPhase = nil
            footTrailPreviousSamplePhase = nil
        }
        if let previous = footTrailPreviousPhase,
           snapshot.phase + 0.35 < previous {
            // Preserve exactly one completed period. The previous version
            // accumulated several laps, then closed that polyline into a
            // jagged scribble.
            completedFootTrails = footTrails
            footTrails.removeAll()
        }
        footTrailPreviousPhase = snapshot.phase
        footTrailSampleCounter += 1
        // Keep the render work below the 50 Hz control stream while retaining
        // enough points for the C² interpolation above.
        guard footTrailSampleCounter % 2 == 0,
              footTrailPreviousSamplePhase.map({ abs($0 - snapshot.phase) > 0.0001 }) ?? true else { return }
        footTrailPreviousSamplePhase = snapshot.phase
        for pose in snapshot.legs {
            var samples = footTrails[pose.leg] ?? []
            // Trail samples are ground-frame foot positions.  A planned
            // torso shift must move the body above a planted foot, not drag
            // that foot through the visual world.
            let worldFoot = robot.convertPosition(pose.points[3].scn, to: nil)
            let point = QuadVector(x: Double(worldFoot.x), y: Double(worldFoot.y), z: Double(worldFoot.z))
            if let last = samples.last, hypot(last.x - point.x, last.z - point.z) < 0.6,
               abs(last.y - point.y) < 0.6 {
                continue
            }
            samples.append(point)
            if samples.count > 64 { samples.removeFirst(samples.count - 64) }
            footTrails[pose.leg] = samples
            let display = completedFootTrails[pose.leg] ?? samples
            setFootTrailGeometry(display, closed: completedFootTrails[pose.leg] != nil,
                                 for: pose.leg)
        }
    }
    private func createDirectionArrow() {
        guard directionArrowRods.isEmpty else { return }
        let arrowMaterial = material(NSColor.systemGreen, metal: 0.15)
        directionArrowRods = (0..<3).map { _ in
            let node = SCNNode(geometry: SCNCylinder(radius: 1.45, height: 1))
            node.geometry?.firstMaterial = arrowMaterial
            node.isHidden = true
            robot.addChildNode(node)
            return node
        }
        directionArrowPivot.geometry?.firstMaterial = arrowMaterial
        directionArrowPivot.isHidden = true
        robot.addChildNode(directionArrowPivot)
    }
    private func setRod(_ node: SCNNode, from start: QuadVector, to end: QuadVector, radius: CGFloat) {
        guard start.isFinite, end.isFinite else { node.isHidden = true; return }
        let a = SIMD3<Float>(Float(start.x), Float(start.y), Float(start.z)), b = SIMD3<Float>(Float(end.x), Float(end.y), Float(end.z)), vector = b - a
        let length = simd_length(vector); guard length.isFinite, length > 0.01 else { node.isHidden = true; return }
        node.isHidden = false
        if let geometry = node.geometry as? SCNCylinder {
            // Leg and frame rods have fixed lengths. Rebuilding their mesh on
            // every 50 Hz sample was the largest SceneKit CPU cost. Geometry
            // is changed only after the user edits a physical dimension.
            if abs(geometry.radius - radius) > 0.01 { geometry.radius = radius }
            if abs(geometry.height - CGFloat(length)) > 0.01 { geometry.height = CGFloat(length) }
        } else {
            let geometry = SCNCylinder(radius: radius, height: CGFloat(length)); geometry.firstMaterial = node.geometry?.firstMaterial
            node.geometry = geometry
        }
        node.position = SCNVector3((a + b) / 2)
        node.simdOrientation = simd_quatf(from: SIMD3<Float>(0, 1, 0), to: simd_normalize(vector))
    }
    private func updateTurnGuides(_ guide: TurnGuide?) {
        guard let guide else {
            if turnGuideSignature != nil {
                turnRings.values.flatMap { $0 }.forEach { $0.isHidden = true }
                turnCentre.isHidden = true
                turnGuideSignature = nil
            }
            return
        }
        if turnRings.isEmpty { createTurnGuides() }
        let signature = ([guide.center.x, guide.center.z] + QuadLeg.allCases.map { guide.radii[$0] ?? -1 })
            .map { String(format: "%.0f", $0) }.joined(separator: ":")
        // Rings belong to the robot node, so they follow its world pose.  Rebuild
        // their geometry only if the circle itself changed, not every render.
        guard signature != turnGuideSignature else { return }
        turnGuideSignature = signature
        turnCentre.isHidden = false
        turnCentre.position = QuadVector(x: guide.center.x, y: 1.2, z: guide.center.z).scn
        for leg in QuadLeg.allCases {
            guard let radius = guide.radii[leg], let ring = turnRings[leg] else { continue }
            for index in ring.indices {
                let first = Double(index) / Double(ring.count) * 2 * Double.pi
                let second = Double(index + 1) / Double(ring.count) * 2 * Double.pi
                let start = QuadVector(x: guide.center.x + radius * cos(first), y: 0.9, z: guide.center.z + radius * sin(first))
                let end = QuadVector(x: guide.center.x + radius * cos(second), y: 0.9, z: guide.center.z + radius * sin(second))
                ring[index].isHidden = false
                setRod(ring[index], from: start, to: end, radius: 0.8)
            }
        }
    }
    private func updateDirectionArrow(_ guide: CalibrationDirectionGuide?) {
        guard let guide else {
            directionArrowRods.forEach { $0.isHidden = true }
            directionArrowPivot.isHidden = true
            return
        }
        createDirectionArrow()
        let joint = guide.axis
        let pivot = guide.reference.points[joint]
        let start = guide.reference.points[joint + 1]
        let end = guide.target.points[joint + 1]
        let movement = SIMD3<Float>(Float(end.x - start.x), Float(end.y - start.y), Float(end.z - start.z))
        guard simd_length(movement) > 0.5 else {
            directionArrowRods.forEach { $0.isHidden = true }
            directionArrowPivot.isHidden = true
            return
        }
        let direction = simd_normalize(movement)
        let radial = SIMD3<Float>(Float(start.x - pivot.x), Float(start.y - pivot.y), Float(start.z - pivot.z))
        var normal = simd_cross(direction, simd_normalize(radial))
        if simd_length(normal) < 0.01 { normal = simd_cross(direction, SIMD3<Float>(0, 1, 0)) }
        normal = simd_normalize(normal)
        let tip = SIMD3<Float>(Float(end.x), Float(end.y), Float(end.z))
        let base = tip - direction * 13
        let wingA = base + normal * 6
        let wingB = base - normal * 6
        let asQuad: (SIMD3<Float>) -> QuadVector = { .init(x: Double($0.x), y: Double($0.y), z: Double($0.z)) }
        setRod(directionArrowRods[0], from: start, to: end, radius: 1.45)
        setRod(directionArrowRods[1], from: asQuad(tip), to: asQuad(wingA), radius: 1.45)
        setRod(directionArrowRods[2], from: asQuad(tip), to: asQuad(wingB), radius: 1.45)
        directionArrowRods.forEach { $0.isHidden = false }
        directionArrowPivot.position = pivot.scn
        directionArrowPivot.isHidden = false
    }
    func bind(_ channel: AuraRobotRenderChannel, model: QuadrupedModel,
              lockPresentationToOrigin: Bool = false,
              showWorld: Bool = true, showFootTrails: Bool = false) {
        self.lockPresentationToOrigin = lockPresentationToOrigin
        self.showWorld = showWorld
        self.showFootTrails = showFootTrails
        // A disconnected board emits no render frames. Apply presentation
        // settings here as well as in `update`, otherwise the floor from the
        // shared SceneKit scene remains visible until the first telemetry
        // packet arrives.
        setWorldVisible(showWorld)
        if !showFootTrails { hideFootTrails() }
        guard frameSubscription == nil else { return }
        frameSubscription = channel.frames
            .receive(on: RunLoop.main)
            .sink { [weak self, weak model] frame in
                guard let self, let model else { return }
                let snapshot = model.poseFromAura(frame.legs, state: frame.state, controller: frame.controller)
                self.lastSnapshot = snapshot
                self.update(model, snapshot: snapshot,
                            lockPresentationToOrigin: self.lockPresentationToOrigin,
                            showWorld: self.showWorld, showFootTrails: self.showFootTrails)
            }
    }
    func refreshGeometry(using model: QuadrupedModel) {
        if appliedCameraResetToken != model.cameraResetToken {
            appliedCameraResetToken = model.cameraResetToken
            cameraRig.position = SCNVector3(0, 0, 0)
            cameraRig.eulerAngles = SCNVector3(0, 0, 0)
            cameraRig.scale = SCNVector3(1, 1, 1)
            camera.position = SCNVector3(400, 285, 440)
            camera.eulerAngles = SCNVector3(0, 0, 0)
        }
        if let lastSnapshot {
            update(model, snapshot: lastSnapshot,
                   lockPresentationToOrigin: lockPresentationToOrigin,
                   showWorld: showWorld, showFootTrails: showFootTrails)
        }
    }
    private func setCameraTarget(lockPresentationToOrigin: Bool) {
        guard cameraTracksPresentationOrigin != lockPresentationToOrigin else { return }
        cameraTracksPresentationOrigin = lockPresentationToOrigin
        let target = SCNLookAtConstraint(target: lockPresentationToOrigin ? presentationOrigin : robot)
        target.isGimbalLockEnabled = true
        camera.constraints = [target]
    }

    private func applyCalibrationAppearance(_ highlightedLeg: QuadLeg?, axis highlightedAxis: Int?) {
        guard calibrationHighlight != highlightedLeg || calibrationHighlightAxis != highlightedAxis else { return }
        calibrationHighlight = highlightedLeg
        calibrationHighlightAxis = highlightedAxis
        for leg in QuadLeg.allCases {
            let isSelected = highlightedLeg == nil || highlightedLeg == leg
            let base = leg.left > 0 ? leftColor : rightColor
            let rodColor: NSColor = highlightedLeg == nil ? base : (isSelected ? NSColor.systemCyan : NSColor(calibratedWhite: 0.34, alpha: 1))
            let jointColor: NSColor = isSelected ? frameColor : NSColor(calibratedWhite: 0.26, alpha: 1)
            rods[leg]?.enumerated().forEach { index, rod in
                let color = isSelected && highlightedAxis == index ? NSColor.systemYellow : rodColor
                rod.geometry?.firstMaterial = material(color, metal: 0.35)
            }
            joints[leg]?.enumerated().forEach { index, joint in
                let color = isSelected && highlightedAxis == index ? NSColor.systemYellow : jointColor
                joint.geometry?.firstMaterial = material(color, metal: 0.25)
            }
            feet[leg]?.geometry?.firstMaterial = material(rodColor, metal: 0.25)
        }
    }
    func update(_ model: QuadrupedModel, snapshot: QuadrupedSnapshot,
                calibrationHighlight highlightedLeg: QuadLeg? = nil,
                calibrationAxis highlightedAxis: Int? = nil,
                calibrationDirection: CalibrationDirectionGuide? = nil,
                lockPresentationToOrigin: Bool = false,
                showWorld: Bool = true,
                showFootTrails: Bool = false) {
        SCNTransaction.begin()
        SCNTransaction.disableActions = true
        setCameraTarget(lockPresentationToOrigin: lockPresentationToOrigin)
        let dimensions = model.visualFrameDimensions()
        let safeHeight = snapshot.bodyHeight.isFinite && (80...350).contains(snapshot.bodyHeight)
            ? snapshot.bodyHeight : model.bodyHeight
        let safePosition = snapshot.position.isFinite && abs(snapshot.position.x) <= 5000 &&
            abs(snapshot.position.y) <= 5000 && abs(snapshot.position.z) <= 5000
            ? snapshot.position : QuadVector(x: 0, y: 0, z: 0)
        let safeYaw = snapshot.yaw.isFinite && abs(snapshot.yaw) <= 32 * .pi ? snapshot.yaw : 0
        let safeRoll = snapshot.roll.isFinite && abs(snapshot.roll) <= 0.8 ? snapshot.roll : 0
        let safePitch = snapshot.pitch.isFinite && abs(snapshot.pitch) <= 0.8 ? snapshot.pitch : 0
        let safeBodyShift = snapshot.bodyShift.isFinite && hypot(snapshot.bodyShift.x, snapshot.bodyShift.z) <= max(250, dimensions.length)
            ? snapshot.bodyShift : QuadVector(x: 0, y: 0, z: 0)
        let halfX = dimensions.length / 2, halfZ = dimensions.width / 2
        let corners = [QuadVector(x: halfX, y: safeHeight, z: halfZ), QuadVector(x: halfX, y: safeHeight, z: -halfZ), QuadVector(x: -halfX, y: safeHeight, z: -halfZ), QuadVector(x: -halfX, y: safeHeight, z: halfZ)]
        for index in 0..<4 { setRod(bodyFrame[index], from: corners[index], to: corners[(index + 1) % 4], radius: 1.8) }
        // `robot` is the actual torso frame. In an inspection view odometry
        // stays centred, but the phase-planned CoM translation remains, so the
        // frame visibly works above stance feet rather than staying rigid.
        let renderPosition = (lockPresentationToOrigin ? QuadVector(x: 0, y: 0, z: 0) : safePosition) + safeBodyShift
        let renderYaw = lockPresentationToOrigin ? 0.0 : safeYaw
        // IK rotates about the torso origin, not the ground at Y=0.
        robot.pivot = SCNMatrix4MakeTranslation(0, CGFloat(safeHeight), 0)
        robot.position = (renderPosition + QuadVector(x: 0, y: safeHeight, z: 0)).scn
        robot.eulerAngles = SCNVector3(Float(safeRoll), Float(renderYaw), Float(safePitch))
        let localCOM = model.centreOfMassLocal(bodyHeight: safeHeight)
        centreOfMassMarker.position = localCOM.scn
        let worldCOM = robot.convertPosition(localCOM.scn, to: nil)
        let projected = QuadVector(x: Double(worldCOM.x), y: 0, z: Double(worldCOM.z))
        centreOfMassProjection.position = SCNVector3(Float(projected.x), 1.1, Float(projected.z))
        let supportColor: NSColor = snapshot.gravity.mode == .stable ? .systemGreen :
            (snapshot.gravity.mode == .edge ? .systemYellow : .systemRed)
        centreOfMassProjection.geometry?.firstMaterial?.diffuse.contents = supportColor
        // An inspection view must stay fixed in the ground frame.  Moving
        // the camera rig with the torso made the planned CoM translation look
        // like a completely rigid body even though the renderer had applied it.
        cameraRig.position = (lockPresentationToOrigin
                              ? QuadVector(x: 0, y: 0, z: 0)
                              : renderPosition).scn
        setWorldVisible(showWorld)
        for pose in snapshot.legs {
            guard let legRods = rods[pose.leg], let legJoints = joints[pose.leg], let foot = feet[pose.leg] else { continue }
            for index in 0..<3 {
                setRod(legRods[index], from: pose.points[index], to: pose.points[index + 1], radius: 2.2)
                legJoints[index].position = pose.points[index].scn
            }
            foot.position = pose.points[3].scn
        }
        updateFootTrails(snapshot, bodyShift: safeBodyShift, enabled: showFootTrails)
        applyCalibrationAppearance(highlightedLeg, axis: highlightedAxis)
        updateTurnGuides(snapshot.turnGuide)
        updateDirectionArrow(calibrationDirection)
        SCNTransaction.commit()
    }
}

private struct QuadrupedSceneView: NSViewRepresentable {
    @ObservedObject var model: QuadrupedModel
    let channel: AuraRobotRenderChannel
    func makeCoordinator() -> QuadrupedScene { QuadrupedScene() }
    func makeNSView(context: Context) -> SCNView {
        context.coordinator.bind(channel, model: model, lockPresentationToOrigin: true,
                                 showWorld: false, showFootTrails: true)
        let view = SCNView(); view.scene = context.coordinator.scene; view.pointOfView = context.coordinator.camera; view.backgroundColor = .black; configureSimulationCamera(view); view.autoenablesDefaultLighting = false; view.antialiasingMode = .none; view.preferredFramesPerSecond = 60; view.rendersContinuously = false
        return view
    }
    func updateNSView(_ view: SCNView, context: Context) {
        context.coordinator.bind(channel, model: model, lockPresentationToOrigin: true,
                                 showWorld: false, showFootTrails: true)
        context.coordinator.refreshGeometry(using: model)
    }
}

// Every simulation is an object-centred inspection view.  SceneKit still
// provides orbit and scroll-wheel zoom, but no pan/translation that could let
// the robot disappear outside the viewport.
private func configureSimulationCamera(_ view: SCNView) {
    view.allowsCameraControl = true
    view.cameraControlConfiguration.allowsTranslation = false
    view.defaultCameraController.interactionMode = .orbitTurntable
    view.defaultCameraController.inertiaEnabled = false
    view.defaultCameraController.minimumVerticalAngle = -12
    view.defaultCameraController.maximumVerticalAngle = 78
}

// A second SceneKit host intentionally has no AuraRobotRenderChannel.  The
// entire Test IK tab is local to the Mac: changing its sliders can never arm a
// servo, issue a UART packet or influence the Wi-Fi connection.
private struct IKLabScene: NSViewRepresentable {
    @ObservedObject var model: QuadrupedModel
    let gait: QuadGait
    let forward: Double
    let lateral: Double
    let turn: Double
    let playing: Bool
    let time: TimeInterval

    final class Coordinator {
        let renderer = QuadrupedScene()

        func makeView() -> SCNView {
            let view = SCNView()
            view.scene = renderer.scene
            view.pointOfView = renderer.camera
            view.backgroundColor = .clear
            configureSimulationCamera(view)
            view.autoenablesDefaultLighting = false
            view.antialiasingMode = .none
            view.preferredFramesPerSecond = 60
            view.rendersContinuously = false
            return view
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeNSView(context: Context) -> SCNView {
        context.coordinator.makeView()
    }

    func updateNSView(_ view: SCNView, context: Context) {
        let snapshot = model.ikLabSnapshot(at: time, gait: gait, forward: forward,
                                           lateral: lateral, turn: turn,
                                           running: playing)
        context.coordinator.renderer.update(model, snapshot: snapshot,
                                            lockPresentationToOrigin: true,
                                            showFootTrails: true)
        view.needsDisplay = true
    }
}

// Offline workbench for the desktop kinematic model. It deliberately owns a
// separate QuadrupedModel and has no connection object, so it remains useful
// with the board unplugged and makes hardware-safe trajectory experiments.
private final class IKLabStore: ObservableObject {
    @Published var gait: QuadGait = .crawl
    @Published var playing = true
    @Published var forward = 1.0
    @Published var lateral = 0.0
    @Published var turn = 0.0
    @Published var probeLeg: QuadLeg = .frontLeft
    @Published var probeForward = 0.0
    @Published var probeLateral = 0.0
    @Published var probeLift = 0.0
}

struct IKLabView: View {
    // Geometry stays local to the Mac.  Gait profiles are deliberately shared
    // with the Quadruped tab so the same stride can be inspected here before
    // it is stored on Aura while the robot is disarmed.
    @ObservedObject var aura: AuraConnection
    @StateObject private var model = QuadrupedModel(persistGeometry: false)
    @StateObject private var store = IKLabStore()

    private func axisControl(_ title: String, value: Binding<Double>, range: ClosedRange<Double>, suffix: String = "") -> some View {
        HStack(spacing: 8) {
            Text(title).font(.caption).frame(width: 74, alignment: .leading)
            Slider(value: value, in: range, step: range.upperBound <= 1 ? 0.01 : 1)
            Text(String(format: range.upperBound <= 1 ? "%+.2f%@" : "%+.0f%@", value.wrappedValue, suffix))
                .font(.caption.monospacedDigit())
                .frame(width: 62, alignment: .trailing)
        }
    }

    @ViewBuilder
    private var gaitProfileEditor: some View {
        GroupBox("Trajektoria kroku · wspólna z Quadruped") {
            if store.gait == .stand {
                Text("Stanie nie ma trajektorii kroku. Wybierz Trot, Krok 1×, Run albo Climb.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .frame(width: 280, alignment: .leading)
            } else {
                let gaitID = store.gait.boardID
                let profile = aura.gaitProfile(gaitID)
                let editable = !aura.robotState.armed
                let stride = Binding<Double>(
                    get: { Double(aura.gaitProfile(gaitID).strideMm) },
                    set: { value in
                        var changed = aura.gaitProfile(gaitID)
                        changed.strideMm = Int(value.rounded())
                        aura.replaceGaitProfileLocally(changed)
                    })
                let lift = Binding<Double>(
                    get: { Double(aura.gaitProfile(gaitID).stepHeightMm) },
                    set: { value in
                        var changed = aura.gaitProfile(gaitID)
                        changed.stepHeightMm = Int(value.rounded())
                        aura.replaceGaitProfileLocally(changed)
                    })
                let maximumFrequency = (store.gait.isSingleFoot || store.gait == .tripod) ? 1.20 : 3.00
                let frequency = Binding<Double>(
                    get: { aura.gaitProfile(gaitID).frequencyHz },
                    set: { value in
                        var changed = aura.gaitProfile(gaitID)
                        changed.frequencyCentiHz = Int((value * 100).rounded())
                        aura.replaceGaitProfileLocally(changed)
                    })
                VStack(alignment: .leading, spacing: 8) {
                    HStack(spacing: 8) {
                        Text("Długość").font(.caption)
                        Slider(value: stride, in: 15...300, step: 1, onEditingChanged: { editing in
                            guard !editing, aura.isConnected, !aura.robotState.armed else { return }
                            aura.setRobotGaitProfile(aura.gaitProfile(gaitID))
                        })
                        .disabled(!editable)
                        Text("\(profile.strideMm) mm")
                            .font(.caption.monospacedDigit())
                            .frame(width: 55, alignment: .trailing)
                    }
                    HStack(spacing: 8) {
                        Text("Wysokość kroku").font(.caption)
                        Slider(value: lift, in: 5...100, step: 1, onEditingChanged: { editing in
                            guard !editing, aura.isConnected, !aura.robotState.armed else { return }
                            aura.setRobotGaitProfile(aura.gaitProfile(gaitID))
                        })
                        .disabled(!editable)
                        Text("\(profile.stepHeightMm) mm")
                            .font(.caption.monospacedDigit())
                            .frame(width: 55, alignment: .trailing)
                    }
                    HStack(spacing: 8) {
                        Text("Częstotliwość").font(.caption)
                        Slider(value: frequency, in: 0.15...maximumFrequency, step: 0.05, onEditingChanged: { editing in
                            guard !editing, aura.isConnected, !aura.robotState.armed else { return }
                            aura.setRobotGaitProfile(aura.gaitProfile(gaitID))
                        })
                        .disabled(!editable)
                        Text(String(format: "%.2f Hz", profile.frequencyHz))
                            .font(.caption.monospacedDigit())
                            .frame(width: 55, alignment: .trailing)
                    }
                    Text(aura.robotState.armed
                         ? "Rozbrój Aurę przed zmianą wspólnego profilu."
                         : aura.isConnected
                            ? "Po puszczeniu suwaka wartość zapisuje się w Aurze; nie wysyła celu serw."
                            : "Zmiana jest widoczna od razu w Test IK i Quadruped. Podłącz Aurę, aby ją zapisać na płytce.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                .frame(width: 280)
            }
        }
    }

    var body: some View {
        TimelineView(.animation(minimumInterval: 1.0 / 60.0, paused: false)) { timeline in
            let neutral = model.ikLabNeutralTarget(for: store.probeLeg)
            let target = neutral + QuadVector(x: store.probeForward, y: store.probeLift, z: store.probeLateral)
            let solution = model.ikLabProbe(leg: store.probeLeg, target: target)
            ScrollView {
                VStack(alignment: .leading, spacing: 14) {
                    HStack(alignment: .firstTextBaseline) {
                        VStack(alignment: .leading, spacing: 3) {
                            Text("Test IK").font(.title2.bold())
                            Text("Lokalna scena 3D i inverse kinematics — wspólny profil trajektorii z Quadruped")
                                .foregroundStyle(.secondary)
                        }
                        Spacer()
                        Label("TYLKO MODEL", systemImage: "desktopcomputer")
                            .font(.caption.weight(.semibold))
                            .foregroundStyle(.mint)
                    }

                    HStack(alignment: .top, spacing: 14) {
                        IKLabScene(model: model, gait: store.gait, forward: store.forward,
                                   lateral: store.lateral, turn: store.turn, playing: store.playing,
                                   time: timeline.date.timeIntervalSinceReferenceDate)
                            .frame(minWidth: 620, minHeight: 520)
                            .clipShape(RoundedRectangle(cornerRadius: 14))
                            .overlay(alignment: .bottom) {
                                HStack {
                                    Text("Przeciągnij: obrót • przewiń: zoom • trajektorie stóp")
                                        .font(.caption).foregroundStyle(.secondary)
                                    Spacer()
                                    Button("Wycentruj widok") { model.resetCamera() }
                                        .controlSize(.small)
                                }
                                .padding(12)
                            }

                        VStack(alignment: .leading, spacing: 12) {
                            GroupBox("Wirtualny pad") {
                                VStack(alignment: .leading, spacing: 9) {
                                    Picker("Chód", selection: $store.gait) {
                                        ForEach(QuadGait.allCases, id: \.self) { gait in
                                            Text(gait.rawValue).tag(gait)
                                        }
                                    }
                                    .pickerStyle(.menu)
                                    if store.gait == .tripod {
                                        Picker("Noga w powietrzu", selection: $model.tripodExcluded) {
                                            ForEach(QuadLeg.allCases) { Text($0.title).tag($0.wireIndex) }
                                        }
                                    }
                                    Toggle("Animacja", isOn: $store.playing)
                                    if store.gait == .stand {
                                        Text("Stanie utrzymuje neutralne pozycje — osie wirtualnego pada są wtedy celowo ignorowane.")
                                            .font(.caption)
                                            .foregroundStyle(.secondary)
                                    } else {
                                        axisControl("Przód", value: $store.forward, range: -1...1)
                                        axisControl("Bok", value: $store.lateral, range: -1...1)
                                        axisControl("Skręt", value: $store.turn, range: -1...1)
                                        Text("Te osie zasilają wyłącznie wirtualny pad tego widoku.")
                                            .font(.caption).foregroundStyle(.secondary)
                                    }
                                }
                                .frame(width: 280)
                            }

                            gaitProfileEditor

                            GroupBox("Wymiary modelu") {
                                VStack(spacing: 7) {
                                    DimensionField(title: "Rama — długość", value: $model.frameLength)
                                    DimensionField(title: "Rama — szerokość", value: $model.frameWidth)
                                    DimensionField(title: "Wysokość stania", value: $model.bodyHeight)
                                    DimensionField(title: "Yaw → hip", value: $model.yawLink)
                                    DimensionField(title: "Hip → knee", value: $model.upperLeg)
                                    DimensionField(title: "Knee → stopa", value: $model.lowerLeg)
                                }
                                .frame(width: 280)
                            }
                        }
                    }

                    GroupBox("Próbnik inverse kinematics") {
                        VStack(alignment: .leading, spacing: 10) {
                            HStack {
                                Picker("Noga", selection: $store.probeLeg) {
                                    ForEach(QuadLeg.allCases) { leg in Text(leg.title).tag(leg) }
                                }
                                .frame(width: 190)
                                Spacer()
                                Label(solution.reachable ? "Punkt osiągalny" : "Punkt poza zasięgiem",
                                      systemImage: solution.reachable ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                                    .foregroundStyle(solution.reachable ? .green : .orange)
                            }
                            HStack(alignment: .top, spacing: 26) {
                                VStack(alignment: .leading, spacing: 7) {
                                    axisControl("Przód", value: $store.probeForward, range: -180...180, suffix: " mm")
                                    axisControl("Bok", value: $store.probeLateral, range: -160...160, suffix: " mm")
                                    axisControl("Podniesienie", value: $store.probeLift, range: 0...180, suffix: " mm")
                                    Text(String(format: "Cel: x %+.0f • y %.0f • z %+.0f mm", target.x, target.y, target.z))
                                        .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                                }
                                Divider().frame(height: 86)
                                if solution.reachable {
                                    Grid(alignment: .leading, horizontalSpacing: 16, verticalSpacing: 6) {
                                        GridRow { Text("Ab/ad").foregroundStyle(.secondary); Text(String(format: "%+.1f°", solution.abduction)).monospacedDigit() }
                                        GridRow { Text("Hip").foregroundStyle(.secondary); Text(String(format: "%+.1f°", solution.hip)).monospacedDigit() }
                                        GridRow { Text("Knee").foregroundStyle(.secondary); Text(String(format: "%+.1f°", solution.knee)).monospacedDigit() }
                                    }
                                } else {
                                    VStack(alignment: .leading, spacing: 4) {
                                        Text("Ten punkt nie ma rozwiązania dla aktualnych długości ogniw.")
                                        Text(String(format: "Odchyłka równania: %.3f", solution.radialError))
                                            .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                                    }
                                    .font(.caption)
                                }
                            }
                            Text("Próbnik liczy dokładnie lokalne równanie 3-DOF używane przez scenę. Nie ma dostępu do Wi‑Fi, Bluetootha, UART ani sterowników serw.")
                                .font(.caption).foregroundStyle(.secondary)
                        }
                        .padding(.vertical, 3)
                    }

                    Text("Kolorowe linie pokazują zapamiętaną trajektorię każdej stopy w bieżącym cyklu. Ten widok służy do dopracowania wymiarów, trajektorii stopy i kierunków przed testem sprzętowym. Statyczna stabilność na fizycznym robocie nadal wymaga bezpiecznych krańców, poprawnej kalibracji oraz później sprzężenia z IMU.")
                        .font(.caption).foregroundStyle(.secondary)
                }
                .padding()
            }
        }
        .onAppear { model.setLocomotionProfiles(aura.gaitProfiles) }
        .onChange(of: aura.gaitProfiles) { _, profiles in
            model.setLocomotionProfiles(profiles)
        }
    }
}

// Used only by Calibration. It is the same SceneKit renderer and the same
// kinematic model as the full Quadruped tab; only its visual emphasis differs.
struct CalibrationQuadrupedScene: NSViewRepresentable {
    let leg: RobotLegID
    let referenceAngles: [Double]
    // Present only on the direction step. The model then draws an arrow from
    // this mechanical reference pose to the +12° target pose.
    let directionReferenceAngles: [Double]?
    let previewMode: CalibrationPreviewMode
    let liveTestTelemetry: Bool
    let allLegLivePreview: Bool
    let highlightedAxis: Int?
    let time: TimeInterval
    let channel: AuraRobotRenderChannel

    final class Coordinator {
        private let renderer = QuadrupedScene()
        private let model = QuadrupedModel()
        private var liveSubscription: AnyCancellable?
        private var liveMode = false
        private var allLegLivePreview = false
        private var selectedLeg: RobotLegID = .lf
        private var selectedAxis: Int?
        private weak var sceneView: SCNView?

        func makeView() -> SCNView {
            let view = SCNView()
            view.scene = renderer.scene
            view.pointOfView = renderer.camera
            view.backgroundColor = .clear
            configureSimulationCamera(view)
            view.autoenablesDefaultLighting = false
            view.antialiasingMode = .none
            view.preferredFramesPerSecond = 60
            view.rendersContinuously = false
            sceneView = view
            return view
        }

        private func bindLiveFrames(_ channel: AuraRobotRenderChannel) {
            guard liveSubscription == nil else { return }
            liveSubscription = channel.frames
                .receive(on: RunLoop.main)
                .sink { [weak self] frame in
                    guard let self, self.liveMode else { return }
                    let snapshot = self.allLegLivePreview
                        ? self.model.calibrationAllLiveSnapshot(frame: frame)
                        : self.model.calibrationLiveSnapshot(
                            selected: self.selectedLeg, frame: frame,
                            at: ProcessInfo.processInfo.systemUptime)
                    self.renderer.update(self.model, snapshot: snapshot,
                                         calibrationHighlight: QuadLeg(robotLeg: self.selectedLeg),
                                         calibrationAxis: self.selectedAxis,
                                         lockPresentationToOrigin: true)
                    // Unlike normal SwiftUI updates, a 50 Hz telemetry frame
                    // mutates SceneKit directly. Request a redraw explicitly;
                    // otherwise an idle SCNView can keep displaying the old
                    // 0 / 0 / ±90 reference pose forever.
                    self.sceneView?.needsDisplay = true
                }
        }

        func update(leg: RobotLegID, referenceAngles: [Double], directionReferenceAngles: [Double]?, previewMode: CalibrationPreviewMode,
                    liveTestTelemetry: Bool, allLegLivePreview: Bool, highlightedAxis: Int?, time: TimeInterval,
                    channel: AuraRobotRenderChannel) {
            selectedLeg = leg
            selectedAxis = highlightedAxis
            self.allLegLivePreview = allLegLivePreview
            let enteringLiveMode = liveTestTelemetry && !liveMode
            liveMode = liveTestTelemetry
            bindLiveFrames(channel)
            // Paint the reference only once while waiting for the first live
            // frame. Repainting it on every SwiftUI update used to overwrite
            // the 50 Hz telemetry animation immediately after it arrived.
            if liveTestTelemetry {
                if enteringLiveMode {
                    let snapshot = model.calibrationSnapshot(selected: leg,
                                                             referenceAngles: referenceAngles,
                                                             at: time, mode: .still)
                    renderer.update(model, snapshot: snapshot,
                                    calibrationHighlight: QuadLeg(robotLeg: leg),
                                    calibrationAxis: highlightedAxis,
                                    lockPresentationToOrigin: true)
                }
                return
            }
            let snapshot = model.calibrationSnapshot(selected: leg,
                                                     referenceAngles: referenceAngles,
                                                     at: time,
                                                     mode: previewMode)
            let directionGuide = directionReferenceAngles.flatMap { reference in
                highlightedAxis.flatMap { axis in
                    model.calibrationDirectionGuide(leg: leg, axis: axis,
                                                    referenceAngles: reference,
                                                    targetAngles: referenceAngles)
                }
            }
            renderer.update(model, snapshot: snapshot,
                            calibrationHighlight: QuadLeg(robotLeg: leg),
                            calibrationAxis: highlightedAxis,
                            calibrationDirection: directionGuide,
                            lockPresentationToOrigin: true)
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeNSView(context: Context) -> SCNView {
        context.coordinator.makeView()
    }

    func updateNSView(_ view: SCNView, context: Context) {
        context.coordinator.update(leg: leg, referenceAngles: referenceAngles,
                                   directionReferenceAngles: directionReferenceAngles,
                                   previewMode: previewMode, liveTestTelemetry: liveTestTelemetry,
                                   allLegLivePreview: allLegLivePreview,
                                   highlightedAxis: highlightedAxis, time: time, channel: channel)
    }
}

private struct DimensionField: View {
    let title: String; @Binding var value: Double
    var body: some View { HStack(spacing: 8) { Text(title).font(.caption).foregroundStyle(.secondary); Spacer(); TextField(title, value: $value, format: .number.precision(.fractionLength(0))).multilineTextAlignment(.trailing).frame(width: 58); Text("mm").font(.caption.monospaced()).foregroundStyle(.secondary) } }
}

struct QuadrupedView: View {
    let channel: AuraRobotRenderChannel
    @ObservedObject var aura: AuraConnection
    @StateObject private var model = QuadrupedModel()
    @StateObject private var hud = QuadrupedHUDStore()
    var body: some View {
        let state = hud.frame.state
        let controller = hud.frame.controller
        let legTelemetry = hud.frame.legs
        let contacts = hud.frame.contacts
        GeometryReader { available in
            ScrollView([.vertical, .horizontal]) { VStack(alignment: .leading, spacing: 14) {
                HStack(alignment: .firstTextBaseline) { VStack(alignment: .leading, spacing: 3) { Text("Quadruped").font(.title2.bold()); Text("Scena 3D • cele osi z Aury • 12 serw TTL").foregroundStyle(.secondary) }; Spacer(); VStack(alignment: .trailing, spacing: 2) { Label(state.tickCount > 0 ? "Aura: planner 50 Hz" : "Oczekiwanie na Aurę", systemImage: "cpu").foregroundStyle(state.tickCount > 0 ? .green : .secondary); Text(state.calibrationLeg != nil ? "KALIBRACJA: 1 NOGA FIZYCZNA" : state.jumping ? "SKOK" : state.spinMode ? "L1: OBRÓT W MIEJSCU" : "R1: \(QuadGait.auraMode(state.selectedGait).rawValue)").font(.caption.monospaced()).foregroundStyle(state.jumping ? .yellow : state.spinMode ? .mint : .secondary) } }
                HSplitView {
                    QuadrupedSceneView(model: model, channel: channel)
                        .frame(minWidth: 680, idealWidth: 860, maxWidth: .infinity, minHeight: 560)
                        .clipShape(RoundedRectangle(cornerRadius: 14))
                        .overlay(alignment: .bottom) {
                            HStack {
                                Text("Przeciągnij: obrót • przewiń: zoom • trajektorie stóp i łuki skrętu")
                                    .font(.caption).foregroundStyle(.secondary)
                                Spacer()
                                Button("Wycentruj widok") { model.resetCamera() }
                                    .controlSize(.small)
                            }
                            .padding(12)
                        }
                    ScrollView(.vertical) {
                        VStack(alignment: .leading, spacing: 12) {
                            if let cadence = state.effectiveFrequencyHz {
                                Text(String(format: "Faktyczny cykl: %.2f Hz • wspólny dla korpusu i stóp", cadence))
                                    .font(.caption).foregroundStyle(.secondary)
                            }
                            if state.constrainedLegMask != 0 {
                                Text("Ograniczenie trajektorii: sprawdź zakresy nóg, wysokość i długość kroku.")
                                    .font(.caption).foregroundStyle(.orange)
                            }
                            GroupBox("Sterowanie") { VStack(alignment: .leading, spacing: 8) { Text("Lewy drążek  ruch przód/bok"); Text("Prawy X  skręt po łukach CoR • prawy Y  korekta wysokości"); Divider(); Text("R1  Stanie → Trot → Krok 1× → Run → Climb → 3 łapy → Stanie"); Text("Krok 1×: trzy nogi podpierają, jedna robi niski krok."); Text("L1  obrót korpusu w miejscu; prawy X steruje obrotem"); Text("×  skok"); Text("Stanie rusza dopiero po wychyleniu drążka; Trot stepuje w miejscu.") }.font(.caption) }
                        GroupBox("Wysokość stania") { VStack(alignment: .leading, spacing: 8) { HStack(spacing: 8) { Slider(value: $model.bodyHeight, in: 120...250, step: 1, onEditingChanged: { editing in if !editing { aura.setRobotBodyHeight(Int(model.bodyHeight.rounded())) } }).disabled(!aura.isConnected || aura.robotState.armed); Text("\(Int(model.bodyHeight.rounded())) mm").font(.caption.monospacedDigit()).frame(width: 62, alignment: .trailing) }; Text(aura.robotState.armed ? "Zmień po rozbrojeniu robota." : "Zapisywane w Aurze; zmiana nie wysyła ruchu.").font(.caption).foregroundStyle(.secondary) }
                        }
                        GroupBox("Rozstaw końców stóp") { VStack(alignment: .leading, spacing: 8) {
                            let spread = Binding<Double>(get: { model.finalFootSpreadMm }, set: { model.finalFootSpreadMm = $0 })
                            HStack(spacing: 8) {
                                Slider(value: spread, in: 170...430, step: 1, onEditingChanged: { editing in
                                    if !editing { aura.setRobotLateralStance(Int(model.lateralStance.rounded())) }
                                }).disabled(!aura.isConnected || aura.robotState.armed)
                                Text("\(Int(model.finalFootSpreadMm.rounded())) mm").font(.caption.monospacedDigit()).frame(width: 62, alignment: .trailing)
                            }
                            Text("Od końca lewej do końca prawej stopy w pozycji stania. Zapisywane tylko po rozbrojeniu; nie wysyła ruchu.").font(.caption).foregroundStyle(.secondary)
                        } }
                        GroupBox("Profil chodu · Aura") {
                            let profile = aura.gaitProfile(model.configuredGait)
                            let editable = aura.isConnected && !aura.robotState.armed
                            let stride = Binding<Double>(
                                get: { Double(aura.gaitProfile(model.configuredGait).strideMm) },
                                set: { value in var changed = aura.gaitProfile(model.configuredGait); changed.strideMm = Int(value.rounded()); aura.replaceGaitProfileLocally(changed) })
                            let lift = Binding<Double>(
                                get: { Double(aura.gaitProfile(model.configuredGait).stepHeightMm) },
                                set: { value in var changed = aura.gaitProfile(model.configuredGait); changed.stepHeightMm = Int(value.rounded()); aura.replaceGaitProfileLocally(changed) })
                            let frequency = Binding<Double>(
                                get: { aura.gaitProfile(model.configuredGait).frequencyHz },
                                set: { value in var changed = aura.gaitProfile(model.configuredGait); changed.frequencyCentiHz = Int((value * 100).rounded()); aura.replaceGaitProfileLocally(changed) })
                            let minimumDuty = (model.configuredGait == 2 || model.configuredGait == 4 || model.configuredGait == 5) ? 76.0 : 45.0
                            let maximumDuty = (model.configuredGait == 2 || model.configuredGait == 4 || model.configuredGait == 5) ? 90.0 : 85.0
                            let maximumFrequency = (model.configuredGait == 2 || model.configuredGait == 4 || model.configuredGait == 5) ? 1.20 : 3.00
                            let duty = Binding<Double>(
                                get: { Double(aura.gaitProfile(model.configuredGait).dutyPercent) },
                                set: { value in var changed = aura.gaitProfile(model.configuredGait); changed.dutyPercent = Int(value.rounded()); aura.replaceGaitProfileLocally(changed) })
                            VStack(alignment: .leading, spacing: 8) {
                                Picker("Chód", selection: $model.configuredGait) {
                                    ForEach(AuraGaitProfile.defaults) { profile in
                                        Text(profile.title).tag(profile.gait)
                                    }
                                }
                                .pickerStyle(.menu)
                                HStack(spacing: 8) {
                                    Text("Długość").font(.caption)
                                    Slider(value: stride, in: 15...300, step: 1, onEditingChanged: { editing in if !editing { aura.setRobotGaitProfile(aura.gaitProfile(model.configuredGait)) } }).disabled(!editable)
                                    Text("\(profile.strideMm) mm").font(.caption.monospacedDigit()).frame(width: 55, alignment: .trailing)
                                }
                                HStack(spacing: 8) {
                                    Text("Wysokość kroku").font(.caption)
                                    Slider(value: lift, in: 5...100, step: 1, onEditingChanged: { editing in if !editing { aura.setRobotGaitProfile(aura.gaitProfile(model.configuredGait)) } }).disabled(!editable)
                                    Text("\(profile.stepHeightMm) mm").font(.caption.monospacedDigit()).frame(width: 55, alignment: .trailing)
                                }
                                HStack(spacing: 8) {
                                    Text("Częstotliwość").font(.caption)
                                    Slider(value: frequency, in: 0.15...maximumFrequency, step: 0.05, onEditingChanged: { editing in if !editing { aura.setRobotGaitProfile(aura.gaitProfile(model.configuredGait)) } }).disabled(!editable)
                                    Text(String(format: "%.2f Hz", profile.frequencyHz)).font(.caption.monospacedDigit()).frame(width: 55, alignment: .trailing)
                                }
                                HStack(spacing: 8) {
                                    Text("Kontakt").font(.caption)
                                    Slider(value: duty, in: minimumDuty...maximumDuty, step: 1, onEditingChanged: { editing in if !editing { aura.setRobotGaitProfile(aura.gaitProfile(model.configuredGait)) } }).disabled(!editable)
                                    Text("\(profile.dutyPercent)%").font(.caption.monospacedDigit()).frame(width: 55, alignment: .trailing)
                                }
                                Text("Długość kroku do 300 mm, podniesienie do 100 mm. Trot i Run do 3 Hz, Krok 1× i Climb do 1,2 Hz. FootSwingTrajectory MIT ma szczyt pionowego łuku dokładnie w połowie przenoszenia. Zmiana jest zapisywana tylko po rozbrojeniu.").font(.caption).foregroundStyle(.secondary)
                            }
                        }
                        GroupBox("Chodzenie na 3 łapach") {
                            VStack(alignment: .leading, spacing: 8) {
                                Toggle("Jedna noga stale w powietrzu", isOn: Binding(
                                    get: { aura.robotState.tripodWalkEnabled },
                                    set: { aura.setTripodWalking($0, excludedLeg: aura.robotState.tripodExcludedLeg) }))
                                Picker("Uniesiona noga", selection: Binding(
                                    get: { aura.robotState.tripodExcludedLeg },
                                    set: { aura.setTripodWalking(aura.robotState.tripodWalkEnabled, excludedLeg: $0) })) {
                                    ForEach(QuadLeg.allCases) { Text($0.title).tag($0.wireIndex) }
                                }
                                Text("Eksperymentalny: wybrana noga jest podniesiona, trzy pozostałe kroczą. Wybór tylko po rozbrojeniu; uzbrojenie nadal przyciskiem Create. R1 wybiera ten tryb jako 5 — pięć białych kropek na padzie; kolejne R1 wraca do Stania.")
                                    .font(.caption).foregroundStyle(.secondary)
                            }.disabled(!aura.isConnected || aura.robotState.armed || !aura.robotState.balanceExtensionAvailable)
                        }
                        GroupBox("Korekcja stóp") {
                            VStack(alignment: .leading, spacing: 6) {
                                Text(state.swingBalanceActive ? "Podparcie + IMU: korekcja aktywna" : "Korekcja oczekuje na chód i świeży odczyt podparcia")
                                if state.constrainedLegMask != 0 {
                                    Text("Zakres ogranicza: " + QuadLeg.allCases.filter {
                                        state.constrainedLegMask & (1 << $0.wireIndex) != 0
                                    }.map(\.rawValue).joined(separator: ", "))
                                        .foregroundStyle(.orange)
                                }
                            }.font(.caption)
                        }
                        GroupBox("Model planowania") {
                            VStack(alignment: .leading, spacing: 7) {
                                Text("Wspólna faza stóp i korpusu • obliczenia na ESP32")
                                    .font(.caption.weight(.semibold))
                                Text("Stopa: trajektoria Béziera MIT. Korpus: podparcie VPSP i okresowa trajektoria LIPM. Podniesione stopy są przeliczane względem pozy zmierzonej przez IMU i enkodery nóg podporowych.")
                                    .font(.caption).foregroundStyle(.secondary)
                                Text("To adaptacja do serw pozycyjnych, nie siłowe MPC. Test IK pokazuje plan; stabilność robota wymaga próby fizycznej.")
                                    .font(.caption).foregroundStyle(.secondary)
                            }
                        }
                        GroupBox("Środek masy · krok 1×") { VStack(alignment: .leading, spacing: 8) {
                            HStack(spacing: 8) { Text("Przód / tył").font(.caption); Slider(value: $model.staticComForward, in: -120...120, step: 1, onEditingChanged: { editing in if !editing { aura.setRobotStaticBalance(comForward: Int(model.staticComForward.rounded()), comLeft: Int(model.staticComLeft.rounded()), supportMargin: Int(model.staticSupportMargin.rounded())) } }).disabled(!aura.isConnected || aura.robotState.armed); Text("\(Int(model.staticComForward.rounded())) mm").font(.caption.monospacedDigit()).frame(width: 52, alignment: .trailing) }
                            HStack(spacing: 8) { Text("Lewo / prawo").font(.caption); Slider(value: $model.staticComLeft, in: -120...120, step: 1, onEditingChanged: { editing in if !editing { aura.setRobotStaticBalance(comForward: Int(model.staticComForward.rounded()), comLeft: Int(model.staticComLeft.rounded()), supportMargin: Int(model.staticSupportMargin.rounded())) } }).disabled(!aura.isConnected || aura.robotState.armed); Text("\(Int(model.staticComLeft.rounded())) mm").font(.caption.monospacedDigit()).frame(width: 52, alignment: .trailing) }
                            HStack(spacing: 8) { Text("Margines podparcia").font(.caption); Slider(value: $model.staticSupportMargin, in: 5...45, step: 1, onEditingChanged: { editing in if !editing { aura.setRobotStaticBalance(comForward: Int(model.staticComForward.rounded()), comLeft: Int(model.staticComLeft.rounded()), supportMargin: Int(model.staticSupportMargin.rounded())) } }).disabled(!aura.isConnected || aura.robotState.armed); Text("\(Int(model.staticSupportMargin.rounded())) mm").font(.caption.monospacedDigit()).frame(width: 52, alignment: .trailing) }
                            Button("Wycentruj środek masy") {
                                model.staticComForward = 0; model.staticComLeft = 0
                                aura.setRobotStaticBalance(comForward: 0, comLeft: 0, supportMargin: Int(model.staticSupportMargin.rounded()))
                            }.disabled(!aura.isConnected || aura.robotState.armed)
                            Text("+ przód / lewo • − tył / prawo. Ustaw przesunięcie baterii względem środka ramy. Zapis nie porusza serwami.").font(.caption).foregroundStyle(.secondary)
                        } }
                        GroupBox("Grawitacja i podparcie") { VStack(alignment: .leading, spacing: 7) {
                            Text(model.gravityStatusText).font(.caption.weight(.semibold))
                            Text("Pomarańczowa kula: środek masy. Znacznik na podłożu: jego rzut. Bez IMU widok nie symuluje swobodnego przechyłu ani upadku — pokazuje wyłącznie margines podparcia.").font(.caption).foregroundStyle(.secondary)
                        } }
                        GroupBox("Kontakt stóp · IMU + serwa") {
                            VStack(alignment: .leading, spacing: 7) {
                                if contacts.feedbackAvailable {
                                    ForEach(QuadLeg.allCases) { leg in
                                        let index = leg.wireIndex
                                        HStack(spacing: 6) {
                                            Circle()
                                                .fill(contacts.isEarlyTouchdown(index) ? .orange :
                                                      contacts.isDetected(index) ? .green : .secondary)
                                                .frame(width: 8, height: 8)
                                            Text(leg.rawValue).font(.caption.monospaced().weight(.semibold))
                                            Text(contacts.isDetected(index) ? "kontakt" : "brak kontaktu")
                                                .font(.caption)
                                            Spacer()
                                            Text("\(contacts.confidence[index])%")
                                                .font(.caption.monospacedDigit())
                                        }
                                    }
                                    Text("Zielony: kontakt wykryty. Pomarańczowy: wczesne dotknięcie w fazie kroku.")
                                        .font(.caption).foregroundStyle(.secondary)
                                } else {
                                    Text("Czekam na pełny odczyt Hip i Knee z każdej nogi.")
                                        .font(.caption).foregroundStyle(.secondary)
                                }
                                Text(contacts.feedbackAvailable
                                     ? "Odczyt kontaktu jest diagnostyczny. Nie zmienia fazy kroku, pozycji stopy ani ruchu korpusu."
                                     : "Brak pełnego odczytu kontaktu; trajektoria pozostaje w całości planowana czasowo.")
                                    .font(.caption).foregroundStyle(.secondary)
                            }
                        }
                        GroupBox("Komenda z pada") { VStack(alignment: .leading, spacing: 7) { Text(String(format: "Przód    %+.2f", state.forward)); Text(String(format: "Bok      %+.2f", state.lateral)); Text(String(format: "Skręt    %+.2f", state.turn)); Text("Wysokość \(state.bodyHeight) mm"); Divider(); Text(String(format: "Pozycja  %.0f / %.0f mm", state.odometryX, state.odometryZ)); Text(String(format: "Kierunek %.0f°", state.odometryYaw * 180 / .pi)) }.font(.caption.monospaced()) }
                            GroupBox("Stan") { VStack(alignment: .leading, spacing: 6) { Text(controller.battery.map { "Bateria pada: \($0)%" } ?? "Bateria pada: —"); Text("Raportów: \(controller.sampleCount)"); Text("Scena: 50 Hz • panel: 10 Hz") }.font(.caption.monospaced()) }
                        }
                        .padding(.trailing, 4)
                    }
                    .frame(minWidth: 320, idealWidth: 340, maxWidth: 380, minHeight: 560, maxHeight: 620, alignment: .topLeading)
                }
                .frame(minWidth: 1050, maxWidth: .infinity, minHeight: 560, maxHeight: 620, alignment: .leading)
                HStack(alignment: .top, spacing: 14) {
                    GroupBox("Rama") { VStack(spacing: 8) { DimensionField(title: "Długość", value: $model.frameLength); DimensionField(title: "Szerokość", value: $model.frameWidth); DimensionField(title: "Odsunięcie narożnika", value: $model.cornerInset) }.frame(minWidth: 220) }
                    GroupBox("Noga") { VStack(spacing: 8) { DimensionField(title: "Yaw → hip", value: $model.yawLink); DimensionField(title: "Hip → knee", value: $model.upperLeg); DimensionField(title: "Knee → stopa", value: $model.lowerLeg) }.frame(minWidth: 245) }
                    GroupBox("12 zadanych kątów") { Grid(alignment: .leading, horizontalSpacing: 12, verticalSpacing: 7) { GridRow { Text("Noga"); Text("Ab/ad"); Text("Hip"); Text("Knee") }.foregroundStyle(.secondary); ForEach(QuadLeg.allCases) { leg in let index = leg.wireIndex; let feedback = legTelemetry.indices.contains(index) ? legTelemetry[index] : AuraRobotLegTelemetry(id: index); let angles = feedback.present.allSatisfy { $0 } ? feedback.measuredDegrees : feedback.targetDegrees; GridRow { Text(leg.rawValue).bold(); Text(String(format: "%+.0f°", angles[0])).monospacedDigit(); Text(String(format: "%+.0f°", angles[1])).monospacedDigit(); Text(String(format: "%+.0f°", angles[2])).monospacedDigit() } } }.font(.caption) }
                }
                .frame(minWidth: 1050, maxWidth: .infinity, alignment: .leading)
            }
            .frame(minWidth: max(1050, available.size.width - 24), alignment: .leading)
            .padding()
            }
        }
            .onAppear { hud.bind(channel) }
            .onAppear { model.setLocomotionProfiles(aura.gaitProfiles) }
            .onChange(of: aura.gaitProfiles) { _, profiles in
                model.setLocomotionProfiles(profiles)
            }
            .onChange(of: aura.robotState.bodyHeight) { _, height in
                guard !aura.robotState.armed, (120...250).contains(height) else { return }
                model.bodyHeight = Double(height)
            }
    }
}
