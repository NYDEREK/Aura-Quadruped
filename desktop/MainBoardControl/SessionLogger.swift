import AppKit
import Combine
import Foundation
import UniformTypeIdentifiers

// One .aura file per session: JSON with a summary, 5 Hz column series,
// per-mode statistics, an event log and, while Aura is armed, a 10 Hz trace
// of every joint, the IMU and the gait state. A session starts with the
// first telemetry from Aura and ends after 5 s without it, or when the app
// quits. Files live on the Mac only (~/Documents/Aura/Sessions); nothing is
// stored on the robot.

struct AuraSessionSummary: Codable, Equatable {
    var duration = 0.0          // s
    var distance = 0.0          // mm, horizontal
    var movingTime = 0.0        // s above 20 mm/s
    var averageSpeed = 0.0      // mm/s while moving
    var maxSpeed = 0.0          // mm/s, 1 s average
    var energy = 0.0            // mAh drawn by the servos
    var voltageStart = 0.0, voltageEnd = 0.0, voltageMin = 0.0, voltageMax = 0.0
    var currentAverage = 0.0, currentMax = 0.0   // A
    var maxServoTemperature = 0.0                // °C
    var armedTime = 0.0                          // s
    var safetyFaults = 0
}

struct AuraSessionSeries: Codable, Equatable {
    var t: [Double] = []
    var voltage: [Double] = []
    var current: [Double] = []
    var power: [Double] = []
    var speed: [Double] = []
    var x: [Double] = [], y: [Double] = [], z: [Double] = []
    var heading: [Double] = []
    var roll: [Double] = [], pitch: [Double] = []
    var servoTemperature: [Double] = []
    var mode: [Int] = []
    var armed: [Bool] = []
}

// How Aura behaved in one pad mode, accumulated from every armed telemetry
// snapshot (10 Hz).
// Sums are stored so files can be merged or re-analysed; the RMS values are
// derived from them.
struct AuraModeStats: Codable, Equatable, Identifiable {
    var mode: Int
    var id: Int { mode }
    var armedTime = 0.0                 // s armed in this mode
    var movingTime = 0.0                // s with a stick command
    var distance = 0.0                  // mm
    var maxSpeed = 0.0                  // mm/s
    var samples = 0                     // armed frames
    var rollSquares = 0.0, pitchSquares = 0.0     // Σ deg², torso against its reference
    var maxRoll = 0.0, maxPitch = 0.0             // |deg|
    var wobbleSquares = 0.0             // Σ (roll rate² + pitch rate²), (deg/s)²
    var correctionSquares = 0.0         // Σ (roll² + pitch²) of the IMU correction, deg²
    var trackingSquares = 0.0           // Σ mean joint tracking error², deg²
    var maxTracking = 0.0               // deg
    var maxTrackingJoint = ""           // e.g. "LF knee"
    var currentSum = 0.0, maxCurrent = 0.0        // A
    var frequencySum = 0.0, frequencySamples = 0  // Hz, while stepping
    var slowedSamples = 0               // gait clock below 95 % (tracking)
    var limitedSamples = 0              // a leg at its range limit
    var contactSamples = 0, contactMismatches = 0 // expected vs detected feet
    var earlyTouchdowns = 0
    var faults = 0

    private func rms(_ sum: Double) -> Double { samples > 0 ? (sum / Double(samples)).squareRoot() : 0 }
    var rollRMS: Double { rms(rollSquares) }
    var pitchRMS: Double { rms(pitchSquares) }
    var wobbleRMS: Double { rms(wobbleSquares) }
    var correctionRMS: Double { rms(correctionSquares) }
    var trackingRMS: Double { rms(trackingSquares) }
    var currentAverage: Double { samples > 0 ? currentSum / Double(samples) : 0 }
    var frequencyAverage: Double { frequencySamples > 0 ? frequencySum / Double(frequencySamples) : 0 }
    var contactMismatchRate: Double { contactSamples > 0 ? Double(contactMismatches) / Double(contactSamples) : 0 }
    var slowedRate: Double { samples > 0 ? Double(slowedSamples) / Double(samples) : 0 }
    var limitedRate: Double { samples > 0 ? Double(limitedSamples) / Double(samples) : 0 }
}

struct AuraSessionEvent: Codable, Equatable {
    var t: Double               // s since the session start
    var kind: String            // start, board start/restart, armed, disarmed, mode, moving, stopped, fault,
                                // fault cleared, pad lost, pad connected, task stalled/resumed
    var detail = ""
}

// Armed-only record at the telemetry rate (10 Hz). Fixed-point integers keep the file small:
// angles 0.01°, joints 0.1°, lengths mm (shift 0.1 mm), currents mA,
// voltages mV. Joint order LF, RF, LR, RR × ab/ad, hip, knee.
struct AuraSessionTrace: Codable, Equatable {
    static let noReply = 99_999         // jointMeasured when the servo did not answer
    static let flagArmed = 1, flagMoving = 2, flagSwingBalance = 4, flagTrackingLimited = 8
    static let flagStill = 16, flagJumping = 32, flagFault = 64, flagAttitudeActive = 128

    var rateHz = 10
    var t: [Double] = []
    var mode: [Int] = []
    var flags: [Int] = []
    var phase: [Int] = []               // × 1000
    var phaseRate: [Int] = []           // × 100
    var frequency: [Int] = []           // 0.01 Hz, 0 when not stepping
    var stick: [[Int]] = [[], [], []]   // forward, lateral, turn × 100
    var roll: [Int] = [], pitch: [Int] = []                     // torso vs reference
    var correctionRoll: [Int] = [], correctionPitch: [Int] = [] // IMU loop output
    var postureRoll: [Int] = [], posturePitch: [Int] = []       // planned torso (mode 2)
    var bodyShift: [[Int]] = [[], []]   // planned CoM shift x, z
    var height: [Int] = []
    var gyro: [[Int]] = [[], [], []]    // deg/s × 10
    var accel: [[Int]] = [[], [], []]   // g × 1000
    var velocity: [[Int]] = [[], []]    // estimator forward, left, mm/s
    var contacts: [[Int]] = [[], [], []] // expected, detected, early-touchdown masks
    var current: [Int] = [], voltage: [Int] = []
    var jointTarget: [[Int]] = Array(repeating: [], count: 12)
    var jointMeasured: [[Int]] = Array(repeating: [], count: 12)

    var isEmpty: Bool { t.isEmpty }
}

extension CodingUserInfoKey {
    // Set while listing sessions: the trace is read only for the open one.
    static let auraSkipTrace = CodingUserInfoKey(rawValue: "auraSkipTrace")!
}

struct AuraSession: Codable, Identifiable, Equatable {
    var format = "aura-session"
    var version = 2
    var id = UUID()
    var start: Date
    var end: Date
    var summary = AuraSessionSummary()
    var series = AuraSessionSeries()
    var modes: [AuraModeStats] = []
    var events: [AuraSessionEvent] = []
    var trace: AuraSessionTrace?

    var fileName: String {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.dateFormat = "yyyy-MM-dd HH-mm-ss"
        return formatter.string(from: start) + ".aura"
    }
}

extension AuraSession {
    private enum CodingKeys: String, CodingKey {
        case format, version, id, start, end, summary, series, modes, events, trace
    }

    // Version 1 files have no modes, events or trace.
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        format = try c.decodeIfPresent(String.self, forKey: .format) ?? "aura-session"
        version = try c.decodeIfPresent(Int.self, forKey: .version) ?? 1
        id = try c.decodeIfPresent(UUID.self, forKey: .id) ?? UUID()
        start = try c.decode(Date.self, forKey: .start)
        end = try c.decode(Date.self, forKey: .end)
        summary = try c.decodeIfPresent(AuraSessionSummary.self, forKey: .summary) ?? AuraSessionSummary()
        series = try c.decodeIfPresent(AuraSessionSeries.self, forKey: .series) ?? AuraSessionSeries()
        modes = try c.decodeIfPresent([AuraModeStats].self, forKey: .modes) ?? []
        events = try c.decodeIfPresent([AuraSessionEvent].self, forKey: .events) ?? []
        trace = decoder.userInfo[.auraSkipTrace] as? Bool == true
            ? nil : try c.decodeIfPresent(AuraSessionTrace.self, forKey: .trace)
    }
}

extension UTType {
    static let auraSession = UTType(exportedAs: "pl.nyderek.aura.session")
}

private let jointNames: [String] = ["LF", "RF", "LR", "RR"].flatMap { leg in
    ["ab/ad", "hip", "knee"].map { "\(leg) \($0)" }
}

@MainActor
final class SessionLogger: ObservableObject {
    @Published private(set) var revision = 0
    @Published private(set) var sessions: [AuraSession] = []
    @Published var selection: UUID?
    // `live` is what the Sessions view shows, published once a second;
    // `current` is the session being recorded.
    @Published private(set) var live: AuraSession?
    private var current: AuraSession?
    private var lastPublished = Date.distantPast
    // The live trace stays out of `live` so appends never copy it.
    private(set) var liveTrace = AuraSessionTrace()

    private var lastFrame = Date.distantPast
    private var lastSaved = Date.distantPast
    private var energyStart: Double?
    private var energyIntegral = 0.0
    private var lastPosition: SIMD3<Double>?
    private var faultLatched = false
    private var timer: AnyCancellable?
    private var termination: AnyCancellable?

    // Per-mode accumulation at the full frame rate.
    private var modeStats: [Int: AuraModeStats] = [:]
    private var events: [AuraSessionEvent] = []
    private var lastFrameTime: Double?
    private var lastModePosition: SIMD3<Double>?
    private var previous: (armed: Bool, mode: Int, moving: Bool, fault: Bool, pad: Bool, earlyMask: UInt8,
                           stalled: Set<String>)?
    private var reportedLowStack: Set<String> = []
    private var lastSlowTickEvent = -10.0

    private var cachedTrace: (id: UUID, trace: AuraSessionTrace?)?

    static var folder: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("Aura/Sessions", isDirectory: true)
    }

    init() {
        loadAll()
        timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect().sink { [weak self] now in
            guard let self, self.current != nil else { return }
            if now.timeIntervalSince(self.lastFrame) > 5 { self.finish() }
            else if now.timeIntervalSince(self.lastSaved) > 30 { self.save() }
            self.revision &+= 1
        }
        termination = NotificationCenter.default.publisher(for: NSApplication.willTerminateNotification)
            .sink { [weak self] _ in MainActor.assumeIsolated { self?.finish() } }
    }

    // One call per telemetry snapshot (10 Hz).
    func ingest(power: AuraPower, state: AuraRobotState, attitude: AuraAttitude, imu: AuraIMU,
                contacts: AuraFootContacts, health: AuraHealth, legs: [AuraRobotLegTelemetry],
                route: RouteRecorder) {
        let now = Date()
        lastFrame = now
        if current == nil {
            current = AuraSession(start: now, end: now)
            live = current
            lastPublished = now
            energyStart = nil
            energyIntegral = 0
            lastPosition = nil
            lastSaved = now
            liveTrace = AuraSessionTrace()
            modeStats = [:]
            events = []
            reportedLowStack = []
            lastSlowTickEvent = -10
            lastFrameTime = nil
            lastModePosition = nil
            previous = nil
        }
        guard let start = current?.start else { return }
        let t = now.timeIntervalSince(start)
        let frameDt = lastFrameTime.map { min(0.1, max(0, t - $0)) } ?? 0
        lastFrameTime = t

        let moving = (state.forward * state.forward + state.lateral * state.lateral + state.turn * state.turn) > 0.0004
        // Mode statistics read the previous frame's flags before the event
        // log replaces them.
        accumulateMode(dt: frameDt, state: state, attitude: attitude, imu: imu, contacts: contacts,
                       legs: legs, power: power, route: route, moving: moving)
        recordEvents(t: t, state: state, moving: moving, contacts: contacts, health: health)
        if state.armed || state.safetyFault.latched {
            appendTrace(t: t, state: state, attitude: attitude, imu: imu, contacts: contacts,
                        legs: legs, power: power, moving: moving)
        }

        // 5 Hz overview samples, by time so any telemetry rate works.
        guard var session = current, t - (session.series.t.last ?? -1) >= 0.19 else { return }
        current = nil       // own the arrays: appends below then never copy them
        let dt = session.series.t.last.map { t - $0 } ?? 0
        let temperature = Double(legs.flatMap(\.temperature).max() ?? 0)
        let position = route.position

        var s = session.summary
        if session.series.t.isEmpty {
            s.voltageStart = power.servoBusVoltage
            s.voltageMin = power.servoBusVoltage
        }
        // Distance only while the legs carry the robot; a re-zero or a jump
        // of more than 0.5 m in 200 ms is not travel.
        if let last = lastPosition, state.armed {
            let step = ((position.x - last.x) * (position.x - last.x) + (position.z - last.z) * (position.z - last.z)).squareRoot()
            if step < 500 { s.distance += step }
        }
        lastPosition = position
        if route.speed > 20 { s.movingTime += dt }
        s.maxSpeed = max(s.maxSpeed, route.speed)
        s.averageSpeed = s.movingTime > 0 ? s.distance / s.movingTime : 0
        if energyStart == nil, power.servoConsumedMilliampHours > 0 { energyStart = power.servoConsumedMilliampHours }
        energyIntegral += max(0, power.current) * 1000 * dt / 3600
        let counter = power.servoConsumedMilliampHours - (energyStart ?? power.servoConsumedMilliampHours)
        s.energy = counter > 0 ? counter : energyIntegral
        s.voltageEnd = power.servoBusVoltage
        if power.servoBusVoltage > 1 { s.voltageMin = s.voltageMin > 1 ? min(s.voltageMin, power.servoBusVoltage) : power.servoBusVoltage }
        s.voltageMax = max(s.voltageMax, power.servoBusVoltage)
        let n = Double(session.series.t.count)
        s.currentAverage = (s.currentAverage * n + power.current) / (n + 1)
        s.currentMax = max(s.currentMax, power.current)
        s.maxServoTemperature = max(s.maxServoTemperature, temperature)
        if state.armed { s.armedTime += dt }
        if state.safetyFault.latched && !faultLatched { s.safetyFaults += 1 }
        faultLatched = state.safetyFault.latched
        s.duration = t

        session.series.t.append(t)
        session.series.voltage.append(power.servoBusVoltage)
        session.series.current.append(power.current)
        session.series.power.append(power.power)
        session.series.speed.append(route.speed)
        session.series.x.append(position.x)
        session.series.y.append(position.y)
        session.series.z.append(position.z)
        session.series.heading.append(route.heading)
        session.series.roll.append(route.roll)
        session.series.pitch.append(route.pitch)
        session.series.servoTemperature.append(temperature)
        session.series.mode.append(state.selectedGait)
        session.series.armed.append(state.armed)

        session.summary = s
        session.modes = modeStats.values.sorted { $0.mode < $1.mode }
        session.events = events
        session.end = now
        current = session
        if now.timeIntervalSince(lastPublished) >= 1 {
            lastPublished = now
            live = session
        }
    }

    private func recordEvents(t: Double, state: AuraRobotState, moving: Bool, contacts: AuraFootContacts,
                              health: AuraHealth) {
        let fault = state.safetyFault.latched
        let mode = state.selectedGait
        let stalled = Set(health.stalledTasks)
        defer {
            previous = (state.armed, mode, moving, fault, state.controllerConnected, contacts.earlyTouchdownMask,
                        stalled)
        }
        guard let previous else {
            events.append(AuraSessionEvent(t: t, kind: "start",
                                           detail: "mode \(mode) · \(state.armed ? "armed" : "disarmed")"))
            // Why the board last started: a crash, watchdog or brownout shows here.
            if health.available {
                events.append(AuraSessionEvent(t: t, kind: health.resetAbnormal ? "board restart" : "board start",
                                               detail: health.resetText))
            }
            return
        }
        for task in stalled.subtracting(previous.stalled).sorted() {
            events.append(AuraSessionEvent(t: t, kind: "task stalled", detail: task))
        }
        for task in previous.stalled.subtracting(stalled).sorted() {
            events.append(AuraSessionEvent(t: t, kind: "task resumed", detail: task))
        }
        // The gait loop has 20 ms per tick; longer ticks starve lower tasks.
        if health.gaitTickMaxMs > 20 && t - lastSlowTickEvent > 1 {
            lastSlowTickEvent = t
            events.append(AuraSessionEvent(t: t, kind: "gait tick slow",
                                           detail: "\(health.gaitTickMaxMs) ms · mode \(mode)"))
        }
        for task in Set(health.lowStackTasks).subtracting(reportedLowStack).sorted() {
            reportedLowStack.insert(task)
            let index = AuraHealth.tasks.firstIndex(of: task) ?? 0
            events.append(AuraSessionEvent(t: t, kind: "low stack",
                                           detail: "\(task) · \(health.stackFree[index]) B free · mode \(mode)"))
        }
        if state.armed != previous.armed {
            events.append(AuraSessionEvent(t: t, kind: state.armed ? "armed" : "disarmed", detail: "mode \(mode)"))
        }
        if mode != previous.mode {
            events.append(AuraSessionEvent(t: t, kind: "mode", detail: "\(previous.mode) → \(mode)"))
        }
        if state.armed && moving != previous.moving {
            events.append(AuraSessionEvent(t: t, kind: moving ? "moving" : "stopped", detail: "mode \(mode)"))
        }
        if fault && !previous.fault {
            let f = state.safetyFault
            events.append(AuraSessionEvent(t: t, kind: "fault", detail: String(
                format: "%@ %@ · ID %d · %.2f A · load %.0f %% · error %.1f° · mode %d",
                f.legTitle, f.axisTitle, f.servoID, f.currentAmperes, f.loadPercent, f.trackingErrorDegrees, mode)))
        } else if !fault && previous.fault {
            events.append(AuraSessionEvent(t: t, kind: "fault cleared"))
        }
        if state.controllerConnected != previous.pad {
            events.append(AuraSessionEvent(t: t, kind: state.controllerConnected ? "pad connected" : "pad lost"))
        }
    }

    private func accumulateMode(dt: Double, state: AuraRobotState, attitude: AuraAttitude, imu: AuraIMU,
                                contacts: AuraFootContacts, legs: [AuraRobotLegTelemetry], power: AuraPower,
                                route: RouteRecorder, moving: Bool) {
        let position = route.position
        defer { lastModePosition = position }
        guard state.armed else { return }
        var m = modeStats[state.selectedGait] ?? AuraModeStats(mode: state.selectedGait)
        m.armedTime += dt
        if moving { m.movingTime += dt }
        if let last = lastModePosition {
            let step = ((position.x - last.x) * (position.x - last.x) + (position.z - last.z) * (position.z - last.z)).squareRoot()
            if step < 100 { m.distance += step }
        }
        m.maxSpeed = max(m.maxSpeed, route.speed)
        m.samples += 1
        m.rollSquares += route.roll * route.roll
        m.pitchSquares += route.pitch * route.pitch
        m.maxRoll = max(m.maxRoll, abs(route.roll))
        m.maxPitch = max(m.maxPitch, abs(route.pitch))
        m.wobbleSquares += imu.gyroscope[0] * imu.gyroscope[0] + imu.gyroscope[2] * imu.gyroscope[2]
        m.correctionSquares += attitude.correctionRollDegrees * attitude.correctionRollDegrees +
            attitude.correctionPitchDegrees * attitude.correctionPitchDegrees
        var squares = 0.0, count = 0
        for (leg, telemetry) in legs.enumerated() {
            for axis in 0..<3 where telemetry.present[axis] {
                let error = abs(telemetry.targetDegrees[axis] - telemetry.measuredDegrees[axis])
                guard error.isFinite, error < 360 else { continue }
                squares += error * error
                count += 1
                if error > m.maxTracking {
                    m.maxTracking = error
                    m.maxTrackingJoint = jointNames[leg * 3 + axis]
                }
            }
        }
        if count > 0 { m.trackingSquares += squares / Double(count) }
        m.currentSum += power.current
        m.maxCurrent = max(m.maxCurrent, power.current)
        if let frequency = state.effectiveFrequencyHz, moving {
            m.frequencySum += frequency
            m.frequencySamples += 1
        }
        if state.phaseRate < 0.95 { m.slowedSamples += 1 }
        if state.trackingLimited || state.constrainedLegMask != 0 { m.limitedSamples += 1 }
        if contacts.feedbackAvailable && contacts.controlActive && moving {
            m.contactSamples += 1
            if contacts.expectedMask != contacts.detectedMask { m.contactMismatches += 1 }
        }
        if let previous {
            m.earlyTouchdowns += (contacts.earlyTouchdownMask & ~previous.earlyMask).nonzeroBitCount
            if state.safetyFault.latched && !previous.fault { m.faults += 1 }
        }
        modeStats[state.selectedGait] = m
    }

    private func appendTrace(t: Double, state: AuraRobotState, attitude: AuraAttitude, imu: AuraIMU,
                             contacts: AuraFootContacts, legs: [AuraRobotLegTelemetry], power: AuraPower,
                             moving: Bool) {
        func fixed(_ value: Double, _ scale: Double) -> Int {
            value.isFinite ? Int((value * scale).rounded()) : 0
        }
        let roll = attitude.rollDegrees - (attitude.referenceCaptured ? attitude.referenceRollDegrees : 0)
        let pitch = attitude.pitchDegrees - (attitude.referenceCaptured ? attitude.referencePitchDegrees : 0)
        var flags = 0
        if state.armed { flags |= AuraSessionTrace.flagArmed }
        if moving { flags |= AuraSessionTrace.flagMoving }
        if state.swingBalanceActive { flags |= AuraSessionTrace.flagSwingBalance }
        if state.trackingLimited || state.constrainedLegMask != 0 { flags |= AuraSessionTrace.flagTrackingLimited }
        if state.estimate.still { flags |= AuraSessionTrace.flagStill }
        if state.jumping { flags |= AuraSessionTrace.flagJumping }
        if state.safetyFault.latched { flags |= AuraSessionTrace.flagFault }
        if attitude.active { flags |= AuraSessionTrace.flagAttitudeActive }

        var trace = liveTrace
        liveTrace = AuraSessionTrace()     // keep the buffers uniquely referenced
        trace.t.append((t * 100).rounded() / 100)
        trace.mode.append(state.selectedGait)
        trace.flags.append(flags)
        trace.phase.append(fixed(state.phase, 1000))
        trace.phaseRate.append(fixed(state.phaseRate, 100))
        trace.frequency.append(fixed(moving ? state.effectiveFrequencyHz ?? 0 : 0, 100))
        trace.stick[0].append(fixed(state.forward, 100))
        trace.stick[1].append(fixed(state.lateral, 100))
        trace.stick[2].append(fixed(state.turn, 100))
        trace.roll.append(fixed(roll, 100))
        trace.pitch.append(fixed(pitch, 100))
        trace.correctionRoll.append(fixed(attitude.correctionRollDegrees, 100))
        trace.correctionPitch.append(fixed(attitude.correctionPitchDegrees, 100))
        trace.postureRoll.append(fixed(state.bodyReferenceRoll * 180 / .pi, 100))
        trace.posturePitch.append(fixed(state.bodyReferencePitch * 180 / .pi, 100))
        trace.bodyShift[0].append(fixed(state.bodyShiftX, 10))
        trace.bodyShift[1].append(fixed(state.bodyShiftZ, 10))
        trace.height.append(state.bodyHeight)
        for axis in 0..<3 {
            trace.gyro[axis].append(fixed(imu.gyroscope[axis], 10))
            trace.accel[axis].append(fixed(imu.acceleration[axis], 1000))
        }
        trace.velocity[0].append(fixed(state.estimate.forward, 1))
        trace.velocity[1].append(fixed(state.estimate.left, 1))
        trace.contacts[0].append(Int(contacts.expectedMask))
        trace.contacts[1].append(Int(contacts.detectedMask))
        trace.contacts[2].append(Int(contacts.earlyTouchdownMask))
        trace.current.append(fixed(power.current, 1000))
        trace.voltage.append(fixed(power.servoBusVoltage, 1000))
        for leg in 0..<4 {
            let telemetry = legs.indices.contains(leg) ? legs[leg] : AuraRobotLegTelemetry(id: leg)
            for axis in 0..<3 {
                trace.jointTarget[leg * 3 + axis].append(fixed(telemetry.targetDegrees[axis], 10))
                trace.jointMeasured[leg * 3 + axis].append(telemetry.present[axis]
                    ? fixed(telemetry.measuredDegrees[axis], 10) : AuraSessionTrace.noReply)
            }
        }
        liveTrace = trace
    }

    // The trace of a session: the live one, or read from its file on demand.
    func trace(for session: AuraSession) -> AuraSessionTrace? {
        if session.id == current?.id { return liveTrace }
        if let cachedTrace, cachedTrace.id == session.id { return cachedTrace.trace }
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        let url = Self.folder.appendingPathComponent(session.fileName)
        let full = try? decoder.decode(AuraSession.self, from: Data(contentsOf: url))
        cachedTrace = (session.id, full?.trace)
        return full?.trace
    }

    // Ends the live session: kept if it lasted 15 s or more.
    func finish() {
        guard let session = current else { return }
        if session.summary.duration >= 15 { save() }
        current = nil
        live = nil
        liveTrace = AuraSessionTrace()
        loadAll()
    }

    func save() {
        guard var session = current else { return }
        lastSaved = Date()
        session.trace = liveTrace.isEmpty ? nil : liveTrace
        write(session, to: Self.folder.appendingPathComponent(session.fileName))
    }

    private func write(_ session: AuraSession, to url: URL) {
        do {
            try FileManager.default.createDirectory(at: Self.folder, withIntermediateDirectories: true)
            let encoder = JSONEncoder()
            encoder.dateEncodingStrategy = .iso8601
            try encoder.encode(session).write(to: url, options: .atomic)
        } catch {
            NSSound.beep()
        }
    }

    func loadAll() {
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        decoder.userInfo[.auraSkipTrace] = true
        let files = (try? FileManager.default.contentsOfDirectory(at: Self.folder, includingPropertiesForKeys: nil)) ?? []
        sessions = files.filter { $0.pathExtension == "aura" }
            .compactMap { try? decoder.decode(AuraSession.self, from: Data(contentsOf: $0)) }
            .sorted { $0.start > $1.start }
        cachedTrace = nil
    }

    // A .aura file opened from Finder; copied into the sessions folder.
    func open(_ url: URL) {
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        guard let session = try? decoder.decode(AuraSession.self, from: Data(contentsOf: url)) else { NSSound.beep(); return }
        let destination = Self.folder.appendingPathComponent(session.fileName)
        if url.standardizedFileURL != destination.standardizedFileURL,
           !FileManager.default.fileExists(atPath: destination.path) {
            write(session, to: destination)
        }
        loadAll()
        selection = sessions.first { $0.start == session.start }?.id ?? session.id
    }

    func delete(_ session: AuraSession) {
        try? FileManager.default.removeItem(at: Self.folder.appendingPathComponent(session.fileName))
        if selection == session.id { selection = nil }
        loadAll()
    }

    func reveal(_ session: AuraSession) {
        NSWorkspace.shared.activateFileViewerSelecting([Self.folder.appendingPathComponent(session.fileName)])
    }

    func exportCSV(_ session: AuraSession) {
        let panel = NSSavePanel()
        panel.allowedContentTypes = [.commaSeparatedText]
        panel.nameFieldStringValue = session.fileName.replacingOccurrences(of: ".aura", with: ".csv")
        guard panel.runModal() == .OK, let url = panel.url else { return }
        let s = session.series
        var lines = ["t_s,voltage_V,current_A,power_W,speed_mm_s,x_mm,y_mm,z_mm,heading_deg,roll_deg,pitch_deg,servo_temp_C,mode,armed"]
        for i in s.t.indices {
            lines.append(String(format: "%.1f,%.3f,%.3f,%.2f,%.0f,%.0f,%.0f,%.0f,%.1f,%.1f,%.1f,%.0f,%d,%d",
                                s.t[i], s.voltage[i], s.current[i], s.power[i], s.speed[i], s.x[i], s.y[i], s.z[i],
                                s.heading[i] * 180 / .pi, s.roll[i], s.pitch[i], s.servoTemperature[i],
                                s.mode[i], s.armed[i] ? 1 : 0))
        }
        try? (lines.joined(separator: "\n") + "\n").write(to: url, atomically: true, encoding: .utf8)
    }
}
