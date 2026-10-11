import AppKit
import Combine
import Foundation
import UniformTypeIdentifiers

// One .aura file per session: JSON with a summary and 5 Hz column series.
// A session starts with the first telemetry from Aura and ends after 5 s
// without it, or when the app quits. Files live on the Mac only
// (~/Documents/Aura/Sessions); nothing is stored on the robot.

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

struct AuraSession: Codable, Identifiable, Equatable {
    var format = "aura-session"
    var version = 1
    var id = UUID()
    var start: Date
    var end: Date
    var summary = AuraSessionSummary()
    var series = AuraSessionSeries()

    var fileName: String {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.dateFormat = "yyyy-MM-dd HH-mm-ss"
        return formatter.string(from: start) + ".aura"
    }
}

extension UTType {
    static let auraSession = UTType(exportedAs: "pl.nyderek.aura.session")
}

@MainActor
final class SessionLogger: ObservableObject {
    @Published private(set) var revision = 0
    @Published private(set) var sessions: [AuraSession] = []
    @Published var selection: UUID?
    @Published private(set) var live: AuraSession?

    private var frameCounter = 0
    private var lastFrame = Date.distantPast
    private var lastSaved = Date.distantPast
    private var energyStart: Double?
    private var energyIntegral = 0.0
    private var lastPosition: SIMD3<Double>?
    private var faultLatched = false
    private var timer: AnyCancellable?
    private var termination: AnyCancellable?

    static var folder: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("Aura/Sessions", isDirectory: true)
    }

    init() {
        loadAll()
        timer = Timer.publish(every: 1, on: .main, in: .common).autoconnect().sink { [weak self] now in
            guard let self, self.live != nil else { return }
            if now.timeIntervalSince(self.lastFrame) > 5 { self.finish() }
            else if now.timeIntervalSince(self.lastSaved) > 30 { self.save() }
            self.revision &+= 1
        }
        termination = NotificationCenter.default.publisher(for: NSApplication.willTerminateNotification)
            .sink { [weak self] _ in MainActor.assumeIsolated { self?.finish() } }
    }

    // 50 Hz from the network path; one sample is kept every 200 ms.
    func ingest(power: AuraPower, state: AuraRobotState, attitude: AuraAttitude,
                legs: [AuraRobotLegTelemetry], route: RouteRecorder) {
        let now = Date()
        lastFrame = now
        if live == nil {
            live = AuraSession(start: now, end: now)
            energyStart = nil
            energyIntegral = 0
            lastPosition = nil
            frameCounter = 0
            lastSaved = now
        }
        frameCounter += 1
        guard frameCounter % 10 == 0, var session = live else { return }
        let t = now.timeIntervalSince(session.start)
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

        var series = session.series
        series.t.append(t)
        series.voltage.append(power.servoBusVoltage)
        series.current.append(power.current)
        series.power.append(power.power)
        series.speed.append(route.speed)
        series.x.append(position.x); series.y.append(position.y); series.z.append(position.z)
        series.heading.append(route.heading)
        series.roll.append(route.roll); series.pitch.append(route.pitch)
        series.servoTemperature.append(temperature)
        series.mode.append(state.selectedGait)
        series.armed.append(state.armed)

        session.summary = s
        session.series = series
        session.end = now
        live = session
    }

    // Ends the live session: kept if it lasted 15 s or more.
    func finish() {
        guard let session = live else { return }
        if session.summary.duration >= 15 { save() }
        live = nil
        loadAll()
    }

    func save() {
        guard let session = live else { return }
        lastSaved = Date()
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
        let files = (try? FileManager.default.contentsOfDirectory(at: Self.folder, includingPropertiesForKeys: nil)) ?? []
        sessions = files.filter { $0.pathExtension == "aura" }
            .compactMap { try? decoder.decode(AuraSession.self, from: Data(contentsOf: $0)) }
            .sorted { $0.start > $1.start }
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
