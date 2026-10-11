@preconcurrency import Network
import Combine
import Foundation
import SwiftUI

struct AuraPower {
    var uptimeMilliseconds: UInt32 = 0
    var servoBusVoltage = 0.0
    var current = 0.0
    var power = 0.0
    var inputVoltage = 0.0
    var inaAvailable = false
    var vinAvailable = false
    var imuAvailable = false
    var imuIdentity: UInt8 = 0
    var servoConsumedMilliampHours = 0.0
}

struct AuraIMU {
    var sampleCount: UInt32 = 0
    var acceleration = [0.0, 0.0, 0.0]
    var gyroscope = [0.0, 0.0, 0.0]
    var temperature = 0.0
}

// Filtered roll/pitch emitted by Aura. The command switch only persists a
// commissioning setting while disarmed; it does not arm or send a target.
struct AuraAttitude: Equatable {
    var sampleCount: UInt32 = 0
    var valid = false
    var enabled = false
    var active = false
    var referenceCaptured = false
    var referencePending = false
    var rollDegrees = 0.0
    var pitchDegrees = 0.0
    var referenceRollDegrees = 0.0
    var referencePitchDegrees = 0.0
    var correctionRollDegrees = 0.0
    var correctionPitchDegrees = 0.0
    var i2cAddress: UInt8 = 0
    var maximumCorrectionDegrees = 7.0
}

// `expectedMask` comes from the gait phase; `detectedMask` is the fused
// estimate from servo effort, tracking residual and MPU6050 impact. Aura's
// firmware owns all physical decisions; this is only its read-only view.
// A percentage is confidence, not a calibrated force measurement.
struct AuraFootContacts: Equatable {
    var feedbackAvailable = false
    var controlActive = false
    var touchdownWaiting = false
    var expectedMask: UInt8 = 0
    var detectedMask: UInt8 = 0
    var earlyTouchdownMask: UInt8 = 0
    var confidence = [Int](repeating: 0, count: 4)

    func isExpected(_ leg: Int) -> Bool { expectedMask & (1 << leg) != 0 }
    func isDetected(_ leg: Int) -> Bool { detectedMask & (1 << leg) != 0 }
    func isEarlyTouchdown(_ leg: Int) -> Bool { earlyTouchdownMask & (1 << leg) != 0 }
}

struct CurrentPoint: Identifiable {
    let id = UUID()
    let time: Date
    let amperes: Double
}

struct AuraToFSensor: Identifiable {
    let id: Int
    var address: Int
    var enabled = true
    var timingBudget = 50
    var intermeasurement = 0
    var offset = 0
    var present = false
    var ranging = false
    var valid = false
    var rangeStatus = 255
    var distance = 0
    var signalPerSpad = 0
    var ambientPerSpad = 0
    var activeSpads = 0
    var sigma = 0
    var sampleCount: UInt32 = 0
    var xtalkKcps = 0
    var signalThresholdKcps = 1024
    var sigmaThresholdMm = 40
    var detectionEnabled = false
    var detectionLowMm = 100
    var detectionHighMm = 300
    var detectionWindow = 3
    var error: UInt32 = 0

    var portNumber: Int { id + 1 }
    var xshutGPIO: Int { id == 0 ? 14 : 15 }
}

enum ToFDetectionWindow: UInt8, CaseIterable, Identifiable {
    case below = 0
    case above = 1
    case outside = 2
    case inside = 3

    var id: UInt8 { rawValue }
    var title: String {
        switch self {
        case .below: "Below low threshold"
        case .above: "Above high threshold"
        case .outside: "Outside window"
        case .inside: "Inside window"
        }
    }
}

enum ServoOperatingMode: UInt8, CaseIterable, Identifiable {
    case servo = 0
    case motor = 1

    var id: UInt8 { rawValue }
    var title: String { self == .servo ? "Servo" : "Motor" }
}

enum DualSenseConnectionState: UInt8 {
    case off = 0
    case ready = 1
    case scanning = 2
    case connecting = 3
    case connected = 4
    case error = 5

    var title: String {
        switch self {
        case .off: "Bluetooth off"
        case .ready: "Ready"
        case .scanning: "Scanning…"
        case .connecting: "Connecting…"
        case .connected: "Connected"
        case .error: "Connection error"
        }
    }
}

struct AuraDualSenseDevice: Identifiable, Equatable {
    var address: [UInt8]
    var name: String
    var rssi: Int

    var id: String { address.map { String(format: "%02X", $0) }.joined(separator: ":") }
    var displayName: String { name.isEmpty ? "Unknown device" : name }
    var signalText: String { rssi == 0 ? "RSSI —" : "\(rssi) dBm" }
}

struct AuraDualSenseButton: Identifiable {
    let title: String
    let pressed: Bool
    var id: String { title }
}

struct AuraDualSense {
    var state = DualSenseConnectionState.off
    var hasSavedController = false
    var hasInput = false
    var battery: Int?
    var address = [UInt8](repeating: 0, count: 6)
    var vendorID = 0
    var productID = 0
    var reportID = 0
    var reportLength = 0
    var leftX = 128
    var leftY = 128
    var rightX = 128
    var rightY = 128
    var leftTrigger = 0
    var rightTrigger = 0
    var buttons = [0, 0, 0]
    var sampleCount: UInt32 = 0

    var isConnected: Bool { state == .connected }
    var addressText: String {
        address.map { String(format: "%02X", $0) }.joined(separator: ":")
    }
    var leftPosition: CGPoint {
        CGPoint(x: Double(leftX) / 255.0, y: Double(leftY) / 255.0)
    }
    var rightPosition: CGPoint {
        CGPoint(x: Double(rightX) / 255.0, y: Double(rightY) / 255.0)
    }
    var buttonStates: [AuraDualSenseButton] {
        let first = buttons[0]
        let second = buttons[1]
        let third = buttons[2]
        let hat = first & 0x0f
        func hatContains(_ values: [Int]) -> Bool { values.contains(hat) }
        return [
            AuraDualSenseButton(title: "↑", pressed: hatContains([0, 1, 7])),
            AuraDualSenseButton(title: "↓", pressed: hatContains([3, 4, 5])),
            AuraDualSenseButton(title: "←", pressed: hatContains([5, 6, 7])),
            AuraDualSenseButton(title: "→", pressed: hatContains([1, 2, 3])),
            AuraDualSenseButton(title: "□", pressed: first & 0x10 != 0),
            AuraDualSenseButton(title: "×", pressed: first & 0x20 != 0),
            AuraDualSenseButton(title: "○", pressed: first & 0x40 != 0),
            AuraDualSenseButton(title: "△", pressed: first & 0x80 != 0),
            AuraDualSenseButton(title: "L1", pressed: second & 0x01 != 0),
            AuraDualSenseButton(title: "R1", pressed: second & 0x02 != 0),
            AuraDualSenseButton(title: "L2", pressed: second & 0x04 != 0),
            AuraDualSenseButton(title: "R2", pressed: second & 0x08 != 0),
            AuraDualSenseButton(title: "Create", pressed: second & 0x10 != 0),
            AuraDualSenseButton(title: "Options", pressed: second & 0x20 != 0),
            AuraDualSenseButton(title: "L3", pressed: second & 0x40 != 0),
            AuraDualSenseButton(title: "R3", pressed: second & 0x80 != 0),
            AuraDualSenseButton(title: "PS", pressed: third & 0x01 != 0),
            AuraDualSenseButton(title: "Touch", pressed: third & 0x02 != 0),
            AuraDualSenseButton(title: "Mic", pressed: third & 0x04 != 0),
        ]
    }
}


struct AuraGaitProfile: Equatable, Identifiable {
    let gait: Int
    var strideMm: Int
    var stepHeightMm: Int
    var frequencyCentiHz: Int
    var dutyPercent: Int

    var id: Int { gait }
    var frequencyHz: Double { Double(frequencyCentiHz) / 100.0 }

    var title: String {
        switch gait {
        case 1: "Mode 4 · Trot"
        case 2: "Crawl · 1 leg"
        case 3: "Mode 3 · Run"
        case 4: "Climb · 1 leg"
        case 5: "Tripod · experimental"
        default: "Unknown gait"
        }
    }

    var isSingleFoot: Bool { gait == 2 || gait == 4 || gait == 5 }

    // Pad mode 4 (Trot) stops only on a tick with all four feet down, so its
    // duty must exceed 50 % and its frequency stay within (duty - 50) x 0.5 Hz,
    // the rule main/robot_gait.c enforces.
    var dutyRange: ClosedRange<Int> { isSingleFoot ? 76...90 : gait == 1 ? 51...85 : 45...85 }
    var frequencyLimitCentiHz: Int {
        isSingleFoot ? 120 : gait == 1 ? min(300, max(15, (dutyPercent - 50) * 50)) : 300
    }

    static let defaults: [AuraGaitProfile] = [
        .init(gait: 1, strideMm: 80, stepHeightMm: 58, frequencyCentiHz: 140, dutyPercent: 58),
        .init(gait: 2, strideMm: 50, stepHeightMm: 30, frequencyCentiHz: 65, dutyPercent: 82),
        .init(gait: 3, strideMm: 90, stepHeightMm: 72, frequencyCentiHz: 190, dutyPercent: 52),
        .init(gait: 4, strideMm: 50, stepHeightMm: 82, frequencyCentiHz: 65, dutyPercent: 82),
        .init(gait: 5, strideMm: 35, stepHeightMm: 40, frequencyCentiHz: 65, dutyPercent: 78),
    ]
}

// These values tune only the two smooth planners that run on Aura while the
// robot walks.  They are not servo speed/acceleration settings and cannot be
// sent while the robot is armed.
struct AuraMotionTuning: Equatable {
    var movingAttitudeGainPermille = 450
    var comMaximumVelocityMmPerSecond = 85
    var comMaximumAccelerationMmPerSecond2 = 360

    var movingAttitudeGain: Double { Double(movingAttitudeGainPermille) / 1000.0 }
}



struct AuraRobotLegTelemetry: Equatable, Identifiable {
    let id: Int
    var configured = [false, false, false]
    var present = [false, false, false]
    var moving = [false, false, false]
    var targetDegrees = [0.0, 0.0, 0.0]
    var measuredDegrees = [0.0, 0.0, 0.0]
    var temperature = [0, 0, 0]
}

// This is emitted by Aura after a calibration capture. The desktop uses it to
// mirror the exact NVS record that will be used by the autonomous gait.
struct AuraCalibrationAxisConfiguration: Equatable {
    var leg: Int
    var axis: Int
    var servoID: Int
    var direction: Int
    var centerTick: Int
    var minimumDegrees: Double
    var maximumDegrees: Double
    var speedLimitRaw: Int
    var acceleration: Int
}

// This is cumulative on purpose. Three calibration replies can arrive in one
// run-loop turn, so a single last-axis event is not enough for SwiftUI to know
// that the whole reference pose was captured.
struct AuraCalibrationZeroProgress: Equatable {
    let leg: Int
    var completedAxes: Set<Int>
    var isComplete: Bool { completedAxes == Set(0..<3) }
}

// The 50 Hz robot sample is a single transport product. SceneKit consumes it
// directly; publishing it separately from the rest of the application avoids
// rebuilding unrelated SwiftUI controls for every control tick.
struct AuraRobotFrame {
    let state: AuraRobotState
    let legs: [AuraRobotLegTelemetry]
    let controller: AuraDualSense
    let contacts: AuraFootContacts
}

final class AuraRobotRenderChannel {
    let frames = PassthroughSubject<AuraRobotFrame, Never>()

    func publish(_ frame: AuraRobotFrame) {
        frames.send(frame)
    }
}

@MainActor
final class AuraConnection: NSObject, ObservableObject {
    // Bonjour remains the primary discovery method. Some macOS sessions
    // temporarily lose their .local resolver even though Aura is reachable
    // on the same WLAN; this is the current DHCP address used only as a
    // bounded final fallback in that situation.
    private static let lastKnownBoardAddress = "192.168.55.11"
    private static let bridgePort: NWEndpoint.Port = 4243
    @Published var connectionText = "Looking for Aura on Wi-Fi…"
    @Published var isConnected = false
    @Published var busy = false
    // High-rate telemetry is committed atomically at the end of a board
    // frame. One explicit objectWillChange below replaces six independent
    // @Published notifications for the same 20 ms sample.
    var power = AuraPower()
    var imu = AuraIMU()
    var attitude = AuraAttitude()
    var footContacts = AuraFootContacts()
    var currentHistory: [CurrentPoint] = []
    @Published var servoIDs: [Int] = []
    @Published var selectedServo = 1
    @Published var feedback: ServoFeedback?
    @Published var torqueEnabled = false
    @Published var operatingMode = ServoOperatingMode.servo
    @Published var angle = 180.0
    @Published var speed = 1000.0
    @Published var motorSpeed = 500.0
    @Published var acceleration = 30.0
    @Published var currentID = 1
    @Published var newID = 2
    @Published var pixelCount = 8
    // Status strip brightness in % as Aura last reported it (command 0x2e).
    @Published var ledBrightness = 20.0
    private var ledBrightnessWrite = false
    @Published var pixelColor = Color.green
    @Published var tofReferenceDistance = 200
    var tofSensors = [
        AuraToFSensor(id: 0, address: 0x30),
        AuraToFSensor(id: 1, address: 0x31),
    ]
    var dualSense = AuraDualSense()
    @Published var dualSenseDevices: [AuraDualSenseDevice] = []
    var robotState = AuraRobotState()
    // Board-owned profiles. The desktop receives and configures them but
    // never generates physical targets from them.
    @Published var gaitProfiles = AuraGaitProfile.defaults
    private var profileEditHold: [Int: Date] = [:]
    @Published var motionTuning = AuraMotionTuning()
    var robotLegTelemetry = (0..<4).map { AuraRobotLegTelemetry(id: $0) }
    @Published var calibrationAxisConfiguration: AuraCalibrationAxisConfiguration?
    // Complete read-only mirror of the axis records currently stored in Aura.
    // It is used to retain a desktop backup even if the board later fails.
    @Published var robotAxisBackup: [AuraCalibrationAxisConfiguration] = []
    @Published var calibrationZeroProgress: AuraCalibrationZeroProgress?
    @Published var calibrationArmedLeg: Int?
    // A read-only encoder stream for all twelve axes. It is separate from
    // both the moving one-leg test and every torque command.
    @Published var calibrationLivePreviewActive = false
    @Published var message = ""
    let robotRenderChannel = AuraRobotRenderChannel()
    let history = TelemetryHistory()
    let route = RouteRecorder()
    let sessions = SessionLogger()

    var detectedToFSensorIndices: [Int] {
        tofSensors.indices.filter { tofSensors[$0].present }
    }

    private var browser: NWBrowser?
    private var connection: NWConnection?
    private var networkBridge: Process?
    private var receiveBuffer = Data()
    private var receivedFirstFrame = false
    private var requestedOperation: UInt8?
    private var retry: DispatchWorkItem?
    private var robotConfigurationQueue: [RobotAxisConfiguration] = []
    private var robotAxisBackupBySlot: [Int: AuraCalibrationAxisConfiguration] = [:]
    private var robotConfigurationTotal = 0
    private var robotConfigurationCompleted = 0
    private var calibrationZeroQueue: [(leg: Int, axis: Int, referenceCdeg: Int16)] = []
    // A zero capture writes no position command.  Keeping its exact in-flight
    // axis lets the app distinguish a late result from the result of the next
    // axis in the three-axis sequence.
    private var calibrationZeroInFlight: (leg: Int, axis: Int, referenceCdeg: Int16)?
    private var calibrationZeroConfigFallback: DispatchWorkItem?
    // A one-axis re-capture is distinct from the normal three-axis sequence.
    // It repairs a reference after an interrupted direction test.
    private var pendingCalibrationReference: (leg: Int, axis: Int)?
    private var pendingCalibrationLimit: (leg: Int, axis: Int, maximum: Bool)?
    private var pendingCalibrationTorque: (leg: Int, enabled: Bool)?
    private var pendingCalibrationPreview = false
    private var pendingRobotState = AuraRobotState()
    private var pendingRobotLegTelemetry = (0..<4).map { AuraRobotLegTelemetry(id: $0) }
    // The board sends seven compact frames per 20 ms control cycle. Keep them
    // off @Published properties until the final leg frame arrives, so one
    // control sample produces one SwiftUI invalidation instead of 7–10.
    private var pendingPower = AuraPower()
    private var pendingIMU = AuraIMU()
    private var pendingAttitude = AuraAttitude()
    private var pendingFootContacts = AuraFootContacts()
    private var pendingHealth = AuraHealth()
    private var healthReference: (time: Double, beats: [Int])?
    @Published private(set) var health = AuraHealth()
    // Silence watchdog: a link that stays "connected" without data (Aura
    // restarted or Wi-Fi dropped) is closed and opened again.
    private var lastTelemetryUptime = 0.0
    private var silenceTimer: Timer?
    private var pendingToFSensors = [AuraToFSensor(id: 0, address: 0x30), AuraToFSensor(id: 1, address: 0x31)]
    private var pendingDualSense = AuraDualSense()
    private var pendingChartPoint: CurrentPoint?
    // The board sends a complete robot snapshot 10 times a second. SceneKit,
    // the route, the history and the session log take every one; the SwiftUI
    // pages (numbers, tables) are refreshed 5 times a second, which reads the
    // same and halves the layout work.
    private var lastUICommitUptime = 0.0
    private let uiCommitInterval = 0.18
    private var lastChartSampleUptime: UInt32 = 0

    override init() {
        super.init()
        startNetworkBridge()
        startScan()
        silenceTimer = Timer.scheduledTimer(withTimeInterval: 1, repeats: true) { [weak self] _ in
            MainActor.assumeIsolated {
                guard let self, self.isConnected, self.lastTelemetryUptime > 0 else { return }
                let quiet = ProcessInfo.processInfo.systemUptime - self.lastTelemetryUptime
                guard quiet > 3 else { return }
                self.lastTelemetryUptime = 0
                self.message = "No data from Aura for 3 s · reconnecting"
                self.reconnect()
            }
        }
    }

    private func startNetworkBridge() {
        guard networkBridge == nil,
              let script = Bundle.main.url(forResource: "aura_network_bridge", withExtension: "py") else {
            return
        }
        let task = Process()
        task.executableURL = URL(fileURLWithPath: "/usr/bin/python3")
        task.arguments = [script.path, "aura-main-board.local", "4242", "\(Self.bridgePort.rawValue)", Self.lastKnownBoardAddress]
        task.standardOutput = FileHandle.nullDevice
        task.standardError = FileHandle.nullDevice
        do {
            try task.run()
            networkBridge = task
        } catch {
            // Native Bonjour remains available on Macs where local-network
            // privacy is healthy. The caller will use it if the bridge fails.
            networkBridge = nil
        }
    }

    func reconnect() {
        retry?.cancel()
        browser?.cancel(); browser = nil
        let old = connection; connection = nil; old?.cancel()
        receiveBuffer.removeAll()
        receivedFirstFrame = false
        isConnected = false; busy = false; requestedOperation = nil
        calibrationLivePreviewActive = false
        startScan()
    }

    private func startScan() {
        guard browser == nil, connection == nil else { return }
        // macOS 26 can incorrectly mark an independently built desktop app
        // as "Local network prohibited". The bundled relay gives this app a
        // loopback-only path while keeping the firmware protocol unchanged.
        // The relay can end (an old copy holding the port, or a crash); start
        // a fresh one instead of falling back to the blocked direct path.
        if networkBridge?.isRunning != true {
            networkBridge = nil
            startNetworkBridge()
        }
        if networkBridge?.isRunning == true {
            connect(to: .hostPort(host: "127.0.0.1", port: Self.bridgePort))
            return
        }
        connectionText = "Looking for Aura Main Board on Wi-Fi…"
        let search = NWBrowser(for: .bonjour(type: "_aura._tcp", domain: nil), using: .tcp)
        browser = search
        search.browseResultsChangedHandler = { [weak self, weak search] results, _ in
            Task { @MainActor in
                guard let self, let search, self.browser === search,
                      self.connection == nil, let result = results.first else { return }
                self.connect(to: result.endpoint)
            }
        }
        search.stateUpdateHandler = { [weak self, weak search] state in
            Task { @MainActor in
                guard let self, let search, self.browser === search else { return }
                if case .failed(let error) = state {
                    self.message = error.localizedDescription
                    self.scheduleRetry()
                }
            }
        }
        search.start(queue: .main)
        // Bonjour can remain silent after the ESP restarts even though the
        // hostname is immediately reachable.  Do not leave the control UI
        // waiting indefinitely for a browse callback in that case.
        DispatchQueue.main.asyncAfter(deadline: .now() + 2) { [weak self, weak search] in
            guard let self, let search, self.browser === search, self.connection == nil else { return }
            self.browser = nil
            search.cancel()
            self.connect(to: .hostPort(host: "aura-main-board.local", port: 4242))
        }
    }

    private func connect(to endpoint: NWEndpoint) {
        browser?.cancel(); browser = nil
        let link = NWConnection(to: endpoint, using: .tcp)
        NSLog("Aura network connecting: %@", String(describing: endpoint))
        connection = link
        connectionText = "Connecting to Aura Main Board over Wi-Fi…"
        link.stateUpdateHandler = { [weak self, weak link] state in
            Task { @MainActor in
                guard let self, let link, self.connection === link else { return }
                NSLog("Aura network state: %@", String(describing: state))
                switch state {
                case .ready:
                    self.isConnected = true
                    self.busy = false
                    self.connectionText = "Connected to Aura Main Board (Wi-Fi)"
                    self.receive(on: link)
                    self.send([0x08], operation: 0x08)
                    self.refreshToFConfigurations()
                    // Read the configuration that is already in Aura. This is
                    // intentionally delayed behind the normal connection
                    // setup and has no motion or NVS side effect.
                    DispatchQueue.main.asyncAfter(deadline: .now() + 0.7) { [weak self, weak link] in
                        guard let self, let link, self.connection === link else { return }
                        self.requestRobotConfiguration()
                    }
                case .failed(let error), .waiting(let error):
                    self.message = error.localizedDescription
                    self.scheduleRetry()
                default: break
                }
            }
        }
        link.start(queue: .main)
        // A Bonjour service may resolve slowly on Macs with multiple interfaces.
        // Fall back to the hostname first, then to the last confirmed LAN
        // address when macOS itself reports .local as unavailable.
        DispatchQueue.main.asyncAfter(deadline: .now() + 6) { [weak self, weak link] in
            guard let self, let link, self.connection === link, !self.isConnected else { return }
            if case .service = endpoint {
                self.connection = nil
                link.cancel()
                self.connect(to: .hostPort(host: "aura-main-board.local", port: 4242))
            } else if case .hostPort(let host, _) = endpoint,
                      String(describing: host) != Self.lastKnownBoardAddress {
                self.connection = nil
                link.cancel()
                self.connect(to: .hostPort(host: NWEndpoint.Host(Self.lastKnownBoardAddress), port: 4242))
            } else {
                self.scheduleRetry()
            }
        }
    }

    private func scheduleRetry() {
        healthReference = nil
        pendingHealth.rates = [Double](repeating: -1, count: AuraHealth.tasks.count)
        let interruptedZeroCapture = !calibrationZeroQueue.isEmpty || calibrationZeroInFlight != nil
        calibrationZeroConfigFallback?.cancel()
        calibrationZeroConfigFallback = nil
        calibrationZeroQueue.removeAll()
        calibrationZeroInFlight = nil
        pendingCalibrationReference = nil
        browser?.cancel(); browser = nil
        let old = connection; connection = nil; old?.cancel()
        receiveBuffer.removeAll()
        receivedFirstFrame = false
        isConnected = false; busy = false; requestedOperation = nil
        calibrationLivePreviewActive = false
        connectionText = "Disconnected · looking for the board…"
        if interruptedZeroCapture {
            message = "Connection lost while saving the reference. Nothing was armed; save this leg again after reconnecting."
        }
        retry?.cancel()
        let work = DispatchWorkItem { [weak self] in self?.startScan() }
        retry = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 3, execute: work)
    }

    private func receive(on link: NWConnection) {
        link.receive(minimumIncompleteLength: 1, maximumLength: 4096) { [weak self, weak link] data, _, complete, error in
            Task { @MainActor in
                guard let self, let link, self.connection === link else { return }
                if let data {
                    self.receiveBuffer.append(data)
                    while self.receiveBuffer.count >= 2 {
                        let header = Array(self.receiveBuffer.prefix(2))
                        let length = Int(header[1])
                        guard (header[0] == 1 || header[0] == 2), (1...20).contains(length) else {
                            self.scheduleRetry(); return
                        }
                        guard self.receiveBuffer.count >= length + 2 else { break }
                        let payload = Data(self.receiveBuffer.dropFirst(2).prefix(length))
                        if !self.receivedFirstFrame {
                            self.receivedFirstFrame = true
                            NSLog("Aura Wi-Fi telemetry received")
                        }
                        self.receiveBuffer.removeFirst(length + 2)
                        if header[0] == 1 { self.parseTelemetry(payload) }
                        else { self.parseEvent(payload) }
                    }
                }
                if complete || error != nil { self.scheduleRetry() }
                else { self.receive(on: link) }
            }
        }
    }

    private func send(_ bytes: [UInt8], operation: UInt8) {
        guard isConnected, let link = connection, !bytes.isEmpty, bytes.count <= 20 else {
            message = "No Wi-Fi connection to Aura Main Board."
            busy = false
            return
        }
        busy = operation != 0x08
        requestedOperation = operation
        link.send(content: Data([0, UInt8(bytes.count)] + bytes), completion: .contentProcessed { [weak self, weak link] error in
            Task { @MainActor in
                guard let self, let link, self.connection === link, let error else { return }
                self.message = error.localizedDescription
                self.scheduleRetry()
            }
        })
    }

    func scanServos() {
        servoIDs = []
        send([0x01], operation: 0x01)
    }

    func refreshServo() { send([0x02, UInt8(selectedServo)], operation: 0x02) }

    func setTorque(_ enabled: Bool) {
        send([0x03, UInt8(selectedServo), enabled ? 1 : 0], operation: 0x03)
        torqueEnabled = enabled
    }

    func moveServo() {
        let position = UInt16(max(0, min(4095, Int((angle / 360.0 * 4095.0).rounded()))))
        let selectedSpeed = UInt16(max(0, min(3400, Int(speed.rounded()))))
        send([0x04, UInt8(selectedServo), UInt8(position & 0xff), UInt8(position >> 8),
              UInt8(selectedSpeed & 0xff), UInt8(selectedSpeed >> 8),
              UInt8(max(0, min(254, Int(acceleration.rounded()))))], operation: 0x04)
    }

    func setOperatingMode() {
        send([0x09, UInt8(selectedServo), operatingMode.rawValue], operation: 0x09)
    }

    func runMotor() {
        let requestedSpeed = Int16(max(-3400, min(3400, Int(motorSpeed.rounded()))))
        let raw = UInt16(bitPattern: requestedSpeed)
        send([0x0a, UInt8(selectedServo), UInt8(raw & 0xff), UInt8(raw >> 8),
              UInt8(max(0, min(254, Int(acceleration.rounded()))))], operation: 0x0a)
    }

    func stopMotor() {
        motorSpeed = 0
        send([0x0a, UInt8(selectedServo), 0, 0,
              UInt8(max(0, min(254, Int(acceleration.rounded()))))], operation: 0x0a)
    }

    func calibrateCenter() { send([0x05, UInt8(selectedServo)], operation: 0x05) }

    func assignID() {
        send([0x06, UInt8(currentID), UInt8(newID)], operation: 0x06)
    }

    func setPixels(mode: UInt8) {
        let color = NSColor(pixelColor).usingColorSpace(.deviceRGB) ?? .green
        send([0x07, mode, UInt8(pixelCount),
              UInt8((color.redComponent * 255).rounded()),
              UInt8((color.greenComponent * 255).rounded()),
              UInt8((color.blueComponent * 255).rounded())], operation: 0x07)
    }

    // Re-zero Aura's position estimate (RAM only, no settings involved).
    func rezeroEstimator() {
        send([0x2f], operation: 0x2f)
    }

    func readLEDBrightness() {
        ledBrightnessWrite = false
        send([0x2e], operation: 0x2e)
    }

    func saveLEDBrightness() {
        ledBrightnessWrite = true
        send([0x2e, UInt8(max(0, min(100, ledBrightness.rounded())))], operation: 0x2e)
    }

    func refreshToFConfigurations() {
        send([0x0c], operation: 0x0c)
    }

    func reinitializeToF() {
        send([0x0d], operation: 0x0d)
    }

    func configureToF(index: Int) {
        guard tofSensors.indices.contains(index) else { return }
        let sensor = tofSensors[index]
        let otherAddress = tofSensors[index == 0 ? 1 : 0].address
        guard (0x08...0x77).contains(sensor.address),
              sensor.address != 0x29, sensor.address != 0x40,
              sensor.address != otherAddress else {
            message = "Address must be unique, within 0x08…0x77, and not 0x29 or 0x40."
            return
        }
        guard (10...200).contains(sensor.timingBudget),
              sensor.intermeasurement == 0 || sensor.intermeasurement > sensor.timingBudget else {
            message = "Timing budget: 10…200 ms. Period must be 0 or longer than the budget."
            return
        }
        let budget = UInt16(sensor.timingBudget)
        let period = UInt16(sensor.intermeasurement)
        send([0x0b, UInt8(index), UInt8(sensor.address), sensor.enabled ? 1 : 0,
              UInt8(budget & 0xff), UInt8(budget >> 8),
              UInt8(period & 0xff), UInt8(period >> 8)], operation: 0x0b)
    }

    func calibrateToF(index: Int, referenceDistance: Int) {
        guard tofSensors.indices.contains(index) else { return }
        let sensor = tofSensors[index]
        guard sensor.valid else {
            message = "Calibration needs a valid reading. Place a matte target at a known distance."
            return
        }
        let newOffset = sensor.offset + referenceDistance - sensor.distance
        guard (-1024...1023).contains(newOffset) else {
            message = "Computed offset \(newOffset) mm is outside −1024…1023 mm."
            return
        }
        let rawOffset = UInt16(bitPattern: Int16(newOffset))
        send([0x0e, UInt8(index), UInt8(rawOffset & 0xff), UInt8(rawOffset >> 8)], operation: 0x0e)
    }

    func resetToFCalibration(index: Int) {
        send([0x0e, UInt8(index), 0, 0], operation: 0x0e)
    }

    func configureToFTuning(index: Int) {
        guard tofSensors.indices.contains(index) else { return }
        let sensor = tofSensors[index]
        guard (0...128).contains(sensor.xtalkKcps),
              (0...16_384).contains(sensor.signalThresholdKcps),
              (0...16_383).contains(sensor.sigmaThresholdMm),
              (0...3).contains(sensor.detectionWindow) else {
            message = "Optical settings are outside the VL53L4CD range."
            return
        }
        if sensor.detectionEnabled {
            guard sensor.intermeasurement > sensor.timingBudget else {
                message = "Threshold detection needs periodic mode: period longer than the timing budget."
                return
            }
            guard sensor.detectionLowMm <= sensor.detectionHighMm else {
                message = "Low threshold cannot be above the high threshold."
                return
            }
        }
        let xtalk = UInt16(sensor.xtalkKcps)
        let signal = UInt16(sensor.signalThresholdKcps)
        let sigma = UInt16(sensor.sigmaThresholdMm)
        let low = UInt16(max(0, min(65_535, sensor.detectionLowMm)))
        let high = UInt16(max(0, min(65_535, sensor.detectionHighMm)))
        send([0x10, UInt8(index),
              UInt8(xtalk & 0xff), UInt8(xtalk >> 8),
              UInt8(signal & 0xff), UInt8(signal >> 8),
              UInt8(sigma & 0xff), UInt8(sigma >> 8),
              UInt8(low & 0xff), UInt8(low >> 8),
              UInt8(high & 0xff), UInt8(high >> 8),
              UInt8(sensor.detectionWindow), sensor.detectionEnabled ? 1 : 0], operation: 0x10)
    }

    func resetToFTuning(index: Int) {
        guard tofSensors.indices.contains(index) else { return }
        tofSensors[index].xtalkKcps = 0
        tofSensors[index].signalThresholdKcps = 1024
        tofSensors[index].sigmaThresholdMm = 40
        tofSensors[index].detectionEnabled = false
        tofSensors[index].detectionLowMm = 100
        tofSensors[index].detectionHighMm = 300
        tofSensors[index].detectionWindow = Int(ToFDetectionWindow.inside.rawValue)
        configureToFTuning(index: index)
    }

    func updateToFTemperature(index: Int) {
        send([0x11, UInt8(index)], operation: 0x11)
    }

    func reinitializeIMU() {
        send([0x0f], operation: 0x0f)
    }

    func pairDualSense() {
        dualSenseDevices = []
        send([0x12], operation: 0x12)
    }
    func connectDualSense(_ device: AuraDualSenseDevice) {
        send([0x16] + device.address, operation: 0x16)
    }
    func disconnectDualSense() { send([0x13], operation: 0x13) }
    func connectSavedDualSense() { send([0x14], operation: 0x14) }
    func forgetDualSense() { send([0x15], operation: 0x15) }

    func configureRobotAxis(_ axis: RobotAxisConfiguration) {
        sendRobotAxisConfiguration(axis)
    }

    func flipCalibrationDirectionAtReference(leg: Int, axis: Int, referenceDegrees: Double) {
        guard isConnected else {
            message = "No Wi-Fi connection to Aura Main Board."
            return
        }
        guard (0..<4).contains(leg), (0..<3).contains(axis) else { return }
        let cdeg = Int16(max(-18000, min(18000, Int((referenceDegrees * 100).rounded()))))
        let raw = UInt16(bitPattern: cdeg)
        // Firmware releases torque, reads the current raw position and changes
        // only the mapping. Unlike the former config+retarget sequence, this
        // command has no position write at all.
        send([0x20, UInt8(leg), UInt8(axis), UInt8(raw & 0xff), UInt8(raw >> 8)], operation: 0x20)
    }

    func repairLegacyDirectionFlipAtReference(leg: Int, axis: Int, referenceDegrees: Double) {
        guard isConnected else {
            message = "No Wi-Fi connection to Aura Main Board."
            return
        }
        guard (0..<4).contains(leg), (0..<3).contains(axis) else { return }
        let cdeg = Int16(max(-18000, min(18000, Int((referenceDegrees * 100).rounded()))))
        let raw = UInt16(bitPattern: cdeg)
        // Recovery for the removed config+retarget behaviour. Retains the
        // current sign, releases torque and only rebuilds zero/limits from the
        // raw position where the user placed the physical joint.
        send([0x21, UInt8(leg), UInt8(axis), UInt8(raw & 0xff), UInt8(raw >> 8)], operation: 0x21)
    }

    // Legacy API retained only so a stale caller cannot send the removed
    // automatic reverse-and-move transaction.  Direction changes must use
    // flipCalibrationDirectionAtReference after manual placement.
    func reverseCalibrationDirectionProbe(leg: Int, axis: Int,
                                          referenceDegrees: Double, probeDegrees: Double = 12) {
        _ = (leg, axis, referenceDegrees, probeDegrees)
        message = "Automatic flip with motion was removed. Release the axis, set it to the reference by hand and use Save reversed direction · no motion."
    }

    private func sendRobotAxisConfiguration(_ axis: RobotAxisConfiguration) {
        let leg = axis.leg.wireIndex
        guard let joint = RobotAxisID.allCases.firstIndex(of: axis.axis) else { return }
        func encoded16(_ value: Int) -> [UInt8] {
            let raw = UInt16(bitPattern: Int16(max(Int(Int16.min), min(Int(Int16.max), value))))
            return [UInt8(raw & 0xff), UInt8(raw >> 8)]
        }
        let minimum = Int((axis.minimumDegrees * 100).rounded())
        let maximum = Int((axis.maximumDegrees * 100).rounded())
        let speed = UInt16(max(0, min(3400, axis.speedLimitRaw)))
        let id = axis.assigned ? UInt8(max(0, min(253, axis.servoID))) : 0xff
        let packet = [0x17, UInt8(leg), UInt8(joint), id,
                      axis.reversed ? 0xff : 1] +
            encoded16(axis.centerTick) + encoded16(minimum) + encoded16(maximum) +
            [UInt8(speed & 0xff), UInt8(speed >> 8), UInt8(max(0, min(254, axis.acceleration)))]
        send(packet, operation: 0x17)
    }

    func configureRobot(_ axes: [RobotAxisConfiguration]) {
        let assigned = axes.filter(\.assigned)
        guard Set(assigned.map(\.servoID)).count == assigned.count else {
            message = "Not saved: the same servo ID is assigned to more than one axis."
            return
        }
        guard !axes.isEmpty else { return }
        // The ESP command queue is deliberately short so it cannot become a
        // source of delayed movement commands.  Configuration is therefore
        // sent one axis at a time and the next record waits for the board's
        // result event.  The old burst of 12 frames could silently drop axes.
        robotConfigurationQueue = axes
        robotConfigurationTotal = axes.count
        robotConfigurationCompleted = 0
        sendNextRobotAxisConfiguration()
    }

    private func sendNextRobotAxisConfiguration() {
        guard isConnected else {
            robotConfigurationQueue.removeAll()
            message = "Connection to Aura lost while saving axes."
            return
        }
        guard !robotConfigurationQueue.isEmpty else {
            message = "Saved \(robotConfigurationCompleted) / \(robotConfigurationTotal) axes on Aura."
            robotConfigurationTotal = 0
            robotConfigurationCompleted = 0
            return
        }
        sendRobotAxisConfiguration(robotConfigurationQueue.removeFirst())
    }

    func setRobotArmed(_ armed: Bool) {
        send([0x18, armed ? 1 : 0], operation: 0x18)
    }

    // Acknowledges a latched physical-stall stop. This command cannot arm
    // torque or move a servo; arming remains a separate explicit action.
    func clearRobotSafetyFault() {
        send([0x24], operation: 0x24)
    }

    // Stores the next neutral stance on Aura. Firmware rejects this while
    // torque is on, so moving the UI slider cannot change a loaded leg.
    func setRobotBodyHeight(_ millimetres: Int) {
        guard !robotState.armed else {
            message = "Disarm Aura to change the stand height."
            return
        }
        let height = UInt16(max(120, min(250, millimetres)))
        send([0x25, UInt8(height & 0xff), UInt8(height >> 8)], operation: 0x25)
    }

    // The parameters are stored on Aura and accepted only before torque is
    // enabled. Editing these sliders therefore cannot change a walking leg.
    func setRobotSingleFootTuning(stride: Int, lift: Int, frequency: Double) {
        guard !robotState.armed else {
            message = "Disarm Aura to change step parameters."
            return
        }
        let safeStride = UInt16(max(20, min(75, stride)))
        let safeLift = UInt16(max(8, min(55, lift)))
        let safeFrequency = UInt16(max(30, min(55, Int((frequency * 100).rounded()))))
        send([0x26, UInt8(safeStride & 0xff), UInt8(safeStride >> 8),
              UInt8(safeLift & 0xff), UInt8(safeLift >> 8),
              UInt8(safeFrequency & 0xff), UInt8(safeFrequency >> 8)], operation: 0x26)
    }

    func gaitProfile(_ gait: Int) -> AuraGaitProfile {
        gaitProfiles.first(where: { $0.gait == gait }) ??
            AuraGaitProfile.defaults.first(where: { $0.gait == gait }) ??
            AuraGaitProfile.defaults[0]
    }

    // An edit in the app. Aura's once-a-second profile report is held off for
    // that gait for a moment, so it cannot pull a slider back mid-drag.
    func replaceGaitProfileLocally(_ profile: AuraGaitProfile) {
        guard let index = gaitProfiles.firstIndex(where: { $0.gait == profile.gait }) else { return }
        profileEditHold[profile.gait] = Date().addingTimeInterval(3)
        if gaitProfiles[index] != profile { gaitProfiles[index] = profile }
    }

    // Aura's stored profile, as it reports it. Unchanged values are not
    // written again, so a report does not redraw the views.
    private func applyBoardGaitProfile(_ profile: AuraGaitProfile) {
        guard let index = gaitProfiles.firstIndex(where: { $0.gait == profile.gait }) else { return }
        if let hold = profileEditHold[profile.gait], hold > Date() { return }
        if gaitProfiles[index] != profile { gaitProfiles[index] = profile }
    }

    // One profile update is stored in Aura's NVS while torque is disabled.
    // It sends no servo target and the planner reads it on Aura itself.
    func setRobotGaitProfile(_ profile: AuraGaitProfile) {
        guard !robotState.armed else {
            message = "Disarm Aura to change the gait profile."
            return
        }
        let stride = UInt16(max(15, min(300, profile.strideMm)))
        let lift = UInt16(max(5, min(100, profile.stepHeightMm)))
        let duty = UInt8(max(profile.dutyRange.lowerBound, min(profile.dutyRange.upperBound, profile.dutyPercent)))
        var bounded = profile
        bounded.dutyPercent = Int(duty)
        let frequency = UInt16(max(15, min(bounded.frequencyLimitCentiHz, profile.frequencyCentiHz)))
        let safe = AuraGaitProfile(gait: profile.gait, strideMm: Int(stride),
                                   stepHeightMm: Int(lift),
                                   frequencyCentiHz: Int(frequency),
                                   dutyPercent: Int(duty))
        replaceGaitProfileLocally(safe)
        send([0x2b, UInt8(profile.gait),
              UInt8(stride & 0xff), UInt8(stride >> 8),
              UInt8(lift & 0xff), UInt8(lift >> 8),
              UInt8(frequency & 0xff), UInt8(frequency >> 8), duty],
             operation: 0x2b)
    }

    // The board owns the resulting 50 Hz body motion. This setter only stores
    // its bounded tuning values for a later arm cycle.
    func setTripodWalking(_ enabled: Bool, excludedLeg: Int) {
        guard isConnected, !robotState.armed, robotState.balanceExtensionAvailable,
              (0..<4).contains(excludedLeg) else { return }
        send([0x2d, enabled ? 1 : 0, UInt8(excludedLeg)], operation: 0x2d)
    }

    func setRobotMotionTuning(_ tuning: AuraMotionTuning) {
        guard !robotState.armed else {
            message = "Disarm Aura to change motion tuning."
            return
        }
        let gain = UInt16(max(100, min(1000, tuning.movingAttitudeGainPermille)))
        let velocity = UInt16(max(20, min(160, tuning.comMaximumVelocityMmPerSecond)))
        let acceleration = UInt16(max(60, min(900, tuning.comMaximumAccelerationMmPerSecond2)))
        motionTuning = AuraMotionTuning(movingAttitudeGainPermille: Int(gain),
                                         comMaximumVelocityMmPerSecond: Int(velocity),
                                         comMaximumAccelerationMmPerSecond2: Int(acceleration))
        send([0x2c,
              UInt8(gain & 0xff), UInt8(gain >> 8),
              UInt8(velocity & 0xff), UInt8(velocity >> 8),
              UInt8(acceleration & 0xff), UInt8(acceleration >> 8)], operation: 0x2c)
    }

    // This is only persisted commissioning data for the static support
    // polygon. Firmware refuses it while torque is on and does not issue a
    // movement command when accepting it.
    func setRobotStaticBalance(comForward: Int, comLeft: Int, supportMargin: Int) {
        guard !robotState.armed else {
            message = "Disarm Aura to change step balance."
            return
        }
        let forward = Int16(max(-120, min(120, comForward)))
        let left = Int16(max(-120, min(120, comLeft)))
        let margin = UInt16(max(5, min(45, supportMargin)))
        let forwardBits = UInt16(bitPattern: forward)
        let leftBits = UInt16(bitPattern: left)
        send([0x27, UInt8(forwardBits & 0xff), UInt8(forwardBits >> 8),
              UInt8(leftBits & 0xff), UInt8(leftBits >> 8),
              UInt8(margin & 0xff), UInt8(margin >> 8)], operation: 0x27)
    }

    func setRobotLateralStance(_ millimetres: Int) {
        guard !robotState.armed else {
            message = "Disarm Aura to change foot spread."
            return
        }
        let stance = UInt16(max(0, min(80, millimetres)))
        send([0x28, UInt8(stance & 0xff), UInt8(stance >> 8)], operation: 0x28)
    }

    // Stores the MPU6050 body-attitude loop preference. Firmware rejects the
    // request while torque or calibration owns any axis, and this call never
    // writes a servo target.
    func setAttitudeBalance(enabled: Bool) {
        guard !robotState.armed else {
            message = "Disarm Aura to change posture correction."
            return
        }
        send([0x29, enabled ? 1 : 0], operation: 0x29)
    }

    // This only persists a controller ceiling while Aura is disarmed. It does
    // not arm torque or send a pose to any joint.
    func setAttitudeCorrectionLimit(degrees: Double) {
        guard !robotState.armed else {
            message = "Disarm Aura to change the IMU correction limit."
            return
        }
        let centidegrees = UInt16(max(300, min(2000, Int((degrees * 100).rounded()))))
        let confirmedDegrees = Double(centidegrees) / 100.0
        attitude.maximumCorrectionDegrees = confirmedDegrees
        pendingAttitude.maximumCorrectionDegrees = confirmedDegrees
        send([0x2a, UInt8(centidegrees & 0xff), UInt8(centidegrees >> 8)], operation: 0x2a)
    }


    // Does not write to the board. Aura replies with all twelve records and
    // the app persists them in its desktop backup.
    func requestRobotConfiguration() {
        // Connection setup already asks Aura for power and ToF state. Keep
        // the read-only robot backup behind that command so it cannot race a
        // calibration sequence or overwrite the UI's pending operation.
        guard !busy, calibrationZeroQueue.isEmpty, calibrationZeroInFlight == nil else {
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.25) { [weak self] in
                self?.requestRobotConfiguration()
            }
            return
        }
        send([0x1f], operation: 0x1f)
    }

    func captureCalibrationZeros(leg: Int) {
        guard (0..<4).contains(leg) else { return }
        guard calibrationZeroQueue.isEmpty else {
            message = "Zero capture for this leg is already running."
            return
        }
        calibrationZeroProgress = AuraCalibrationZeroProgress(leg: leg, completedAxes: [])
        pendingCalibrationReference = nil
        calibrationZeroConfigFallback?.cancel()
        calibrationZeroConfigFallback = nil
        calibrationZeroQueue = (0..<3).map { axis in
            // The robot kinematics fold the two knee chains in opposite
            // directions. Their commissioning pose is +/-90°, while hip and
            // ab/ad are referenced at 0°.
            let kneeReference: Int16 = (leg == 0 || leg == 2) ? 9000 : -9000
            return (leg: leg, axis: axis, referenceCdeg: axis == 2 ? kneeReference : 0)
        }
        sendNextCalibrationZero()
    }

    private func sendNextCalibrationZero() {
        guard !calibrationZeroQueue.isEmpty else {
            calibrationZeroInFlight = nil
            calibrationZeroConfigFallback?.cancel()
            calibrationZeroConfigFallback = nil
            message = "Three zero positions stored on Aura."
            return
        }
        let next = calibrationZeroQueue[0]
        calibrationZeroInFlight = next
        let reference = UInt16(bitPattern: next.referenceCdeg)
        send([0x19, UInt8(next.leg), UInt8(next.axis),
              UInt8(reference & 0xff), UInt8(reference >> 8)], operation: 0x19)
    }

    private func completeCalibrationZero(_ captured: (leg: Int, axis: Int, referenceCdeg: Int16)) {
        guard let first = calibrationZeroQueue.first,
              first.leg == captured.leg, first.axis == captured.axis else { return }
        calibrationZeroConfigFallback?.cancel()
        calibrationZeroConfigFallback = nil
        var progress = calibrationZeroProgress?.leg == captured.leg
            ? calibrationZeroProgress!
            : AuraCalibrationZeroProgress(leg: captured.leg, completedAxes: [])
        progress.completedAxes.insert(captured.axis)
        calibrationZeroProgress = progress
        calibrationZeroQueue.removeFirst()
        calibrationZeroInFlight = nil
        sendNextCalibrationZero()
    }

    private func acknowledgeCalibrationZeroConfiguration(leg: Int, axis: Int) {
        guard let expected = calibrationZeroInFlight,
              expected.leg == leg, expected.axis == axis else { return }
        // Firmware emits an axis-configuration event only after a successful
        // zero capture.  Use it as a bounded fallback if the following result
        // frame is lost while Wi-Fi remains otherwise connected.
        calibrationZeroConfigFallback?.cancel()
        let fallback = DispatchWorkItem { [weak self] in
            guard let self, let current = self.calibrationZeroInFlight,
                  current.leg == expected.leg, current.axis == expected.axis else { return }
            self.completeCalibrationZero(current)
        }
        calibrationZeroConfigFallback = fallback
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.35, execute: fallback)
    }

    func recaptureCalibrationAxisReference(leg: Int, axis: Int, referenceDegrees: Double) {
        guard (0..<4).contains(leg), (0..<3).contains(axis) else { return }
        let cdeg = Int16(max(-18000, min(18000, Int((referenceDegrees * 100).rounded()))))
        let raw = UInt16(bitPattern: cdeg)
        pendingCalibrationReference = (leg, axis)
        send([0x19, UInt8(leg), UInt8(axis), UInt8(raw & 0xff), UInt8(raw >> 8)], operation: 0x19)
    }

    func nudgeCalibrationAxis(leg: Int, axis: Int, degrees: Double) {
        guard (0..<4).contains(leg), (0..<3).contains(axis) else { return }
        let cdeg = Int16(max(-18000, min(18000, Int((degrees * 100).rounded()))))
        let raw = UInt16(bitPattern: cdeg)
        send([0x1a, UInt8(leg), UInt8(axis), UInt8(raw & 0xff), UInt8(raw >> 8)], operation: 0x1a)
    }

    func releaseCalibrationAxis(leg: Int, axis: Int) {
        guard (0..<4).contains(leg), (0..<3).contains(axis) else { return }
        send([0x1b, UInt8(leg), UInt8(axis)], operation: 0x1b)
    }

    func captureCalibrationLimit(leg: Int, axis: Int, maximum: Bool) {
        guard (0..<4).contains(leg), (0..<3).contains(axis) else { return }
        pendingCalibrationLimit = (leg, axis, maximum)
        send([0x1c, UInt8(leg), UInt8(axis), maximum ? 1 : 0], operation: 0x1c)
    }

    func setCalibrationLegTest(leg: Int, enabled: Bool) {
        guard (0..<4).contains(leg) else { return }
        send([0x1d, UInt8(leg), enabled ? 1 : 0], operation: 0x1d)
    }

    func setCalibrationLegTorque(leg: Int, enabled: Bool) {
        guard (0..<4).contains(leg) else { return }
        pendingCalibrationTorque = (leg, enabled)
        send([0x1e, UInt8(leg), enabled ? 1 : 0], operation: 0x1e)
    }

    func setCalibrationLivePreview(enabled: Bool) {
        pendingCalibrationPreview = enabled
        // The leg byte is retained by firmware command framing; the live
        // preview itself continuously reads all 12 configured axes.
        send([0x23, 0, enabled ? 1 : 0], operation: 0x23)
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

    private func i32(_ data: Data, _ offset: Int) -> Int32 {
        Int32(bitPattern: u32(data, offset))
    }

    // Counters wrap at 16 bits; rates are measured over at least one second.
    private func applyHealthFrame(_ data: Data) {
        let now = ProcessInfo.processInfo.systemUptime
        let beats = (0..<AuraHealth.tasks.count).map { Int(u16(data, 2 + $0 * 2)) }
        pendingHealth.available = true
        pendingHealth.resetReason = Int(data[1])
        pendingHealth.beats = beats
        if let reference = healthReference {
            let elapsed = now - reference.time
            guard elapsed >= 1 else { return }
            // A board restart resets the counters; that one bogus jump reads as unknown.
            pendingHealth.rates = zip(beats, reference.beats).map {
                let rate = Double(($0 - $1 + 65536) % 65536) / elapsed
                return rate > 2000 ? -1 : rate
            }
        }
        healthReference = (now, beats)
    }

    private func parseTelemetry(_ data: Data) {
        guard data.count == 20 else { return }
        lastTelemetryUptime = ProcessInfo.processInfo.systemUptime
        if data[0] == 0x10 {
            let flags = data[18]
            let nextPower = AuraPower(
                uptimeMilliseconds: u32(data, 2),
                servoBusVoltage: Double(u16(data, 6)) / 1000.0,
                current: Double(i32(data, 8)) / 1000.0,
                power: Double(i32(data, 12)) / 1000.0,
                inputVoltage: Double(u16(data, 16)) / 1000.0,
                inaAvailable: flags & 1 != 0,
                vinAvailable: flags & 2 != 0,
                imuAvailable: flags & 4 != 0,
                imuIdentity: data[19],
                servoConsumedMilliampHours: pendingPower.servoConsumedMilliampHours)
            pendingPower = nextPower
            // At most one current-chart point per 100 ms of board time.
            if lastChartSampleUptime == 0 ||
               nextPower.uptimeMilliseconds &- lastChartSampleUptime >= 100 {
                pendingChartPoint = CurrentPoint(time: Date(), amperes: nextPower.current)
                lastChartSampleUptime = nextPower.uptimeMilliseconds
            }
        } else if data[0] == 0x11 {
            pendingIMU = AuraIMU(
                sampleCount: u32(data, 2),
                acceleration: (0..<3).map { Double(i16(data, 6 + $0 * 2)) / 1000.0 },
                gyroscope: (0..<3).map { Double(i16(data, 12 + $0 * 2)) / 10.0 },
                temperature: Double(i16(data, 18)) / 100.0)
        } else if data[0] == 0x2d {
            let flags = data[1]
            pendingAttitude = AuraAttitude(
                sampleCount: u32(data, 2),
                valid: flags & 1 != 0,
                enabled: flags & 2 != 0,
                active: flags & 4 != 0,
                referenceCaptured: flags & 8 != 0,
                referencePending: flags & 16 != 0,
                rollDegrees: Double(i16(data, 6)) / 100.0,
                pitchDegrees: Double(i16(data, 8)) / 100.0,
                referenceRollDegrees: Double(i16(data, 10)) / 100.0,
                referencePitchDegrees: Double(i16(data, 12)) / 100.0,
                correctionRollDegrees: Double(i16(data, 14)) / 100.0,
                correctionPitchDegrees: Double(i16(data, 16)) / 100.0,
                i2cAddress: data[18],
                maximumCorrectionDegrees: data[19] >= 30 ? Double(data[19]) / 10.0 : 7.0)
        } else if data[0] == 0x2e {
            let flags = data[1]
            pendingFootContacts = AuraFootContacts(
                feedbackAvailable: flags & 1 != 0,
                controlActive: flags & 2 != 0,
                touchdownWaiting: flags & 4 != 0,
                expectedMask: data[2],
                detectedMask: data[3],
                earlyTouchdownMask: data[4],
                confidence: (0..<4).map { Int(data[5 + $0]) })
        } else if data[0] == 0x12 {
            let flags = data[2]
            // Preserve fields edited in the configuration UI; telemetry owns
            // only the current range sample.
            pendingToFSensors = tofSensors
            for index in 0..<2 {
                let offset = 4 + index * 8
                pendingToFSensors[index].present = flags & (1 << index) != 0
                pendingToFSensors[index].ranging = flags & (1 << (index + 2)) != 0
                pendingToFSensors[index].valid = flags & (1 << (index + 4)) != 0
                pendingToFSensors[index].address = Int(data[offset])
                pendingToFSensors[index].rangeStatus = Int(data[offset + 1])
                pendingToFSensors[index].distance = Int(u16(data, offset + 2))
                pendingToFSensors[index].signalPerSpad = Int(u16(data, offset + 4))
                pendingToFSensors[index].sigma = Int(u16(data, offset + 6))
            }
        } else if data[0] == 0x13 {
            let flags = data[2]
            for index in 0..<2 {
                let offset = 4 + index * 8
                if flags & (1 << index) != 0 {
                    pendingToFSensors[index].ambientPerSpad = Int(u16(data, offset))
                    pendingToFSensors[index].activeSpads = Int(u16(data, offset + 2))
                    pendingToFSensors[index].sampleCount = u32(data, offset + 4)
                }
            }
        } else if data[0] == 0x14 {
            let flags = data[2]
            pendingDualSense.hasInput = flags & 4 != 0
            pendingDualSense.reportID = Int(data[3])
            pendingDualSense.leftX = Int(data[4])
            pendingDualSense.leftY = Int(data[5])
            pendingDualSense.rightX = Int(data[6])
            pendingDualSense.rightY = Int(data[7])
            pendingDualSense.leftTrigger = Int(data[8])
            pendingDualSense.rightTrigger = Int(data[9])
            pendingDualSense.buttons = [Int(data[10]), Int(data[11]), Int(data[12])]
            pendingDualSense.battery = flags & 8 != 0 ? Int(data[13]) : nil
            pendingDualSense.reportLength = Int(data[14])
            pendingDualSense.sampleCount = u32(data, 16)
            if flags & 1 != 0 { pendingDualSense.state = .connected }
            else if flags & 2 != 0 { pendingDualSense.state = .scanning }
        } else if data[0] == 0x29 {
            pendingPower.servoConsumedMilliampHours = Double(u32(data, 2)) / 1000.0
        } else if data[0] == 0x2c {
            pendingRobotState.safetyFault = AuraRobotSafetyFault(
                latched: data[1] != 0,
                leg: data[2] == 0xff ? -1 : Int(data[2]),
                axis: data[3] == 0xff ? -1 : Int(data[3]),
                servoID: Int(data[4]),
                currentRaw: Int(i16(data, 5)),
                loadRaw: Int(i16(data, 7)),
                trackingErrorDegrees: Double(u16(data, 9)) / 100.0,
                confirmations: Int(data[11]))
        } else if data[0] == 0x27 {
            pendingRobotState.applyCoreFrame(data)
        } else if data[0] == 0x31 {
            pendingRobotState.applyBodyFrame(data)
        } else if data[0] == 0x32 {
            pendingRobotState.applyEstimateFrame(data)
        } else if data[0] == 0x33 {
            applyHealthFrame(data)
        } else if data[0] == 0x35 {
            guard data[1] == 1 else { return }
            pendingHealth.timingAvailable = true
            pendingHealth.sectionMs = (0..<AuraHealth.sections.count).map { Double(u16(data, 2 + $0 * 2)) / 10 }
            pendingHealth.heapFree = Int(u32(data, 12))
            pendingHealth.heapMinimum = Int(u32(data, 16))
        } else if data[0] == 0x34 {
            pendingHealth.stackAvailable = true
            pendingHealth.gaitTickMaxMs = Int(data[1])
            pendingHealth.stackFree = (0..<AuraHealth.tasks.count).map { Int(u16(data, 2 + $0 * 2)) }
        } else if data[0] == 0x2f || data[0] == 0x30 {
            // Aura reports its stored gait profiles and motion tuning on the
            // telemetry channel; the parser for them lives with the events.
            parseEvent(data)
        } else if data[0] == 0x2a {
            pendingRobotState.odometryValid = data[1] & 1 != 0
            pendingRobotState.odometryX = Double(i32(data, 2)) / 10.0
            pendingRobotState.odometryZ = Double(i32(data, 6)) / 10.0
            pendingRobotState.odometryYaw = Double(i16(data, 10)) * .pi / 18000.0
        } else if data[0] == 0x28 {
            let leg = Int(data[1]); guard pendingRobotLegTelemetry.indices.contains(leg) else { return }
            var telemetry = pendingRobotLegTelemetry[leg]
            let configuredFlags = data[2], stateFlags = data[3]
            telemetry.configured = (0..<3).map { configuredFlags & (1 << ($0 + 1)) != 0 }
            telemetry.present = (0..<3).map { stateFlags & (1 << $0) != 0 }
            telemetry.moving = (0..<3).map { stateFlags & (1 << ($0 + 3)) != 0 }
            telemetry.targetDegrees = (0..<3).map { Double(i16(data, 4 + $0 * 2)) / 100.0 }
            telemetry.measuredDegrees = (0..<3).map { Double(i16(data, 10 + $0 * 2)) / 100.0 }
            telemetry.temperature = [Int(data[16]), Int(data[17]), Int(data[18])]
            pendingRobotLegTelemetry[leg] = telemetry
            // Firmware always emits status followed by physical slots
            // RF, LF, RR, LR. Publish
            // each complete 20 ms robot frame directly to SceneKit.  The
            // normal SwiftUI state is deliberately rate-limited below.
            if leg == 3 {
                robotRenderChannel.publish(AuraRobotFrame(state: pendingRobotState,
                                                           legs: pendingRobotLegTelemetry,
                                                           controller: pendingDualSense,
                                                           contacts: pendingFootContacts))
                recordHistorySample()
                route.ingest(state: pendingRobotState, imu: pendingIMU, attitude: pendingAttitude)
                sessions.ingest(power: pendingPower, state: pendingRobotState, attitude: pendingAttitude,
                                imu: pendingIMU, contacts: pendingFootContacts, health: pendingHealth,
                                legs: pendingRobotLegTelemetry, route: route)

                let now = ProcessInfo.processInfo.systemUptime
                guard now - lastUICommitUptime >= uiCommitInterval else { return }
                lastUICommitUptime = now
                objectWillChange.send()
                power = pendingPower
                imu = pendingIMU
                attitude = pendingAttitude
                footContacts = pendingFootContacts
                if health != pendingHealth { health = pendingHealth }
                tofSensors = pendingToFSensors
                dualSense = pendingDualSense
                if let pendingChartPoint {
                    currentHistory.append(pendingChartPoint)
                    if currentHistory.count > 600 { currentHistory.removeFirst(currentHistory.count - 600) }
                    self.pendingChartPoint = nil
                }
                robotState = pendingRobotState
                robotLegTelemetry = pendingRobotLegTelemetry
            }
        }
    }

    private func recordHistorySample() {
        let power = pendingPower, imu = pendingIMU, attitude = pendingAttitude
        let state = pendingRobotState, legs = pendingRobotLegTelemetry, contacts = pendingFootContacts
        history.append { time in
            TelemetrySample(
                time: time, busVoltage: power.servoBusVoltage, current: power.current, power: power.power,
                roll: attitude.rollDegrees, pitch: attitude.pitchDegrees,
                referenceRoll: attitude.referenceCaptured ? attitude.referenceRollDegrees : 0,
                referencePitch: attitude.referenceCaptured ? attitude.referencePitchDegrees : 0,
                correctionRoll: attitude.correctionRollDegrees, correctionPitch: attitude.correctionPitchDegrees,
                gyro: SIMD3(imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]),
                accel: SIMD3(imu.acceleration[0], imu.acceleration[1], imu.acceleration[2]),
                imuTemperature: imu.temperature,
                servoTemperature: legs.flatMap { $0.temperature.map(Double.init) },
                trackingError: legs.flatMap { leg in
                    (0..<3).map { leg.present[$0] ? leg.targetDegrees[$0] - leg.measuredDegrees[$0] : 0 } },
                bodyHeight: Double(state.bodyHeight), phase: state.phase,
                forward: state.forward, lateral: state.lateral, turn: state.turn,
                bodyShiftX: state.bodyShiftX, bodyShiftZ: state.bodyShiftZ,
                odometryX: state.odometryX, odometryZ: state.odometryZ, odometryYaw: state.odometryYaw,
                armed: state.armed, mode: state.selectedGait, contacts: contacts.detectedMask)
        }
    }

    private func parseEvent(_ data: Data) {
        guard data.count == 20 else { return }
        switch data[0] {
        case 0x20:
            let operation = data[1]
            let result = u32(data, 2)
            busy = false
            requestedOperation = nil
            if result != 0 {
                if operation == 0x20 || operation == 0x21 {
                    message = String(format: "Direction change not saved (ESP 0x%X). The axis stays released; set it to the reference by hand and try again.", result)
                    return
                }
                if operation == 0x22 {
                    message = "Automatic direction flip was removed for safety. No motion sent, calibration unchanged."
                    return
                }
                if operation == 0x1a {
                    switch result {
                    case 0x102:
                        message = "Direction test stopped: no reliable servo position reading. No motion sent."
                    case 0x106:
                        message = "The +12° test would leave the encoder's 0…4095 range. No torque, no motion."
                    case 0x103:
                        message = "Direction test blocked: Aura is armed, the axis is not in position mode, it is away from its stored reference, or the previous test was not released. No motion sent."
                    default:
                        message = String(format: "Direction test blocked by Aura (ESP 0x%X). No motion sent.", result)
                    }
                    return
                }
                if operation == 0x17, robotConfigurationTotal != 0 {
                    robotConfigurationQueue.removeAll()
                    message = "Axis save stopped after \(robotConfigurationCompleted) / \(robotConfigurationTotal): ESP 0x\(String(result, radix: 16))."
                    robotConfigurationTotal = 0
                    robotConfigurationCompleted = 0
                    return
                }
                if operation == 0x14 {
                    message = "No paired pad, or the pad is off."
                    return
                }
                if operation == 0x12 {
                    message = "Could not start DualSense pairing."
                    return
                }
                if operation == 0x13 {
                    message = "No active pad to disconnect."
                    return
                }
                if operation == 0x0f {
                    message = String(format: "MPU6050 not responding · WHO_AM_I 0x%02X.", data[6])
                    return
                }
                if operation == 0x29 {
                    message = "Posture correction unchanged. Aura must be disarmed and not calibrating."
                    return
                }
                if operation == 0x2a {
                    message = "IMU correction limit unchanged. Aura must be disarmed and not calibrating."
                    return
                }
                if operation == 0x18 {
                    if pendingRobotState.safetyFault.latched {
                        message = "Aura stays disarmed after an emergency stop. Check the mechanics and clear the stop before arming again."
                    } else if result == 0x106, data[6] < 12 {
                        let leg = ["LF", "RF", "LR", "RR"][Int(data[6]) / 3]
                        let axis = ["Ab/ad", "Hip", "Knee"][Int(data[6]) % 3]
                        message = "Not armed: \(leg) / \(axis) is outside its stored limits in the stand pose. Check and save that axis's limits again."
                    } else if result == 0x10d {
                        message = String(format: "Not armed: battery at %.2f V, 10.0 V is 0 %%. Charge the battery.", power.servoBusVoltage)
                    } else if result == 0x103 {
                        message = "Aura is still in calibration. In Calibration, stop the TTL preview and disarm every leg, then press Create again."
                    } else {
                        message = "Not armed: save a complete set of 12 assignments with unique IDs."
                    }
                    return
                }
                if operation == 0x1c {
                    let side = pendingCalibrationLimit?.maximum == true ? "+" : "−"
                    pendingCalibrationLimit = nil
                    switch result {
                    case 0x102:
                        message = "\(side) limit not saved: the reading is on the wrong side of the other limit. Release the axis, move it further that way and check its direction."
                    case 0x103:
                        message = "\(side) limit not saved: Aura is armed. Disarm and try again."
                    case 0x107:
                        message = "\(side) limit not saved: no TTL reply from this servo."
                    default:
                        message = String(format: "\(side) limit not saved (ESP 0x%X).", result)
                    }
                    return
                }
                if operation == 0x19, result == 0x102 {
                    calibrationZeroConfigFallback?.cancel()
                    calibrationZeroConfigFallback = nil
                    calibrationZeroQueue.removeAll()
                    calibrationZeroInFlight = nil
                    pendingCalibrationReference = nil
                    message = "Axis reference is outside the safe configured range. Release the axis, set the reference by hand and save again."
                    return
                }
                if (0x19...0x1d).contains(operation) {
                    calibrationZeroConfigFallback?.cancel()
                    calibrationZeroConfigFallback = nil
                    calibrationZeroQueue.removeAll()
                    calibrationZeroInFlight = nil
                    if operation == 0x19 { pendingCalibrationReference = nil }
                    message = String(format: "Axis calibration failed (ESP 0x%X). Check the servo ID, the leg position and that Aura is disarmed.", result)
                    return
                }
                if operation == 0x0b || operation == 0x10 {
                    message = String(format: "Configuration saved, but the sensors did not start (ESP 0x%X).", result)
                    refreshToFConfigurations()
                    return
                }
                if operation == 0x0d {
                    message = String(format: "No VL53L4CD found (ESP 0x%X).", result)
                    return
                }
                message = String(format: "Aura command 0x%02X: ESP error 0x%X.", operation, result)
                if operation == 0x03 { torqueEnabled.toggle() }
                return
            }
            switch operation {
            case 0x01:
                message = "Scan done · \(data[6]) servos found."
            case 0x02: message = "Servo read done."
            case 0x03: message = torqueEnabled ? "Servo torque on." : "Servo torque off."
            case 0x04: message = "Position sent to servo."
            case 0x05: message = "Current position saved as servo centre."
            case 0x06:
                selectedServo = Int(data[6])
                message = "New servo ID: \(data[6])."
                scanServos()
            case 0x07: message = "LED command done."
            case 0x2e:
                ledBrightness = Double(data[6])
                if ledBrightnessWrite { message = "Strip brightness saved on Aura: \(data[6]) %." }
                ledBrightnessWrite = false
            case 0x09:
                torqueEnabled = false
                if let mode = ServoOperatingMode(rawValue: data[6]) { operatingMode = mode }
                message = "\(operatingMode.title) mode saved. Servo torque is off."
            case 0x0a:
                torqueEnabled = data[6] != 0
                message = torqueEnabled ? "Motor running at the set speed." : "Motor stopped, torque off."
            case 0x0b: message = "VL53L4CD configuration saved and applied."
            case 0x0c: break
            case 0x0d:
                let count = Int(data[6])
                message = count == 1 ? "One VL53L4CD found." : "\(count) VL53L4CD sensors found."
            case 0x0e:
                message = "Distance offset saved in the sensor and ESP."
            case 0x10:
                message = "VL53L4CD optical settings saved and applied."
            case 0x11:
                message = "VL53L4CD temperature update done."
            case 0x0f:
                message = result == 0
                    ? String(format: "MPU6050 gotowy — WHO_AM_I 0x%02X.", data[6])
                    : String(format: "MPU6050 not responding · WHO_AM_I 0x%02X.", data[6])
            case 0x29:
                message = data[6] != 0
                    ? "Posture correction applies at the next arming."
                    : "Posture correction off."
            case 0x2a:
                message = "Posture correction limit saved: \(data[6])°"
            case 0x2b:
                message = "Gait profile saved on Aura; used from the next step."
            case 0x2c:
                message = "Motion tuning saved on Aura; used at the next arming."
            case 0x12: message = "Looking for DualSense. Hold Create and PS until the light bar blinks."
            case 0x13: message = "DualSense disconnected."
            case 0x14: message = "Connecting to the paired pad…"
            case 0x15: message = "DualSense pairing removed."
            case 0x16: message = "Connecting to the selected device…"
            case 0x17:
                if robotConfigurationTotal != 0 {
                    robotConfigurationCompleted += 1
                    sendNextRobotAxisConfiguration()
                } else {
                    message = "Axis assignment saved on Aura."
                }
            case 0x18: message = data[6] != 0 ? "Aura servos armed." : "Aura servos disarmed."
            case 0x19:
                let recordedSlot = Int(data[6])
                if let captured = calibrationZeroInFlight,
                   recordedSlot == captured.leg * 3 + captured.axis {
                    completeCalibrationZero(captured)
                } else if let captured = pendingCalibrationReference {
                    pendingCalibrationReference = nil
                    message = "Axis \(captured.axis + 1) reference read and saved without motion."
                }
            case 0x1a: message = "Single +12° axis test done; torque released."
            case 0x1b: message = "Axis released; it can be moved by hand."
            case 0x1c:
                pendingCalibrationLimit = nil
                message = "Axis limit saved on Aura."
            case 0x1d: message = "Leg test state updated."
            case 0x1e:
                if let pending = pendingCalibrationTorque, result == 0 {
                    calibrationArmedLeg = pending.enabled ? pending.leg : nil
                }
                pendingCalibrationTorque = nil
                message = calibrationArmedLeg == nil
                    ? "The leg's three servos are disarmed."
                    : "Only the selected leg's three servos are armed."
            case 0x20:
                message = "Axis direction reversed at the reference. Torque stays off; run a new +12° test after checking the model."
            case 0x21:
                message = "Older direction flip repaired: sign kept, zero and limits recomputed at the reference. Torque stays off."
            case 0x22:
                message = "Automatic direction flip is disabled."
            case 0x23:
                calibrationLivePreviewActive = pendingCalibrationPreview
                message = calibrationLivePreviewActive
                    ? "TTL preview on: reading 12 servos, torque off."
                    : "TTL preview off."
            case 0x24:
                message = "Emergency stop cleared. Servos stay disarmed."
            case 0x25:
                message = "Stand height saved on Aura. Check the model before arming."
            case 0x26:
                message = "Crawl stride, height and speed saved on Aura."
            case 0x28:
                message = "Foot spread saved on Aura."
            case 0x1f:
                message = "Axis configuration read from Aura; backup saved on this Mac."
            default: break
            }
        case 0x21:
            let id = Int(data[1])
            if !servoIDs.contains(id) { servoIDs.append(id); servoIDs.sort() }
        case 0x22:
            let position = Int(i16(data, 4))
            feedback = ServoFeedback(
                id: Int(data[1]), status_error: Int(data[2]), position: position,
                angle_deg: Double(position) * 360.0 / 4096.0,
                speed_raw: Int(i16(data, 6)), load_raw: Int(i16(data, 8)),
                voltage_v: Double(data[10]) / 10.0, temperature_c: Int(data[11]),
                moving: data[3] != 0, current_raw: Int(i16(data, 12)))
            if let feedback { angle = feedback.angle_deg }
            if let mode = ServoOperatingMode(rawValue: data[14]) { operatingMode = mode }
        case 0x23:
            let index = Int(data[1])
            guard tofSensors.indices.contains(index) else { return }
            tofSensors[index].address = Int(data[2])
            tofSensors[index].enabled = data[3] != 0
            tofSensors[index].timingBudget = Int(u16(data, 4))
            tofSensors[index].intermeasurement = Int(u16(data, 6))
            tofSensors[index].present = data[8] != 0
            tofSensors[index].ranging = data[9] != 0
            tofSensors[index].error = u32(data, 10)
            tofSensors[index].offset = Int(i16(data, 14))
        case 0x24:
            let index = Int(data[1])
            guard tofSensors.indices.contains(index) else { return }
            tofSensors[index].xtalkKcps = Int(u16(data, 2))
            tofSensors[index].signalThresholdKcps = Int(u16(data, 4))
            tofSensors[index].sigmaThresholdMm = Int(u16(data, 6))
            tofSensors[index].detectionLowMm = Int(u16(data, 8))
            tofSensors[index].detectionHighMm = Int(u16(data, 10))
            tofSensors[index].detectionWindow = Int(data[12])
            tofSensors[index].detectionEnabled = data[13] != 0
        case 0x25:
            pendingDualSense.state = DualSenseConnectionState(rawValue: data[1]) ?? .error
            pendingDualSense.hasSavedController = data[2] != 0
            pendingDualSense.battery = data[3] == 0xff ? nil : Int(data[3])
            pendingDualSense.address = Array(data[4..<10])
            pendingDualSense.vendorID = Int(u16(data, 10))
            pendingDualSense.productID = Int(u16(data, 12))
            pendingDualSense.reportID = Int(data[14])
            pendingDualSense.hasInput = data[15] != 0
            pendingDualSense.sampleCount = u32(data, 16)
        case 0x26:
            let length = min(Int(data[3]), 10)
            let name = String(bytes: data[10..<(10 + length)], encoding: .utf8) ?? ""
            let device = AuraDualSenseDevice(
                address: Array(data[4..<10]), name: name,
                rssi: Int(Int8(bitPattern: data[2])))
            if let index = dualSenseDevices.firstIndex(where: { $0.address == device.address }) {
                dualSenseDevices[index] = device
            } else {
                dualSenseDevices.append(device)
            }
        case 0x2f:
            guard data[1] == 1, (1...5).contains(Int(data[2])) else { return }
            let profile = AuraGaitProfile(
                gait: Int(data[2]), strideMm: Int(u16(data, 3)),
                stepHeightMm: Int(u16(data, 5)), frequencyCentiHz: Int(u16(data, 7)),
                dutyPercent: Int(data[9]))
            guard (15...300).contains(profile.strideMm),
                  (5...100).contains(profile.stepHeightMm),
                  (15...(profile.gait == 2 || profile.gait == 4 || profile.gait == 5 ? 120 : 300)).contains(profile.frequencyCentiHz),
                  ((profile.gait == 2 || profile.gait == 4 || profile.gait == 5)
                      ? (76...90).contains(profile.dutyPercent)
                      : (45...85).contains(profile.dutyPercent)) else { return }
            applyBoardGaitProfile(profile)
        case 0x30:
            guard data[1] == 1 else { return }
            let tuning = AuraMotionTuning(
                movingAttitudeGainPermille: Int(u16(data, 2)),
                comMaximumVelocityMmPerSecond: Int(u16(data, 4)),
                comMaximumAccelerationMmPerSecond2: Int(u16(data, 6)))
            guard (100...1000).contains(tuning.movingAttitudeGainPermille),
                  (20...160).contains(tuning.comMaximumVelocityMmPerSecond),
                  (60...900).contains(tuning.comMaximumAccelerationMmPerSecond2) else { return }
            if motionTuning != tuning { motionTuning = tuning }
        case 0x2b:
            guard (0..<4).contains(Int(data[1])), (0..<3).contains(Int(data[2])) else { return }
            let configuration = AuraCalibrationAxisConfiguration(
                leg: Int(data[1]), axis: Int(data[2]), servoID: Int(data[3]),
                direction: Int(Int8(bitPattern: data[4])), centerTick: Int(i16(data, 5)),
                minimumDegrees: Double(i16(data, 7)) / 100.0,
                maximumDegrees: Double(i16(data, 9)) / 100.0,
                speedLimitRaw: Int(u16(data, 11)), acceleration: Int(data[13]))
            calibrationAxisConfiguration = configuration
            robotAxisBackupBySlot[configuration.leg * 3 + configuration.axis] = configuration
            robotAxisBackup = robotAxisBackupBySlot.values.sorted {
                ($0.leg, $0.axis) < ($1.leg, $1.axis)
            }
            acknowledgeCalibrationZeroConfiguration(leg: configuration.leg, axis: configuration.axis)
        default: break
        }
    }
}
