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
        case .below: "Poniżej dolnego progu"
        case .above: "Powyżej górnego progu"
        case .outside: "Poza oknem"
        case .inside: "Wewnątrz okna"
        }
    }
}

enum ServoOperatingMode: UInt8, CaseIterable, Identifiable {
    case servo = 0
    case motor = 1

    var id: UInt8 { rawValue }
    var title: String { self == .servo ? "Serwo" : "Motor" }
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
        case .off: "Bluetooth wyłączony"
        case .ready: "Gotowy"
        case .scanning: "Szukam pada…"
        case .connecting: "Łączę…"
        case .connected: "Połączony"
        case .error: "Błąd połączenia"
        }
    }
}

struct AuraDualSenseDevice: Identifiable, Equatable {
    var address: [UInt8]
    var name: String
    var rssi: Int

    var id: String { address.map { String(format: "%02X", $0) }.joined(separator: ":") }
    var displayName: String { name.isEmpty ? "Nieznane urządzenie" : name }
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
        case 1: "Trot · 2 nogi"
        case 2: "Krok 1×"
        case 3: "Run · szybki diagonalny"
        case 4: "Climb · 1 noga"
        case 5: "3 łapy · eksperymentalny"
        default: "Nieznany chód"
        }
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
    private static let lastKnownBoardAddress = "192.168.0.69"
    private static let bridgePort: NWEndpoint.Port = 4243
    @Published var connectionText = "Szukam Aury w sieci Wi-Fi…"
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
    private var pendingToFSensors = [AuraToFSensor(id: 0, address: 0x30), AuraToFSensor(id: 1, address: 0x31)]
    private var pendingDualSense = AuraDualSense()
    private var pendingChartPoint: CurrentPoint?
    // The board still delivers a complete robot frame at 50 Hz, but SwiftUI
    // must not rebuild every tab, chart and control at that rate.  SceneKit
    // consumes every frame through robotRenderChannel; dashboard controls are
    // committed at 10 Hz, which also makes 600 chart samples a full minute.
    private var lastUICommitUptime = 0.0
    private let uiCommitInterval = 0.10
    private var lastChartSampleUptime: UInt32 = 0

    override init() {
        super.init()
        startNetworkBridge()
        startScan()
    }

    private func startNetworkBridge() {
        guard networkBridge == nil,
              let script = Bundle.main.url(forResource: "aura_network_bridge", withExtension: "py") else {
            return
        }
        let task = Process()
        task.executableURL = URL(fileURLWithPath: "/usr/bin/python3")
        task.arguments = [script.path, Self.lastKnownBoardAddress, "4242", "\(Self.bridgePort.rawValue)"]
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
        if networkBridge?.isRunning == true {
            connect(to: .hostPort(host: "127.0.0.1", port: Self.bridgePort))
            return
        }
        connectionText = "Szukam Aura Main Board w sieci Wi-Fi…"
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
        connectionText = "Łączę z Aura Main Board przez Wi-Fi…"
        link.stateUpdateHandler = { [weak self, weak link] state in
            Task { @MainActor in
                guard let self, let link, self.connection === link else { return }
                NSLog("Aura network state: %@", String(describing: state))
                switch state {
                case .ready:
                    self.isConnected = true
                    self.busy = false
                    self.connectionText = "Połączono z Aura Main Board (Wi-Fi)"
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
        connectionText = "Brak połączenia — szukam płytki w sieci…"
        if interruptedZeroCapture {
            message = "Połączenie przerwano podczas zapisu referencji. Nic nie zostało uzbrojone — po ponownym połączeniu zapisz tę nogę od początku."
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
            message = "Brak połączenia Wi-Fi z Aura Main Board."
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
            message = "Adres musi być unikalny, mieścić się w 0x08…0x77 i nie może być 0x29 ani 0x40."
            return
        }
        guard (10...200).contains(sensor.timingBudget),
              sensor.intermeasurement == 0 || sensor.intermeasurement > sensor.timingBudget else {
            message = "Budżet pomiaru: 10…200 ms. Okres ma być równy 0 albo większy od budżetu."
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
            message = "Do kalibracji potrzebny jest poprawny pomiar. Ustaw matowy cel na znanej odległości."
            return
        }
        let newOffset = sensor.offset + referenceDistance - sensor.distance
        guard (-1024...1023).contains(newOffset) else {
            message = "Wyliczona korekta \(newOffset) mm przekracza zakres −1024…1023 mm."
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
            message = "Ustawienia optyczne są poza zakresem VL53L4CD."
            return
        }
        if sensor.detectionEnabled {
            guard sensor.intermeasurement > sensor.timingBudget else {
                message = "Próg detekcji wymaga trybu okresowego: okres musi być większy od budżetu pomiaru."
                return
            }
            guard sensor.detectionLowMm <= sensor.detectionHighMm else {
                message = "Dolny próg odległości nie może być większy od górnego."
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
            message = "Brak połączenia Wi-Fi z Aura Main Board."
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
            message = "Brak połączenia Wi-Fi z Aura Main Board."
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
        message = "Automatyczne odwrócenie z ruchem zostało usunięte. Zwolnij moment, ustaw oś ręcznie na referencji i użyj „Zapisz odwrócony kierunek — bez ruchu”."
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
            message = "Nie zapisuję: ten sam ID serwa jest przypisany do więcej niż jednej osi."
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
            message = "Połączenie z Aurą zostało przerwane podczas zapisu osi."
            return
        }
        guard !robotConfigurationQueue.isEmpty else {
            message = "Zapisano \(robotConfigurationCompleted) / \(robotConfigurationTotal) osi w Aurze."
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
            message = "Najpierw rozbrój robota, aby zmienić wysokość stania."
            return
        }
        let height = UInt16(max(120, min(250, millimetres)))
        send([0x25, UInt8(height & 0xff), UInt8(height >> 8)], operation: 0x25)
    }

    // The parameters are stored on Aura and accepted only before torque is
    // enabled. Editing these sliders therefore cannot change a walking leg.
    func setRobotSingleFootTuning(stride: Int, lift: Int, frequency: Double) {
        guard !robotState.armed else {
            message = "Najpierw rozbrój robota, aby zmienić parametry kroku."
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

    func replaceGaitProfileLocally(_ profile: AuraGaitProfile) {
        guard let index = gaitProfiles.firstIndex(where: { $0.gait == profile.gait }) else { return }
        gaitProfiles[index] = profile
    }

    // One profile update is stored in Aura's NVS while torque is disabled.
    // It sends no servo target and the planner reads it on Aura itself.
    func setRobotGaitProfile(_ profile: AuraGaitProfile) {
        guard !robotState.armed else {
            message = "Najpierw rozbrój robota, aby zmienić profil chodu."
            return
        }
        let singleFoot = profile.gait == 2 || profile.gait == 4 || profile.gait == 5
        let stride = UInt16(max(15, min(300, profile.strideMm)))
        let lift = UInt16(max(5, min(100, profile.stepHeightMm)))
        let frequency = UInt16(max(15, min(singleFoot ? 120 : 300, profile.frequencyCentiHz)))
        let duty = UInt8(max(singleFoot ? 76 : 45, min(singleFoot ? 90 : 85,
                                                        profile.dutyPercent)))
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
            message = "Najpierw rozbrój robota, aby zmienić strojenie ruchu."
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
            message = "Najpierw rozbrój robota, aby zmienić równowagę kroku."
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
            message = "Najpierw rozbrój robota, aby zmienić rozstaw stóp."
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
            message = "Najpierw rozbrój robota, aby zmienić korekcję postawy."
            return
        }
        send([0x29, enabled ? 1 : 0], operation: 0x29)
    }

    // This only persists a controller ceiling while Aura is disarmed. It does
    // not arm torque or send a pose to any joint.
    func setAttitudeCorrectionLimit(degrees: Double) {
        guard !robotState.armed else {
            message = "Najpierw rozbrój robota, aby zmienić limit korekcji IMU."
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
            message = "Trwa już zapis pozycji zerowych tej nogi."
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
            message = "Trzy pozycje zerowe zapisane w Aurze."
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

    private func parseTelemetry(_ data: Data) {
        guard data.count == 20 else { return }
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
            // INA226 is received at 50 Hz, while the chart only needs a
            // stable 10 Hz history.  This avoids rebuilding a 3,000-point
            // SwiftUI path on every incoming control cycle.
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

                let now = ProcessInfo.processInfo.systemUptime
                guard now - lastUICommitUptime >= uiCommitInterval else { return }
                lastUICommitUptime = now
                objectWillChange.send()
                power = pendingPower
                imu = pendingIMU
                attitude = pendingAttitude
                footContacts = pendingFootContacts
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
                    message = String(format: "Nie zapisano bezpiecznej korekty kierunku (ESP 0x%X). Moment osi pozostaje wyłączony; ustaw ją ręcznie na pozycję referencyjną i spróbuj ponownie.", result)
                    return
                }
                if operation == 0x22 {
                    message = "Automatyczne odwrócenie kierunku zostało usunięte ze względów bezpieczeństwa. Nie wysłano ruchu ani nie zmieniono kalibracji."
                    return
                }
                if operation == 0x1a {
                    switch result {
                    case 0x102:
                        message = "Test kierunku został zatrzymany, bo Aura nie dostała wiarygodnego odczytu pozycji serwa. Nie wysłano ruchu."
                    case 0x106:
                        message = "Test +12° przekroczyłby fizyczny zakres 0…4095 bieżącego enkodera. Aura nie włączyła momentu i nie wysłała ruchu."
                    case 0x103:
                        message = "Test kierunku został zablokowany: robot jest uzbrojony, oś nie jest w trybie pozycyjnym, nie wróciła do zapisanej referencji albo poprzedni test nie został zwolniony. Nie wysłano ruchu."
                    default:
                        message = String(format: "Test kierunku został zablokowany przez Aurę (ESP 0x%X). Nie wysłano ruchu.", result)
                    }
                    return
                }
                if operation == 0x17, robotConfigurationTotal != 0 {
                    robotConfigurationQueue.removeAll()
                    message = "Zapis osi przerwany po \(robotConfigurationCompleted) / \(robotConfigurationTotal): ESP 0x\(String(result, radix: 16))."
                    robotConfigurationTotal = 0
                    robotConfigurationCompleted = 0
                    return
                }
                if operation == 0x14 {
                    message = "Nie ma zapamiętanego pada albo pad jest wyłączony."
                    return
                }
                if operation == 0x12 {
                    message = "Nie udało się uruchomić parowania DualSense."
                    return
                }
                if operation == 0x13 {
                    message = "Nie ma aktywnego pada do rozłączenia."
                    return
                }
                if operation == 0x0f {
                    message = String(format: "MPU6050 nie odpowiedział — WHO_AM_I 0x%02X.", data[6])
                    return
                }
                if operation == 0x29 {
                    message = "Nie zmieniono korekcji postawy. Robot musi być rozbrojony, bez aktywnej kalibracji."
                    return
                }
                if operation == 0x2a {
                    message = "Nie zmieniono limitu korekcji IMU. Robot musi być rozbrojony, bez aktywnej kalibracji."
                    return
                }
                if operation == 0x18 {
                    if pendingRobotState.safetyFault.latched {
                        message = "Aura pozostaje rozbrojona po zatrzymaniu awaryjnym. Sprawdź mechanikę i skasuj blokadę, zanim ponownie uzbroisz serwa."
                    } else if result == 0x106, data[6] < 12 {
                        let leg = ["LF", "RF", "LR", "RR"][Int(data[6]) / 3]
                        let axis = ["Ab/ad", "Hip", "Knee"][Int(data[6]) % 3]
                        message = "Aura nie uzbroiła robota: \(leg) / \(axis) nie mieści się w zapisanych krańcach pozycji stania. Sprawdź i ponownie zapisz krańce tej osi."
                    } else if result == 0x103 {
                        message = "Aura pozostaje w trybie kalibracji. W zakładce Calibration wyłącz podgląd TTL i rozbrój każdą nogę, a potem ponownie naciśnij Create."
                    } else {
                        message = "Aura nie uzbroiła serw: zapisz poprawne, kompletne 12 przypisań z unikalnymi ID."
                    }
                    return
                }
                if operation == 0x1c {
                    let side = pendingCalibrationLimit?.maximum == true ? "+" : "−"
                    pendingCalibrationLimit = nil
                    switch result {
                    case 0x102:
                        message = "Kraniec \(side) nie został zapisany: odczytana pozycja leży po niewłaściwej stronie obecnego drugiego krańca. Zwolnij moment, przesuń oś dalej w wybranym kierunku i sprawdź kierunek osi."
                    case 0x103:
                        message = "Kraniec \(side) nie został zapisany, bo robot jest uzbrojony. Rozbrój go i spróbuj ponownie."
                    case 0x107:
                        message = "Kraniec \(side) nie został zapisany: Aura nie dostała odpowiedzi TTL z tego serwa."
                    default:
                        message = String(format: "Kraniec \(side) nie został zapisany (ESP 0x%X).", result)
                    }
                    return
                }
                if operation == 0x19, result == 0x102 {
                    calibrationZeroConfigFallback?.cancel()
                    calibrationZeroConfigFallback = nil
                    calibrationZeroQueue.removeAll()
                    calibrationZeroInFlight = nil
                    pendingCalibrationReference = nil
                    message = "Referencja osi nie mieści się w bezpiecznym zakresie skonfigurowanego ruchu. Zwolnij moment, ustaw referencję ręcznie i zapisz ją ponownie."
                    return
                }
                if (0x19...0x1d).contains(operation) {
                    calibrationZeroConfigFallback?.cancel()
                    calibrationZeroConfigFallback = nil
                    calibrationZeroQueue.removeAll()
                    calibrationZeroInFlight = nil
                    if operation == 0x19 { pendingCalibrationReference = nil }
                    message = String(format: "Kalibracja osi nie została wykonana (ESP 0x%X). Sprawdź ID serwa, pozycję nogi i to, że robot jest rozbrojony.", result)
                    return
                }
                if operation == 0x0b || operation == 0x10 {
                    message = String(format: "Konfigurację zapisano, ale czujniki nie uruchomiły się (ESP 0x%X).", result)
                    refreshToFConfigurations()
                    return
                }
                if operation == 0x0d {
                    message = String(format: "Nie wykryto żadnego VL53L4CD (ESP 0x%X).", result)
                    return
                }
                message = String(format: "Polecenie Aury 0x%02X: błąd ESP 0x%X.", operation, result)
                if operation == 0x03 { torqueEnabled.toggle() }
                return
            }
            switch operation {
            case 0x01:
                message = "Skanowanie zakończone — znaleziono \(data[6]) serw."
            case 0x02: message = "Odczyt serwa zakończony."
            case 0x03: message = torqueEnabled ? "Moment serwa włączony." : "Moment serwa wyłączony."
            case 0x04: message = "Pozycja wysłana do serwa."
            case 0x05: message = "Bieżąca pozycja zapisana jako środek serwa."
            case 0x06:
                selectedServo = Int(data[6])
                message = "Nowy adres serwa: \(data[6])."
                scanServos()
            case 0x07: message = "Polecenie LED wykonane."
            case 0x09:
                torqueEnabled = false
                if let mode = ServoOperatingMode(rawValue: data[6]) { operatingMode = mode }
                message = "Tryb \(operatingMode.title) zapisany. Moment serwa jest wyłączony."
            case 0x0a:
                torqueEnabled = data[6] != 0
                message = torqueEnabled ? "Motor pracuje z zadaną prędkością." : "Motor zatrzymany, moment wyłączony."
            case 0x0b: message = "Konfiguracja VL53L4CD zapisana i zastosowana."
            case 0x0c: break
            case 0x0d:
                let count = Int(data[6])
                message = count == 1 ? "Wykryto jeden VL53L4CD." : "Wykryto \(count) czujniki VL53L4CD."
            case 0x0e:
                message = "Korekta odległości zapisana w czujniku i ESP."
            case 0x10:
                message = "Parametry optyczne VL53L4CD zapisane i zastosowane."
            case 0x11:
                message = "Wykonano aktualizację temperaturową VL53L4CD."
            case 0x0f:
                message = result == 0
                    ? String(format: "MPU6050 gotowy — WHO_AM_I 0x%02X.", data[6])
                    : String(format: "MPU6050 nie odpowiedział — WHO_AM_I 0x%02X.", data[6])
            case 0x29:
                message = data[6] != 0
                    ? "Korekcja postawy zostanie użyta przy następnym uzbrojeniu."
                    : "Korekcja postawy wyłączona."
            case 0x2a:
                message = "Limit korekcji postawy zapisany: \(data[6])°"
            case 0x2b:
                message = "Profil chodu zapisany w Aurze. Zostanie użyty przy następnym kroku."
            case 0x2c:
                message = "Strojenie ruchu zapisane w Aurze. Zostanie użyte przy następnym uzbrojeniu."
            case 0x12: message = "Szukam DualSense. Przytrzymaj Create i PS, aż pasek zacznie migać."
            case 0x13: message = "DualSense rozłączony."
            case 0x14: message = "Łączenie z zapamiętanym padem…"
            case 0x15: message = "Zapamiętane parowanie DualSense usunięte."
            case 0x16: message = "Łączenie z wybranym urządzeniem…"
            case 0x17:
                if robotConfigurationTotal != 0 {
                    robotConfigurationCompleted += 1
                    sendNextRobotAxisConfiguration()
                } else {
                    message = "Przypisanie osi zapisane w pamięci Aury."
                }
            case 0x18: message = data[6] != 0 ? "Serwa Aury uzbrojone." : "Serwa Aury rozbrojone."
            case 0x19:
                let recordedSlot = Int(data[6])
                if let captured = calibrationZeroInFlight,
                   recordedSlot == captured.leg * 3 + captured.axis {
                    completeCalibrationZero(captured)
                } else if let captured = pendingCalibrationReference {
                    pendingCalibrationReference = nil
                    message = "Referencja osi \(captured.axis + 1) została odczytana i zapisana bez ruchu."
                }
            case 0x1a: message = "Wykonano pojedynczy test osi +12° i automatycznie wyłączono moment."
            case 0x1b: message = "Moment wybranej osi jest wyłączony — można ją ustawić ręcznie."
            case 0x1c:
                pendingCalibrationLimit = nil
                message = "Krańcowa pozycja osi została zapisana w Aurze."
            case 0x1d: message = "Stan testu kalibracyjnego nogi został zaktualizowany."
            case 0x1e:
                if let pending = pendingCalibrationTorque, result == 0 {
                    calibrationArmedLeg = pending.enabled ? pending.leg : nil
                }
                pendingCalibrationTorque = nil
                message = calibrationArmedLeg == nil
                    ? "Trzy serwa wybranej nogi są rozbrojone."
                    : "Uzbrojone są wyłącznie trzy serwa wybranej nogi."
            case 0x20:
                message = "Kierunek osi został odwrócony przy pozycji referencyjnej. Moment pozostał wyłączony — wykonaj nowy test +12° dopiero po sprawdzeniu modelu."
            case 0x21:
                message = "Naprawiono wcześniejsze odwrócenie kierunku: znak został zachowany, a zero i krańce przeliczono przy referencji. Moment pozostał wyłączony."
            case 0x22:
                message = "Automatyczne odwrócenie kierunku jest wyłączone."
            case 0x23:
                calibrationLivePreviewActive = pendingCalibrationPreview
                message = calibrationLivePreviewActive
                    ? "Podgląd pozycji TTL działa: tylko odczyt 12 serw, moment wyłączony."
                    : "Podgląd pozycji TTL jest wyłączony."
            case 0x24:
                message = "Skasowano blokadę awaryjną. Serwa pozostają rozbrojone."
            case 0x25:
                message = "Wysokość stania zapisana w Aurze. Przed uzbrojeniem sprawdź model."
            case 0x26:
                message = "Długość, wysokość i szybkość kroku 1× zapisane w Aurze."
            case 0x28:
                message = "Rozstaw końców stóp zapisany w Aurze."
            case 0x1f:
                message = "Pobrano konfigurację osi z Aury i zapisano kopię na komputerze."
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
            replaceGaitProfileLocally(profile)
        case 0x30:
            guard data[1] == 1 else { return }
            let tuning = AuraMotionTuning(
                movingAttitudeGainPermille: Int(u16(data, 2)),
                comMaximumVelocityMmPerSecond: Int(u16(data, 4)),
                comMaximumAccelerationMmPerSecond2: Int(u16(data, 6)))
            guard (100...1000).contains(tuning.movingAttitudeGainPermille),
                  (20...160).contains(tuning.comMaximumVelocityMmPerSecond),
                  (60...900).contains(tuning.comMaximumAccelerationMmPerSecond2) else { return }
            motionTuning = tuning
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
