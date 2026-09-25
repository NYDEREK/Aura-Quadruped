import Foundation
import SwiftUI

enum RobotLegID: String, CaseIterable, Codable, Identifiable {
    case lf = "LF", rf = "RF", lr = "LR", rr = "RR"
    var id: String { rawValue }

    // Matches Aura's stable logical slots: LF, RF, LR, RR.
    var wireIndex: Int {
        switch self {
        case .lf: 0
        case .rf: 1
        case .lr: 2
        case .rr: 3
        }
    }

    init?(wireIndex: Int) {
        switch wireIndex {
        case 0: self = .lf
        case 1: self = .rf
        case 2: self = .lr
        case 3: self = .rr
        default: return nil
        }
    }

    var isLeft: Bool { self == .lf || self == .lr }

    static let physicalOrder: [Self] = [.lf, .lr, .rf, .rr]
    var displayCode: String {
        switch self {
        case .lf: "FL"
        case .lr: "RL"
        case .rf: "FR"
        case .rr: "RR"
        }
    }
    var title: String {
        switch self {
        case .lf: "Lewy przód"
        case .rf: "Prawy przód"
        case .lr: "Lewy tył"
        case .rr: "Prawy tył"
        }
    }
}

enum RobotAxisID: String, CaseIterable, Codable, Identifiable {
    case abduction = "Ab/ad", hip = "Hip", knee = "Knee"
    var id: String { rawValue }
}

struct RobotAxisConfiguration: Identifiable, Codable, Equatable {
    var leg: RobotLegID
    var axis: RobotAxisID
    var assigned = false
    var servoID = 1
    var reversed = false
    var centerTick = 2048
    var minimumDegrees = -90.0
    var maximumDegrees = 90.0
    var speedLimitRaw = 3400
    var acceleration = 254
    var targetDegrees = 0.0
    var measuredDegrees: Double?
    var id: String { "\(leg.rawValue)-\(axis.rawValue)" }

    var directionText: String { reversed ? "−" : "+" }
    var status: String {
        guard assigned else { return "nieprzypisane" }
        return measuredDegrees == nil ? "gotowe do testu" : "telemetria"
    }
}

struct RobotCalibrationBackup: Codable {
    let formatVersion: Int
    let exportedAt: Date
    let axes: [RobotAxisConfiguration]
}

@MainActor
final class RobotControlStore: ObservableObject {
    @Published var axes: [RobotAxisConfiguration]
    @Published private(set) var telemetry = (0..<4).map { AuraRobotLegTelemetry(id: $0) }
    // This gates only the local preview. It never sends a torque or movement
    // command to a board; the physical arm state will remain separate.
    @Published var simulationArmed = true

    private let persistenceKey = "aura.robot.axis-configuration.v1"
    private let canonicalLayoutMigrationKey = "aura.robot.canonical-layout.v3"
    private let backupURL = FileManager.default.homeDirectoryForCurrentUser
        .appendingPathComponent("Documents/Aura/robot-calibration-backup.json")

    var backupPath: String { backupURL.path }

    private static func defaultAxis(leg: RobotLegID, axis: RobotAxisID) -> RobotAxisConfiguration {
        var result = RobotAxisConfiguration(leg: leg, axis: axis)
        // The knee's kinematic coordinate is measured at the joint and needs
        // more than ±90° around the configured mechanical zero. Hip and ab/ad
        // remain conservative until they are commissioned individually.
        if axis == .knee {
            result.minimumDegrees = -170
            result.maximumDegrees = 170
        }
        return result
    }

    private static func migrateKneeDefaults(_ source: [RobotAxisConfiguration]) -> [RobotAxisConfiguration] {
        source.map { axis in
            guard axis.axis == .knee, axis.minimumDegrees == -90, axis.maximumDegrees == 90 else {
                return axis
            }
            var migrated = axis
            migrated.minimumDegrees = -170
            migrated.maximumDegrees = 170
            return migrated
        }
    }

    init() {
        if let data = UserDefaults.standard.data(forKey: persistenceKey),
           let decoded = try? JSONDecoder().decode([RobotAxisConfiguration].self, from: data),
           decoded.count == RobotLegID.allCases.count * RobotAxisID.allCases.count {
            var migrated = Self.migrateKneeDefaults(decoded)
            if !UserDefaults.standard.bool(forKey: canonicalLayoutMigrationKey) {
                migrated = Self.canonicalizeNumberedLayout(migrated)
                UserDefaults.standard.set(true, forKey: canonicalLayoutMigrationKey)
            }
            axes = migrated
            if axes != decoded, let encoded = try? JSONEncoder().encode(axes) {
                UserDefaults.standard.set(encoded, forKey: persistenceKey)
            }
        } else {
            axes = RobotLegID.allCases.flatMap { leg in
                RobotAxisID.allCases.map { axis in
                    Self.defaultAxis(leg: leg, axis: axis)
                }
            }
        }
    }

    private static func canonicalizeNumberedLayout(_ source: [RobotAxisConfiguration]) -> [RobotAxisConfiguration] {
        source.map { configuration in
            guard configuration.assigned, (1...12).contains(configuration.servoID) else { return configuration }
            var canonical = configuration
            switch (configuration.servoID - 1) / 3 {
            case 0: canonical.leg = .lf
            case 1: canonical.leg = .lr
            case 2: canonical.leg = .rf
            default: canonical.leg = .rr
            }
            canonical.axis = RobotAxisID.allCases[(configuration.servoID - 1) % 3]
            return canonical
        }
    }

    func save() {
        guard let encoded = try? JSONEncoder().encode(axes) else { return }
        UserDefaults.standard.set(encoded, forKey: persistenceKey)
        let backup = RobotCalibrationBackup(formatVersion: 1, exportedAt: Date(), axes: axes)
        let encoder = JSONEncoder()
        encoder.dateEncodingStrategy = .iso8601
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        guard let archive = try? encoder.encode(backup) else { return }
        do {
            try FileManager.default.createDirectory(at: backupURL.deletingLastPathComponent(),
                                                    withIntermediateDirectories: true)
            try archive.write(to: backupURL, options: .atomic)
        } catch {
            // UserDefaults remains the local live copy; a transient Documents
            // permission or volume error must not alter robot configuration.
        }
    }

    func reset() {
        axes = RobotLegID.allCases.flatMap { leg in
            RobotAxisID.allCases.map { Self.defaultAxis(leg: leg, axis: $0) }
        }
        save()
    }

    // Commissioning starts with the IDs found on the live half-duplex bus.
    // This only creates an axis map in the app; it never enables torque or
    // sends a position to a servo.  Per-axis direction, centre and limits
    // remain editable before the explicit arm action.
    @discardableResult
    func assignDetectedServos(_ ids: [Int]) -> Bool {
        let ordered = Array(Set(ids)).sorted()
        guard ordered.count >= axes.count else { return false }
        for (legOffset, leg) in RobotLegID.physicalOrder.enumerated() {
            for (axisOffset, axis) in RobotAxisID.allCases.enumerated() {
                guard let index = axes.firstIndex(where: { $0.leg == leg && $0.axis == axis }) else { continue }
                axes[index].assigned = true
                axes[index].servoID = ordered[legOffset * RobotAxisID.allCases.count + axisOffset]
            }
        }
        save()
        return true
    }

    func hasDuplicateAssignedID(_ candidate: RobotAxisConfiguration) -> Bool {
        axes.contains { $0.id != candidate.id && $0.assigned && $0.servoID == candidate.servoID }
    }

    var assignedCount: Int { axes.filter(\.assigned).count }
    var isComplete: Bool { assignedCount == axes.count && !axes.contains { hasDuplicateAssignedID($0) } }

    func apply(_ samples: [AuraRobotLegTelemetry]) {
        guard samples.count == RobotLegID.allCases.count else { return }
        telemetry = samples
    }

    func applyCalibration(_ configuration: AuraCalibrationAxisConfiguration) {
        applyAuraConfigurations([configuration])
    }

    // The board is authoritative for a recovered configuration. Applying a
    // received record updates only this desktop mirror and its backup file;
    // there is deliberately no command sent back to Aura here.
    func applyAuraConfigurations(_ configurations: [AuraCalibrationAxisConfiguration]) {
        guard !configurations.isEmpty else { return }
        var updated = axes
        var changed = false
        for configuration in configurations {
            guard let leg = RobotLegID(wireIndex: configuration.leg),
                  RobotAxisID.allCases.indices.contains(configuration.axis) else { continue }
            let axis = RobotAxisID.allCases[configuration.axis]
            guard let index = updated.firstIndex(where: { $0.leg == leg && $0.axis == axis }) else { continue }
            updated[index].assigned = configuration.servoID != 0xff
            updated[index].servoID = configuration.servoID
            updated[index].reversed = configuration.direction < 0
            updated[index].centerTick = configuration.centerTick
            updated[index].minimumDegrees = configuration.minimumDegrees
            updated[index].maximumDegrees = configuration.maximumDegrees
            updated[index].speedLimitRaw = configuration.speedLimitRaw
            updated[index].acceleration = configuration.acceleration
            changed = true
        }
        guard changed else { return }
        axes = updated
        save()
    }

    func telemetry(for axis: RobotAxisConfiguration) -> AuraRobotLegTelemetry {
        let leg = axis.leg.wireIndex
        return telemetry[leg]
    }
}

private struct RobotAxisCard: View {
    @Binding var axis: RobotAxisConfiguration
    let duplicateID: Bool
    let telemetry: AuraRobotLegTelemetry
    let save: () -> Void

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text(axis.axis.rawValue).font(.headline)
                Spacer()
                Toggle("", isOn: $axis.assigned).labelsHidden().toggleStyle(.switch)
            }
            HStack {
                Text("ID").foregroundStyle(.secondary)
                Stepper(value: $axis.servoID, in: 0...253) {
                    Text("\(axis.servoID)").monospacedDigit()
                }
            }
            Picker("Kierunek", selection: $axis.reversed) {
                Text("+").tag(false); Text("−").tag(true)
            }.pickerStyle(.segmented)
            Grid(alignment: .leading, horizontalSpacing: 8, verticalSpacing: 7) {
                GridRow { Text("Zero").foregroundStyle(.secondary); TextField("", value: $axis.centerTick, format: .number).multilineTextAlignment(.trailing); Text("tick").foregroundStyle(.secondary) }
                GridRow { Text("Min").foregroundStyle(.secondary); TextField("", value: $axis.minimumDegrees, format: .number.precision(.fractionLength(0))).multilineTextAlignment(.trailing); Text("°").foregroundStyle(.secondary) }
                GridRow { Text("Max").foregroundStyle(.secondary); TextField("", value: $axis.maximumDegrees, format: .number.precision(.fractionLength(0))).multilineTextAlignment(.trailing); Text("°").foregroundStyle(.secondary) }
                GridRow { Text("Prędkość").foregroundStyle(.secondary); TextField("", value: $axis.speedLimitRaw, format: .number).multilineTextAlignment(.trailing); Text("raw").foregroundStyle(.secondary) }
                GridRow { Text("Accel").foregroundStyle(.secondary); TextField("", value: $axis.acceleration, format: .number).multilineTextAlignment(.trailing); Text("raw").foregroundStyle(.secondary) }
            }.font(.caption.monospacedDigit())
            Divider()
            let joint = RobotAxisID.allCases.firstIndex(of: axis.axis) ?? 0
            Text(String(format: "Cel z Aury  %+.1f°", telemetry.targetDegrees[joint]))
                .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
            if telemetry.present[joint] {
                Text(String(format: "Odczyt       %+.1f°  •  %d°C", telemetry.measuredDegrees[joint], telemetry.temperature[joint]))
                    .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
            } else if axis.assigned {
                Text("Brak zwrotu z serwa").font(.caption).foregroundStyle(.orange)
            }
            Label(duplicateID ? "Ten ID jest przypisany drugi raz" : axis.status,
                  systemImage: duplicateID ? "exclamationmark.triangle.fill" : axis.assigned ? "checkmark.circle" : "circle")
                .font(.caption).foregroundStyle(duplicateID ? .orange : .secondary)
        }
        .padding(12)
        .frame(width: 225, alignment: .leading)
        .background(.quaternary.opacity(0.35), in: RoundedRectangle(cornerRadius: 12))
        .onChange(of: axis) { _, _ in save() }
    }
}

struct RobotControlView: View {
    @EnvironmentObject private var robot: RobotControlStore
    @EnvironmentObject private var aura: AuraConnection

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                HStack(alignment: .firstTextBaseline) {
                    VStack(alignment: .leading, spacing: 3) {
                        Text("Robot").font(.title2.bold())
                        Text("Serwo → oś → noga → ciało").foregroundStyle(.secondary)
                    }
                    Spacer()
                    Label(aura.robotState.armed ? "Serwa uzbrojone" : "Serwa rozbrojone",
                          systemImage: aura.robotState.armed ? "bolt.fill" : "bolt.slash")
                        .foregroundStyle(aura.robotState.armed ? .orange : .secondary)
                    Button("Pobierz z Aury") { aura.requestRobotConfiguration() }
                        .disabled(!aura.isConnected || aura.busy)
                    Button("Zapisz do Aury") { aura.configureRobot(robot.axes) }
                        .disabled(!aura.isConnected || aura.robotState.armed)
                    Button(aura.robotState.armed ? "Rozbrój" : "Uzbrój 12 serw") {
                        aura.setRobotArmed(!aura.robotState.armed)
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(!aura.isConnected || aura.robotState.safetyFault.latched ||
                              (!aura.robotState.armed && !robot.isComplete))
                    Button("Wyczyść") { robot.reset() }.buttonStyle(.bordered)
                }

                if aura.robotState.safetyFault.latched {
                    let fault = aura.robotState.safetyFault
                    GroupBox {
                        HStack(alignment: .center, spacing: 14) {
                            Image(systemName: "exclamationmark.octagon.fill")
                                .font(.title2).foregroundStyle(.red)
                            VStack(alignment: .leading, spacing: 3) {
                                Text("Zatrzymanie awaryjne — moment wyłączony")
                                    .font(.headline).foregroundStyle(.red)
                                Text(String(format: "%@ · %@ · ID %d  |  %.2f A, %.0f%% PWM, błąd %.1f°",
                                            fault.legTitle, fault.axisTitle, fault.servoID,
                                            fault.currentAmperes, fault.loadPercent,
                                            fault.trackingErrorDegrees))
                                    .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                                Text("Sprawdź nogę i mechanikę. Skasowanie blokady nie uzbraja serw ani nie wysyła ruchu.")
                                    .font(.caption).foregroundStyle(.secondary)
                            }
                            Spacer()
                            Button("Skasuj blokadę") { aura.clearRobotSafetyFault() }
                                .buttonStyle(.bordered)
                                .disabled(!aura.isConnected || aura.busy || aura.robotState.armed)
                        }
                        .padding(.vertical, 4)
                    } label: {
                        Text("Ochrona serw")
                    }
                }

                HStack(spacing: 10) {
                    Button("Skanuj magistralę") { aura.scanServos() }
                        .disabled(!aura.isConnected || aura.busy || aura.robotState.armed)
                    Button("Wstaw wykryte ID") {
                        if !robot.assignDetectedServos(aura.servoIDs) {
                            aura.message = "Do automatycznego przypisania potrzebne jest 12 różnych serw na magistrali."
                        }
                    }
                    .disabled(!aura.isConnected || aura.servoIDs.count < 12 || aura.robotState.armed)
                    Text("Wykryte: \(aura.servoIDs.count) / 12")
                        .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                }

                HStack(spacing: 12) {
                    Metric(title: "Przypisane osie", value: "\(robot.assignedCount) / 12")
                    Metric(title: "Backend", value: "Aura / ESP32")
                    Metric(title: "Sterownik", value: "50 Hz")
                    Metric(title: "Pakiet", value: "SYNC_WRITE")
                }

                if aura.robotState.trackingLimited {
                    Label(String(format: "Aura płynnie redukuje tempo kroku do %.0f%%, bo odczyt serw jest za celem.", aura.robotState.phaseRate * 100), systemImage: "speedometer")
                        .font(.callout).foregroundStyle(.orange)
                }

                Text("Najpierw zeskanuj magistralę i wstaw wykryte ID. To nie włącza momentu. Kierunek, zero i limity są własnością osi, więc lustrzane nogi nie wymagają wyjątków w kinematyce.")
                    .font(.callout).foregroundStyle(.secondary)
                Text("Kopia konfiguracji: \(robot.backupPath)")
                    .font(.caption.monospaced()).foregroundStyle(.secondary)

                ForEach(RobotLegID.physicalOrder) { leg in
                    GroupBox(leg.title) {
                        HStack(alignment: .top, spacing: 12) {
                            ForEach(robot.axes.indices.filter { robot.axes[$0].leg == leg }, id: \.self) { index in
                                RobotAxisCard(axis: $robot.axes[index], duplicateID: robot.hasDuplicateAssignedID(robot.axes[index]), telemetry: robot.telemetry(for: robot.axes[index]), save: robot.save)
                            }
                        }
                    }
                }

                GroupBox("Stan wykonawczy") {
                    VStack(alignment: .leading, spacing: 6) {
                        Text("Aura zapisuje przypisania osi w swojej pamięci i liczy gait oraz IK co 20 ms. Komputer odbiera tylko cele i ostatnie odczyty zwrotne serw.")
                        Text("Wybranie „Uzbrój 12 serw” jest możliwe dopiero po przypisaniu wszystkich osi. Bez serw można bezpiecznie zapisać konfigurację — firmware pozostaje rozbrojony.")
                    }.font(.caption).foregroundStyle(.secondary)
                }
            }
            .padding()
        }
        .onChange(of: aura.robotLegTelemetry) { _, samples in robot.apply(samples) }
    }
}
