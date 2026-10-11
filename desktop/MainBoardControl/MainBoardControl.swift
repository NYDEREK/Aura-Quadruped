import AppKit
import Foundation
import SwiftUI


struct ServoFeedback: Decodable {
    let id: Int
    let status_error: Int
    let position: Int
    let angle_deg: Double
    let speed_raw: Int
    let load_raw: Int
    let voltage_v: Double
    let temperature_c: Int
    let moving: Bool
    let current_raw: Int
}

struct Metric: View {
    let title: String
    let value: String
    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(title).font(Theme.label).foregroundStyle(Theme.muted).lineLimit(1)
            Text(value).font(Theme.mono).foregroundStyle(Theme.text).lineLimit(1).minimumScaleFactor(0.7)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(.horizontal, 10).padding(.vertical, 7)
        .background(Theme.canvas, in: RoundedRectangle(cornerRadius: 5))
        .overlay(RoundedRectangle(cornerRadius: 5).stroke(Theme.line, lineWidth: 1))
    }
}

// Redraws with the history (4 Hz), not with every state update of the page.
private struct IMUCharts: View {
    @EnvironmentObject var history: TelemetryHistory

    var body: some View {
        let _ = history.revision
        let samples = history.window(seconds: 30, maximumPoints: 150)
        LazyVGrid(columns: [GridItem(.adaptive(minimum: 420), spacing: 12)], spacing: 12) {
            TimeChart(title: "Tilt", unit: "°", samples: samples, series: [
                Series(name: "roll", color: Theme.accent) { $0.roll },
                Series(name: "pitch", color: Theme.accent2Line) { $0.pitch }], format: "%+.1f")
            TimeChart(title: "Gyroscope", unit: "°/s", samples: samples, series: [
                Series(name: "x", color: Theme.red) { $0.gyro.x },
                Series(name: "y", color: Theme.green) { $0.gyro.y },
                Series(name: "z", color: Theme.blue) { $0.gyro.z }], format: "%+.1f")
            TimeChart(title: "Acceleration", unit: "g", samples: samples, series: [
                Series(name: "x", color: Theme.red) { $0.accel.x },
                Series(name: "y", color: Theme.green) { $0.accel.y },
                Series(name: "z", color: Theme.blue) { $0.accel.z }], format: "%+.2f")
        }
    }
}

struct IMUView: View {
    @EnvironmentObject var aura: AuraConnection

    var body: some View {
        let attitude = aura.attitude
        let available = aura.power.imuAvailable
        Page {
            HStack(spacing: 12) {
                StatusDot(color: available ? Theme.green : Theme.orange,
                          text: available ? "MPU6050 · I²C 0x68 · S1" : "MPU6050 not responding")
                Text(String(format: "WHO_AM_I 0x%02X", aura.power.imuIdentity))
                    .font(Theme.monoSmall).foregroundStyle(Theme.muted)
                Spacer()
                Button("Check MPU6050") { aura.reinitializeIMU() }
                    .controlSize(.small)
                    .disabled(!aura.isConnected || aura.busy)
            }
            if available {
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 120), spacing: 8)], spacing: 8) {
                    Stat(label: "Roll", value: String(format: "%+.1f°", attitude.rollDegrees))
                    Stat(label: "Pitch", value: String(format: "%+.1f°", attitude.pitchDegrees))
                    Stat(label: "Reference", value: String(format: "%+.1f° %+.1f°",
                                                             attitude.referenceRollDegrees, attitude.referencePitchDegrees))
                    Stat(label: "Correction", value: String(format: "%+.1f° %+.1f°",
                                                           attitude.correctionRollDegrees, attitude.correctionPitchDegrees))
                    Stat(label: "Filter", value: attitude.valid ? "ready" : "settling",
                         tint: attitude.valid ? Theme.green : Theme.yellow)
                    Stat(label: "Temperature", value: String(format: "%.1f °C", aura.imu.temperature))
                }
                HStack(alignment: .top, spacing: 12) {
                    Panel("Orientation") {
                        IMUAttitudePreview(rollDegrees: attitude.rollDegrees, pitchDegrees: attitude.pitchDegrees,
                                           valid: attitude.valid)
                            .frame(height: 240)
                            .clipShape(RoundedRectangle(cornerRadius: 4))
                    }
                    .help("Sign test, torque off: lift the nose and the red edge rises; lift the left side and the left edge rises.")
                    Panel("Posture correction") {
                        Toggle("On after arming", isOn: Binding(
                            get: { aura.attitude.enabled }, set: { aura.setAttitudeBalance(enabled: $0) }))
                            .disabled(!aura.isConnected || aura.busy || aura.robotState.armed)
                        Picker("Limit", selection: Binding(
                            get: { Int(aura.attitude.maximumCorrectionDegrees.rounded()) },
                            set: { aura.setAttitudeCorrectionLimit(degrees: Double($0)) })) {
                            Text("7°").tag(7); Text("10°").tag(10); Text("15°").tag(15); Text("20°").tag(20)
                        }
                        .pickerStyle(.segmented)
                        .disabled(!aura.isConnected || aura.busy || aura.robotState.armed)
                        ValueRow(label: "State", value: attitude.referencePending ? "capturing reference"
                                 : attitude.active ? "active" : "off",
                                 tint: attitude.active ? Theme.green : Theme.muted)
                        ValueRow(label: "Samples", value: "\(aura.imu.sampleCount)")
                    }
                    .frame(width: 260)
                }
                IMUCharts()
            } else {
                Text("SDA GPIO25 · SCL GPIO26 · DATA_RDY GPIO14").font(Theme.monoSmall).foregroundStyle(Theme.muted)
            }
        }
    }
}

struct ServoView: View {
    @EnvironmentObject var aura: AuraConnection
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                HStack {
                    if aura.servoIDs.isEmpty {
                        Text("No servos found").font(Theme.body).foregroundStyle(Theme.muted)
                    } else {
                        Picker("Servo", selection: $aura.selectedServo) {
                            ForEach(aura.servoIDs, id: \.self) { Text("ID \($0)").tag($0) }
                        }
                        .frame(maxWidth: 220)
                        .onChange(of: aura.selectedServo) { _, _ in aura.refreshServo() }
                    }
                    Spacer()
                    Button("Scan 0…253") { aura.scanServos() }.disabled(aura.busy || !aura.isConnected)
                }
                GroupBox("Control") {
                    VStack(alignment: .leading, spacing: 12) {
                        HStack {
                            Button(aura.torqueEnabled ? "Torque off" : "Torque on") {
                                aura.setTorque(!aura.torqueEnabled)
                            }
                            Button("Read") { aura.refreshServo() }
                        }
                        Picker("Mode", selection: $aura.operatingMode) {
                            ForEach(ServoOperatingMode.allCases) { mode in
                                Text(mode.title).tag(mode)
                            }
                        }
                        .pickerStyle(.segmented)
                        HStack {
                            Button("Write \(aura.operatingMode.title) mode to servo") {
                                aura.setOperatingMode()
                            }
                            .disabled(aura.busy || !aura.isConnected)
                        }
                        HStack { Text("Acceleration"); Slider(value: $aura.acceleration, in: 0...254); Text("\(Int(aura.acceleration))").monospacedDigit().frame(width: 45) }
                        if aura.operatingMode == .servo {
                            HStack {
                                Button("Set current position as centre") { aura.calibrateCenter() }
                            }
                            HStack { Text("Angle"); Slider(value: $aura.angle, in: 0...360); Text("\(Int(aura.angle))°").monospacedDigit().frame(width: 50) }
                            HStack { Text("Move speed"); Slider(value: $aura.speed, in: 0...3400); Text("\(Int(aura.speed))").monospacedDigit().frame(width: 55) }
                            Button("Move") { aura.moveServo() }.buttonStyle(AuraButtonStyle(prominent: true))
                                .disabled(aura.busy || !aura.isConnected)
                        } else {
                            HStack {
                                Text("Motor speed")
                                Slider(value: $aura.motorSpeed, in: -3400...3400)
                                Text("\(Int(aura.motorSpeed))").monospacedDigit().frame(width: 65)
                            }
                            HStack {
                                Button("Run / set speed") { aura.runMotor() }
                                    .buttonStyle(AuraButtonStyle(prominent: true))
                                    .disabled(aura.busy || !aura.isConnected || Int(aura.motorSpeed) == 0)
                                Button("STOP", role: .destructive) { aura.stopMotor() }
                                    .keyboardShortcut(.escape, modifiers: [])
                                    .disabled(!aura.isConnected)
                            }
                        }
                        if let feedback = aura.feedback {
                            Divider()
                            Text("Position \(feedback.position) • \(String(format: "%.1f°", feedback.angle_deg)) • \(String(format: "%.1f V", feedback.voltage_v)) • \(feedback.temperature_c) °C")
                            Text("Speed \(feedback.speed_raw) • load \(feedback.load_raw) • current raw \(feedback.current_raw)").foregroundStyle(.secondary)
                        }
                    }.frame(maxWidth: .infinity, alignment: .leading)
                }
                GroupBox("Servo ID") {
                    VStack(alignment: .leading, spacing: 10) {
                        Text("Only one servo on the bus").font(Theme.label).foregroundStyle(Theme.yellow)
                        HStack {
                            Stepper("Current ID: \(aura.currentID)", value: $aura.currentID, in: 0...253)
                            Stepper("New ID: \(aura.newID)", value: $aura.newID, in: 0...253)
                            Button("Assign ID") { aura.assignID() }
                                .disabled(aura.currentID == aura.newID || aura.busy || !aura.isConnected)
                        }
                    }.frame(maxWidth: .infinity, alignment: .leading)
                }
            }.padding()
        }
    }
}

struct LEDsView: View {
    @EnvironmentObject var aura: AuraConnection

    var body: some View {
        Page {
            Panel("Strip brightness") {
                HStack {
                    Slider(value: $aura.ledBrightness, in: 0...100, step: 5) { editing in
                        if !editing { aura.saveLEDBrightness() }
                    }
                    Text("\(Int(aura.ledBrightness)) %").font(Theme.mono).frame(width: 50, alignment: .trailing)
                }
                .disabled(!aura.isConnected)
            }
            .frame(maxWidth: 480)
            Panel("Pixels") {
                Grid(alignment: .leading, horizontalSpacing: 18, verticalSpacing: 6) {
                    GridRow { Text("1").foregroundStyle(Theme.muted); Text("state: orange disarmed · green armed · red calibration · blinking stop") }
                    GridRow { Text("2").foregroundStyle(Theme.muted); Text("mode: 1 blue · 2 turquoise · 3 purple · 4 yellow · 5 pink") }
                    GridRow { Text("3–8").foregroundStyle(Theme.muted); Text("battery 10.0–12.6 V · orange below 11.0 V · red at 10.0 V") }
                }
                .font(Theme.body).foregroundStyle(Theme.text)
            }
        }
        .onAppear { if aura.isConnected { aura.readLEDBrightness() } }
    }
}

struct ToFFieldOfView: View {
    let sensor: AuraToFSensor
    let mountingYaw: Double
    let zoneLimits: [Int]

    private var activeZone: Int? {
        guard sensor.valid else { return nil }
        for (index, limit) in zoneLimits.enumerated() where sensor.distance < limit {
            return index
        }
        return nil
    }

    private func color(for zone: Int) -> Color {
        let palette: [Color] = [.red, .orange, .yellow, .green]
        return activeZone == zone ? palette[zone] : Color.secondary.opacity(0.20)
    }

    private var proximityLabel: String {
        guard let activeZone else { return sensor.valid ? "Out of zones" : "—" }
        return ["Red", "Orange", "Yellow", "Green"][activeZone]
    }

    private var zoneLabels: [String] {
        [
            "<\(zoneLimits[0])",
            "\(zoneLimits[0])–\(zoneLimits[1])",
            "\(zoneLimits[1])–\(zoneLimits[2])",
            "\(zoneLimits[2])–\(zoneLimits[3])",
        ]
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text("Distance zones")
                    .font(.headline)
                Spacer()
                Text(proximityLabel)
                    .foregroundStyle(activeZone.map { color(for: $0) } ?? Color.secondary)
                    .font(.subheadline.weight(.medium))
                Text(String(format: "18° · %+.0f°", mountingYaw))
                    .font(.caption).foregroundStyle(.secondary)
            }
            Canvas { context, size in
                let origin = CGPoint(x: size.width / 2, y: size.height + 8)
                let radii: [CGFloat] = [46, 78, 112, 148]
                for (zone, radius) in radii.enumerated() {
                    var arc = Path()
                    arc.addArc(center: origin, radius: radius,
                               startAngle: .degrees(212), endAngle: .degrees(328), clockwise: false)
                    context.stroke(arc, with: .color(color(for: zone)),
                                   style: StrokeStyle(lineWidth: activeZone == zone ? 9 : 6,
                                                      lineCap: .round))
                }

                let car = CGRect(x: origin.x - 28, y: origin.y - 28, width: 56, height: 32)
                context.fill(Path(roundedRect: car, cornerRadius: 9), with: .color(.primary))
                context.fill(Path(roundedRect: car.insetBy(dx: 8, dy: 6), cornerRadius: 5),
                             with: .color(Color.primary.opacity(0.15)))

                if sensor.valid {
                    let progress = min(1.0, max(0.10, CGFloat(sensor.distance) / CGFloat(zoneLimits[3])))
                    let target = CGPoint(x: origin.x, y: origin.y - 148 * progress)
                    context.fill(Path(ellipseIn: CGRect(x: target.x - 5, y: target.y - 5,
                                                        width: 10, height: 10)),
                                 with: .color(activeZone.map { color(for: $0) } ?? Color.secondary))
                }
            }
            .frame(height: 178)
            .background(.quaternary.opacity(0.45), in: RoundedRectangle(cornerRadius: 16))
            HStack(spacing: 0) {
                ForEach(zoneLabels, id: \.self) { label in
                    Text(label + " mm")
                        .frame(maxWidth: .infinity)
                }
            }
            .font(.caption).foregroundStyle(.secondary)
        }
    }
}

struct ToFSensorCard: View {
    @Binding var sensor: AuraToFSensor
    @Binding var mountingYaw: Double
    let zoneLimits: [Int]
    let isConnected: Bool
    let busy: Bool
    let save: () -> Void
    let saveTuning: () -> Void
    let resetTuning: () -> Void
    let updateTemperature: () -> Void
    let calibrate: (Int) -> Void
    let resetCalibration: () -> Void
    @Binding var referenceDistance: Int

    private var detectionMode: Binding<Int> { $sensor.detectionWindow }

    var body: some View {
        let continuous = Binding<Bool>(
            get: { sensor.intermeasurement == 0 },
            set: { enabled in
                sensor.intermeasurement = enabled ? 0 : max(100, sensor.timingBudget + 1)
            })
        GroupBox {
            VStack(alignment: .leading, spacing: 15) {
                HStack(alignment: .firstTextBaseline) {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Sensor \(sensor.portNumber)").font(.title3.bold())
                        Text("Port \(sensor.portNumber) • XSHUT GPIO\(sensor.xshutGPIO)")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                    Spacer()
                    Label(sensor.present ? (sensor.ranging ? "Ranging" : "Detected") : "No response",
                          systemImage: sensor.present ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                        .foregroundStyle(sensor.present ? Color.green : Color.orange)
                }

                HStack(alignment: .firstTextBaseline, spacing: 8) {
                    Text(sensor.valid ? "\(sensor.distance)" : "—")
                        .font(.system(size: 52, weight: .semibold, design: .rounded))
                        .monospacedDigit()
                    Text("mm").font(.title3).foregroundStyle(.secondary)
                    Spacer()
                    VStack(alignment: .trailing, spacing: 3) {
                        Text("Status: \(sensor.rangeStatus)")
                        Text("Samples: \(sensor.sampleCount)")
                        Text(sensor.valid ? "valid" : "rejected")
                    }
                    .font(.caption.monospacedDigit()).foregroundStyle(sensor.valid ? Color.secondary : Color.orange)
                }

                ToFFieldOfView(sensor: sensor, mountingYaw: mountingYaw, zoneLimits: zoneLimits)
                HStack {
                    Text("Mounting direction")
                    Slider(value: $mountingYaw, in: -180...180, step: 1)
                    Text(String(format: "%+.0f°", mountingYaw)).monospacedDigit().frame(width: 46)
                }
                GroupBox("Measurement") {
                    VStack(alignment: .leading, spacing: 10) {
                        HStack {
                            Metric(title: "Signal / SPAD", value: "\(sensor.signalPerSpad) kcps")
                            Metric(title: "Ambient / SPAD", value: "\(sensor.ambientPerSpad) kcps")
                            Metric(title: "Sigma", value: "\(sensor.sigma) mm")
                            Metric(title: "Active SPADs", value: "\(sensor.activeSpads)")
                        }
                        Toggle("Enable ranging", isOn: $sensor.enabled)
                        Stepper(value: $sensor.address, in: 0x08...0x77) {
                            Text(String(format: "I²C address: 0x%02X", sensor.address)).monospacedDigit()
                        }
                        Stepper(value: $sensor.timingBudget, in: 10...200, step: 5) {
                            Text("Timing budget: \(sensor.timingBudget) ms").monospacedDigit()
                        }
                        Toggle("Continuous", isOn: continuous)
                        if !continuous.wrappedValue {
                            Stepper(value: $sensor.intermeasurement,
                                    in: max(11, sensor.timingBudget + 1)...5000, step: 10) {
                                Text("Period: \(sensor.intermeasurement) ms").monospacedDigit()
                            }
                        }
                        Button("Save settings", action: save)
                            .buttonStyle(AuraButtonStyle(prominent: true))
                            .disabled(!isConnected || busy)
                    }
                }

                GroupBox("Distance calibration") {
                    VStack(alignment: .leading, spacing: 10) {
                        Stepper(value: $referenceDistance, in: 10...4000, step: 10) {
                            Text("True target distance: \(referenceDistance) mm")
                        }
                        HStack {
                            Button("Calibrate from current reading") { calibrate(referenceDistance) }
                                .disabled(!isConnected || busy || !sensor.valid)
                            Button("Reset offset", action: resetCalibration)
                                .disabled(!isConnected || busy || sensor.offset == 0)
                        }
                        Text("Offset: \(sensor.offset >= 0 ? "+" : "")\(sensor.offset) mm")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                }

                GroupBox("Optical filters and detection") {
                    VStack(alignment: .leading, spacing: 10) {
                        Stepper(value: $sensor.xtalkKcps, in: 0...128) {
                            Text("XTalk: \(sensor.xtalkKcps) kcps")
                        }
                        Stepper(value: $sensor.signalThresholdKcps, in: 0...16_384, step: 64) {
                            Text("Min signal: \(sensor.signalThresholdKcps) kcps")
                        }
                        Stepper(value: $sensor.sigmaThresholdMm, in: 0...16_383, step: 1) {
                            Text("Max sigma: \(sensor.sigmaThresholdMm) mm")
                        }
                        Divider()
                        Toggle("Threshold detection", isOn: $sensor.detectionEnabled)
                        if sensor.detectionEnabled {
                            if sensor.intermeasurement == 0 {
                                Label("Use a periodic measurement: detection needs a period longer than the budget.",
                                      systemImage: "exclamationmark.triangle.fill")
                                    .font(.caption).foregroundStyle(.orange)
                            }
                            Stepper(value: $sensor.detectionLowMm, in: 0...4000, step: 10) {
                                Text("Low threshold: \(sensor.detectionLowMm) mm")
                            }
                            Stepper(value: $sensor.detectionHighMm, in: 0...4000, step: 10) {
                                Text("High threshold: \(sensor.detectionHighMm) mm")
                            }
                            Picker("Detection condition", selection: detectionMode) {
                                ForEach(ToFDetectionWindow.allCases) { mode in
                                    Text(mode.title).tag(Int(mode.rawValue))
                                }
                            }
                            .pickerStyle(.menu)
                        }
                        HStack {
                            Button("Save filters and thresholds", action: saveTuning)
                                .buttonStyle(AuraButtonStyle(prominent: true))
                                .disabled(!isConnected || busy)
                            Button("Restore safe values", action: resetTuning)
                                .disabled(!isConnected || busy)
                        }
                        Divider()
                        Button("Update temperature compensation", action: updateTemperature)
                            .disabled(!isConnected || busy || !sensor.present)
                    }
                }

                if sensor.error != 0 && !sensor.present {
                    Text(String(format: "Last ESP error: 0x%X", sensor.error))
                        .font(.caption.monospacedDigit()).foregroundStyle(.orange)
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(.vertical, 4)
        }
    }
}

struct ToFView: View {
    @EnvironmentObject var aura: AuraConnection
    @AppStorage("aura.tof.mount_yaw.port_1") private var mountingYaw1 = 0.0
    @AppStorage("aura.tof.mount_yaw.port_2") private var mountingYaw2 = 0.0
    @AppStorage("aura.tof.zone.red") private var redZone = 250
    @AppStorage("aura.tof.zone.orange") private var orangeZone = 500
    @AppStorage("aura.tof.zone.yellow") private var yellowZone = 900
    @AppStorage("aura.tof.zone.green") private var greenZone = 1300

    private var zoneLimits: [Int] { [redZone, orangeZone, yellowZone, greenZone] }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                HStack {
                    Text("VL53L4CD · detected: \(aura.detectedToFSensorIndices.count)")
                        .font(Theme.body).foregroundStyle(Theme.muted)
                    Spacer()
                    Button("Detect S2 again") { aura.reinitializeToF() }
                        .controlSize(.small)
                        .disabled(aura.busy || !aura.isConnected)
                }

                GroupBox("Colour zones") {
                    VStack(alignment: .leading, spacing: 8) {
                        Stepper(value: $redZone, in: 10...max(10, orangeZone - 10), step: 10) {
                            Text("Red: up to \(redZone) mm")
                        }
                        Stepper(value: $orangeZone, in: redZone + 10...max(redZone + 10, yellowZone - 10), step: 10) {
                            Text("Orange: up to \(orangeZone) mm")
                        }
                        Stepper(value: $yellowZone, in: orangeZone + 10...max(orangeZone + 10, greenZone - 10), step: 10) {
                            Text("Yellow: up to \(yellowZone) mm")
                        }
                        Stepper(value: $greenZone, in: yellowZone + 10...4000, step: 10) {
                            Text("Green: up to \(greenZone) mm")
                        }
                    }
                    .monospacedDigit()
                }

                if aura.detectedToFSensorIndices.isEmpty {
                    Text("No VL53L4CD on S2").font(Theme.body).foregroundStyle(Theme.muted)
                } else {
                    ForEach(aura.detectedToFSensorIndices, id: \.self) { index in
                        ToFSensorCard(sensor: $aura.tofSensors[index],
                                      mountingYaw: index == 0 ? $mountingYaw1 : $mountingYaw2,
                                      zoneLimits: zoneLimits,
                                      isConnected: aura.isConnected,
                                      busy: aura.busy,
                                      save: { aura.configureToF(index: index) },
                                      saveTuning: { aura.configureToFTuning(index: index) },
                                      resetTuning: { aura.resetToFTuning(index: index) },
                                      updateTemperature: { aura.updateToFTemperature(index: index) },
                                      calibrate: { referenceDistance in
                            aura.calibrateToF(index: index, referenceDistance: referenceDistance)
                        }, resetCalibration: {
                            aura.resetToFCalibration(index: index)
                        }, referenceDistance: $aura.tofReferenceDistance)
                    }
                }
            }
            .padding()
        }
    }
}

struct DualSenseStick: View {
    let title: String
    let position: CGPoint

    var body: some View {
        VStack(spacing: 7) {
            GeometryReader { geometry in
                let inset: CGFloat = 18
                let x = inset + CGFloat(position.x) * max(0, geometry.size.width - inset * 2)
                let y = inset + CGFloat(position.y) * max(0, geometry.size.height - inset * 2)
                ZStack {
                    RoundedRectangle(cornerRadius: 18)
                        .fill(.quaternary.opacity(0.48))
                    Path { path in
                        path.move(to: CGPoint(x: geometry.size.width / 2, y: 14))
                        path.addLine(to: CGPoint(x: geometry.size.width / 2, y: geometry.size.height - 14))
                        path.move(to: CGPoint(x: 14, y: geometry.size.height / 2))
                        path.addLine(to: CGPoint(x: geometry.size.width - 14, y: geometry.size.height / 2))
                    }
                    .stroke(.secondary.opacity(0.34), style: StrokeStyle(lineWidth: 1, dash: [3, 3]))
                    Circle()
                        .fill(.blue.gradient)
                        .frame(width: 28, height: 28)
                        .shadow(color: .blue.opacity(0.22), radius: 5, y: 2)
                        .position(x: x, y: y)
                }
            }
            .frame(width: 132, height: 132)
            Text(title).font(.caption).foregroundStyle(.secondary)
        }
    }
}

struct DualSenseButtonTile: View {
    let button: AuraDualSenseButton

    var body: some View {
        HStack(spacing: 7) {
            Circle()
                .fill(button.pressed ? Color.accentColor : Color.secondary.opacity(0.24))
                .frame(width: 8, height: 8)
            Text(button.title).font(.caption.weight(.medium))
            Spacer(minLength: 0)
        }
        .padding(.horizontal, 9).padding(.vertical, 7)
        .background(button.pressed ? Color.accentColor.opacity(0.13) : Color.gray.opacity(0.11),
                    in: RoundedRectangle(cornerRadius: 8))
    }
}

struct DualSenseView: View {
    @EnvironmentObject var aura: AuraConnection

    private var statusColor: Color {
        switch aura.dualSense.state {
        case .connected: .green
        case .scanning, .connecting: .orange
        case .error: .red
        default: .secondary
        }
    }

    private var batteryText: String {
        guard let battery = aura.dualSense.battery else { return "—" }
        return "\(battery)%"
    }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                HStack(spacing: 10) {
                    StatusDot(color: statusColor, text: aura.dualSense.isConnected && !aura.dualSense.hasInput
                              ? "No current data" : aura.dualSense.state.title)
                    Spacer()
                    if aura.dualSense.state == .scanning || aura.dualSense.state == .connecting {
                        ProgressView().controlSize(.small)
                    }
                }

                HStack(spacing: 10) {
                    Button("Connect") { aura.connectSavedDualSense() }
                        .buttonStyle(AuraButtonStyle(prominent: true))
                        .disabled(!aura.isConnected || aura.busy || !aura.dualSense.hasSavedController || aura.dualSense.isConnected)
                    Button("New pairing") { aura.pairDualSense() }
                        .disabled(!aura.isConnected || aura.busy || aura.dualSense.isConnected)
                    Button("Disconnect") { aura.disconnectDualSense() }
                        .disabled(!aura.isConnected || aura.busy || !aura.dualSense.isConnected)
                    Spacer()
                    Button("Forget") { aura.forgetDualSense() }
                        .disabled(!aura.isConnected || aura.busy || !aura.dualSense.hasSavedController || aura.dualSense.isConnected)
                }

                GroupBox("Devices") {
                    if aura.dualSenseDevices.isEmpty {
                        Text(aura.dualSense.state == .scanning
                             ? "Scanning. Pairing mode: hold Create + PS."
                             : "Press New pairing to list nearby devices.")
                            .font(.callout).foregroundStyle(.secondary)
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .padding(.vertical, 4)
                    } else {
                        VStack(spacing: 8) {
                            ForEach(aura.dualSenseDevices) { device in
                                HStack(spacing: 10) {
                                    Image(systemName: "gamecontroller")
                                        .foregroundStyle(device.name.contains("Controller") ? Color.accentColor : Color.secondary)
                                    VStack(alignment: .leading, spacing: 2) {
                                        Text(device.displayName).font(.callout.weight(.medium))
                                        Text("\(device.id)  •  \(device.signalText)")
                                            .font(.caption.monospaced()).foregroundStyle(.secondary)
                                    }
                                    Spacer()
                                    Button("Select") { aura.connectDualSense(device) }
                                        .buttonStyle(AuraButtonStyle())
                                        .disabled(!aura.isConnected || aura.busy || aura.dualSense.isConnected)
                                }
                                .padding(.vertical, 2)
                            }
                        }
                    }
                }

                GroupBox {
                    HStack {
                        DualSenseStick(title: "Left", position: aura.dualSense.leftPosition)
                        Spacer(minLength: 28)
                        DualSenseStick(title: "Right", position: aura.dualSense.rightPosition)
                    }
                    .frame(maxWidth: 340)
                    .padding(.vertical, 5)
                }

                HStack {
                    Metric(title: "L2", value: "\(aura.dualSense.leftTrigger)")
                    Metric(title: "R2", value: "\(aura.dualSense.rightTrigger)")
                    Metric(title: "Battery", value: batteryText)
                    Metric(title: "Reports", value: "\(aura.dualSense.sampleCount)")
                }

                GroupBox("Buttons") {
                    LazyVGrid(columns: Array(repeating: GridItem(.flexible(), spacing: 7), count: 4), spacing: 7) {
                        ForEach(aura.dualSense.buttonStates) { button in
                            DualSenseButtonTile(button: button)
                        }
                    }
                    .padding(.vertical, 3)
                }

                if aura.dualSense.hasInput {
                    Text("HID report \(String(format: "0x%02X", aura.dualSense.reportID))  •  \(aura.dualSense.reportLength) B")
                        .font(.caption.monospaced()).foregroundStyle(.secondary)
                }

                if aura.dualSense.hasSavedController {
                    Text("\(aura.dualSense.addressText)  •  VID \(String(format: "%04X", aura.dualSense.vendorID))  •  PID \(String(format: "%04X", aura.dualSense.productID))")
                        .font(.caption.monospaced()).foregroundStyle(.secondary)
                }
            }
            .padding()
        }
    }
}

private enum AuraSection: String, CaseIterable, Identifiable {
    case overview, charts, route, sessions, robot3d, robot, calibration, ikLab, servos, imu, pad, sensors, leds, firmware

    var id: Self { self }
    var title: String {
        switch self {
        case .overview: "Overview"
        case .charts: "Charts"
        case .route: "Route"
        case .sessions: "Sessions"
        case .robot3d: "Robot 3D"
        case .robot: "Configuration"
        case .calibration: "Calibration"
        case .ikLab: "IK Test"
        case .servos: "Servos"
        case .imu: "IMU"
        case .pad: "Controller"
        case .sensors: "Sensors"
        case .leds: "LED"
        case .firmware: "Firmware"
        }
    }
    static let groups: [(String, [AuraSection])] = [
        ("Monitor", [.overview, .charts, .route, .sessions, .robot3d]),
        ("Robot", [.robot, .calibration, .ikLab, .servos]),
        ("Board", [.imu, .pad, .sensors, .leds, .firmware]),
    ]
}

private final class AuraNavigationStore: ObservableObject {
    @Published var section: AuraSection = .overview
}

private struct Sidebar: View {
    @ObservedObject var navigation: AuraNavigationStore

    var body: some View {
        VStack(alignment: .leading, spacing: 0) {
            Text("Aura").font(.system(size: 13, weight: .semibold)).foregroundStyle(Theme.text)
                .padding(.horizontal, 14).padding(.top, 12).padding(.bottom, 10)
            ForEach(Array(AuraSection.groups.enumerated()), id: \.offset) { _, group in
                Text(group.0).font(Theme.label).foregroundStyle(Theme.faint)
                    .padding(.horizontal, 14).padding(.top, 10).padding(.bottom, 4)
                ForEach(group.1) { section in
                    let index = AuraSection.allCases.firstIndex(of: section) ?? 0
                    let selected = navigation.section == section
                    Button { navigation.section = section } label: {
                        Text(section.title)
                            .font(.system(size: 12))
                            .foregroundStyle(selected ? Theme.text : Theme.muted)
                            .frame(maxWidth: .infinity, alignment: .leading)
                            .padding(.horizontal, 10).padding(.vertical, 5)
                            .background(selected ? Theme.accent2.opacity(0.55) : .clear,
                                        in: RoundedRectangle(cornerRadius: 4))
                            .contentShape(Rectangle())
                    }
                    .buttonStyle(.plain)
                    .padding(.horizontal, 6)
                    .keyboardShortcut(index < 9 ? KeyEquivalent(Character("\(index + 1)")) : KeyEquivalent(" "),
                                      modifiers: index < 9 ? .command : [.command, .option, .control, .shift])
                }
            }
            Spacer()
        }
        .frame(width: 168)
        .background(Theme.panel)
    }
}

private struct TopBar: View {
    @EnvironmentObject var aura: AuraConnection
    let title: String

    var body: some View {
        let state = aura.robotState
        let voltage = aura.power.servoBusVoltage
        HStack(spacing: 16) {
            Text(title).font(.system(size: 13, weight: .semibold)).foregroundStyle(Theme.text)
            Spacer(minLength: 8)
            if aura.isConnected {
                StatusDot(color: state.safetyFault.latched ? Theme.red : state.armed ? Theme.green : Theme.yellow,
                          text: state.safetyFault.latched ? "Fault" : state.armed ? "Armed" : "Disarmed")
                Text(AuraPadMode.title(state.selectedGait)).font(Theme.label).foregroundStyle(Theme.muted)
                    .lineLimit(1)
                Text(String(format: "%.2f V", voltage)).font(Theme.monoSmall)
                    .foregroundStyle(voltage <= 10 ? Theme.red : voltage < 11 ? Theme.orange : Theme.text)
            }
            StatusDot(color: aura.isConnected ? Theme.green : Theme.orange,
                      text: aura.isConnected ? "Wi-Fi" : "Searching")
            if aura.busy { ProgressView().controlSize(.mini) }
            Button("Reconnect") { aura.reconnect() }.buttonStyle(AuraButtonStyle(compact: true))
        }
        .padding(.horizontal, 14)
        .frame(height: 38)
        .background(Theme.panel)
        .overlay(alignment: .bottom) { Rectangle().fill(Theme.line).frame(height: 1) }
    }
}

private struct StatusBar: View {
    @EnvironmentObject var aura: AuraConnection

    var body: some View {
        HStack(spacing: 12) {
            if !aura.message.isEmpty {
                Text(aura.message).font(Theme.label).foregroundStyle(Theme.text).lineLimit(1)
                    .truncationMode(.tail).help(aura.message)
                Button("×") { aura.message = "" }.buttonStyle(.plain).foregroundStyle(Theme.muted)
            }
            Spacer(minLength: 8)
            if aura.isConnected && !aura.health.stalledTasks.isEmpty {
                Text("stalled: " + aura.health.stalledTasks.joined(separator: ", "))
                    .font(Theme.monoSmall).foregroundStyle(Theme.red).lineLimit(1)
            }
            if aura.isConnected && aura.health.resetAbnormal {
                Text("last start: " + aura.health.resetText)
                    .font(Theme.monoSmall).foregroundStyle(Theme.orange).lineLimit(1)
            }
            if aura.isConnected {
                Text(String(format: "up %.0f s", Double(aura.power.uptimeMilliseconds) / 1000))
                    .font(Theme.monoSmall).foregroundStyle(Theme.faint)
            }
        }
        .padding(.horizontal, 12)
        .frame(height: 24)
        .background(Theme.panel)
        .overlay(alignment: .top) { Rectangle().fill(Theme.line).frame(height: 1) }
    }
}

// Files opened from Finder (double-clicked .aura sessions).
final class AuraAppDelegate: NSObject, NSApplicationDelegate {
    static let openSession = Notification.Name("AuraOpenSession")

    func application(_ application: NSApplication, open urls: [URL]) {
        for url in urls where url.pathExtension == "aura" {
            NotificationCenter.default.post(name: Self.openSession, object: url)
        }
    }
}

// The window's long-lived models. Nothing here is published, so the window
// root never redraws on telemetry: each page observes only the models it
// shows, and a 10 Hz update redraws that page instead of the whole window.
@MainActor
private final class AuraModels: ObservableObject {
    let aura = AuraConnection()
    let robot = RobotControlStore()
    let calibration = CalibrationStore()
}

// Keeps the Mac's calibration backup in step with what Aura reports, as the
// root used to; in its own view so the root need not observe the connection.
private struct AxisBackupSync: View {
    @EnvironmentObject var aura: AuraConnection
    @EnvironmentObject var robot: RobotControlStore

    var body: some View {
        Color.clear
            .frame(width: 0, height: 0)
            .onChange(of: aura.robotAxisBackup) { _, records in
                robot.applyAuraConfigurations(records)
            }
    }
}

struct ContentView: View {
    @StateObject private var models = AuraModels()
    @StateObject private var navigation = AuraNavigationStore()
    private var aura: AuraConnection { models.aura }

    @ViewBuilder
    private var content: some View {
        // One live panel at a time: a TabView would keep every chart and
        // 3D scene alive behind the visible one.
        switch navigation.section {
        case .overview: OverviewView()
        case .charts: ChartsView()
        case .route: RouteView()
        case .sessions: SessionsView()
        case .robot3d: QuadrupedView(channel: aura.robotRenderChannel, aura: aura)
        case .robot: RobotControlView()
        case .calibration: CalibrationView()
        case .ikLab: IKLabView(aura: aura)
        case .servos: ServoView()
        case .imu: IMUView()
        case .pad: DualSenseView()
        case .sensors: ToFView()
        case .leds: LEDsView()
        case .firmware: FirmwareView()
        }
    }

    var body: some View {
        HStack(spacing: 0) {
            Sidebar(navigation: navigation)
            Rectangle().fill(Theme.line).frame(width: 1)
            VStack(spacing: 0) {
                TopBar(title: navigation.section.title)
                content
                    .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
                    .background(Theme.canvas)
                StatusBar()
            }
        }
        .frame(minWidth: 900, minHeight: 600)
        .background(Theme.canvas)
        .preferredColorScheme(.dark)
        .environment(\.locale, Locale(identifier: "en_US"))
        .tint(Theme.accent2)
        .buttonStyle(AuraButtonStyle())
        .groupBoxStyle(AuraGroupBoxStyle())
        .background(AxisBackupSync())
        .environmentObject(aura)
        .environmentObject(aura.history)
        .environmentObject(aura.route)
        .environmentObject(aura.sessions)
        .environmentObject(models.robot)
        .environmentObject(models.calibration)
        .onReceive(NotificationCenter.default.publisher(for: AuraAppDelegate.openSession)) { note in
            guard let url = note.object as? URL else { return }
            aura.sessions.open(url)
            navigation.section = .sessions
        }
    }
}

@main
struct MainBoardControlApp: App {
    @NSApplicationDelegateAdaptor(AuraAppDelegate.self) private var delegate

    init() { NSApplication.shared.appearance = NSAppearance(named: .darkAqua) }

    var body: some Scene {
        WindowGroup("Aura") { ContentView() }
            .windowStyle(.titleBar)
            .windowToolbarStyle(.unifiedCompact)
            .defaultSize(width: 1280, height: 820)
    }
}
