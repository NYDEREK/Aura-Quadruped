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

struct IMUView: View {
    @EnvironmentObject var aura: AuraConnection
    @EnvironmentObject var history: TelemetryHistory

    var body: some View {
        let _ = history.revision
        let samples = history.window(seconds: 30, maximumPoints: 240)
        let attitude = aura.attitude
        let available = aura.power.imuAvailable
        Page {
            HStack(spacing: 12) {
                StatusDot(color: available ? Theme.green : Theme.orange,
                          text: available ? "MPU6050 · I²C 0x68 · S1" : "MPU6050 nie odpowiada")
                Text(String(format: "WHO_AM_I 0x%02X", aura.power.imuIdentity))
                    .font(Theme.monoSmall).foregroundStyle(Theme.muted)
                Spacer()
                Button("Sprawdź MPU6050") { aura.reinitializeIMU() }
                    .controlSize(.small)
                    .disabled(!aura.isConnected || aura.busy)
            }
            if available {
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 120), spacing: 8)], spacing: 8) {
                    Stat(label: "Roll", value: String(format: "%+.1f°", attitude.rollDegrees))
                    Stat(label: "Pitch", value: String(format: "%+.1f°", attitude.pitchDegrees))
                    Stat(label: "Referencja", value: String(format: "%+.1f° %+.1f°",
                                                             attitude.referenceRollDegrees, attitude.referencePitchDegrees))
                    Stat(label: "Korekcja", value: String(format: "%+.1f° %+.1f°",
                                                           attitude.correctionRollDegrees, attitude.correctionPitchDegrees))
                    Stat(label: "Filtr", value: attitude.valid ? "gotowy" : "stabilizacja",
                         tint: attitude.valid ? Theme.green : Theme.yellow)
                    Stat(label: "Temperatura", value: String(format: "%.1f °C", aura.imu.temperature))
                }
                HStack(alignment: .top, spacing: 12) {
                    Panel("Orientacja") {
                        IMUAttitudePreview(rollDegrees: attitude.rollDegrees, pitchDegrees: attitude.pitchDegrees,
                                           valid: attitude.valid)
                            .frame(height: 240)
                            .clipShape(RoundedRectangle(cornerRadius: 4))
                    }
                    .help("Test znaku bez momentu: unieś przód — czerwona krawędź modelu idzie w górę; unieś lewy bok — lewa krawędź idzie w górę.")
                    Panel("Korekcja postawy") {
                        Toggle("Włączona po uzbrojeniu", isOn: Binding(
                            get: { aura.attitude.enabled }, set: { aura.setAttitudeBalance(enabled: $0) }))
                            .disabled(!aura.isConnected || aura.busy || aura.robotState.armed)
                        Picker("Limit", selection: Binding(
                            get: { Int(aura.attitude.maximumCorrectionDegrees.rounded()) },
                            set: { aura.setAttitudeCorrectionLimit(degrees: Double($0)) })) {
                            Text("7°").tag(7); Text("10°").tag(10); Text("15°").tag(15); Text("20°").tag(20)
                        }
                        .pickerStyle(.segmented)
                        .disabled(!aura.isConnected || aura.busy || aura.robotState.armed)
                        ValueRow(label: "Stan", value: attitude.referencePending ? "ustalanie referencji"
                                 : attitude.active ? "aktywna" : "wyłączona",
                                 tint: attitude.active ? Theme.green : Theme.muted)
                        ValueRow(label: "Próbki", value: "\(aura.imu.sampleCount)")
                    }
                    .frame(width: 260)
                }
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 420), spacing: 12)], spacing: 12) {
                    TimeChart(title: "Przechył", unit: "°", samples: samples, series: [
                        Series(name: "roll", color: Theme.blue) { $0.roll },
                        Series(name: "pitch", color: Theme.orange) { $0.pitch }], format: "%+.1f")
                    TimeChart(title: "Żyroskop", unit: "°/s", samples: samples, series: [
                        Series(name: "x", color: Theme.red) { $0.gyro.x },
                        Series(name: "y", color: Theme.green) { $0.gyro.y },
                        Series(name: "z", color: Theme.blue) { $0.gyro.z }], format: "%+.1f")
                    TimeChart(title: "Przyspieszenie", unit: "g", samples: samples, series: [
                        Series(name: "x", color: Theme.red) { $0.accel.x },
                        Series(name: "y", color: Theme.green) { $0.accel.y },
                        Series(name: "z", color: Theme.blue) { $0.accel.z }], format: "%+.2f")
                }
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
                        Text("Brak wykrytych serw").font(Theme.body).foregroundStyle(Theme.muted)
                    } else {
                        Picker("Serwo", selection: $aura.selectedServo) {
                            ForEach(aura.servoIDs, id: \.self) { Text("ID \($0)").tag($0) }
                        }
                        .frame(maxWidth: 220)
                        .onChange(of: aura.selectedServo) { _, _ in aura.refreshServo() }
                    }
                    Spacer()
                    Button("Skanuj 0…253") { aura.scanServos() }.disabled(aura.busy || !aura.isConnected)
                }
                GroupBox("Sterowanie") {
                    VStack(alignment: .leading, spacing: 12) {
                        HStack {
                            Button(aura.torqueEnabled ? "Wyłącz moment" : "Włącz moment") {
                                aura.setTorque(!aura.torqueEnabled)
                            }
                            Button("Odczytaj") { aura.refreshServo() }
                        }
                        Picker("Tryb pracy", selection: $aura.operatingMode) {
                            ForEach(ServoOperatingMode.allCases) { mode in
                                Text(mode.title).tag(mode)
                            }
                        }
                        .pickerStyle(.segmented)
                        HStack {
                            Button("Zapisz tryb \(aura.operatingMode.title) w serwie") {
                                aura.setOperatingMode()
                            }
                            .disabled(aura.busy || !aura.isConnected)
                        }
                        HStack { Text("Przyspieszenie"); Slider(value: $aura.acceleration, in: 0...254); Text("\(Int(aura.acceleration))").monospacedDigit().frame(width: 45) }
                        if aura.operatingMode == .servo {
                            HStack {
                                Button("Ustaw bieżącą pozycję jako środek") { aura.calibrateCenter() }
                            }
                            HStack { Text("Kąt"); Slider(value: $aura.angle, in: 0...360); Text("\(Int(aura.angle))°").monospacedDigit().frame(width: 50) }
                            HStack { Text("Prędkość ruchu"); Slider(value: $aura.speed, in: 0...3400); Text("\(Int(aura.speed))").monospacedDigit().frame(width: 55) }
                            Button("Ustaw pozycję") { aura.moveServo() }.buttonStyle(.borderedProminent)
                                .disabled(aura.busy || !aura.isConnected)
                        } else {
                            HStack {
                                Text("Prędkość motoru")
                                Slider(value: $aura.motorSpeed, in: -3400...3400)
                                Text("\(Int(aura.motorSpeed))").monospacedDigit().frame(width: 65)
                            }
                            HStack {
                                Button("Uruchom / ustaw prędkość") { aura.runMotor() }
                                    .buttonStyle(.borderedProminent)
                                    .disabled(aura.busy || !aura.isConnected || Int(aura.motorSpeed) == 0)
                                Button("STOP", role: .destructive) { aura.stopMotor() }
                                    .keyboardShortcut(.escape, modifiers: [])
                                    .disabled(!aura.isConnected)
                            }
                        }
                        if let feedback = aura.feedback {
                            Divider()
                            Text("Pozycja \(feedback.position) • \(String(format: "%.1f°", feedback.angle_deg)) • \(String(format: "%.1f V", feedback.voltage_v)) • \(feedback.temperature_c) °C")
                            Text("Prędkość \(feedback.speed_raw) • obciążenie \(feedback.load_raw) • prąd raw \(feedback.current_raw)").foregroundStyle(.secondary)
                        }
                    }.frame(maxWidth: .infinity, alignment: .leading)
                }
                GroupBox("Nadawanie adresu") {
                    VStack(alignment: .leading, spacing: 10) {
                        Text("Tylko jedno serwo na magistrali").font(Theme.label).foregroundStyle(Theme.yellow)
                        HStack {
                            Stepper("Obecny ID: \(aura.currentID)", value: $aura.currentID, in: 0...253)
                            Stepper("Nowy ID: \(aura.newID)", value: $aura.newID, in: 0...253)
                            Button("Nadaj adres") { aura.assignID() }
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
            Panel("Jasność paska") {
                HStack {
                    Slider(value: $aura.ledBrightness, in: 0...100, step: 5) { editing in
                        if !editing { aura.saveLEDBrightness() }
                    }
                    Text("\(Int(aura.ledBrightness)) %").font(Theme.mono).frame(width: 50, alignment: .trailing)
                }
                .disabled(!aura.isConnected)
            }
            .frame(maxWidth: 480)
            Panel("Diody") {
                Grid(alignment: .leading, horizontalSpacing: 18, verticalSpacing: 6) {
                    GridRow { Text("1").foregroundStyle(Theme.muted); Text("stan: pomarańczowy rozbrojony · zielony uzbrojony · czerwony kalibracja · miga blokada") }
                    GridRow { Text("2").foregroundStyle(Theme.muted); Text("tryb: 1 niebieski · 2 turkusowy · 3 fioletowy · 4 żółty · 5 różowy") }
                    GridRow { Text("3–8").foregroundStyle(Theme.muted); Text("bateria 10,0–12,6 V · pomarańczowy < 11,0 V · czerwony ≤ 10,0 V") }
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
        guard let activeZone else { return sensor.valid ? "Poza strefami" : "—" }
        return ["Czerwona", "Pomarańczowa", "Żółta", "Zielona"][activeZone]
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
                Text("Strefy odległości")
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
                        Text("Czujnik \(sensor.portNumber)").font(.title3.bold())
                        Text("Port \(sensor.portNumber) • XSHUT GPIO\(sensor.xshutGPIO)")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                    Spacer()
                    Label(sensor.present ? (sensor.ranging ? "Pomiar aktywny" : "Wykryty") : "Brak odpowiedzi",
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
                        Text("Próbki: \(sensor.sampleCount)")
                        Text(sensor.valid ? "wynik poprawny" : "wynik odrzucony")
                    }
                    .font(.caption.monospacedDigit()).foregroundStyle(sensor.valid ? Color.secondary : Color.orange)
                }

                ToFFieldOfView(sensor: sensor, mountingYaw: mountingYaw, zoneLimits: zoneLimits)
                HStack {
                    Text("Kierunek montażu")
                    Slider(value: $mountingYaw, in: -180...180, step: 1)
                    Text(String(format: "%+.0f°", mountingYaw)).monospacedDigit().frame(width: 46)
                }
                GroupBox("Pomiar") {
                    VStack(alignment: .leading, spacing: 10) {
                        HStack {
                            Metric(title: "Sygnał / SPAD", value: "\(sensor.signalPerSpad) kcps")
                            Metric(title: "Tło / SPAD", value: "\(sensor.ambientPerSpad) kcps")
                            Metric(title: "Sigma", value: "\(sensor.sigma) mm")
                            Metric(title: "Aktywne SPAD", value: "\(sensor.activeSpads)")
                        }
                        Toggle("Włącz pomiary", isOn: $sensor.enabled)
                        Stepper(value: $sensor.address, in: 0x08...0x77) {
                            Text(String(format: "Adres I²C: 0x%02X", sensor.address)).monospacedDigit()
                        }
                        Stepper(value: $sensor.timingBudget, in: 10...200, step: 5) {
                            Text("Budżet pomiaru: \(sensor.timingBudget) ms").monospacedDigit()
                        }
                        Toggle("Pomiar ciągły", isOn: continuous)
                        if !continuous.wrappedValue {
                            Stepper(value: $sensor.intermeasurement,
                                    in: max(11, sensor.timingBudget + 1)...5000, step: 10) {
                                Text("Okres pomiaru: \(sensor.intermeasurement) ms").monospacedDigit()
                            }
                        }
                        Button("Zapisz podstawowe ustawienia", action: save)
                            .buttonStyle(.borderedProminent)
                            .disabled(!isConnected || busy)
                    }
                }

                GroupBox("Kalibracja odległości") {
                    VStack(alignment: .leading, spacing: 10) {
                        Stepper(value: $referenceDistance, in: 10...4000, step: 10) {
                            Text("Rzeczywista odległość celu: \(referenceDistance) mm")
                        }
                        HStack {
                            Button("Skalibruj z bieżącego pomiaru") { calibrate(referenceDistance) }
                                .disabled(!isConnected || busy || !sensor.valid)
                            Button("Wyzeruj korektę", action: resetCalibration)
                                .disabled(!isConnected || busy || sensor.offset == 0)
                        }
                        Text("Korekta: \(sensor.offset >= 0 ? "+" : "")\(sensor.offset) mm")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                }

                GroupBox("Filtry optyczne i detekcja") {
                    VStack(alignment: .leading, spacing: 10) {
                        Stepper(value: $sensor.xtalkKcps, in: 0...128) {
                            Text("XTalk: \(sensor.xtalkKcps) kcps")
                        }
                        Stepper(value: $sensor.signalThresholdKcps, in: 0...16_384, step: 64) {
                            Text("Minimalny sygnał: \(sensor.signalThresholdKcps) kcps")
                        }
                        Stepper(value: $sensor.sigmaThresholdMm, in: 0...16_383, step: 1) {
                            Text("Maksymalna sigma: \(sensor.sigmaThresholdMm) mm")
                        }
                        Divider()
                        Toggle("Detekcja progu", isOn: $sensor.detectionEnabled)
                        if sensor.detectionEnabled {
                            if sensor.intermeasurement == 0 {
                                Label("Ustaw pomiar okresowy — detekcja progu wymaga okresu większego od budżetu.",
                                      systemImage: "exclamationmark.triangle.fill")
                                    .font(.caption).foregroundStyle(.orange)
                            }
                            Stepper(value: $sensor.detectionLowMm, in: 0...4000, step: 10) {
                                Text("Dolny próg: \(sensor.detectionLowMm) mm")
                            }
                            Stepper(value: $sensor.detectionHighMm, in: 0...4000, step: 10) {
                                Text("Górny próg: \(sensor.detectionHighMm) mm")
                            }
                            Picker("Warunek detekcji", selection: detectionMode) {
                                ForEach(ToFDetectionWindow.allCases) { mode in
                                    Text(mode.title).tag(Int(mode.rawValue))
                                }
                            }
                            .pickerStyle(.menu)
                        }
                        HStack {
                            Button("Zapisz filtry i progi", action: saveTuning)
                                .buttonStyle(.borderedProminent)
                                .disabled(!isConnected || busy)
                            Button("Przywróć bezpieczne wartości", action: resetTuning)
                                .disabled(!isConnected || busy)
                        }
                        Divider()
                        Button("Aktualizuj kompensację temperatury", action: updateTemperature)
                            .disabled(!isConnected || busy || !sensor.present)
                    }
                }

                if sensor.error != 0 && !sensor.present {
                    Text(String(format: "Ostatni błąd ESP: 0x%X", sensor.error))
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
                    Text("VL53L4CD · wykryte: \(aura.detectedToFSensorIndices.count)")
                        .font(Theme.body).foregroundStyle(Theme.muted)
                    Spacer()
                    Button("Wykryj S2 ponownie") { aura.reinitializeToF() }
                        .controlSize(.small)
                        .disabled(aura.busy || !aura.isConnected)
                }

                GroupBox("Strefy kolorów") {
                    VStack(alignment: .leading, spacing: 8) {
                        Stepper(value: $redZone, in: 10...max(10, orangeZone - 10), step: 10) {
                            Text("Czerwona: do \(redZone) mm")
                        }
                        Stepper(value: $orangeZone, in: redZone + 10...max(redZone + 10, yellowZone - 10), step: 10) {
                            Text("Pomarańczowa: do \(orangeZone) mm")
                        }
                        Stepper(value: $yellowZone, in: orangeZone + 10...max(orangeZone + 10, greenZone - 10), step: 10) {
                            Text("Żółta: do \(yellowZone) mm")
                        }
                        Stepper(value: $greenZone, in: yellowZone + 10...4000, step: 10) {
                            Text("Zielona: do \(greenZone) mm")
                        }
                    }
                    .monospacedDigit()
                }

                if aura.detectedToFSensorIndices.isEmpty {
                    Text("Nie wykryto VL53L4CD na S2").font(Theme.body).foregroundStyle(Theme.muted)
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
                              ? "Brak aktualnych danych" : aura.dualSense.state.title)
                    Spacer()
                    if aura.dualSense.state == .scanning || aura.dualSense.state == .connecting {
                        ProgressView().controlSize(.small)
                    }
                }

                HStack(spacing: 10) {
                    Button("Połącz") { aura.connectSavedDualSense() }
                        .buttonStyle(.borderedProminent)
                        .disabled(!aura.isConnected || aura.busy || !aura.dualSense.hasSavedController || aura.dualSense.isConnected)
                    Button("Nowe parowanie") { aura.pairDualSense() }
                        .disabled(!aura.isConnected || aura.busy || aura.dualSense.isConnected)
                    Button("Rozłącz") { aura.disconnectDualSense() }
                        .disabled(!aura.isConnected || aura.busy || !aura.dualSense.isConnected)
                    Spacer()
                    Button("Zapomnij") { aura.forgetDualSense() }
                        .disabled(!aura.isConnected || aura.busy || !aura.dualSense.hasSavedController || aura.dualSense.isConnected)
                }

                GroupBox("Urządzenia") {
                    if aura.dualSenseDevices.isEmpty {
                        Text(aura.dualSense.state == .scanning
                             ? "Skanowanie trwa. Włącz tryb parowania: Create + PS."
                             : "Naciśnij „Nowe parowanie”, aby zobaczyć urządzenia w pobliżu.")
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
                                    Button("Wybierz") { aura.connectDualSense(device) }
                                        .buttonStyle(.bordered)
                                        .disabled(!aura.isConnected || aura.busy || aura.dualSense.isConnected)
                                }
                                .padding(.vertical, 2)
                            }
                        }
                    }
                }

                GroupBox {
                    HStack {
                        DualSenseStick(title: "Lewy", position: aura.dualSense.leftPosition)
                        Spacer(minLength: 28)
                        DualSenseStick(title: "Prawy", position: aura.dualSense.rightPosition)
                    }
                    .frame(maxWidth: 340)
                    .padding(.vertical, 5)
                }

                HStack {
                    Metric(title: "L2", value: "\(aura.dualSense.leftTrigger)")
                    Metric(title: "R2", value: "\(aura.dualSense.rightTrigger)")
                    Metric(title: "Bateria", value: batteryText)
                    Metric(title: "Raporty", value: "\(aura.dualSense.sampleCount)")
                }

                GroupBox("Przyciski") {
                    LazyVGrid(columns: Array(repeating: GridItem(.flexible(), spacing: 7), count: 4), spacing: 7) {
                        ForEach(aura.dualSense.buttonStates) { button in
                            DualSenseButtonTile(button: button)
                        }
                    }
                    .padding(.vertical, 3)
                }

                if aura.dualSense.hasInput {
                    Text("Raport HID \(String(format: "0x%02X", aura.dualSense.reportID))  •  \(aura.dualSense.reportLength) B")
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
    case overview, charts, route, robot3d, robot, calibration, ikLab, servos, imu, pad, sensors, leds, firmware

    var id: Self { self }
    var title: String {
        switch self {
        case .overview: "Pulpit"
        case .charts: "Wykresy"
        case .route: "Trasa"
        case .robot3d: "Robot 3D"
        case .robot: "Konfiguracja"
        case .calibration: "Kalibracja"
        case .ikLab: "Test IK"
        case .servos: "Serwa"
        case .imu: "IMU"
        case .pad: "Pad"
        case .sensors: "Czujniki"
        case .leds: "LED"
        case .firmware: "Firmware"
        }
    }
    static let groups: [(String, [AuraSection])] = [
        ("Podgląd", [.overview, .charts, .route, .robot3d]),
        ("Robot", [.robot, .calibration, .ikLab, .servos]),
        ("Płytka", [.imu, .pad, .sensors, .leds, .firmware]),
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
                            .background(selected ? Theme.accent.opacity(0.28) : .clear,
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
                          text: state.safetyFault.latched ? "Blokada" : state.armed ? "Uzbrojony" : "Rozbrojony")
                Text(AuraPadMode.title(state.selectedGait)).font(Theme.label).foregroundStyle(Theme.muted)
                    .lineLimit(1)
                Text(String(format: "%.2f V", voltage)).font(Theme.monoSmall)
                    .foregroundStyle(voltage <= 10 ? Theme.red : voltage < 11 ? Theme.orange : Theme.text)
            }
            StatusDot(color: aura.isConnected ? Theme.green : Theme.orange,
                      text: aura.isConnected ? "Wi-Fi" : "Szukam Aury")
            if aura.busy { ProgressView().controlSize(.mini) }
            Button("Połącz ponownie") { aura.reconnect() }.controlSize(.small)
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
            Text(aura.connectionText).font(Theme.label).foregroundStyle(Theme.faint).lineLimit(1)
            if aura.isConnected {
                Text(String(format: "%.0f s", Double(aura.power.uptimeMilliseconds) / 1000))
                    .font(Theme.monoSmall).foregroundStyle(Theme.faint)
            }
        }
        .padding(.horizontal, 12)
        .frame(height: 24)
        .background(Theme.panel)
        .overlay(alignment: .top) { Rectangle().fill(Theme.line).frame(height: 1) }
    }
}

struct ContentView: View {
    @StateObject private var aura = AuraConnection()
    @StateObject private var robot = RobotControlStore()
    @StateObject private var calibration = CalibrationStore()
    @StateObject private var navigation = AuraNavigationStore()

    @ViewBuilder
    private var content: some View {
        // One live panel at a time: a TabView would keep every chart and
        // 3D scene alive behind the visible one.
        switch navigation.section {
        case .overview: OverviewView()
        case .charts: ChartsView()
        case .route: RouteView()
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
        .tint(Theme.accent)
        .groupBoxStyle(AuraGroupBoxStyle())
        .environmentObject(aura)
        .environmentObject(aura.history)
        .environmentObject(aura.route)
        .environmentObject(robot)
        .environmentObject(calibration)
        .onChange(of: aura.robotAxisBackup) { _, records in
            robot.applyAuraConfigurations(records)
        }
    }
}

@main
struct MainBoardControlApp: App {
    init() { NSApplication.shared.appearance = NSAppearance(named: .darkAqua) }

    var body: some Scene {
        WindowGroup("Aura") { ContentView() }
            .windowStyle(.titleBar)
            .windowToolbarStyle(.unifiedCompact)
            .defaultSize(width: 1280, height: 820)
    }
}
