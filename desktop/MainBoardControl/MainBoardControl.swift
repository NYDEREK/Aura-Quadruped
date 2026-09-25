import AppKit
import Foundation
import SwiftUI


private struct AuraGroupBoxStyle: GroupBoxStyle {
    func makeBody(configuration: Configuration) -> some View {
        VStack(alignment: .leading, spacing: 10) {
            configuration.label
                .font(.headline)
            configuration.content
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(14)
        .background(Color.white.opacity(0.055), in: RoundedRectangle(cornerRadius: 14, style: .continuous))
        .overlay {
            RoundedRectangle(cornerRadius: 14, style: .continuous)
                .stroke(Color.white.opacity(0.09), lineWidth: 1)
        }
    }
}

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
        VStack(alignment: .leading, spacing: 5) {
            Text(title).font(.caption).foregroundStyle(.secondary)
            Text(value).font(.title3.monospacedDigit()).fontWeight(.semibold)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(12).background(.quaternary.opacity(0.5), in: RoundedRectangle(cornerRadius: 10))
    }
}

struct CurrentChart: View {
    let samples: [CurrentPoint]

    var body: some View {
        GeometryReader { geometry in
            let values = samples.isEmpty ? [0.0] : samples.map(\.amperes)
            let low = min(values.min() ?? 0, 0)
            let high = max(values.max() ?? 0, 0)
            let span = max(high - low, 0.01)
            let width = geometry.size.width
            let height = geometry.size.height
            let step = samples.count > 1 ? width / CGFloat(samples.count - 1) : 0
            let zeroY = height - CGFloat((0 - low) / span) * height

            ZStack(alignment: .topLeading) {
                RoundedRectangle(cornerRadius: 8).fill(.quaternary.opacity(0.35))
                Path { path in
                    path.move(to: CGPoint(x: 0, y: zeroY))
                    for (index, sample) in samples.enumerated() {
                        let y = height - CGFloat((sample.amperes - low) / span) * height
                        path.addLine(to: CGPoint(x: CGFloat(index) * step, y: y))
                    }
                    path.addLine(to: CGPoint(x: width, y: zeroY))
                    path.closeSubpath()
                }
                .fill(.blue.opacity(0.15))
                Path { path in
                    for (index, sample) in samples.enumerated() {
                        let point = CGPoint(
                            x: CGFloat(index) * step,
                            y: height - CGFloat((sample.amperes - low) / span) * height)
                        if index == 0 { path.move(to: point) } else { path.addLine(to: point) }
                    }
                }
                .stroke(.blue, style: StrokeStyle(lineWidth: 2, lineJoin: .round))
                Path { path in
                    path.move(to: CGPoint(x: 0, y: zeroY))
                    path.addLine(to: CGPoint(x: width, y: zeroY))
                }
                .stroke(.secondary.opacity(0.45), style: StrokeStyle(lineWidth: 1, dash: [4, 4]))
                VStack(alignment: .leading) {
                    Text(String(format: "%.3f A", high))
                    Spacer()
                    Text(String(format: "%.3f A", low))
                }
                .font(.caption2.monospacedDigit())
                .foregroundStyle(.secondary)
                .padding(6)
            }
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Wykres prądu serw z ostatniej minuty")
        .accessibilityValue(String(format: "%.3f A", samples.last?.amperes ?? 0))
    }
}

struct DashboardView: View {
    @EnvironmentObject var aura: AuraConnection
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                Text("Aura Main Board").font(.title2.bold())
                if aura.isConnected {
                    HStack {
                        Metric(title: "Magistrala serw", value: String(format: "%.2f V", aura.power.servoBusVoltage))
                        Metric(title: "Prąd", value: String(format: "%.3f A", aura.power.current))
                        Metric(title: "Moc", value: String(format: "%.3f W", aura.power.power))
                        Metric(title: "Zużycie serw", value: String(format: "%.3f mAh", aura.power.servoConsumedMilliampHours))
                        Metric(title: "VIN", value: String(format: "%.2f V", aura.power.inputVoltage))
                    }
                    GroupBox("Prąd serw • ostatnia minuta") {
                        CurrentChart(samples: aura.currentHistory)
                        .frame(height: 210)
                        .padding(.top, 6)
                    }
                    GroupBox("MPU6050 · S1") {
                        if aura.power.imuAvailable {
                            VStack(alignment: .leading) {
                                Text(String(format: "Accel: %.3f  %.3f  %.3f g", aura.imu.acceleration[0], aura.imu.acceleration[1], aura.imu.acceleration[2]))
                                Text(String(format: "Roll / pitch: %.1f° / %.1f°", aura.attitude.rollDegrees, aura.attitude.pitchDegrees))
                                Text(String(format: "Temperatura: %.1f °C", aura.imu.temperature))
                            }.frame(maxWidth: .infinity, alignment: .leading)
                        } else {
                            Text(String(format: "Brak odpowiedzi — WHO_AM_I 0x%02X", aura.power.imuIdentity))
                                .foregroundStyle(.orange).frame(maxWidth: .infinity, alignment: .leading)
                        }
                    }
                } else {
                    ContentUnavailableView("Brak danych", systemImage: "waveform.slash", description: Text("Połącz aplikację z płytką przez Wi-Fi."))
                }
            }.padding()
        }
    }
}

struct IMUView: View {
    @EnvironmentObject var aura: AuraConnection

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                HStack {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("MPU6050 · S1").font(.title2.bold())
                        Text("I²C na GPIO25/26, DATA_RDY na GPIO14.")
                            .foregroundStyle(.secondary)
                    }
                    Spacer()
                    Button("Sprawdź MPU6050") { aura.reinitializeIMU() }
                        .buttonStyle(.borderedProminent)
                        .disabled(!aura.isConnected || aura.busy)
                }

                GroupBox("Stan komunikacji") {
                    VStack(alignment: .leading, spacing: 10) {
                        HStack {
                            Label(aura.power.imuAvailable ? "MPU6050 odpowiada" : "Brak odpowiedzi MPU6050",
                                  systemImage: aura.power.imuAvailable
                                      ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                                .foregroundStyle(aura.power.imuAvailable ? .green : .orange)
                            Spacer()
                            Text(String(format: "WHO_AM_I: 0x%02X", aura.power.imuIdentity))
                                .monospacedDigit()
                        }
                        HStack {
                            Metric(title: "Próbki", value: "\(aura.imu.sampleCount)")
                            Metric(title: "Oczekiwane ID", value: "0x68")
                            Metric(title: "Interfejs", value: "I²C · S1")
                        }
                    }
                }

                if aura.power.imuAvailable {
                    GroupBox("Postawa korpusu") {
                        HStack {
                            Metric(title: "Roll", value: String(format: "%.1f°", aura.attitude.rollDegrees))
                            Metric(title: "Pitch", value: String(format: "%.1f°", aura.attitude.pitchDegrees))
                            Metric(title: "Filtr", value: aura.attitude.valid ? "gotowy" : "stabilizacja…")
                        }
                        .padding(.bottom, 6)
                        HStack {
                            Metric(title: "Referencja roll", value: String(format: "%.1f°", aura.attitude.referenceRollDegrees))
                            Metric(title: "Referencja pitch", value: String(format: "%.1f°", aura.attitude.referencePitchDegrees))
                            Metric(title: "Korekta", value: aura.attitude.referencePending
                                ? "ustalanie pozycji…"
                                : (aura.attitude.active ? "aktywna" : "wyłączona"))
                        }

                        Toggle("Korekcja postawy po uzbrojeniu", isOn: Binding(
                            get: { aura.attitude.enabled },
                            set: { aura.setAttitudeBalance(enabled: $0) }
                        ))
                        .disabled(!aura.isConnected || aura.busy || aura.robotState.armed)
                        .padding(.top, 10)

                        Picker("Maksymalna korekta", selection: Binding(
                            get: { Int(aura.attitude.maximumCorrectionDegrees.rounded()) },
                            set: { aura.setAttitudeCorrectionLimit(degrees: Double($0)) }
                        )) {
                            Text("7°").tag(7)
                            Text("10°").tag(10)
                            Text("15°").tag(15)
                            Text("20°").tag(20)
                        }
                        .pickerStyle(.segmented)
                        .disabled(!aura.isConnected || aura.busy || aura.robotState.armed)

                        Text("Po Create Aura najpierw spokojnie dojeżdża do pozycji stania, a potem zapisuje ją jako referencję IMU. Dzięki temu luźna pozycja sprzed uzbrojenia nie staje się błędnym poziomem korpusu. Korekta działa również w staniu. Wybrany limit jest \(Int(aura.attitude.maximumCorrectionDegrees.rounded()))°; przy błędzie postawy większym niż 25° Aura zatrzymuje korektę zamiast próbować ratować upadek dużym ruchem.")
                            .font(.caption).foregroundStyle(.secondary)
                    }

                    GroupBox("Kalibracja kierunku IMU") {
                        IMUAttitudePreview(rollDegrees: aura.attitude.rollDegrees,
                                           pitchDegrees: aura.attitude.pitchDegrees,
                                           valid: aura.attitude.valid)
                            .frame(height: 300)
                            .clipShape(RoundedRectangle(cornerRadius: 10))

                        VStack(alignment: .leading, spacing: 5) {
                            Text("Model ma stałą kamerę i jest obracany tylko przez filtr MPU6050. Czerwona krawędź oznacza przód ramy; oś czerwona X wskazuje przód, zielona Y górę, niebieska Z lewą stronę.")
                            Text("Test bez momentu: unieś przód robota — czerwona krawędź na modelu też musi pójść do góry. Następnie unieś lewą stronę — lewa krawędź modelu ma pójść do góry. Jeśli którykolwiek znak jest odwrotny, nie włączaj korekcji postawy; zmienimy mapowanie osi w jednym miejscu w firmware.")
                        }
                        .font(.caption)
                        .foregroundStyle(.secondary)
                        .padding(.top, 6)
                    }

                    GroupBox("Przyspieszenie i prędkość kątowa") {
                        HStack {
                            Metric(title: "Accel X", value: String(format: "%.3f g", aura.imu.acceleration[0]))
                            Metric(title: "Accel Y", value: String(format: "%.3f g", aura.imu.acceleration[1]))
                            Metric(title: "Accel Z", value: String(format: "%.3f g", aura.imu.acceleration[2]))
                            Metric(title: "Temperatura", value: String(format: "%.1f °C", aura.imu.temperature))
                        }
                        HStack {
                            Metric(title: "Gyro X", value: String(format: "%.2f °/s", aura.imu.gyroscope[0]))
                            Metric(title: "Gyro Y", value: String(format: "%.2f °/s", aura.imu.gyroscope[1]))
                            Metric(title: "Gyro Z", value: String(format: "%.2f °/s", aura.imu.gyroscope[2]))
                        }
                        .padding(.top, 8)
                    }
                } else {
                    ContentUnavailableView(
                        "MPU6050 nie odpowiada po I²C",
                        systemImage: "waveform.path.ecg.rectangle",
                        description: Text("Sprawdź zasilanie 3,3 V oraz SDA GPIO25, SCL GPIO26 i ponów test."))
                }

                GroupBox("Montaż i połączenia") {
                    VStack(alignment: .leading, spacing: 5) {
                        Text("SDA GPIO25    SCL GPIO26    INT / DATA_RDY GPIO14")
                            .font(.body.monospaced())
                        Text("Kod zakłada montaż: oś X modułu do przodu, Y w lewo, Z do góry. Przed włączeniem korekcji sprawdź na rozbrojonym robocie, czy przechylenie w każdą stronę daje właściwy znak roll/pitch.")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                }
            }
            .padding()
        }
    }
}

struct ServoView: View {
    @EnvironmentObject var aura: AuraConnection
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                HStack {
                    Text("Serwa ST3215").font(.title2.bold())
                    Spacer()
                    Button("Skanuj 0…253") { aura.scanServos() }.disabled(aura.busy || !aura.isConnected)
                }
                if aura.servoIDs.isEmpty {
                    Text("Nie znaleziono jeszcze serw. Podłącz zasilanie magistrali i uruchom skanowanie.").foregroundStyle(.secondary)
                } else {
                    Picker("Serwo", selection: $aura.selectedServo) {
                        ForEach(aura.servoIDs, id: \.self) { Text("ID \($0)").tag($0) }
                    }.onChange(of: aura.selectedServo) { _, _ in aura.refreshServo() }
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
                            Text("Zmiana trybu zatrzymuje napęd i wyłącza moment.")
                                .foregroundStyle(.secondary)
                        }
                        HStack { Text("Przyspieszenie"); Slider(value: $aura.acceleration, in: 0...254); Text("\(Int(aura.acceleration))").monospacedDigit().frame(width: 45) }
                        if aura.operatingMode == .servo {
                            HStack {
                                Button("Ustaw bieżącą pozycję jako środek") { aura.calibrateCenter() }
                                Text("Tryb pozycyjny 0–360°").foregroundStyle(.secondary)
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
                            Text("Wartość ujemna obraca w przeciwną stronę, 0 zatrzymuje.")
                                .foregroundStyle(.secondary)
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
                        Text("Podłącz tylko jedno serwo. Operacja odblokowuje EEPROM, zmienia ID, blokuje EEPROM i sprawdza nowy adres.").foregroundStyle(.secondary)
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
    var body: some View {
        VStack(alignment: .leading, spacing: 16) {
            Text("Sygnalizacja stanu • GPIO32").font(.title2.bold())
            Grid(alignment: .leading, horizontalSpacing: 18, verticalSpacing: 10) {
                GridRow { Text("Rozbrojony"); Text("pomarańczowy") }
                GridRow { Text("Kalibracja"); Text("czerwony") }
                GridRow { Text("Uzbrojony"); Text("zielony") }
                GridRow { Text("Awaria bezpieczeństwa"); Text("czerwony migający") }
            }.foregroundStyle(.secondary)
            Text("Pasek robota i lightbar pada pokazują ten sam stan. Aura odświeża kolor okresowo, więc test ręcznego koloru ani ponowne połączenie pada nie zostawią starego koloru.").foregroundStyle(.secondary)
            Spacer()
        }.padding()
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
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Czujniki odległości VL53L4CD").font(.title2.bold())
                        Text("S2 działa jako VL53L4CD; S1 / GPIO14 jest zarezerwowane dla MPU6050 DATA_RDY.")
                            .foregroundStyle(.secondary)
                    }
                    Spacer()
                    Text("Wykryto: \(aura.detectedToFSensorIndices.count)")
                        .font(.headline.monospacedDigit())
                        .padding(.horizontal, 10).padding(.vertical, 5)
                        .background(.quaternary, in: Capsule())
                    Button("Wykryj S2 ponownie") { aura.reinitializeToF() }
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
                    ContentUnavailableView(
                        "Nie wykryto VL53L4CD",
                        systemImage: "sensor.tag.radiowaves.forward",
                        description: Text("Podłącz VL53L4CD do S2 i wybierz „Wykryj S2 ponownie”."))
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
                    Circle().fill(statusColor).frame(width: 10, height: 10)
                    Text("DualSense").font(.title2.bold())
                    Text(aura.dualSense.isConnected && !aura.dualSense.hasInput ? "Brak aktualnych danych" : aura.dualSense.state.title).foregroundStyle(.secondary)
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

private enum AuraTab: String, CaseIterable, Identifiable {
    case dashboard, imu, servos, robot, calibration, quadruped, ikLab, dualSense, sensors, leds

    var id: Self { self }
    var title: String {
        switch self {
        case .dashboard: "Płytka"
        case .imu: "IMU"
        case .servos: "Serwa"
        case .robot: "Robot"
        case .calibration: "Calibration"
        case .quadruped: "Quadruped"
        case .ikLab: "Test IK"
        case .dualSense: "DualSense"
        case .sensors: "Czujniki"
        case .leds: "LED"
        }
    }
    var icon: String {
        switch self {
        case .dashboard: "gauge.with.dots.needle.50percent"
        case .imu: "gyroscope"
        case .servos: "move.3d"
        case .robot: "point.3.connected.trianglepath.dotted"
        case .calibration: "slider.horizontal.3"
        case .quadruped: "figure.walk.motion"
        case .ikLab: "point.3.connected.trianglepath.dotted"
        case .dualSense: "gamecontroller"
        case .sensors: "ruler"
        case .leds: "lightbulb.led"
        }
    }
}

private final class AuraNavigationStore: ObservableObject {
    @Published var selectedTab: AuraTab = .dashboard
}

private struct AuraTabButton: View {
    let tab: AuraTab
    let selected: Bool
    let select: () -> Void

    var body: some View {
        Button(action: select) {
            HStack(spacing: 6) {
                Image(systemName: tab.icon)
                Text(tab.title)
            }
            .font(.subheadline.weight(selected ? .semibold : .regular))
            .foregroundStyle(selected ? .primary : .secondary)
            .padding(.horizontal, 10)
            .padding(.vertical, 7)
            .background(selected ? Color.primary.opacity(0.14) : Color.clear,
                        in: Capsule())
        }
        .buttonStyle(.plain)
    }
}

struct ContentView: View {
    @StateObject private var aura = AuraConnection()
    @StateObject private var robot = RobotControlStore()
    @StateObject private var calibration = CalibrationStore()
    @StateObject private var navigation = AuraNavigationStore()

    @ViewBuilder
    private var selectedContent: some View {
        switch navigation.selectedTab {
        case .dashboard: DashboardView()
        case .imu: IMUView()
        case .servos: ServoView()
        case .robot: RobotControlView()
        case .calibration: CalibrationView()
        case .quadruped: QuadrupedView(channel: aura.robotRenderChannel, aura: aura)
        case .ikLab: IKLabView(aura: aura)
        case .dualSense: DualSenseView()
        case .sensors: ToFView()
        case .leds: LEDsView()
        }
    }

    var body: some View {
        VStack(spacing: 0) {
            HStack {
                Text("Aura").font(.title2.bold()).foregroundStyle(.black)
                    .padding(.horizontal, 11).padding(.vertical, 5).background(.white, in: RoundedRectangle(cornerRadius: 8))
                Circle().fill(aura.isConnected ? Color.green : Color.orange).frame(width: 9, height: 9)
                Text(aura.connectionText)
                if aura.busy { ProgressView().controlSize(.small) }
                Spacer()
                Button("Szukaj ponownie") { aura.reconnect() }
            }.padding(12).background(.bar)
            // TabView retains all of its child trees.  With live telemetry,
            // that meant the chart, sensor cards and 3D scene stayed alive at
            // once.  This bar constructs exactly one live panel.
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 5) {
                    ForEach(AuraTab.allCases) { tab in
                        AuraTabButton(tab: tab, selected: tab == navigation.selectedTab) {
                            navigation.selectedTab = tab
                        }
                    }
                }
                .padding(.horizontal, 12)
                .padding(.vertical, 7)
            }
            .background(.bar.opacity(0.55))
            selectedContent
                .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
            if !aura.message.isEmpty {
                HStack { Text(aura.message); Spacer(); Button("×") { aura.message = "" }.buttonStyle(.plain) }
                    .padding(10).background(.regularMaterial)
            }
        }
        .frame(minWidth: 900, minHeight: 620)
        .groupBoxStyle(AuraGroupBoxStyle())
        .environmentObject(aura)
        .environmentObject(robot)
        .environmentObject(calibration)
        .onChange(of: aura.robotAxisBackup) { _, records in
            robot.applyAuraConfigurations(records)
        }
    }
}

@main
struct MainBoardControlApp: App {
    var body: some Scene {
        WindowGroup("Aura") { ContentView() }
            .windowStyle(.titleBar)
            .defaultSize(width: 1280, height: 820)
    }
}
