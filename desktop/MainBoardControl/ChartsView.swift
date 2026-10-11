import AppKit
import Charts
import SwiftUI
import UniformTypeIdentifiers

struct Series: Identifiable {
    let name: String
    let color: Color
    let value: (TelemetrySample) -> Double
    var id: String { name }
}

// Time chart over the history window: x is seconds before now, the header
// carries the series names with their latest values instead of a legend.
struct TimeChart: View {
    let title: String
    let unit: String
    let samples: [TelemetrySample]
    let series: [Series]
    var height: CGFloat = 140
    var format = "%.2f"

    var body: some View {
        let end = samples.last?.time ?? 0
        Panel(title, trailing: {
            HStack(spacing: 10) {
                ForEach(series) { s in
                    HStack(spacing: 4) {
                        Circle().fill(s.color).frame(width: 6, height: 6)
                        Text(series.count > 1 ? s.name : "").font(Theme.label).foregroundStyle(Theme.muted)
                        Text(samples.last.map { String(format: format, s.value($0)) } ?? "—")
                            .font(Theme.monoSmall).foregroundStyle(Theme.text)
                    }
                    .fixedSize()
                }
                Text(unit).font(Theme.label).foregroundStyle(Theme.faint)
            }
        }) {
            Chart {
                ForEach(series) { s in
                    ForEach(Array(samples.enumerated()), id: \.offset) { _, sample in
                        LineMark(x: .value("t", sample.time - end), y: .value(s.name, s.value(sample)),
                                 series: .value("seria", s.name))
                            .foregroundStyle(s.color)
                            .lineStyle(StrokeStyle(lineWidth: 1.3))
                            .interpolationMethod(.linear)
                    }
                }
            }
            .chartXAxis {
                AxisMarks(values: .automatic(desiredCount: 6)) { value in
                    AxisGridLine().foregroundStyle(Theme.line)
                    AxisValueLabel {
                        if let t = value.as(Double.self) { Text(String(format: "%.0f s", t)) }
                    }
                    .font(Theme.label).foregroundStyle(Theme.muted)
                }
            }
            .chartYAxis {
                AxisMarks(position: .leading, values: .automatic(desiredCount: 4)) { _ in
                    AxisGridLine().foregroundStyle(Theme.line)
                    AxisValueLabel().font(Theme.label).foregroundStyle(Theme.muted)
                }
            }
            .chartYScale(domain: .automatic(includesZero: false))
            .chartLegend(.hidden)
            .frame(height: height)
            .overlay {
                if samples.isEmpty { Text("Brak danych").font(Theme.label).foregroundStyle(Theme.faint) }
            }
        }
    }
}

enum ChartWindow: Int, CaseIterable, Hashable {
    case s10 = 10, s30 = 30, m1 = 60, m5 = 300
    var title: String {
        switch self { case .s10: "10 s"; case .s30: "30 s"; case .m1: "1 min"; case .m5: "5 min" }
    }
}

private func legMax(_ values: [Double], _ leg: Int, absolute: Bool = false) -> Double {
    let slice = values.count >= leg * 3 + 3 ? Array(values[(leg * 3)..<(leg * 3 + 3)]) : [0]
    return (absolute ? slice.map(abs) : slice).max() ?? 0
}

enum ChartSets {
    static let legs = ["LF", "RF", "LR", "RR"]

    static func power(_ s: [TelemetrySample]) -> [TimeChart] {
        [TimeChart(title: "Napięcie", unit: "V", samples: s,
                   series: [Series(name: "VBUS", color: Theme.green) { $0.busVoltage }]),
         TimeChart(title: "Prąd serw", unit: "A", samples: s,
                   series: [Series(name: "I", color: Theme.blue) { $0.current }], format: "%.3f"),
         TimeChart(title: "Moc", unit: "W", samples: s,
                   series: [Series(name: "P", color: Theme.orange) { $0.power }], format: "%.1f")]
    }

    static func imu(_ s: [TelemetrySample]) -> [TimeChart] {
        [TimeChart(title: "Przechył", unit: "°", samples: s, series: [
            Series(name: "roll", color: Theme.blue) { $0.roll - $0.referenceRoll },
            Series(name: "pitch", color: Theme.orange) { $0.pitch - $0.referencePitch }], format: "%+.1f"),
         TimeChart(title: "Korekcja postawy", unit: "°", samples: s, series: [
            Series(name: "roll", color: Theme.blue) { $0.correctionRoll },
            Series(name: "pitch", color: Theme.orange) { $0.correctionPitch }], format: "%+.1f"),
         TimeChart(title: "Żyroskop", unit: "°/s", samples: s, series: [
            Series(name: "x", color: Theme.red) { $0.gyro.x },
            Series(name: "y", color: Theme.green) { $0.gyro.y },
            Series(name: "z", color: Theme.blue) { $0.gyro.z }], format: "%+.1f"),
         TimeChart(title: "Przyspieszenie", unit: "g", samples: s, series: [
            Series(name: "x", color: Theme.red) { $0.accel.x },
            Series(name: "y", color: Theme.green) { $0.accel.y },
            Series(name: "z", color: Theme.blue) { $0.accel.z }], format: "%+.2f"),
         TimeChart(title: "Temperatura IMU", unit: "°C", samples: s,
                   series: [Series(name: "T", color: Theme.yellow) { $0.imuTemperature }], format: "%.1f")]
    }

    static func servos(_ s: [TelemetrySample]) -> [TimeChart] {
        [TimeChart(title: "Temperatura serw (max w nodze)", unit: "°C", samples: s, series: (0..<4).map { leg in
            Series(name: legs[leg], color: Theme.series[leg]) { legMax($0.servoTemperature, leg) } }, format: "%.0f"),
         TimeChart(title: "Błąd śledzenia (max w nodze)", unit: "°", samples: s, series: (0..<4).map { leg in
            Series(name: legs[leg], color: Theme.series[leg]) { legMax($0.trackingError, leg, absolute: true) } },
                   format: "%.1f")]
    }

    static func gait(_ s: [TelemetrySample]) -> [TimeChart] {
        [TimeChart(title: "Komenda", unit: "", samples: s, series: [
            Series(name: "przód", color: Theme.blue) { $0.forward },
            Series(name: "bok", color: Theme.green) { $0.lateral },
            Series(name: "skręt", color: Theme.orange) { $0.turn }], format: "%+.2f"),
         TimeChart(title: "Wysokość tułowia", unit: "mm", samples: s,
                   series: [Series(name: "h", color: Theme.teal) { $0.bodyHeight }], format: "%.0f"),
         TimeChart(title: "Przesunięcie tułowia", unit: "mm", samples: s, series: [
            Series(name: "x", color: Theme.blue) { $0.bodyShiftX },
            Series(name: "z", color: Theme.orange) { $0.bodyShiftZ }], format: "%+.1f"),
         TimeChart(title: "Faza chodu", unit: "", samples: s,
                   series: [Series(name: "φ", color: Theme.purple) { $0.phase }], format: "%.2f"),
         TimeChart(title: "Odometria", unit: "mm", samples: s, series: [
            Series(name: "x", color: Theme.blue) { $0.odometryX },
            Series(name: "z", color: Theme.orange) { $0.odometryZ }], format: "%.0f")]
    }
}

enum ChartGroup: String, Hashable { case power, imu, servos, gait }

struct ChartsView: View {
    @EnvironmentObject var history: TelemetryHistory
    @AppStorage("aura.charts.group") private var group: ChartGroup = .power
    @AppStorage("aura.charts.window") private var window: ChartWindow = .s30

    var body: some View {
        let _ = history.revision
        let samples = history.window(seconds: Double(window.rawValue), maximumPoints: 300)
        VStack(spacing: 0) {
            HStack(spacing: 12) {
                TextTabs(tabs: [(.power, "Zasilanie"), (.imu, "IMU"), (.servos, "Serwa"), (.gait, "Chód")],
                         selection: $group)
                Spacer()
                Picker("", selection: $window) {
                    ForEach(ChartWindow.allCases, id: \.self) { Text($0.title).tag($0) }
                }
                .pickerStyle(.segmented)
                .labelsHidden()
                .frame(width: 220)
                Button(history.paused ? "Wznów" : "Pauza") { history.paused.toggle() }
                Button("CSV") { exportCSV() }.disabled(history.count == 0)
            }
            .controlSize(.small)
            .padding(.horizontal, 12).padding(.vertical, 6)
            .background(Theme.panel)
            Divider().overlay(Theme.line)
            if samples.isEmpty {
                Placeholder(text: "Brak danych z Aury")
            } else {
                ScrollView {
                    let charts: [TimeChart] = switch group {
                    case .power: ChartSets.power(samples)
                    case .imu: ChartSets.imu(samples)
                    case .servos: ChartSets.servos(samples)
                    case .gait: ChartSets.gait(samples)
                    }
                    LazyVGrid(columns: [GridItem(.adaptive(minimum: 420), spacing: 12)], spacing: 12) {
                        ForEach(Array(charts.enumerated()), id: \.offset) { _, chart in chart }
                    }
                    .padding(14)
                }
                .background(Theme.canvas)
            }
        }
    }

    private func exportCSV() {
        let panel = NSSavePanel()
        panel.allowedContentTypes = [.commaSeparatedText]
        panel.nameFieldStringValue = "aura-telemetria.csv"
        guard panel.runModal() == .OK, let url = panel.url else { return }
        try? history.csv().write(to: url, atomically: true, encoding: .utf8)
    }
}

struct OverviewView: View {
    @EnvironmentObject var aura: AuraConnection
    @EnvironmentObject var history: TelemetryHistory

    private var batteryPercent: Double {
        min(100, max(0, (aura.power.servoBusVoltage - 10.0) / 2.6 * 100))
    }

    var body: some View {
        let _ = history.revision
        let samples = history.window(seconds: 60, maximumPoints: 240)
        let state = aura.robotState
        let legs = aura.robotLegTelemetry
        let temperatures = legs.flatMap(\.temperature)
        let maxTemperature = temperatures.max() ?? 0
        let voltage = aura.power.servoBusVoltage
        Page {
            if !aura.isConnected && samples.isEmpty {
                Placeholder(text: "Brak połączenia z Aurą").frame(height: 300)
            } else {
                if state.safetyFault.latched {
                    Panel("Blokada awaryjna") {
                        Text("\(state.safetyFault.legTitle) · \(state.safetyFault.axisTitle) · ID \(state.safetyFault.servoID) · "
                             + String(format: "błąd %.1f° · %.2f A · obciążenie %.0f%%",
                                      state.safetyFault.trackingErrorDegrees, state.safetyFault.currentAmperes,
                                      state.safetyFault.loadPercent))
                            .font(Theme.monoSmall).foregroundStyle(Theme.red)
                    }
                }
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 130), spacing: 8)], spacing: 8) {
                    Stat(label: "Bateria", value: String(format: "%.2f V · %.0f%%", voltage, batteryPercent),
                         tint: voltage <= 10 ? Theme.red : voltage < 11 ? Theme.orange : Theme.green)
                    Stat(label: "Prąd", value: String(format: "%.2f A", aura.power.current))
                    Stat(label: "Moc", value: String(format: "%.1f W", aura.power.power))
                    Stat(label: "Zużycie", value: String(format: "%.0f mAh", aura.power.servoConsumedMilliampHours))
                    Stat(label: "Tryb", value: AuraPadMode.title(state.selectedGait))
                    Stat(label: "Serwa", value: state.safetyFault.latched ? "blokada" : state.armed ? "uzbrojone" : "rozbrojone",
                         tint: state.safetyFault.latched ? Theme.red : state.armed ? Theme.green : Theme.yellow)
                    Stat(label: "Roll / pitch", value: String(format: "%+.1f° %+.1f°",
                         aura.attitude.rollDegrees - aura.attitude.referenceRollDegrees,
                         aura.attitude.pitchDegrees - aura.attitude.referencePitchDegrees))
                    Stat(label: "Temp. serw max", value: "\(maxTemperature) °C",
                         tint: maxTemperature >= 65 ? Theme.red : maxTemperature >= 55 ? Theme.orange : Theme.text)
                    Stat(label: "Wysokość", value: "\(state.bodyHeight) mm")
                    Stat(label: "Pad", value: aura.dualSense.isConnected ? aura.dualSense.battery.map { "\($0)%" } ?? "połączony" : "brak",
                         tint: aura.dualSense.isConnected ? Theme.text : Theme.muted)
                }
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 420), spacing: 12)], spacing: 12) {
                    TimeChart(title: "Napięcie", unit: "V", samples: samples,
                              series: [Series(name: "VBUS", color: Theme.green) { $0.busVoltage }])
                    TimeChart(title: "Prąd serw", unit: "A", samples: samples,
                              series: [Series(name: "I", color: Theme.blue) { $0.current }], format: "%.3f")
                    TimeChart(title: "Przechył", unit: "°", samples: samples, series: [
                        Series(name: "roll", color: Theme.blue) { $0.roll - $0.referenceRoll },
                        Series(name: "pitch", color: Theme.orange) { $0.pitch - $0.referencePitch }], format: "%+.1f")
                    TimeChart(title: "Temperatura serw", unit: "°C", samples: samples, series: (0..<4).map { leg in
                        Series(name: ChartSets.legs[leg], color: Theme.series[leg]) { legMax($0.servoTemperature, leg) } },
                              format: "%.0f")
                }
                Panel("Osie") {
                    Grid(alignment: .trailing, horizontalSpacing: 18, verticalSpacing: 5) {
                        GridRow {
                            Text("").gridColumnAlignment(.leading)
                            ForEach(["Ab/ad", "Hip", "Knee"], id: \.self) { axis in
                                Text(axis).gridCellColumns(3).frame(maxWidth: .infinity)
                            }
                        }
                        .font(Theme.label).foregroundStyle(Theme.muted)
                        GridRow {
                            Text("")
                            ForEach(0..<3, id: \.self) { _ in
                                Text("kąt"); Text("błąd"); Text("°C")
                            }
                        }
                        .font(Theme.label).foregroundStyle(Theme.faint)
                        ForEach(0..<4, id: \.self) { leg in
                            let telemetry = legs.indices.contains(leg) ? legs[leg] : AuraRobotLegTelemetry(id: leg)
                            GridRow {
                                Text(ChartSets.legs[leg]).font(Theme.heading).foregroundStyle(Theme.text)
                                ForEach(0..<3, id: \.self) { axis in
                                    let error = telemetry.targetDegrees[axis] - telemetry.measuredDegrees[axis]
                                    let present = telemetry.present[axis]
                                    Text(present ? String(format: "%+.1f", telemetry.measuredDegrees[axis]) : "—")
                                        .foregroundStyle(present ? Theme.text : Theme.faint)
                                    Text(present ? String(format: "%+.1f", error) : "—")
                                        .foregroundStyle(abs(error) >= 5 ? Theme.yellow : Theme.muted)
                                    Text("\(telemetry.temperature[axis])")
                                        .foregroundStyle(telemetry.temperature[axis] >= 60 ? Theme.orange : Theme.muted)
                                }
                            }
                            .font(Theme.monoSmall)
                        }
                    }
                }
            }
        }
    }
}
