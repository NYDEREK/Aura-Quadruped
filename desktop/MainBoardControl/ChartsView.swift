import AppKit
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
        // Plain number arrays: the chart never touches the full samples.
        let times = samples.map { $0.time - end }
        let lines = series.map { s in samples.map(s.value) }
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
            LineCanvas(times: times, lines: lines, colors: series.map(\.color))
                .frame(height: height)
                .overlay {
                    if samples.isEmpty { Text("No data").font(Theme.label).foregroundStyle(Theme.faint) }
                }
        }
    }
}

// Line chart drawn straight into a Canvas: grid, axis labels and one path
// per series. Swift Charts cost a few percent of CPU per chart at 4 Hz; this
// costs next to nothing for the same picture.
struct LineCanvas: View {
    let times: [Double]          // s, negative = before now
    let lines: [[Double]]
    let colors: [Color]

    private static func label(_ value: Double, span: Double) -> String {
        let format = span < 0.02 ? "%.4f" : span < 0.2 ? "%.3f" : span < 2 ? "%.2f" : span < 20 ? "%.1f" : "%.0f"
        return String(format: format, value)
    }

    private static func timeStep(_ span: Double) -> Double {
        [1.0, 2, 5, 10, 15, 30, 60, 120, 300].first { span / $0 <= 6 } ?? 600
    }

    var body: some View {
        Canvas { context, size in
            let values = lines.flatMap { $0 }.filter(\.isFinite)
            guard times.count > 1, var low = values.min(), var high = values.max() else { return }
            if high - low < 1e-6 { low -= 0.5; high += 0.5 }
            let pad = (high - low) * 0.08
            low -= pad; high += pad
            let start = times.first ?? -1, end = times.last ?? 0
            let span = max(1e-6, end - start)
            let rows = (0...3).map { low + (high - low) * Double($0) / 3 }
            let labels = rows.map { Self.label($0, span: high - low) }
            // Room for the widest value label, so none is cut at the edge.
            let labelWidth = labels.map { context.resolve(Text($0).font(Theme.label)).measure(in: size).width }.max() ?? 30
            let plot = CGRect(x: labelWidth + 8, y: 4,
                              width: max(1, size.width - labelWidth - 22), height: max(1, size.height - 22))
            func x(_ t: Double) -> CGFloat { plot.minX + CGFloat((t - start) / span) * plot.width }
            func y(_ v: Double) -> CGFloat { plot.maxY - CGFloat((v - low) / (high - low)) * plot.height }

            var grid = Path()
            for (value, label) in zip(rows, labels) {
                let row = y(value)
                grid.move(to: CGPoint(x: plot.minX, y: row))
                grid.addLine(to: CGPoint(x: plot.maxX, y: row))
                context.draw(Text(label).font(Theme.label).foregroundColor(Theme.muted),
                             at: CGPoint(x: plot.minX - 6, y: row), anchor: .trailing)
            }
            let step = Self.timeStep(span)
            var tick = (end / step).rounded(.down) * step
            while tick >= start {
                let column = x(tick)
                grid.move(to: CGPoint(x: column, y: plot.minY))
                grid.addLine(to: CGPoint(x: column, y: plot.maxY))
                context.draw(Text(String(format: "%.0f s", tick)).font(Theme.label).foregroundColor(Theme.muted),
                             at: CGPoint(x: column, y: plot.maxY + 4), anchor: .top)
                tick -= step
            }
            context.stroke(grid, with: .color(Theme.line), lineWidth: 0.5)

            for (index, line) in lines.enumerated() where line.count == times.count {
                var path = Path()
                var drawing = false
                for i in times.indices {
                    guard line[i].isFinite else { drawing = false; continue }
                    let point = CGPoint(x: x(times[i]), y: y(line[i]))
                    if drawing { path.addLine(to: point) } else { path.move(to: point); drawing = true }
                }
                context.stroke(path, with: .color(colors.indices.contains(index) ? colors[index] : Theme.text),
                               style: StrokeStyle(lineWidth: 1.5, lineJoin: .round))
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
        [TimeChart(title: "Voltage", unit: "V", samples: s,
                   series: [Series(name: "VBUS", color: Theme.accent) { $0.busVoltage }]),
         TimeChart(title: "Servo current", unit: "A", samples: s,
                   series: [Series(name: "I", color: Theme.accent2Line) { $0.current }], format: "%.3f"),
         TimeChart(title: "Power", unit: "W", samples: s,
                   series: [Series(name: "P", color: Theme.orange) { $0.power }], format: "%.1f")]
    }

    static func imu(_ s: [TelemetrySample]) -> [TimeChart] {
        [TimeChart(title: "Tilt", unit: "°", samples: s, series: [
            Series(name: "roll", color: Theme.accent) { $0.roll - $0.referenceRoll },
            Series(name: "pitch", color: Theme.accent2Line) { $0.pitch - $0.referencePitch }], format: "%+.1f"),
         TimeChart(title: "Posture correction", unit: "°", samples: s, series: [
            Series(name: "roll", color: Theme.accent) { $0.correctionRoll },
            Series(name: "pitch", color: Theme.accent2Line) { $0.correctionPitch }], format: "%+.1f"),
         TimeChart(title: "Gyroscope", unit: "°/s", samples: s, series: [
            Series(name: "x", color: Theme.red) { $0.gyro.x },
            Series(name: "y", color: Theme.green) { $0.gyro.y },
            Series(name: "z", color: Theme.blue) { $0.gyro.z }], format: "%+.1f"),
         TimeChart(title: "Acceleration", unit: "g", samples: s, series: [
            Series(name: "x", color: Theme.red) { $0.accel.x },
            Series(name: "y", color: Theme.green) { $0.accel.y },
            Series(name: "z", color: Theme.blue) { $0.accel.z }], format: "%+.2f"),
         TimeChart(title: "IMU temperature", unit: "°C", samples: s,
                   series: [Series(name: "T", color: Theme.yellow) { $0.imuTemperature }], format: "%.1f")]
    }

    static func servos(_ s: [TelemetrySample]) -> [TimeChart] {
        [TimeChart(title: "Servo temperature (max per leg)", unit: "°C", samples: s, series: (0..<4).map { leg in
            Series(name: legs[leg], color: Theme.series[leg]) { legMax($0.servoTemperature, leg) } }, format: "%.0f"),
         TimeChart(title: "Tracking error (max per leg)", unit: "°", samples: s, series: (0..<4).map { leg in
            Series(name: legs[leg], color: Theme.series[leg]) { legMax($0.trackingError, leg, absolute: true) } },
                   format: "%.1f")]
    }

    static func gait(_ s: [TelemetrySample]) -> [TimeChart] {
        [TimeChart(title: "Command", unit: "", samples: s, series: [
            Series(name: "forward", color: Theme.accent) { $0.forward },
            Series(name: "lateral", color: Theme.accent2Line) { $0.lateral },
            Series(name: "turn", color: Theme.orange) { $0.turn }], format: "%+.2f"),
         TimeChart(title: "Torso height", unit: "mm", samples: s,
                   series: [Series(name: "h", color: Theme.teal) { $0.bodyHeight }], format: "%.0f"),
         TimeChart(title: "Torso shift", unit: "mm", samples: s, series: [
            Series(name: "x", color: Theme.blue) { $0.bodyShiftX },
            Series(name: "z", color: Theme.orange) { $0.bodyShiftZ }], format: "%+.1f"),
         TimeChart(title: "Gait phase", unit: "", samples: s,
                   series: [Series(name: "φ", color: Theme.purple) { $0.phase }], format: "%.2f"),
         TimeChart(title: "Odometry", unit: "mm", samples: s, series: [
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
        let samples = history.window(seconds: Double(window.rawValue), maximumPoints: 200)
        VStack(spacing: 0) {
            HStack(spacing: 12) {
                TextTabs(tabs: [(.power, "Power"), (.imu, "IMU"), (.servos, "Servos"), (.gait, "Gait")],
                         selection: $group)
                Spacer()
                Picker("", selection: $window) {
                    ForEach(ChartWindow.allCases, id: \.self) { Text($0.title).tag($0) }
                }
                .pickerStyle(.segmented)
                .labelsHidden()
                .frame(width: 220)
                Button(history.paused ? "Resume" : "Pause") { history.paused.toggle() }
                Button("CSV") { exportCSV() }.disabled(history.count == 0)
            }
            .controlSize(.small)
            .padding(.horizontal, 12).padding(.vertical, 6)
            .background(Theme.panel)
            Divider().overlay(Theme.line)
            if samples.isEmpty {
                Placeholder(text: "No data from Aura")
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
        panel.nameFieldStringValue = "aura-telemetry.csv"
        guard panel.runModal() == .OK, let url = panel.url else { return }
        try? history.csv().write(to: url, atomically: true, encoding: .utf8)
    }
}

// The page follows the connection only; its charts follow the history in
// their own view, so the page is not laid out again for every chart update.
struct OverviewView: View {
    @EnvironmentObject var aura: AuraConnection

    private var batteryPercent: Double {
        min(100, max(0, (aura.power.servoBusVoltage - 10.0) / 2.6 * 100))
    }

    var body: some View {
        let state = aura.robotState
        let legs = aura.robotLegTelemetry
        let temperatures = legs.flatMap(\.temperature)
        let maxTemperature = temperatures.max() ?? 0
        let voltage = aura.power.servoBusVoltage
        Page {
            if !aura.isConnected && aura.history.count == 0 {
                Placeholder(text: "Not connected to Aura").frame(height: 300)
            } else {
                if state.safetyFault.latched {
                    Panel("Safety stop") {
                        Text("\(state.safetyFault.legTitle) · \(state.safetyFault.axisTitle) · ID \(state.safetyFault.servoID) · "
                             + String(format: "error %.1f° · %.2f A · load %.0f%%",
                                      state.safetyFault.trackingErrorDegrees, state.safetyFault.currentAmperes,
                                      state.safetyFault.loadPercent))
                            .font(Theme.monoSmall).foregroundStyle(Theme.red)
                    }
                }
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 130), spacing: 8)], spacing: 8) {
                    Stat(label: "Battery", value: String(format: "%.2f V · %.0f%%", voltage, batteryPercent),
                         tint: voltage <= 10 ? Theme.red : voltage < 11 ? Theme.orange : Theme.green)
                    Stat(label: "Current", value: String(format: "%.2f A", aura.power.current))
                    Stat(label: "Power", value: String(format: "%.1f W", aura.power.power))
                    Stat(label: "Consumed", value: String(format: "%.0f mAh", aura.power.servoConsumedMilliampHours))
                    Stat(label: "Mode", value: AuraPadMode.title(state.selectedGait))
                    Stat(label: "Servos", value: state.safetyFault.latched ? "stopped" : state.armed ? "armed" : "disarmed",
                         tint: state.safetyFault.latched ? Theme.red : state.armed ? Theme.green : Theme.yellow)
                    Stat(label: "Roll / pitch", value: String(format: "%+.1f° %+.1f°",
                         aura.attitude.rollDegrees - aura.attitude.referenceRollDegrees,
                         aura.attitude.pitchDegrees - aura.attitude.referencePitchDegrees))
                    Stat(label: "Servo temp max", value: "\(maxTemperature) °C",
                         tint: maxTemperature >= 65 ? Theme.red : maxTemperature >= 55 ? Theme.orange : Theme.text)
                    Stat(label: "Height", value: "\(state.bodyHeight) mm")
                    Stat(label: "Controller", value: aura.dualSense.isConnected ? aura.dualSense.battery.map { "\($0)%" } ?? "connected" : "none",
                         tint: aura.dualSense.isConnected ? Theme.text : Theme.muted)
                }
                OverviewCharts()
                Panel("Axes") {
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
                                Text("angle"); Text("error"); Text("°C")
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

// Overview's charts in their own view: it redraws with the history (4 Hz),
// not with every 10 Hz state update of the page around it.
private struct OverviewCharts: View {
    @EnvironmentObject var history: TelemetryHistory

    var body: some View {
        let _ = history.revision
        let samples = history.window(seconds: 60, maximumPoints: 150)
        LazyVGrid(columns: [GridItem(.adaptive(minimum: 420), spacing: 12)], spacing: 12) {
            TimeChart(title: "Voltage", unit: "V", samples: samples,
                      series: [Series(name: "VBUS", color: Theme.accent) { $0.busVoltage }])
            TimeChart(title: "Servo current", unit: "A", samples: samples,
                      series: [Series(name: "I", color: Theme.accent2Line) { $0.current }], format: "%.3f")
            TimeChart(title: "Tilt", unit: "°", samples: samples, series: [
                Series(name: "roll", color: Theme.accent) { $0.roll - $0.referenceRoll },
                Series(name: "pitch", color: Theme.accent2Line) { $0.pitch - $0.referencePitch }], format: "%+.1f")
            TimeChart(title: "Servo temperature", unit: "°C", samples: samples, series: (0..<4).map { leg in
                Series(name: ChartSets.legs[leg], color: Theme.series[leg]) { legMax($0.servoTemperature, leg) } },
                      format: "%.0f")
        }
    }
}
