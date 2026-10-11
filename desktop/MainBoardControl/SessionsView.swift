import Charts
import SwiftUI

private let dayFormatter: DateFormatter = {
    let f = DateFormatter(); f.locale = Locale(identifier: "en_US"); f.dateFormat = "EEE, MMM d, yyyy"; return f
}()
private let timeFormatter: DateFormatter = {
    let f = DateFormatter(); f.locale = Locale(identifier: "en_US"); f.dateFormat = "HH:mm"; return f
}()

private func durationText(_ seconds: Double) -> String {
    let s = Int(seconds.rounded())
    return s >= 3600 ? String(format: "%d h %02d min", s / 3600, (s % 3600) / 60)
        : s >= 60 ? String(format: "%d min %02d s", s / 60, s % 60) : "\(s) s"
}

private func metres(_ mm: Double) -> String { String(format: "%.1f m", mm / 1000) }

struct SessionsView: View {
    @EnvironmentObject var logger: SessionLogger

    var body: some View {
        let _ = logger.revision
        let selected = logger.live.flatMap { logger.selection == $0.id ? $0 : nil }
            ?? logger.sessions.first { $0.id == logger.selection }
            ?? logger.live ?? logger.sessions.first
        HStack(spacing: 0) {
            SessionList(logger: logger, selectedID: selected?.id)
            Rectangle().fill(Theme.line).frame(width: 1)
            if let selected {
                SessionDetail(logger: logger, session: selected, live: selected.id == logger.live?.id)
            } else {
                Placeholder(text: "No sessions yet")
            }
        }
    }
}

private struct SessionList: View {
    @ObservedObject var logger: SessionLogger
    let selectedID: UUID?

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 2) {
                if let live = logger.live { row(live, live: true) }
                ForEach(logger.sessions) { row($0, live: false) }
            }
            .padding(6)
        }
        .frame(width: 220)
        .background(Theme.panel)
    }

    private func row(_ session: AuraSession, live: Bool) -> some View {
        let selected = session.id == selectedID
        return Button { logger.selection = session.id } label: {
            VStack(alignment: .leading, spacing: 2) {
                HStack(spacing: 6) {
                    if live { Circle().fill(Theme.red).frame(width: 6, height: 6) }
                    Text(dayFormatter.string(from: session.start)).font(.system(size: 12))
                        .foregroundStyle(Theme.text)
                    Spacer(minLength: 0)
                    Text(timeFormatter.string(from: session.start)).font(Theme.monoSmall).foregroundStyle(Theme.muted)
                }
                Text(durationText(session.summary.duration) + " · " + metres(session.summary.distance))
                    .font(Theme.label).foregroundStyle(Theme.muted)
            }
            .padding(.horizontal, 8).padding(.vertical, 6)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(selected ? Theme.accent2.opacity(0.55) : .clear, in: RoundedRectangle(cornerRadius: 4))
            .contentShape(Rectangle())
        }
        .buttonStyle(.plain)
    }
}

private struct SessionDetail: View {
    @ObservedObject var logger: SessionLogger
    let session: AuraSession
    let live: Bool

    var body: some View {
        let s = session.summary
        let series = session.series
        Page {
            HStack(alignment: .firstTextBaseline) {
                VStack(alignment: .leading, spacing: 3) {
                    Text(dayFormatter.string(from: session.start)).font(.system(size: 15, weight: .semibold))
                        .foregroundStyle(Theme.text)
                    Text("\(timeFormatter.string(from: session.start))–\(timeFormatter.string(from: session.end))"
                         + (live ? " · live" : ""))
                        .font(Theme.monoSmall).foregroundStyle(live ? Theme.red : Theme.muted)
                }
                Spacer()
                if !live {
                    Button("Show in Finder") { logger.reveal(session) }
                    Button("CSV") { logger.exportCSV(session) }
                    Button("Delete") { logger.delete(session) }
                }
            }
            .buttonStyle(AuraButtonStyle(compact: true))
            LazyVGrid(columns: [GridItem(.adaptive(minimum: 130), spacing: 8)], spacing: 8) {
                Stat(label: "Duration", value: durationText(s.duration))
                Stat(label: "Distance", value: metres(s.distance))
                Stat(label: "Average speed", value: String(format: "%.0f mm/s", s.averageSpeed))
                Stat(label: "Max speed", value: String(format: "%.0f mm/s", s.maxSpeed), tint: Theme.accent)
                Stat(label: "Energy", value: String(format: "%.0f mAh", s.energy))
                Stat(label: "Battery", value: String(format: "%.2f→%.2f V", s.voltageStart, s.voltageEnd))
                Stat(label: "Current avg / max", value: String(format: "%.2f / %.2f A", s.currentAverage, s.currentMax))
                Stat(label: "Servo temp max", value: String(format: "%.0f °C", s.maxServoTemperature),
                     tint: s.maxServoTemperature >= 65 ? Theme.red : s.maxServoTemperature >= 55 ? Theme.orange : Theme.text)
                Stat(label: "Armed", value: durationText(s.armedTime))
                Stat(label: "Faults", value: "\(s.safetyFaults)", tint: s.safetyFaults > 0 ? Theme.red : Theme.text)
            }
            LazyVGrid(columns: [GridItem(.adaptive(minimum: 420), spacing: 12)], spacing: 12) {
                SeriesChart(title: "Voltage", unit: "V", t: series.t, values: series.voltage, color: Theme.accent)
                SeriesChart(title: "Current", unit: "A", t: series.t, values: series.current, color: Theme.accent2Line,
                            format: "%.2f")
                SeriesChart(title: "Speed", unit: "mm/s", t: series.t, values: series.speed, color: Theme.orange,
                            format: "%.0f")
                Panel("Route") {
                    RouteMap(x: series.x, z: series.z).frame(height: 140)
                }
            }
        }
    }
}

// Line chart over a whole session, thinned to about 400 points.
struct SeriesChart: View {
    let title: String
    let unit: String
    let t: [Double]
    let values: [Double]
    let color: Color
    var format = "%.2f"

    var body: some View {
        let step = max(1, t.count / 400)
        let indices = Array(stride(from: 0, to: min(t.count, values.count), by: step))
        Panel(title, trailing: {
            HStack(spacing: 6) {
                if let min = values.min(), let max = values.max() {
                    Text(String(format: format, min) + " – " + String(format: format, max))
                        .font(Theme.monoSmall).foregroundStyle(Theme.text)
                }
                Text(unit).font(Theme.label).foregroundStyle(Theme.faint)
            }
        }) {
            Chart(indices, id: \.self) { i in
                LineMark(x: .value("t", t[i] / 60), y: .value(title, values[i]))
                    .foregroundStyle(color)
                    .lineStyle(StrokeStyle(lineWidth: 1.5))
            }
            .chartXAxis {
                AxisMarks(values: .automatic(desiredCount: 6)) { value in
                    AxisGridLine().foregroundStyle(Theme.line)
                    AxisValueLabel {
                        if let m = value.as(Double.self) { Text(String(format: "%.0f min", m)) }
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
            .frame(height: 140)
            .overlay { if indices.isEmpty { Text("No data").font(Theme.label).foregroundStyle(Theme.faint) } }
        }
    }
}

// Top view of a route: Aura's forward to the right, its left up.
struct RouteMap: View {
    let x: [Double]
    let z: [Double]

    var body: some View {
        Canvas { context, size in
            guard x.count > 1, x.count == z.count, let minX = x.min(), let maxX = x.max(),
                  let minZ = z.min(), let maxZ = z.max() else {
                context.draw(Text("No route").font(Theme.label).foregroundColor(Theme.faint),
                             at: CGPoint(x: size.width / 2, y: size.height / 2))
                return
            }
            let span = max(maxX - minX, maxZ - minZ, 500)
            let scale = min(size.width, size.height) * 0.9 / span
            let cx = (minX + maxX) / 2, cz = (minZ + maxZ) / 2
            func point(_ i: Int) -> CGPoint {
                CGPoint(x: size.width / 2 + (x[i] - cx) * scale, y: size.height / 2 - (z[i] - cz) * scale)
            }
            var path = Path()
            path.move(to: point(0))
            for i in 1..<x.count { path.addLine(to: point(i)) }
            context.stroke(path, with: .color(Theme.accent), lineWidth: 1.5)
            let start = point(0), end = point(x.count - 1)
            context.fill(Path(ellipseIn: CGRect(x: start.x - 4, y: start.y - 4, width: 8, height: 8)), with: .color(Theme.accent2Line))
            context.fill(Path(ellipseIn: CGRect(x: end.x - 4, y: end.y - 4, width: 8, height: 8)), with: .color(Theme.text))
        }
    }
}
