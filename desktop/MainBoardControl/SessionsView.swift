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
            if !session.modes.isEmpty {
                ModeTable(modes: session.modes)
            }
            if let plot = TracePlot(logger.trace(for: session)) {
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 420), spacing: 12)], spacing: 12) {
                    TraceChart(title: "Torso", unit: "°", plot: plot, lines: [
                        .init(id: "roll", color: Theme.accent, values: plot.roll),
                        .init(id: "pitch", color: Theme.accent2Line, values: plot.pitch)])
                    TraceChart(title: "IMU correction", unit: "°", plot: plot, lines: [
                        .init(id: "roll", color: Theme.accent, values: plot.correctionRoll),
                        .init(id: "pitch", color: Theme.accent2Line, values: plot.correctionPitch)])
                    TraceChart(title: "Joint error, worst joint", unit: "°", plot: plot, lines: [
                        .init(id: "error", color: Theme.orange, values: plot.jointError)])
                    TraceChart(title: "Sticks", unit: "", plot: plot, lines: [
                        .init(id: "forward", color: Theme.accent, values: plot.forward),
                        .init(id: "lateral", color: Theme.teal, values: plot.lateral),
                        .init(id: "turn", color: Theme.accent2Line, values: plot.turn)])
                }
            }
            if !session.events.isEmpty {
                EventList(events: session.events)
            }
        }
    }
}

func modeColor(_ mode: Int) -> Color {
    switch mode {
    case 1: Theme.blue
    case 2: Theme.teal
    case 3: Theme.purple
    case 4: Theme.yellow
    case 5: Theme.red
    default: Theme.muted
    }
}

private func clock(_ seconds: Double) -> String {
    let s = Int(max(0, seconds).rounded(.down))
    return String(format: "%d:%02d", s / 60, s % 60)
}

// Per-mode behaviour: one column per pad mode that was armed.
private struct ModeTable: View {
    let modes: [AuraModeStats]

    var body: some View {
        Panel("Modes") {
            ScrollView(.horizontal, showsIndicators: false) {
                Grid(alignment: .trailing, horizontalSpacing: 22, verticalSpacing: 5) {
                    GridRow {
                        Text("").gridColumnAlignment(.leading)
                        ForEach(modes) { mode in
                            HStack(spacing: 5) {
                                Circle().fill(modeColor(mode.mode)).frame(width: 7, height: 7)
                                Text(AuraPadMode.title(mode.mode)).font(Theme.label).foregroundStyle(Theme.text)
                            }
                        }
                    }
                    row("Armed", modes.map { durationText($0.armedTime) })
                    row("Moving", modes.map { durationText($0.movingTime) })
                    row("Distance", modes.map { metres($0.distance) })
                    row("Max speed", modes.map { String(format: "%.0f mm/s", $0.maxSpeed) })
                    row("Tilt RMS roll / pitch", modes.map { String(format: "%.1f / %.1f°", $0.rollRMS, $0.pitchRMS) })
                    row("Tilt max roll / pitch", modes.map { String(format: "%.1f / %.1f°", $0.maxRoll, $0.maxPitch) })
                    row("Wobble RMS", modes.map { String(format: "%.1f °/s", $0.wobbleRMS) })
                    row("IMU correction RMS", modes.map { String(format: "%.2f°", $0.correctionRMS) })
                    row("Joint error RMS", modes.map { String(format: "%.1f°", $0.trackingRMS) })
                    row("Joint error max", modes.map { String(format: "%.1f° ", $0.maxTracking) + $0.maxTrackingJoint })
                    row("Current avg / max", modes.map { String(format: "%.2f / %.2f A", $0.currentAverage, $0.maxCurrent) })
                    row("Gait frequency", modes.map { $0.frequencySamples > 0 ? String(format: "%.2f Hz", $0.frequencyAverage) : "—" })
                    row("Gait slowed", modes.map { percent($0.slowedRate) })
                    row("At range limit", modes.map { percent($0.limitedRate) })
                    row("Contact mismatch", modes.map { $0.contactSamples > 0 ? percent($0.contactMismatchRate) : "—" })
                    row("Early touchdowns", modes.map { "\($0.earlyTouchdowns)" })
                    row("Faults", modes.map { "\($0.faults)" }, tints: modes.map { $0.faults > 0 ? Theme.red : Theme.text })
                }
            }
        }
    }

    private func percent(_ value: Double) -> String { String(format: "%.0f %%", value * 100) }

    private func row(_ label: String, _ values: [String], tints: [Color]? = nil) -> some View {
        GridRow {
            Text(label).font(Theme.label).foregroundStyle(Theme.muted).gridColumnAlignment(.leading)
            ForEach(values.indices, id: \.self) { index in
                Text(values[index]).font(Theme.monoSmall).foregroundStyle(tints?[index] ?? Theme.text)
            }
        }
    }
}

// The armed 25 Hz trace thinned to about 500 points for drawing. The joint
// error keeps the worst value of each thinned window so short spikes stay.
struct TracePlot {
    struct Band: Identifiable { let id: Int; let mode: Int; let start: Double; let end: Double }
    struct Line: Identifiable { let id: String; let color: Color; let values: [Double] }

    var t: [Double] = []
    var segment: [Int] = []       // breaks lines across disarmed gaps
    var bands: [Band] = []
    var roll: [Double] = [], pitch: [Double] = []
    var correctionRoll: [Double] = [], correctionPitch: [Double] = []
    var jointError: [Double] = []
    var forward: [Double] = [], lateral: [Double] = [], turn: [Double] = []

    init?(_ trace: AuraSessionTrace?) {
        guard let trace, trace.t.count > 1 else { return nil }
        let count = trace.t.count
        var start = 0
        for i in 1...count {
            if i == count || trace.mode[i] != trace.mode[start] || trace.t[i] - trace.t[i - 1] > 0.3 {
                bands.append(Band(id: bands.count, mode: trace.mode[start], start: trace.t[start], end: trace.t[i - 1]))
                start = i
            }
        }
        let step = max(1, count / 500)
        var currentSegment = 0
        for i in stride(from: 0, to: count, by: step) {
            if let last = t.last, trace.t[i] - last > 0.3 + Double(step) * 0.06 { currentSegment += 1 }
            t.append(trace.t[i])
            segment.append(currentSegment)
            roll.append(Double(trace.roll[i]) / 100)
            pitch.append(Double(trace.pitch[i]) / 100)
            correctionRoll.append(Double(trace.correctionRoll[i]) / 100)
            correctionPitch.append(Double(trace.correctionPitch[i]) / 100)
            forward.append(Double(trace.stick[0][i]) / 100)
            lateral.append(Double(trace.stick[1][i]) / 100)
            turn.append(Double(trace.stick[2][i]) / 100)
            var worst = 0.0
            for j in i..<min(count, i + step) {
                for joint in 0..<12 where trace.jointMeasured[joint][j] != AuraSessionTrace.noReply {
                    worst = max(worst, Double(abs(trace.jointTarget[joint][j] - trace.jointMeasured[joint][j])) / 10)
                }
            }
            jointError.append(worst)
        }
    }
}

private struct TraceChart: View {
    let title: String
    let unit: String
    let plot: TracePlot
    let lines: [TracePlot.Line]

    var body: some View {
        Panel(title, trailing: {
            HStack(spacing: 8) {
                if lines.count > 1 {
                    ForEach(lines) { line in
                        HStack(spacing: 4) {
                            Rectangle().fill(line.color).frame(width: 10, height: 2)
                            Text(line.id).font(Theme.label).foregroundStyle(Theme.muted)
                        }
                    }
                }
                Text(unit).font(Theme.label).foregroundStyle(Theme.faint)
            }
        }) {
            Chart {
                ForEach(plot.bands) { band in
                    RectangleMark(xStart: .value("t", band.start), xEnd: .value("t", band.end))
                        .foregroundStyle(modeColor(band.mode).opacity(0.13))
                }
                ForEach(lines) { line in
                    ForEach(plot.t.indices, id: \.self) { i in
                        LineMark(x: .value("t", plot.t[i]), y: .value(title, line.values[i]),
                                 series: .value("line", "\(line.id)-\(plot.segment[i])"))
                            .foregroundStyle(line.color)
                            .lineStyle(StrokeStyle(lineWidth: 1.2))
                    }
                }
            }
            .chartXAxis {
                AxisMarks(values: .automatic(desiredCount: 6)) { value in
                    AxisGridLine().foregroundStyle(Theme.line)
                    AxisValueLabel {
                        if let seconds = value.as(Double.self) { Text(clock(seconds)) }
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
            .frame(height: 150)
        }
    }
}

private struct EventList: View {
    let events: [AuraSessionEvent]

    private func tint(_ kind: String) -> Color {
        switch kind {
        case "fault", "pad lost", "task stalled", "board restart", "low stack", "gait tick slow": Theme.red
        case "armed": Theme.green
        case "disarmed", "fault cleared": Theme.orange
        case "mode": Theme.accent
        default: Theme.muted
        }
    }

    var body: some View {
        Panel("Events", trailing: {
            Text("\(events.count)").font(Theme.monoSmall).foregroundStyle(Theme.muted)
        }) {
            ScrollView {
                LazyVStack(alignment: .leading, spacing: 3) {
                    ForEach(events.indices, id: \.self) { index in
                        let event = events[index]
                        HStack(spacing: 10) {
                            Text(clock(event.t)).font(Theme.monoSmall).foregroundStyle(Theme.faint)
                                .frame(width: 44, alignment: .leading)
                            Text(event.kind).font(Theme.monoSmall).foregroundStyle(tint(event.kind))
                                .frame(width: 96, alignment: .leading)
                            Text(event.detail).font(Theme.monoSmall).foregroundStyle(Theme.text)
                                .lineLimit(1)
                            Spacer(minLength: 0)
                        }
                    }
                }
            }
            .frame(maxHeight: 220)
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
                LineMark(x: .value("t", t[i]), y: .value(title, values[i]))
                    .foregroundStyle(color)
                    .lineStyle(StrokeStyle(lineWidth: 1.5))
            }
            .chartXAxis {
                AxisMarks(values: .automatic(desiredCount: 6)) { value in
                    AxisGridLine().foregroundStyle(Theme.line)
                    AxisValueLabel {
                        if let seconds = value.as(Double.self) { Text(clock(seconds)) }
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
