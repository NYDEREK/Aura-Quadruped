import AppKit
import SwiftUI

// Dark palette after the JetBrains New UI: an editor-grey canvas, slightly
// lighter tool panels and hairline separators, with Aura's two accents:
// lime #BECF00 for highlights and purple #5E43B7 for selection and controls.
// Every view takes its colours from here.
enum Theme {
    static let canvas = Color(rgb: 0x1E1F22)
    static let panel = Color(rgb: 0x2B2D30)
    static let raised = Color(rgb: 0x393B40)
    static let line = Color(rgb: 0x393B40)
    static let text = Color(rgb: 0xDFE1E5)
    static let muted = Color(rgb: 0x8C8F96)
    static let faint = Color(rgb: 0x5A5D63)
    static let accent = Color(rgb: 0xBECF00)       // lime: underline, highlight, first series
    static let accent2 = Color(rgb: 0x5E43B7)      // purple: selection, controls
    static let accent2Line = Color(rgb: 0x8E78DA)  // purple lightened for thin lines on dark

    static let blue = Color(rgb: 0x548AF7)
    static let green = Color(rgb: 0x5FB865)
    static let yellow = Color(rgb: 0xD6AE58)
    static let orange = Color(rgb: 0xE08855)
    static let red = Color(rgb: 0xE55765)
    static let purple = Color(rgb: 0xB189F5)
    static let teal = Color(rgb: 0x2AACB8)
    static let series: [Color] = [accent, accent2Line, blue, orange, teal, yellow, red]

    static let canvasNS = NSColor(rgb: 0x1E1F22)
    static let panelNS = NSColor(rgb: 0x2B2D30)
    static let accentNS = NSColor(rgb: 0xBECF00)
    static let accent2NS = NSColor(rgb: 0x5E43B7)
    static let accent2LineNS = NSColor(rgb: 0x8E78DA)

    static let label = Font.system(size: 11)
    static let body = Font.system(size: 12)
    static let mono = Font.system(size: 12, design: .monospaced)
    static let monoSmall = Font.system(size: 11, design: .monospaced)
    static let heading = Font.system(size: 12, weight: .semibold)
}

extension Color {
    init(rgb: UInt32, opacity: Double = 1) {
        self.init(.sRGB, red: Double((rgb >> 16) & 0xff) / 255, green: Double((rgb >> 8) & 0xff) / 255,
                  blue: Double(rgb & 0xff) / 255, opacity: opacity)
    }
}

extension NSColor {
    convenience init(rgb: UInt32) {
        self.init(srgbRed: CGFloat((rgb >> 16) & 0xff) / 255, green: CGFloat((rgb >> 8) & 0xff) / 255,
                  blue: CGFloat(rgb & 0xff) / 255, alpha: 1)
    }
}

// Flat tool panel: optional small header, hairline border, no shadow.
struct Panel<Content: View, Trailing: View>: View {
    let title: String?
    let trailing: Trailing
    let content: Content

    init(_ title: String? = nil, @ViewBuilder trailing: () -> Trailing,
         @ViewBuilder content: () -> Content) {
        self.title = title
        self.trailing = trailing()
        self.content = content()
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            if title != nil || Trailing.self != EmptyView.self {
                HStack(spacing: 8) {
                    if let title { Text(title).font(Theme.heading).foregroundStyle(Theme.text) }
                    Spacer(minLength: 0)
                    trailing
                }
            }
            content
        }
        .frame(maxWidth: .infinity, alignment: .topLeading)
        .padding(10)
        .background(Theme.panel, in: RoundedRectangle(cornerRadius: 6))
        .overlay(RoundedRectangle(cornerRadius: 6).stroke(Theme.line, lineWidth: 1))
    }
}

extension Panel where Trailing == EmptyView {
    init(_ title: String? = nil, @ViewBuilder content: () -> Content) {
        self.init(title, trailing: { EmptyView() }, content: content)
    }
}

// The existing GroupBoxes take the same flat panel look.
struct AuraGroupBoxStyle: GroupBoxStyle {
    func makeBody(configuration: Configuration) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            configuration.label
                .font(Theme.heading)
                .foregroundStyle(Theme.text)
            configuration.content
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(10)
        .background(Theme.panel, in: RoundedRectangle(cornerRadius: 6))
        .overlay(RoundedRectangle(cornerRadius: 6).stroke(Theme.line, lineWidth: 1))
    }
}

// Compact numeric tile: small label above a monospaced value.
struct Stat: View {
    let label: String
    let value: String
    var tint: Color = Theme.text

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(label).font(Theme.label).foregroundStyle(Theme.muted).lineLimit(1)
            Text(value).font(.system(size: 15, weight: .medium, design: .monospaced))
                .foregroundStyle(tint).lineLimit(1).minimumScaleFactor(0.7)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(.horizontal, 10).padding(.vertical, 8)
        .background(Theme.panel, in: RoundedRectangle(cornerRadius: 6))
        .overlay(RoundedRectangle(cornerRadius: 6).stroke(Theme.line, lineWidth: 1))
    }
}

// One label/value line inside a panel.
struct ValueRow: View {
    let label: String
    let value: String
    var tint: Color = Theme.text

    var body: some View {
        HStack(spacing: 8) {
            Text(label).font(Theme.label).foregroundStyle(Theme.muted).lineLimit(1)
            Spacer(minLength: 8)
            Text(value).font(Theme.monoSmall).foregroundStyle(tint).lineLimit(1)
        }
    }
}

// Text tabs with an accent underline, as in JetBrains tool windows.
struct TextTabs<Tab: Hashable>: View {
    let tabs: [(Tab, String)]
    @Binding var selection: Tab

    var body: some View {
        HStack(spacing: 2) {
            ForEach(tabs, id: \.0) { tab, title in
                Button { selection = tab } label: {
                    VStack(spacing: 5) {
                        Text(title)
                            .font(.system(size: 12, weight: selection == tab ? .medium : .regular))
                            .foregroundStyle(selection == tab ? Theme.text : Theme.muted)
                            .lineLimit(1)
                            .fixedSize()
                        Rectangle().fill(selection == tab ? Theme.accent : .clear).frame(height: 2)
                    }
                    .padding(.horizontal, 8)
                    .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
            }
        }
    }
}

// Small status dot with a label, for the toolbar and status bar.
struct StatusDot: View {
    let color: Color
    let text: String

    var body: some View {
        HStack(spacing: 6) {
            Circle().fill(color).frame(width: 7, height: 7)
            Text(text).font(Theme.label).foregroundStyle(Theme.text).lineLimit(1)
        }
    }
}

// Content page scaffold: a scrolling column with uniform padding.
struct Page<Content: View>: View {
    let content: Content
    init(@ViewBuilder content: () -> Content) { self.content = content() }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 12) { content }
                .padding(14)
                .frame(maxWidth: .infinity, alignment: .topLeading)
        }
        .background(Theme.canvas)
    }
}

// Shown in place of data while Aura is not connected.
struct Placeholder: View {
    let text: String
    var body: some View {
        Text(text).font(Theme.body).foregroundStyle(Theme.muted)
            .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
}
