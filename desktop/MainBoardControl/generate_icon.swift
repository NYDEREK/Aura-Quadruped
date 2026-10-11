import AppKit
import Foundation

// JetBrains-style product icon: angular gradient shards in the two Aura
// accents behind a black square with a large white "Au".
// Coordinates are in a 1024 px canvas, origin bottom-left; everything stays
// inside the 100…924 macOS icon safe area.

guard CommandLine.arguments.count == 2 else { fatalError("output PNG path required") }

func color(_ rgb: UInt32) -> NSColor {
    NSColor(srgbRed: CGFloat((rgb >> 16) & 0xFF) / 255, green: CGFloat((rgb >> 8) & 0xFF) / 255,
            blue: CGFloat(rgb & 0xFF) / 255, alpha: 1)
}

let lime = color(0xBECF00)
let limeLight = color(0xE6F25A)
let purple = color(0x5E43B7)
let purpleLight = color(0x8E78DA)
let purpleDark = color(0x3F2A8C)

guard let bitmap = NSBitmapImageRep(bitmapDataPlanes: nil, pixelsWide: 1024, pixelsHigh: 1024,
                                    bitsPerSample: 8, samplesPerPixel: 4, hasAlpha: true, isPlanar: false,
                                    colorSpaceName: .deviceRGB, bytesPerRow: 0, bitsPerPixel: 0),
      let context = NSGraphicsContext(bitmapImageRep: bitmap) else { fatalError("no bitmap") }
bitmap.size = NSSize(width: 1024, height: 1024)
NSGraphicsContext.saveGraphicsState()
NSGraphicsContext.current = context
context.imageInterpolation = .high

func shard(_ points: [(CGFloat, CGFloat)], _ colors: [NSColor], angle: CGFloat) {
    let path = NSBezierPath()
    path.move(to: NSPoint(x: points[0].0, y: points[0].1))
    for point in points.dropFirst() { path.line(to: NSPoint(x: point.0, y: point.1)) }
    path.close()
    NSGradient(colors: colors)?.draw(in: path, angle: angle)
}

// Back: right-hand violet shard.
shard([(560, 900), (924, 700), (924, 190), (610, 120)], [purpleLight, purple], angle: -60)
// Bottom-left: deep purple shard.
shard([(100, 540), (170, 100), (780, 100), (520, 420)], [purpleDark, purple], angle: 15)
// Top-left: lime shard over both.
shard([(120, 924), (730, 896), (440, 430), (100, 600)], [lime, limeLight], angle: -30)

// Black product square.
NSColor.black.setFill()
NSRect(x: 252, y: 252, width: 520, height: 520).fill()

// "Au" as large as the square allows, centred on its actual glyph bounds.
let letters = NSAttributedString(string: "Au", attributes: [
    .font: NSFont.systemFont(ofSize: 350, weight: .heavy),
    .foregroundColor: NSColor.white,
    .kern: -10,
])
let line = CTLineCreateWithAttributedString(letters)
let glyphs = CTLineGetImageBounds(line, context.cgContext)
context.cgContext.textPosition = CGPoint(x: 512 - glyphs.midX, y: 512 - glyphs.midY)
CTLineDraw(line, context.cgContext)

NSGraphicsContext.restoreGraphicsState()
guard let png = bitmap.representation(using: .png, properties: [:]) else { fatalError("could not render icon") }
try png.write(to: URL(fileURLWithPath: CommandLine.arguments[1]), options: .atomic)
