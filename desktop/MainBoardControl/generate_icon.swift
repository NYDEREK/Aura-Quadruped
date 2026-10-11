import AppKit
import Foundation

guard CommandLine.arguments.count == 2 else { fatalError("output PNG path required") }
let size = NSSize(width: 1024, height: 1024)
let image = NSImage(size: size)
image.lockFocus()

NSColor(srgbRed: 0x1E / 255.0, green: 0x1F / 255.0, blue: 0x22 / 255.0, alpha: 1).setFill()
NSBezierPath(roundedRect: NSRect(origin: .zero, size: size), xRadius: 190, yRadius: 190).fill()
NSColor(srgbRed: 0x39 / 255.0, green: 0x3B / 255.0, blue: 0x40 / 255.0, alpha: 1).setStroke()
let border = NSBezierPath(roundedRect: NSRect(x: 22, y: 22, width: 980, height: 980), xRadius: 170, yRadius: 170)
border.lineWidth = 18
border.stroke()

let text = "Aura" as NSString
let paragraph = NSMutableParagraphStyle()
paragraph.alignment = .center
let attributes: [NSAttributedString.Key: Any] = [
    .font: NSFont.systemFont(ofSize: 250, weight: .black),
    .foregroundColor: NSColor(srgbRed: 0xBE / 255.0, green: 0xCF / 255.0, blue: 0x00 / 255.0, alpha: 1),
    .paragraphStyle: paragraph,
    .kern: -8,
]
let bounds = text.boundingRect(with: size, options: [.usesLineFragmentOrigin], attributes: attributes)
let textRect = NSRect(x: 0, y: (1024 - bounds.height) / 2 - 8, width: 1024, height: bounds.height + 20)
text.draw(in: textRect, withAttributes: attributes)
NSColor(srgbRed: 0x5E / 255.0, green: 0x43 / 255.0, blue: 0xB7 / 255.0, alpha: 1).setFill()
NSBezierPath(roundedRect: NSRect(x: 312, y: 300, width: 400, height: 34), xRadius: 17, yRadius: 17).fill()
image.unlockFocus()

guard let tiff = image.tiffRepresentation,
      let bitmap = NSBitmapImageRep(data: tiff),
      let png = bitmap.representation(using: .png, properties: [:]) else {
    fatalError("could not render icon")
}
try png.write(to: URL(fileURLWithPath: CommandLine.arguments[1]), options: .atomic)
