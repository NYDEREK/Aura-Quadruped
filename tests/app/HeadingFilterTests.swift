// swiftc -O desktop/MainBoardControl/HeadingFilter.swift tests/app/HeadingFilterTests.swift -o /tmp/hf && /tmp/hf
import Foundation

func check(_ condition: Bool, _ message: String) {
    if !condition { print("FAIL: \(message)"); exit(1) }
}

@main
struct HeadingFilterTests {
    static func main() {
        let dt = 0.02, bias = 1.5 * .pi / 180          // a 1.5 °/s gyro offset
        var rng = SystemRandomNumberGenerator()
        func noise(_ sigma: Double) -> Double { Double.random(in: -sigma...sigma, using: &rng) }

        // Standing still for 10 s: the zero-rate updates learn the bias, so
        // the heading stops drifting.
        var filter = HeadingFilter()
        for _ in 0..<500 {
            let gyro = bias + noise(0.3 * .pi / 180)
            filter.predict(gyroRate: gyro, dt: dt)
            filter.observeStill(gyroRate: gyro)
        }
        check(abs(filter.bias - bias) < 0.15 * .pi / 180, "bias learned while still")
        check(abs(filter.heading) < 1.0 * .pi / 180, "no heading drift while still")

        // Then a 90° left turn at 30 °/s: integrated with the learned bias.
        let rate = 30.0 * .pi / 180
        for _ in 0..<Int(3.0 / dt) { filter.predict(gyroRate: rate + bias + noise(0.3 * .pi / 180), dt: dt) }
        check(abs(filter.heading - .pi / 2) < 1.5 * .pi / 180, "90° turn integrated")
        check(filter.headingSigma > 0, "heading uncertainty grows while moving")

        // Without bias estimation the same 13 s would drift by ~20°.
        var naive = HeadingFilter()
        for _ in 0..<500 { naive.predict(gyroRate: bias, dt: dt) }
        check(abs(naive.heading) > 10 * .pi / 180, "uncorrected bias drifts")
        print("heading filter tests passed")
    }
}
