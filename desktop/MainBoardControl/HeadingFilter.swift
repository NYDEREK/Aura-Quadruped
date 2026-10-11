import Foundation

// Heading Kalman filter, state [heading, gyro bias]. The gyro rate drives the
// prediction; whenever the robot is provably still, a zero-rate measurement
// observes the bias. Heading itself has no absolute reference (no compass),
// so its variance only grows - shown as the heading uncertainty.
struct HeadingFilter {
    private(set) var heading = 0.0
    private(set) var bias = 0.0
    private static let initialBiasSigma: Double = 2.0 * .pi / 180
    private var p00 = 0.0, p01 = 0.0, p11 = initialBiasSigma * initialBiasSigma
    private let gyroNoise: Double = 0.6 * .pi / 180     // rad/s, white
    private let biasWalk: Double = 0.02 * .pi / 180     // rad/s/sqrt(s)
    private let stillNoise: Double = 0.25 * .pi / 180   // rad/s, zero-rate update

    var headingSigma: Double { p00.squareRoot() }
    var biasSigma: Double { p11.squareRoot() }

    mutating func reset(keepBias: Bool) {
        heading = 0; p00 = 0; p01 = 0
        if !keepBias { bias = 0; p11 = Self.initialBiasSigma * Self.initialBiasSigma }
    }

    mutating func predict(gyroRate: Double, dt: Double) {
        heading += (gyroRate - bias) * dt
        let qHeading: Double = gyroNoise * dt * gyroNoise * dt
        let qBias: Double = biasWalk * biasWalk * dt
        let next00: Double = p00 - 2 * dt * p01 + dt * dt * p11 + qHeading
        p00 = next00
        p01 = p01 - dt * p11
        p11 = p11 + qBias
    }

    // z = true yaw rate (0 while still) = gyroRate - bias + noise; H = [0, -1].
    mutating func observeStill(gyroRate: Double) {
        let s = p11 + stillNoise * stillNoise
        let residual = 0 - (gyroRate - bias)
        let k0 = -p01 / s, k1 = -p11 / s
        heading += k0 * residual
        bias += k1 * residual
        let n00 = p00 - p01 * p01 / s, n01 = p01 * (1 - p11 / s), n11 = p11 * (1 - p11 / s)
        p00 = n00; p01 = n01; p11 = n11
    }
}
