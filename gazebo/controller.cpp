// Minimal 50 Hz joint-command process for the Aura Gazebo model.
//
// The published topics are deliberately the same boundary a future serial / Wi-Fi
// bridge will use.  This process does not fake body translation: Gazebo decides
// whether feet grip, slip, or the body falls from the commanded joint angles.

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

namespace {
constexpr std::array<const char *, 4> kLegs{"lf", "rf", "lr", "rr"};
constexpr std::array<const char *, 3> kAxes{"abduction", "hip", "knee"};
constexpr double kHz = 50.0;

struct JointTargets {
  // Ordering: lf ab/hip/knee, rf..., lr..., rr...
  std::array<double, 12> value{};
};

JointTargets StandPose() {
  JointTargets pose;
  for (std::size_t leg = 0; leg < kLegs.size(); ++leg) {
    // Fully extended neutral pose.  Ab/ad is roll about the forward axis;
    // hip and knee are pitch about lateral axis.  There is no yaw command.
    pose.value[leg * 3 + 0] = 0.0;
    pose.value[leg * 3 + 1] = 0.0;
    pose.value[leg * 3 + 2] = 0.0;
  }
  return pose;
}

JointTargets MechanicalDemo(double seconds) {
  // A bounded, diagonal-pair test of the three real axes.  It is intentionally
  // not called a walking gait: a real gait will be added only after physical
  // dimensions and contact behaviour have been calibrated in Gazebo.
  JointTargets pose = StandPose();
  const double wave = std::sin(2.0 * M_PI * 0.45 * seconds);
  for (std::size_t leg = 0; leg < kLegs.size(); ++leg) {
    const bool diagonalA = leg == 0 || leg == 3;
    const double phase = diagonalA ? wave : -wave;
    pose.value[leg * 3 + 0] = (leg == 0 || leg == 2 ? 1.0 : -1.0) * 0.07;
    pose.value[leg * 3 + 1] = 0.18 * phase;
    pose.value[leg * 3 + 2] = -0.34 * std::max(0.0, phase);
  }
  return pose;
}
}  // namespace

int main(int argc, char **argv) {
  const bool demo = argc == 2 && std::strcmp(argv[1], "--demo") == 0;
  if (argc > 1 && !demo) {
    std::cerr << "Usage: aura_controller [--demo]\\n";
    return 2;
  }

  gz::transport::Node node;
  std::array<gz::transport::Node::Publisher, 12> publishers;
  for (std::size_t leg = 0; leg < kLegs.size(); ++leg) {
    for (std::size_t axis = 0; axis < kAxes.size(); ++axis) {
      const auto i = leg * kAxes.size() + axis;
      const std::string topic = std::string("/aura/") + kLegs[leg] + "_" +
          kAxes[axis] + "/cmd_pos";
      publishers[i] = node.Advertise<gz::msgs::Double>(topic);
      if (!publishers[i]) {
        std::cerr << "Cannot advertise " << topic << "\\n";
        return 1;
      }
    }
  }

  std::cout << "Aura controller: " << (demo ? "mechanical demo" : "stand")
            << " at " << kHz << " Hz.  Ctrl-C stops it.\\n";
  const auto started = std::chrono::steady_clock::now();
  const auto period = std::chrono::microseconds(static_cast<int>(1'000'000 / kHz));
  for (auto next = started;; next += period) {
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    const auto target = demo ? MechanicalDemo(elapsed) : StandPose();
    for (std::size_t i = 0; i < publishers.size(); ++i) {
      gz::msgs::Double message;
      message.set_data(target.value[i]);
      publishers[i].Publish(message);
    }
    std::this_thread::sleep_until(next + period);
  }
}
