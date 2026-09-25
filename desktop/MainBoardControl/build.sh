#!/bin/bash
set -euo pipefail

source_dir="$(cd -P -- "$(dirname -- "$0")" && pwd)"
project_dir="$(cd -P -- "$source_dir/../.." && pwd)"
app_dir="$source_dir/dist/Aura.app"
icon_source="$source_dir/dist/AppIcon-1024.png"
iconset="$source_dir/dist/AppIcon.iconset"

rm -rf -- "$app_dir"
mkdir -p -- "$app_dir/Contents/MacOS" "$app_dir/Contents/Resources"
cp -- "$source_dir/Info.plist" "$app_dir/Contents/Info.plist"
cp -- "$source_dir/aura_network_bridge.py" "$app_dir/Contents/Resources/aura_network_bridge.py"

xcrun swift "$source_dir/generate_icon.swift" "$icon_source"
rm -rf -- "$iconset"
mkdir -p -- "$iconset"
for spec in "16 icon_16x16.png" "32 icon_16x16@2x.png" "32 icon_32x32.png" \
            "64 icon_32x32@2x.png" "128 icon_128x128.png" "256 icon_128x128@2x.png" \
            "256 icon_256x256.png" "512 icon_256x256@2x.png" "512 icon_512x512.png" \
            "1024 icon_512x512@2x.png"; do
    pixels="${spec%% *}"
    filename="${spec#* }"
    sips -z "$pixels" "$pixels" "$icon_source" --out "$iconset/$filename" >/dev/null
done
iconutil -c icns "$iconset" -o "$app_dir/Contents/Resources/AppIcon.icns"

# Compile the portable ESP planner once for the desktop Test IK preview.
planner_objects=()
mkdir -p "$source_dir/dist/planner"
for unit in robot_body_trajectory robot_gait_profile robot_locomotion robot_predictive_support robot_kinematics; do
    object="$source_dir/dist/planner/$unit.o"
    xcrun clang -std=c11 -O2 -I "$project_dir/main" -c "$project_dir/main/$unit.c" -o "$object"
    planner_objects+=("$object")
done

xcrun swiftc -parse-as-library -O \
    -import-objc-header "$source_dir/LocomotionBridge.h" -I "$project_dir/main" \
    "${planner_objects[@]}" \
    -framework SwiftUI -framework AppKit -framework Foundation -framework SceneKit \
    -framework Network -framework Charts \
    "$source_dir/MainBoardControl.swift" "$source_dir/AuraConnection.swift" "$source_dir/RobotControl.swift" "$source_dir/RobotTelemetry.swift" "$source_dir/QuadrupedView.swift" "$source_dir/CalibrationView.swift" "$source_dir/IMUAttitudePreview.swift" \
    -o "$app_dir/Contents/MacOS/MainBoardControl"

codesign --force --deep --sign - "$app_dir"
echo "$app_dir"
