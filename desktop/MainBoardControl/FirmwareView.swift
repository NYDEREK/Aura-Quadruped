import SwiftUI
import Foundation

// Firmware tab: builds the ESP-IDF project this app was built from and
// flashes it over USB with tools/flash_firmware.sh. The script never erases
// NVS, so servo calibration, profiles and the paired pad are kept; it backs
// NVS up before flashing and compares it afterwards.
private final class TextBox: @unchecked Sendable { var text = "" }

@MainActor
final class FirmwareUpdater: ObservableObject {
    @Published var log = ""
    @Published var running = false
    @Published var lastResult: Int32?
    @Published var commit = ""
    // View state lives here: the command-line toolchain has no SwiftUI
    // macro plugin, so @State cannot be used in this app.
    @Published var pullFirst = true
    @Published var confirm = false

    // build.sh records the repository path in the bundle.
    let projectPath: String? = {
        guard let url = Bundle.main.url(forResource: "project_path", withExtension: "txt"),
              let text = try? String(contentsOf: url, encoding: .utf8) else { return nil }
        let path = text.trimmingCharacters(in: .whitespacesAndNewlines)
        return FileManager.default.fileExists(atPath: path + "/tools/flash_firmware.sh") ? path : nil
    }()

    private var process: Process?

    func refreshCommit() {
        guard let projectPath else { return }
        run(["/usr/bin/git", "-C", projectPath, "log", "-1", "--format=%h  %s  (%cr)"], capture: true) { [weak self] out, _ in
            self?.commit = out.trimmingCharacters(in: .whitespacesAndNewlines)
        }
    }

    func flash(pull: Bool, completion: @escaping (Bool) -> Void) {
        guard let projectPath, !running else { return }
        log = ""; lastResult = nil; running = true
        var arguments = [projectPath + "/tools/flash_firmware.sh"]
        if pull { arguments.append("--pull") }
        run(["/bin/bash"] + arguments, capture: false) { [weak self] _, status in
            guard let self else { return }
            self.running = false
            self.lastResult = status
            self.refreshCommit()
            completion(status == 0)
        }
    }

    private func run(_ command: [String], capture: Bool,
                     done: @escaping (String, Int32) -> Void) {
        let task = Process()
        task.executableURL = URL(fileURLWithPath: command[0])
        task.arguments = Array(command.dropFirst())
        var environment = ProcessInfo.processInfo.environment
        environment["PATH"] = "/usr/local/bin:/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin"
        task.environment = environment
        let pipe = Pipe()
        task.standardOutput = pipe
        task.standardError = pipe
        let collected = TextBox()
        pipe.fileHandleForReading.readabilityHandler = { [weak self] handle in
            let data = handle.availableData
            guard !data.isEmpty, let text = String(data: data, encoding: .utf8) else { return }
            Task { @MainActor in
                if capture { collected.text += text } else { self?.log += text }
            }
        }
        task.terminationHandler = { [weak self] finished in
            pipe.fileHandleForReading.readabilityHandler = nil
            let rest = pipe.fileHandleForReading.readDataToEndOfFile()
            let tail = String(data: rest, encoding: .utf8) ?? ""
            let status = finished.terminationStatus
            Task { @MainActor in
                if capture { collected.text += tail } else { self?.log += tail }
                self?.process = nil
                done(collected.text, status)
            }
        }
        do {
            try task.run()
            if !capture { process = task }
        } catch {
            log += "Could not start: \(error.localizedDescription)\n"
            done("", -1)
        }
    }
}

struct FirmwareView: View {
    @EnvironmentObject private var aura: AuraConnection
    @StateObject private var updater = FirmwareUpdater()

    private var armed: Bool { aura.isConnected && aura.robotState.armed }

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            BoardHealthPanel()
            GroupBox("Aura firmware") {
                VStack(alignment: .leading, spacing: 10) {
                    if let path = updater.projectPath {
                        LabeledContent("Project", value: path)
                        LabeledContent("Repository version", value: updater.commit.isEmpty ? "—" : updater.commit)
                    } else {
                        Text("Repository path unknown — build the app with desktop/MainBoardControl/build.sh.")
                            .foregroundStyle(.orange)
                    }
                    Toggle("Pull the latest code from GitHub first (git pull)", isOn: $updater.pullFirst)
                    Text("USB-C · settings and calibration are kept · NVS backup in backups/")
                        .font(Theme.label).foregroundStyle(Theme.muted)
                    HStack {
                        Button {
                            updater.confirm = true
                        } label: {
                            Label("Flash latest code to Aura", systemImage: "arrow.down.circle.fill")
                        }
                        .buttonStyle(AuraButtonStyle(prominent: true))
                        .disabled(updater.projectPath == nil || updater.running || armed)
                        if updater.running { ProgressView().controlSize(.small); Text("Flashing…") }
                        if let result = updater.lastResult, !updater.running {
                            Text(result == 0 ? "Done" : "Failed (code \(result)) — see the log")
                                .foregroundStyle(result == 0 ? .green : .red)
                        }
                    }
                    if armed {
                        Text("Aura is armed — disarm it (Create) before flashing.")
                            .foregroundStyle(.orange)
                    }
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }
            GroupBox("Log") {
                ScrollViewReader { proxy in
                    ScrollView {
                        Text(updater.log.isEmpty ? "—" : updater.log)
                            .font(.system(.caption, design: .monospaced))
                            .textSelection(.enabled)
                            .frame(maxWidth: .infinity, alignment: .leading)
                        Color.clear.frame(height: 1).id("end")
                    }
                    .onChange(of: updater.log) { _, _ in proxy.scrollTo("end", anchor: .bottom) }
                }
                .frame(minHeight: 260)
            }
        }
        .padding(16)
        .onAppear { updater.refreshCommit() }
        .alert("Flash firmware to Aura?", isPresented: $updater.confirm) {
            Button("Flash") {
                updater.flash(pull: updater.pullFirst) { ok in
                    // The ESP restarts after flashing; reconnect once it is back.
                    DispatchQueue.main.asyncAfter(deadline: .now() + (ok ? 8 : 1)) { aura.reconnect() }
                }
            }
            Button("Cancel", role: .cancel) {}
        } message: {
            Text("The ESP restarts and the servos disarm. Let the robot lie down or support it.")
        }
    }
}

// Why the board last started and how fast each of its tasks loops. A task at
// 0 Hz while the rest keep running has stopped.
private struct BoardHealthPanel: View {
    @EnvironmentObject private var aura: AuraConnection

    var body: some View {
        let health = aura.health
        GroupBox("Board health") {
            VStack(alignment: .leading, spacing: 5) {
                ValueRow(label: "Last start", value: health.available ? health.resetText : "—",
                         tint: health.resetAbnormal ? Theme.orange : Theme.text)
                if health.stackAvailable {
                    ValueRow(label: "Gait tick (budget 20 ms)", value: "\(health.gaitTickMaxMs) ms",
                             tint: health.gaitTickMaxMs > 20 ? Theme.red : health.gaitTickMaxMs > 14 ? Theme.orange : Theme.text)
                }
                if health.cpuAvailable {
                    ValueRow(label: "CPU 0 / 1 (\(health.cpuMHz) MHz)",
                             value: "\(health.coreLoad[0]) / \(health.coreLoad[1]) %",
                             tint: health.coreLoad.max() ?? 0 >= AuraHealth.busyCorePercent ? Theme.red : Theme.text)
                    ValueRow(label: "Gait CPU per tick",
                             value: String(format: "%.1f ms · %d Hz", Double(health.gaitTickCPUus) / 1000,
                                           health.gaitTicksPerSecond),
                             tint: health.gaitTickCPUus > 14_000 ? Theme.orange : Theme.text)
                    ValueRow(label: "Planning parts (ms)", value: health.planPartsText)
                    ValueRow(label: "Busiest", value: health.busiestText)
                        .help(health.busiestTasks.map { "\($0.name): \($0.percent)% of a core, " +
                            ($0.core.map { "pinned to core \($0)" } ?? "either core") }.joined(separator: "\n"))
                }
                if health.crash.received {
                    let crash = health.crash
                    ValueRow(label: "Last crash",
                             value: !crash.supported ? "not recorded" : crash.present ? crash.summary : "none",
                             tint: crash.present && crash.thisBoot ? Theme.red : Theme.text)
                        .help(crash.present ? crash.summary + "\nBacktrace: " + crash.backtraceText : "")
                        .textSelection(.enabled)
                }
                if health.timingAvailable {
                    ValueRow(label: "Tick parts", value: zip(AuraHealth.sections, health.sectionMs)
                        .map { "\($0) \(String(format: "%.1f", $1))" }.joined(separator: " · "))
                    ValueRow(label: "Free RAM / lowest", value: "\(health.heapFree / 1024) / \(health.heapMinimum / 1024) KB",
                             tint: health.heapMinimum < AuraHealth.lowHeapBytes ? Theme.red : Theme.text)
                }
                ForEach(AuraHealth.tasks.indices, id: \.self) { task in
                    let rate = health.rates[task]
                    let stack = health.stackFree[task]
                    ValueRow(label: AuraHealth.tasks[task],
                             value: (!aura.isConnected || rate < 0 ? "—" : String(format: "%.0f Hz", rate))
                                 + (stack > 0 ? " · \(stack) B free" : ""),
                             tint: rate == 0 || (stack > 0 && stack < AuraHealth.lowStackBytes) ? Theme.red : Theme.text)
                }
            }
            .frame(maxWidth: 360, alignment: .leading)
        }
    }
}
