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
            log += "Nie udało się uruchomić: \(error.localizedDescription)\n"
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
            GroupBox("Firmware Aury") {
                VStack(alignment: .leading, spacing: 10) {
                    if let path = updater.projectPath {
                        LabeledContent("Projekt", value: path)
                        LabeledContent("Wersja w repozytorium", value: updater.commit.isEmpty ? "—" : updater.commit)
                    } else {
                        Text("Nie znam ścieżki do repozytorium Aura-Quadruped. Zbuduj aplikację przez desktop/MainBoardControl/build.sh.")
                            .foregroundStyle(.orange)
                    }
                    Toggle("Najpierw pobierz najnowszy kod z GitHuba (git pull)", isOn: $updater.pullFirst)
                    Text("Podłącz Aurę kablem USB-C. Wgrywanie nie kasuje kalibracji serw ani ustawień — kopia pamięci ustawień trafia do backups/.")
                        .font(.callout).foregroundStyle(.secondary)
                    HStack {
                        Button {
                            updater.confirm = true
                        } label: {
                            Label("Wgraj najnowszy kod do Aury", systemImage: "arrow.down.circle.fill")
                        }
                        .buttonStyle(.borderedProminent)
                        .disabled(updater.projectPath == nil || updater.running || armed)
                        if updater.running { ProgressView().controlSize(.small); Text("Wgrywanie…") }
                        if let result = updater.lastResult, !updater.running {
                            Text(result == 0 ? "Gotowe" : "Błąd (kod \(result)) — szczegóły w logu")
                                .foregroundStyle(result == 0 ? .green : .red)
                        }
                    }
                    if armed {
                        Text("Aura jest uzbrojona — rozbrój ją (Create) przed wgraniem. Restart ESP zwalnia serwa.")
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
        .alert("Wgrać firmware do Aury?", isPresented: $updater.confirm) {
            Button("Wgraj") {
                updater.flash(pull: updater.pullFirst) { ok in
                    // The ESP restarts after flashing; reconnect once it is back.
                    DispatchQueue.main.asyncAfter(deadline: .now() + (ok ? 8 : 1)) { aura.reconnect() }
                }
            }
            Button("Anuluj", role: .cancel) {}
        } message: {
            Text("ESP zrestartuje się, a serwa zostaną rozbrojone. Robot powinien leżeć lub być podparty.")
        }
    }
}
