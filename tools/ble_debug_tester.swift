import CoreBluetooth
import Foundation

let defaultDeviceNamePrefix = "DCtrl"
let serviceUUID = CBUUID(string: "7b3a0001-8d4d-4b9a-b5c7-0f7c4c415243")
let rxUUID = CBUUID(string: "7b3a0002-8d4d-4b9a-b5c7-0f7c4c415243")
let txUUID = CBUUID(string: "7b3a0003-8d4d-4b9a-b5c7-0f7c4c415243")

struct Config {
    var deviceName = ""
    var deviceNamePrefix = defaultDeviceNamePrefix
    var command = #"{"type":"get_telemetry"}"#
    var timeoutSeconds = 15.0
    var logRoot = "logs/ble-debug-tests"
    var writeChunkBytes = 0
    var appendNewline = false
    var expectPrefix: String?
}

final class Logger {
    let folder: URL
    private let eventsURL: URL
    private var events = 0

    init(root: String) throws {
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyyMMdd-HHmmss"
        let stamp = formatter.string(from: Date())
        folder = URL(fileURLWithPath: root).appendingPathComponent(stamp)
        try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
        eventsURL = folder.appendingPathComponent("events.jsonl")
        FileManager.default.createFile(atPath: eventsURL.path, contents: nil)
    }

    func event(_ type: String, _ fields: [String: Any] = [:]) {
        var payload = fields
        payload["type"] = type
        payload["ts"] = ISO8601DateFormatter().string(from: Date())
        if let data = try? JSONSerialization.data(withJSONObject: payload, options: []),
           let line = String(data: data, encoding: .utf8),
           let handle = try? FileHandle(forWritingTo: eventsURL) {
            handle.seekToEndOfFile()
            handle.write((line + "\n").data(using: .utf8)!)
            try? handle.close()
            events += 1
        }
        print("[\(type)] \(fields)")
    }

    func summary(status: String, checks: [(String, Bool)], response: String?, notes: [String]) {
        let passed = checks.filter { $0.1 }.count
        let failed = checks.count - passed
        var lines: [String] = []
        lines.append("# BLE Debug Test Summary")
        lines.append("")
        lines.append("- Status: \(status)")
        lines.append("- Checks: \(passed) passed, \(failed) failed")
        lines.append("- Events: \(events)")
        lines.append("- Log folder: \(folder.path)")
        lines.append("")
        lines.append("## Checks")
        lines.append("")
        for (name, ok) in checks {
            lines.append("- \(ok ? "PASS" : "FAIL") \(name)")
        }
        if !notes.isEmpty {
            lines.append("")
            lines.append("## Notes")
            lines.append("")
            for note in notes {
                lines.append("- \(note)")
            }
        }
        if let response {
            lines.append("")
            lines.append("## Response")
            lines.append("")
            lines.append("```")
            lines.append(response)
            lines.append("```")
        }
        try? lines.joined(separator: "\n").write(to: folder.appendingPathComponent("summary.md"),
                                                 atomically: true,
                                                 encoding: .utf8)
    }
}

final class BLEDebugTester: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private let config: Config
    private let logger: Logger
    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var rxCharacteristic: CBCharacteristic?
    private var txCharacteristic: CBCharacteristic?
    private var response: String?
    private var expectedResponseBytes: Int?
    private var chunkBuffer = ""
    private var readRequested = false
    private var checks: [(String, Bool)] = []
    private var notes: [String] = []
    private var finished = false

    init(config: Config, logger: Logger) {
        self.config = config
        self.logger = logger
        super.init()
        central = CBCentralManager(delegate: self, queue: DispatchQueue.main)
    }

    func startTimeout() {
        DispatchQueue.main.asyncAfter(deadline: .now() + config.timeoutSeconds) {
            self.finish(false, reason: "timeout after \(self.config.timeoutSeconds)s")
        }
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        logger.event("central_state", ["state": stateName(central.state)])
        guard central.state == .poweredOn else {
            if central.state == .poweredOff || central.state == .unauthorized || central.state == .unsupported {
                finish(false, reason: "bluetooth central state is \(stateName(central.state))")
            }
            return
        }
        checks.append(("Bluetooth powered on", true))
        logger.event("scan_start", [
            "device_name": config.deviceName,
            "device_name_prefix": config.deviceNamePrefix
        ])
        central.scanForPeripherals(withServices: nil, options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        let advertisedName = advertisementData[CBAdvertisementDataLocalNameKey] as? String
        let name = advertisedName ?? peripheral.name ?? ""
        let matchesExactName = !config.deviceName.isEmpty && name == config.deviceName
        let matchesPrefix = config.deviceName.isEmpty && name.hasPrefix(config.deviceNamePrefix)
        if !matchesExactName && !matchesPrefix {
            return
        }
        checks.append(("Found \(name)", true))
        logger.event("device_found", [
            "name": name,
            "identifier": peripheral.identifier.uuidString,
            "rssi": RSSI.intValue
        ])
        self.peripheral = peripheral
        peripheral.delegate = self
        central.stopScan()
        logger.event("connect_start")
        central.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        checks.append(("Connected", true))
        logger.event("connected", ["identifier": peripheral.identifier.uuidString])
        peripheral.discoverServices([serviceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral,
                        error: Error?) {
        logger.event("connect_failed", ["error": error?.localizedDescription ?? "unknown"])
        finish(false, reason: "connect failed")
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        logger.event("disconnected", ["error": error?.localizedDescription ?? "none"])
        if !finished {
            finish(false, reason: "disconnected before response")
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error {
            logger.event("discover_services_failed", ["error": error.localizedDescription])
            finish(false, reason: "service discovery failed")
            return
        }
        guard let service = peripheral.services?.first(where: { $0.uuid == serviceUUID }) else {
            finish(false, reason: "service not found")
            return
        }
        checks.append(("Service found", true))
        logger.event("service_found", ["uuid": service.uuid.uuidString])
        peripheral.discoverCharacteristics([rxUUID, txUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        if let error {
            logger.event("discover_characteristics_failed", ["error": error.localizedDescription])
            finish(false, reason: "characteristic discovery failed")
            return
        }
        for characteristic in service.characteristics ?? [] {
            if characteristic.uuid == rxUUID {
                rxCharacteristic = characteristic
            } else if characteristic.uuid == txUUID {
                txCharacteristic = characteristic
            }
        }
        checks.append(("RX characteristic found", rxCharacteristic != nil))
        checks.append(("TX characteristic found", txCharacteristic != nil))
        guard let rx = rxCharacteristic, let tx = txCharacteristic else {
            finish(false, reason: "missing RX or TX characteristic")
            return
        }

        logger.event("characteristics_found", [
            "rx": rx.uuid.uuidString,
            "tx": tx.uuid.uuidString
        ])
        if tx.properties.contains(.notify) {
            peripheral.setNotifyValue(true, for: tx)
        } else {
            writeCommand()
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateNotificationStateFor characteristic: CBCharacteristic,
                    error: Error?) {
        logger.event("notify_state", [
            "uuid": characteristic.uuid.uuidString,
            "is_notifying": characteristic.isNotifying,
            "error": error?.localizedDescription ?? "none"
        ])
        writeCommand()
    }

    private func writeCommand() {
        guard let rx = rxCharacteristic,
              let data = config.command.data(using: .utf8) else {
            finish(false, reason: "invalid command")
            return
        }
        checks.append(("Command encoded", true))
        logger.event("write_command", [
            "command": config.command,
            "bytes": data.count,
            "write_chunk_bytes": config.writeChunkBytes,
            "append_newline": config.appendNewline
        ])
        writeData(data, to: rx, offset: 0)
    }

    private func writeData(_ originalData: Data, to characteristic: CBCharacteristic, offset: Int) {
        var data = originalData
        if config.appendNewline && offset == 0 {
            data.append(0x0a)
        }

        let chunkSize = config.writeChunkBytes > 0 ? config.writeChunkBytes : data.count
        let end = min(offset + chunkSize, data.count)
        let chunk = data.subdata(in: offset..<end)
        logger.event("write_chunk", ["offset": offset, "bytes": chunk.count, "done": end >= data.count])
        peripheral?.writeValue(chunk, for: characteristic, type: .withResponse)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let error {
            logger.event("write_failed", ["error": error.localizedDescription])
            finish(false, reason: "write failed")
            return
        }
        checks.append(("Command write acknowledged", true))
        logger.event("write_ack", ["uuid": characteristic.uuid.uuidString])
        if config.writeChunkBytes > 0,
           let rx = rxCharacteristic,
           var commandData = config.command.data(using: .utf8) {
            if config.appendNewline {
                commandData.append(0x0a)
            }
            let acknowledgedBytes = min(checks.filter { $0.0 == "Command write acknowledged" }.count * config.writeChunkBytes,
                                        commandData.count)
            if acknowledgedBytes < commandData.count {
                let end = min(acknowledgedBytes + config.writeChunkBytes, commandData.count)
                let chunk = commandData.subdata(in: acknowledgedBytes..<end)
                logger.event("write_chunk", [
                    "offset": acknowledgedBytes,
                    "bytes": chunk.count,
                    "done": end >= commandData.count
                ])
                peripheral.writeValue(chunk, for: rx, type: .withResponse)
                return
            }
        }
        if expectedResponseBytes == nil {
            requestRead(peripheral: peripheral, reason: "write_ack")
        }
    }

    private func requestRead(peripheral: CBPeripheral, reason: String) {
        if readRequested {
            logger.event("read_response_skip", ["reason": reason, "why": "already requested"])
            return
        }
        readRequested = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.15) {
            if let tx = self.txCharacteristic {
                self.logger.event("read_response_start", ["reason": reason])
                peripheral.readValue(for: tx)
            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let error {
            logger.event("read_failed", ["error": error.localizedDescription])
            finish(false, reason: "read failed")
            return
        }
        guard characteristic.uuid == txUUID,
              let data = characteristic.value,
              let text = String(data: data, encoding: .utf8) else {
            finish(false, reason: "response was not UTF-8")
            return
        }
        if text.contains(#""type":"ble_response_ready""#) {
            logger.event("response_ready_notification", ["bytes": data.count, "text": text])
            if let jsonData = text.data(using: .utf8),
               let object = try? JSONSerialization.jsonObject(with: jsonData) as? [String: Any],
               let bytes = object["bytes"] as? Int {
                expectedResponseBytes = bytes
            }
            requestRead(peripheral: peripheral, reason: "notification")
            return
        }
        if let jsonData = text.data(using: .utf8),
           let object = try? JSONSerialization.jsonObject(with: jsonData) as? [String: Any],
           object["type"] as? String == "ble_chunk" {
            logger.event("response_chunk", ["bytes": data.count, "text": text])
            guard let chunk = object["data"] as? String,
                  let done = object["done"] as? Bool,
                  let total = object["total"] as? Int else {
                checks.append(("Chunk envelope valid", false))
                finish(false, reason: "invalid chunk envelope")
                return
            }
            checks.append(("Chunk envelope valid", true))
            chunkBuffer += chunk
            expectedResponseBytes = expectedResponseBytes ?? total
            if done {
                finishResponseText(chunkBuffer)
            } else {
                readRequested = false
                requestRead(peripheral: peripheral, reason: "next_chunk")
            }
            return
        }
        logger.event("response_read", ["bytes": data.count, "text": text])
        finishResponseText(text)
    }

    private func finishResponseText(_ text: String) {
        response = text
        checks.append(("Response is UTF-8", true))
        if let expectedResponseBytes {
            let actualBytes = text.data(using: .utf8)?.count ?? 0
            let bytesMatch = actualBytes == expectedResponseBytes
            checks.append(("Response bytes match notification", bytesMatch))
            if !bytesMatch {
                notes.append("Notification advertised \(expectedResponseBytes) response bytes, but the TX read returned \(actualBytes) bytes.")
            }
        }
        if let jsonData = text.data(using: .utf8),
           let object = try? JSONSerialization.jsonObject(with: jsonData),
           JSONSerialization.isValidJSONObject(object) {
            checks.append(("Response is JSON", true))
        } else if isCompactResponse(text) {
            checks.append(("Response is compact BLE frame", true))
        } else {
            checks.append(("Response is JSON or compact BLE frame", false))
            notes.append("Response was readable but not valid JSON. If it is truncated, increase BLE response chunk handling or test with a smaller command.")
        }
        if let prefix = config.expectPrefix {
            checks.append(("Response matches expected prefix \(prefix)", text.hasPrefix(prefix)))
        }
        finish(checks.allSatisfy { $0.1 }, reason: "completed")
    }

    private func finish(_ success: Bool, reason: String) {
        if finished {
            return
        }
        finished = true
        logger.event("finish", ["success": success, "reason": reason])
        if !success {
            notes.append(reason)
        }
        central.stopScan()
        if let peripheral {
            central.cancelPeripheralConnection(peripheral)
        }
        logger.summary(status: success ? "PASS" : "FAIL", checks: checks, response: response, notes: notes)
        print("Log folder: \(logger.folder.path)")
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.2) {
            exit(success ? 0 : 2)
        }
    }
}

func stateName(_ state: CBManagerState) -> String {
    switch state {
    case .unknown: return "unknown"
    case .resetting: return "resetting"
    case .unsupported: return "unsupported"
    case .unauthorized: return "unauthorized"
    case .poweredOff: return "poweredOff"
    case .poweredOn: return "poweredOn"
    @unknown default: return "future"
    }
}

func parseArgs() -> Config {
    var config = Config()
    var args = Array(CommandLine.arguments.dropFirst())
    while !args.isEmpty {
        let arg = args.removeFirst()
        switch arg {
        case "--name":
            if !args.isEmpty { config.deviceName = args.removeFirst() }
        case "--command":
            if !args.isEmpty { config.command = args.removeFirst() }
        case "--timeout":
            if !args.isEmpty { config.timeoutSeconds = Double(args.removeFirst()) ?? config.timeoutSeconds }
        case "--log-root":
            if !args.isEmpty { config.logRoot = args.removeFirst() }
        case "--help", "-h":
            print("""
            Usage:
              swift tools/ble_debug_tester.swift [options]

            Options:
              --name NAME          Exact BLE device name. If omitted, scans by DCtrl prefix.
              --name-prefix TEXT   BLE device name prefix, default DCtrl
              --command JSON       JSON debug command, default {"type":"get_telemetry"}
              --timeout SECONDS    Overall timeout, default 15
              --log-root PATH      Log root, default logs/ble-debug-tests
              --write-chunk BYTES  Split BLE writes, e.g. 20 to simulate WeChat
              --append-newline     Append newline terminator for split-write framing
              --expect-prefix TEXT Check that response starts with TEXT
            """)
            exit(0)
        case "--write-chunk":
            if !args.isEmpty { config.writeChunkBytes = Int(args.removeFirst()) ?? config.writeChunkBytes }
        case "--name-prefix":
            if !args.isEmpty { config.deviceNamePrefix = args.removeFirst() }
        case "--append-newline":
            config.appendNewline = true
        case "--expect-prefix":
            if !args.isEmpty { config.expectPrefix = args.removeFirst() }
        default:
            print("Unknown argument: \(arg)")
            exit(64)
        }
    }
    return config
}

func isCompactResponse(_ text: String) -> Bool {
    let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
    return trimmed == "OK" ||
        trimmed.hasPrefix("P") ||
        trimmed.hasPrefix("N") ||
        trimmed.hasPrefix("C:") ||
        trimmed.hasPrefix("E:")
}

let config = parseArgs()
let logger = try Logger(root: config.logRoot)
logger.event("test_start", [
    "device_name": config.deviceName,
    "device_name_prefix": config.deviceNamePrefix,
    "service_uuid": serviceUUID.uuidString,
    "rx_uuid": rxUUID.uuidString,
    "tx_uuid": txUUID.uuidString,
    "command": config.command,
    "timeout_seconds": config.timeoutSeconds,
    "write_chunk_bytes": config.writeChunkBytes,
    "append_newline": config.appendNewline,
    "expect_prefix": config.expectPrefix ?? ""
])
let tester = BLEDebugTester(config: config, logger: logger)
tester.startTimeout()
RunLoop.main.run()
