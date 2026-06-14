import CoreBluetooth
import Foundation

let defaultDeviceNamePrefix = "DCtrl"
let remoteServiceUUID = CBUUID(string: "6d1f0001-5b9b-4f4e-9c67-7b1f4d430001")
let remoteRxUUID = CBUUID(string: "6d1f0002-5b9b-4f4e-9c67-7b1f4d430001")
let remoteTxUUID = CBUUID(string: "6d1f0003-5b9b-4f4e-9c67-7b1f4d430001")

struct Config {
    var deviceName = ""
    var deviceNamePrefix = defaultDeviceNamePrefix
    var motion = "zero"
    var timeoutSeconds = 15.0
    var durationMs = 500
    var intervalMs = 50
}

func appendInt32LE(_ value: Int32, to data: inout Data) {
    var little = value.littleEndian
    withUnsafeBytes(of: &little) { data.append(contentsOf: $0) }
}

func appendUInt16LE(_ value: UInt16, to data: inout Data) {
    var little = value.littleEndian
    withUnsafeBytes(of: &little) { data.append(contentsOf: $0) }
}

func dflinkFixedF32(_ value: Double) -> Int32 {
    return Int32((value * 10000.0).rounded())
}

func degreesToRadians(_ value: Double) -> Double {
    return value * Double.pi / 180.0
}

func motionVelocityFrame(vx: Double, vy: Double, vzDegPerCommand: Double) -> Data {
    var payload = Data()
    appendInt32LE(dflinkFixedF32(vx), to: &payload)
    appendInt32LE(dflinkFixedF32(vy), to: &payload)
    appendInt32LE(dflinkFixedF32(degreesToRadians(vzDegPerCommand)), to: &payload)

    var frame = Data([0xdf, 0x01, 0x97, 0x02, 0x62, 0x0c])
    frame.append(payload)
    frame.append(0xfd)
    let checksum = frame.reduce(UInt16(0)) { ($0 &+ UInt16($1)) & 0xffff }
    appendUInt16LE(checksum, to: &frame)
    return frame
}

func remoteFrames(motion: String, durationMs: Int, intervalMs: Int) -> [Data] {
    let active: Data
    switch motion {
    case "forward":
        active = motionVelocityFrame(vx: 0.5, vy: 0, vzDegPerCommand: 0)
    case "backward":
        active = motionVelocityFrame(vx: -0.5, vy: 0, vzDegPerCommand: 0)
    case "strafe":
        active = motionVelocityFrame(vx: 0, vy: 0.5, vzDegPerCommand: 0)
    case "yaw":
        active = motionVelocityFrame(vx: 0, vy: 0, vzDegPerCommand: 5)
    default:
        return [motionVelocityFrame(vx: 0, vy: 0, vzDegPerCommand: 0)]
    }

    let safeInterval = max(1, intervalMs)
    let activeCount = max(1, Int((max(durationMs, safeInterval) + safeInterval - 1) / safeInterval))
    let zero = motionVelocityFrame(vx: 0, vy: 0, vzDegPerCommand: 0)
    return Array(repeating: active, count: activeCount) + [zero, zero, zero]
}

func hexString(_ data: Data) -> String {
    return data.map { String(format: "%02X", $0) }.joined(separator: " ")
}

final class BLERemoteTester: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private let config: Config
    private let frames: [Data]
    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var rxCharacteristic: CBCharacteristic?
    private var txCharacteristic: CBCharacteristic?
    private var writeIndex = 0
    private var waitingForErrorWindow = false
    private var finished = false

    init(config: Config) {
        self.config = config
        self.frames = remoteFrames(motion: config.motion,
                                   durationMs: config.durationMs,
                                   intervalMs: config.intervalMs)
        super.init()
        central = CBCentralManager(delegate: self, queue: DispatchQueue.main)
    }

    func startTimeout() {
        DispatchQueue.main.asyncAfter(deadline: .now() + config.timeoutSeconds) {
            self.finish(false, "timeout after \(self.config.timeoutSeconds)s")
        }
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        print("central_state=\(stateName(central.state))")
        guard central.state == .poweredOn else {
            if central.state == .poweredOff || central.state == .unauthorized || central.state == .unsupported {
                finish(false, "bluetooth central state is \(stateName(central.state))")
            }
            return
        }
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
        print("device_found name=\(name) rssi=\(RSSI)")
        self.peripheral = peripheral
        peripheral.delegate = self
        central.stopScan()
        central.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("connected id=\(peripheral.identifier.uuidString)")
        peripheral.discoverServices([remoteServiceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral,
                        error: Error?) {
        finish(false, "connect failed: \(error?.localizedDescription ?? "unknown")")
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        if !finished {
            finish(false, "disconnected before completion: \(error?.localizedDescription ?? "none")")
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error {
            finish(false, "service discovery failed: \(error.localizedDescription)")
            return
        }
        guard let service = peripheral.services?.first(where: { $0.uuid == remoteServiceUUID }) else {
            finish(false, "remote service not found")
            return
        }
        print("service_found uuid=\(service.uuid.uuidString)")
        peripheral.discoverCharacteristics([remoteRxUUID, remoteTxUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        if let error {
            finish(false, "characteristic discovery failed: \(error.localizedDescription)")
            return
        }
        for characteristic in service.characteristics ?? [] {
            if characteristic.uuid == remoteRxUUID {
                rxCharacteristic = characteristic
            } else if characteristic.uuid == remoteTxUUID {
                txCharacteristic = characteristic
            }
        }
        guard let tx = txCharacteristic, rxCharacteristic != nil else {
            finish(false, "missing remote RX or TX characteristic")
            return
        }
        print("characteristics_found rx=\(remoteRxUUID.uuidString) tx=\(remoteTxUUID.uuidString)")
        peripheral.setNotifyValue(true, for: tx)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateNotificationStateFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let error {
            finish(false, "notify failed: \(error.localizedDescription)")
            return
        }
        writeNextFrame()
    }

    private func writeNextFrame() {
        guard let rx = rxCharacteristic else {
            finish(false, "missing RX characteristic")
            return
        }
        if writeIndex >= frames.count {
            finishAfterQuietWindow()
            return
        }
        let payload = frames[writeIndex]
        print("write_motion motion=\(config.motion) frame=\(writeIndex + 1)/\(frames.count) bytes=\(payload.count) hex=\(hexString(payload))")
        peripheral?.writeValue(payload, for: rx, type: .withResponse)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let error {
            finish(false, "write failed: \(error.localizedDescription)")
            return
        }
        print("write_ack frame=\(writeIndex + 1)/\(frames.count)")
        writeIndex += 1
        if writeIndex >= frames.count {
            finishAfterQuietWindow()
        } else {
            let interval = Double(max(1, config.intervalMs)) / 1000.0
            DispatchQueue.main.asyncAfter(deadline: .now() + interval) {
                self.writeNextFrame()
            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let error {
            finish(false, "notify read failed: \(error.localizedDescription)")
            return
        }
        guard characteristic.uuid == remoteTxUUID,
              let data = characteristic.value,
              let text = String(data: data, encoding: .utf8) else {
            finish(false, "remote response was not UTF-8")
            return
        }
        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        print("remote_response=\(trimmed)")
        if trimmed.hasPrefix("E:") {
            finish(false, "remote returned \(trimmed)")
        } else {
            print("remote_notice_ignored=\(trimmed)")
        }
    }

    private func finishAfterQuietWindow() {
        if waitingForErrorWindow {
            return
        }
        waitingForErrorWindow = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.35) {
            self.finish(true, "all DFLink Motion_Velocity writes completed without remote error")
        }
    }

    private func finish(_ success: Bool, _ reason: String) {
        if finished {
            return
        }
        finished = true
        print("result=\(success ? "PASS" : "FAIL") reason=\(reason)")
        central.stopScan()
        if let peripheral {
            central.cancelPeripheralConnection(peripheral)
        }
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
        case "--name-prefix":
            if !args.isEmpty { config.deviceNamePrefix = args.removeFirst() }
        case "--motion":
            if !args.isEmpty { config.motion = args.removeFirst() }
        case "--timeout":
            if !args.isEmpty { config.timeoutSeconds = Double(args.removeFirst()) ?? config.timeoutSeconds }
        case "--duration-ms":
            if !args.isEmpty { config.durationMs = Int(args.removeFirst()) ?? config.durationMs }
        case "--interval-ms":
            if !args.isEmpty { config.intervalMs = Int(args.removeFirst()) ?? config.intervalMs }
        case "--help", "-h":
            print("""
            Usage:
              swift tools/ble_remote_tester.swift [options]

            Options:
              --name NAME          Exact BLE device name. If omitted, scans by DCtrl prefix.
              --name-prefix TEXT   BLE device name prefix, default DCtrl
              --motion NAME        zero, forward, backward, strafe, or yaw. Default zero.
              --duration-ms MS     Non-zero motion repeat duration, default 500
              --interval-ms MS     Repeat interval, default 50
              --timeout SECONDS    Overall timeout, default 15
            """)
            exit(0)
        default:
            print("Unknown argument: \(arg)")
            exit(64)
        }
    }
    return config
}

let config = parseArgs()
print("test_start service=\(remoteServiceUUID.uuidString) motion=\(config.motion)")
let tester = BLERemoteTester(config: config)
tester.startTimeout()
RunLoop.main.run()
