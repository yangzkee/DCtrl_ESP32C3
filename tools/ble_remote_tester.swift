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
}

func appendInt16LE(_ value: Int16, to data: inout Data) {
    var little = value.littleEndian
    withUnsafeBytes(of: &little) { data.append(contentsOf: $0) }
}

func appendInt32LE(_ value: Int32, to data: inout Data) {
    var little = value.littleEndian
    withUnsafeBytes(of: &little) { data.append(contentsOf: $0) }
}

func remotePayload(motion: String) -> Data {
    let tuple: (Int16, Int16, Int32)
    switch motion {
    case "forward":
        tuple = (500, 0, 0)
    case "backward":
        tuple = (-500, 0, 0)
    case "strafe":
        tuple = (0, 500, 0)
    case "yaw":
        tuple = (0, 0, 1500)
    default:
        tuple = (0, 0, 0)
    }

    var data = Data([0x44, 0x43, 0x01, 0x01])
    appendInt16LE(tuple.0, to: &data)
    appendInt16LE(tuple.1, to: &data)
    appendInt32LE(tuple.2, to: &data)
    let checksum = data.reduce(UInt8(0)) { $0 &+ $1 }
    data.append(checksum)
    return data
}

final class BLERemoteTester: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private let config: Config
    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var rxCharacteristic: CBCharacteristic?
    private var txCharacteristic: CBCharacteristic?
    private var finished = false

    init(config: Config) {
        self.config = config
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
            finish(false, "disconnected before OK: \(error?.localizedDescription ?? "none")")
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
        writeMotion()
    }

    private func writeMotion() {
        guard let rx = rxCharacteristic else {
            finish(false, "missing RX characteristic")
            return
        }
        let payload = remotePayload(motion: config.motion)
        print("write_motion motion=\(config.motion) bytes=\(payload.count)")
        peripheral?.writeValue(payload, for: rx, type: .withResponse)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let error {
            finish(false, "write failed: \(error.localizedDescription)")
            return
        }
        print("write_ack")
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
        if trimmed == "OK" {
            finish(true, "remote motion accepted")
        } else {
            finish(false, "remote returned \(trimmed)")
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
        case "--help", "-h":
            print("""
            Usage:
              swift tools/ble_remote_tester.swift [options]

            Options:
              --name NAME          Exact BLE device name. If omitted, scans by DCtrl prefix.
              --name-prefix TEXT   BLE device name prefix, default DCtrl
              --motion NAME        zero, forward, backward, strafe, or yaw. Default zero.
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
