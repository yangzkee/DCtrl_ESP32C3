#!/usr/bin/env python3
"""Build, validate and replace the two ESP32-C3 line-following releases (300s total)."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "output/.firmware-build"
RELEASE = ROOT / "release-packages"
STATE = ROOT / ".codex/line-release-budget.json"
LOG = ROOT / ".codex/line-release-build.log"
VERSION = "1.1.0"


def remove_tree(path):
    def ignore_missing(_function, _path, error):
        if not isinstance(error[1], FileNotFoundError):
            raise error[1]
    shutil.rmtree(path, onerror=ignore_missing)


def main():
    STATE.parent.mkdir(exist_ok=True)
    previous = json.loads(STATE.read_text()) if STATE.exists() else {}
    started = previous.get("started", time.time()) if previous.get("status") != "complete" else time.time()
    deadline = started + 300
    resuming = previous.get("status") not in (None, "complete")
    state = {"started": started, "status": "running",
             "completed_steps": previous.get("completed_steps", []) if resuming else [],
             "outputs": previous.get("outputs", {}) if resuming else {}}
    STATE.write_text(json.dumps(state, indent=2))

    def expire(_signum, _frame):
        raise TimeoutError("300 second cumulative build/validation budget exhausted")

    signal.signal(signal.SIGALRM, expire)
    signal.setitimer(signal.ITIMER_REAL, max(0.01, deadline - time.time()))

    def run(args, label, env=None):
        if label in state["completed_steps"]:
            if label.endswith("application image integrity"):
                return state["outputs"].get(label) or LOG.read_text(errors="replace").rsplit("\n" + label + "\n", 1)[-1]
            return ""
        remaining = deadline - time.time()
        if remaining <= 0:
            raise TimeoutError("300 second cumulative build/validation budget exhausted")
        print(label, flush=True)
        with LOG.open("a") as log:
            log.write("\n" + label + "\n")
            log.flush()
            output_start = LOG.stat().st_size
            process = subprocess.Popen([str(x) for x in args], cwd=ROOT, stdout=log,
                                       stderr=subprocess.STDOUT, env=env, start_new_session=True)
            try:
                code = process.wait(timeout=remaining)
            except (subprocess.TimeoutExpired, TimeoutError):
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                raise TimeoutError("300 second cumulative build/validation budget exhausted")
        if code:
            raise RuntimeError(label + " failed:\n" + "\n".join(LOG.read_text(errors="replace").splitlines()[-28:]))
        state["completed_steps"].append(label)
        output = LOG.read_bytes()[output_start:].decode(errors="replace")
        if label.endswith("application image integrity"):
            state["outputs"][label] = output
        STATE.write_text(json.dumps(state, indent=2))
        return output

    try:
        run([sys.executable, ROOT / "tools/check_feature_boundaries.py"], "Feature ownership checks")
        with tempfile.TemporaryDirectory(prefix="dctrl-regression-") as temp:
            temp = Path(temp)
            (temp / "esp_err.h").write_text(
                "#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n"
                "#define ESP_ERR_INVALID_ARG 1\n#define ESP_ERR_INVALID_RESPONSE 2\n")
            (temp / "board_profile.h").write_text(
                "#pragma once\ntypedef struct board_uart_config board_uart_config_t;\n")
            includes = [temp] + [ROOT / "components" / component / "include"
                                 for component in ("chassis", "line_sensor", "control", "vehicle_state")]
            sources = [ROOT / "tools/line_following_regression.c",
                       ROOT / "components/line_sensor/line_sensor_protocol.c",
                       ROOT / "components/chassis/dflink_motion_codec.c"]
            sources += [ROOT / "components/control" / (name + ".c") for name in
                        ("motion_contracts", "motion_inputs", "line_interpreter", "motion_policy",
                         "motion_executor", "line_trace_policy")]
            for color, white in (("black", 0), ("white", 1)):
                executable = temp / color
                args = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1",
                        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                        "-DDCTRL_TRACK_WHITE=" + str(white)]
                for include in includes:
                    args += ["-I", include]
                run(args + sources + ["-o", executable], color + " native regression compilation")
                run([executable], color + " sensor/steering/recovery/wire regressions")

        BUILD.mkdir(parents=True, exist_ok=True)
        config = BUILD / "sdkconfig"
        if not config.exists():
            shutil.copyfile(ROOT / "sdkconfig.esp32c3", config)
        staged = BUILD / "release"
        staged.mkdir(exist_ok=True)
        idf_python = next((ROOT / ".idf-tools/python_env").glob("*/bin/python"), None)
        if idf_python is None:
            raise RuntimeError("Project ESP-IDF Python environment is missing")
        records = []
        colors = ("black", "white")
        published = all(
            (RELEASE / ("DCtrl_V" + VERSION + "_" + color + "-line.bin")).exists()
            and color + " application image integrity" in state["completed_steps"]
            for color in colors)
        if published:
            for color in colors:
                name = "DCtrl_V" + VERSION + "_" + color + "-line.bin"
                data = (RELEASE / name).read_bytes()
                records.append({"file": name, "line_color": color, "version": VERSION + "-" + color,
                                "chip": "esp32c3", "flash_address": "0x0", "bytes": len(data),
                                "sha256": hashlib.sha256(data).hexdigest()})
        for color in (() if published else colors):
            run([ROOT / "scripts/idf.sh", "-B", BUILD, "-D", "SDKCONFIG=" + str(config),
                 "-D", "DCTRL_LINE_COLOR=" + color, "build"], color + " ESP32-C3 firmware build")
            flash = json.loads((BUILD / "flasher_args.json").read_text())
            description = json.loads((BUILD / "project_description.json").read_text())
            assert description["target"] == "esp32c3"
            assert description["project_version"] == VERSION + "-" + color
            name = "DCtrl_V" + VERSION + "_" + color + "-line.bin"
            target = staged / name
            files = sorted((int(offset, 0), BUILD / name) for offset, name in flash["flash_files"].items())
            args = [idf_python, "-m", "esptool", "--chip", "esp32c3", "merge_bin", "-o", target,
                    "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "4MB",
                    "--fill-flash-size", "4MB"]
            for offset, path in files:
                args += [hex(offset), path]
            run(args, color + " merged image at address 0x0")
            details = run([idf_python, "-m", "esptool", "--chip", "esp32c3", "image_info",
                           "--version", "2", BUILD / flash["app"]["file"]], color + " application image integrity")
            assert "checksum:" in details.lower() and "validation hash:" in details.lower()
            assert details.count("(valid)") >= 2 and "invalid" not in details.lower()
            image = target.read_bytes()
            assert len(image) == 4 * 1024 * 1024
            assert image[0] == 0xE9 and struct.unpack_from("<H", image, 12)[0] == 5
            for offset, path in files:
                assert image[offset:offset + path.stat().st_size] == path.read_bytes()
            app_offset = int(flash["app"]["offset"], 0)
            assert image[app_offset] == 0xE9
            assert struct.unpack_from("<H", image, app_offset + 12)[0] == 5
            # ESP-IDF application descriptor begins in the first data segment.
            assert struct.unpack_from("<I", image, app_offset + 32)[0] == 0xABCD5432
            assert image[app_offset + 48:app_offset + 80].split(b"\0")[0].decode() == VERSION + "-" + color
            assert image[0x9000:0xF000] == b"\xff" * 0x6000  # Old NVS settings are cleared.
            records.append({"file": name, "line_color": color, "version": VERSION + "-" + color,
                            "chip": "esp32c3", "flash_address": "0x0", "bytes": len(image),
                            "sha256": hashlib.sha256(image).hexdigest()})
        assert records[0]["sha256"] != records[1]["sha256"]
        if time.time() >= deadline:
            raise TimeoutError("300 second cumulative build/validation budget exhausted")
        RELEASE.mkdir(exist_ok=True)
        for record in records:
            if not published:
                os.replace(staged / record["file"], RELEASE / record["file"])
        keep = {record["file"] for record in records}
        removed = []
        for path in RELEASE.iterdir():
            if path.is_file() and path.name not in keep and path.suffix in {".bin", ".gz", ".zip", ".elf", ".hex", ".uf2"}:
                removed.append(str(path.relative_to(ROOT)))
                path.unlink()
        for directory in (ROOT / "build-esp32c3", ROOT / "backups", ROOT / "releases/v1-first-tuning-stable"):
            if directory.exists():
                removed.append(str(directory.relative_to(ROOT)))
                remove_tree(directory)
        report = {"version": VERSION, "status": "passed", "firmwares": records,
                  "checks": state["completed_steps"], "removed_old_artifacts": removed,
                  "hardware_tested": False, "elapsed_seconds": round(time.time() - started, 1)}
        (RELEASE / "manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        (RELEASE / "README.md").write_text(
            "# DCtrl ESP32-C3 循迹固件 V" + VERSION + "\n\n"
            "- 黑线版：`" + records[0]["file"] + "`，用于浅色背景上的黑线。\n"
            "- 白线版：`" + records[1]["file"] + "`，用于深色背景上的白线。\n\n"
            "两份均为 ESP32-C3 / 4MB 完整合并固件，烧录地址 **0x0**。包含引导程序、分区表与应用。"
            "烧录会清除旧参数，PID 和档位恢复默认；识别颜色固定在固件中，不受旧参数影响。\n\n"
            "DF-Link 使用 `02/62/LEN12` 持续速度，单位 m/s、m/s、rad/s，正值左转。"
            "上电待机，准备后启动循迹；找线先左后右，找线超时或收到停止命令时停车。\n\n"
            "已通过黑白传感器解析、纠偏、找线、停车及协议编码离线测试和两版 ESP-IDF 编译；"
            "尚未实车验证。首次使用从一档开始，检查左右纠偏、弯道、找线与停车。"
            "原有速度/PID数值保留，不代表已在新版底盘完成标定。\n\n"
            "校验和与检查记录见 `manifest.json`。重新构建：`python3 tools/build_line_firmwares.py`。\n")
        remove_tree(BUILD)
        # Installed tools stay available; downloaded installation archives are disposable.
        shutil.rmtree(ROOT / ".idf-tools/dist", ignore_errors=True)
        state.update(status="complete", elapsed_seconds=round(time.time() - started, 1))
        signal.setitimer(signal.ITIMER_REAL, 0)
        STATE.write_text(json.dumps(state, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    except Exception as exc:
        signal.setitimer(signal.ITIMER_REAL, 0)
        state.update(status="failed", error=repr(exc), elapsed_seconds=round(time.time() - started, 1))
        STATE.write_text(json.dumps(state, indent=2))
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
