"""Process-isolated background render chunk supervisor.

This module intentionally has no Blender imports.  The supervisor is loaded in a
factory-startup Blender process and launches a fresh scene-loading Blender process
for every render chunk.
"""

from __future__ import annotations

import ctypes
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path


FATAL_LOG_PATTERNS = {
    "out_of_memory": re.compile(
        r"(?:out of memory|cuda_error_out_of_memory|failed to allocate[^\n]*(?:gpu|device|memory))",
        re.IGNORECASE,
    ),
    "cuda": re.compile(
        r"(?:cuda|cudart|cumem)[^\n]*(?:error|failed|failure|illegal|device lost)",
        re.IGNORECASE,
    ),
    "optix": re.compile(
        r"optix[^\n]*(?:error|failed|failure|exception|device lost)",
        re.IGNORECASE,
    ),
    "device": re.compile(
        r"(?:device error|device lost|illegal memory access|unspecified launch failure)",
        re.IGNORECASE,
    ),
}


def atomic_write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, path)


def write_progress(path, state, current=0, total=0, message="", error=""):
    payload = {
        "state": state,
        "current": int(current),
        "total": int(total),
        "progress": 0.0 if total <= 0 else min(1.0, max(0.0, current / total)),
        "message": message,
        "error": error,
        "elapsed": None,
        "eta": None,
    }
    try:
        atomic_write_json(path, payload)
    except Exception:
        pass


def detect_fatal_log(text):
    return [name for name, pattern in FATAL_LOG_PATTERNS.items() if pattern.search(text or "")]


def _windows_ram_bytes(pid):
    from ctypes import wintypes

    class ProcessMemoryCounters(ctypes.Structure):
        _fields_ = [
            ("cb", wintypes.DWORD),
            ("PageFaultCount", wintypes.DWORD),
            ("PeakWorkingSetSize", ctypes.c_size_t),
            ("WorkingSetSize", ctypes.c_size_t),
            ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
            ("QuotaPagedPoolUsage", ctypes.c_size_t),
            ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
            ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
            ("PagefileUsage", ctypes.c_size_t),
            ("PeakPagefileUsage", ctypes.c_size_t),
        ]

    handle = ctypes.windll.kernel32.OpenProcess(0x1000 | 0x0010, False, int(pid))
    if not handle:
        return None
    try:
        counters = ProcessMemoryCounters()
        counters.cb = ctypes.sizeof(counters)
        ok = ctypes.windll.psapi.GetProcessMemoryInfo(
            handle, ctypes.byref(counters), counters.cb
        )
        return int(counters.WorkingSetSize) if ok else None
    finally:
        ctypes.windll.kernel32.CloseHandle(handle)


def process_ram_bytes(pid):
    try:
        if os.name == "nt":
            return _windows_ram_bytes(pid)
        status = Path(f"/proc/{int(pid)}/status").read_text(encoding="utf-8", errors="ignore")
        match = re.search(r"^VmRSS:\s+(\d+)\s+kB", status, re.MULTILINE)
        return int(match.group(1)) * 1024 if match else None
    except Exception:
        return None


def _run_nvidia_smi(arguments):
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    try:
        completed = subprocess.run(
            ["nvidia-smi", *arguments],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=3,
            creationflags=creationflags,
            check=False,
        )
    except Exception:
        return ""
    return completed.stdout if completed.returncode == 0 else ""


def process_vram_mb(pid):
    output = _run_nvidia_smi(
        ["--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"]
    )
    values = []
    for line in output.splitlines():
        fields = [field.strip() for field in line.split(",")]
        if len(fields) >= 2 and fields[0].isdigit() and int(fields[0]) == int(pid):
            try:
                values.append(float(fields[1]))
            except ValueError:
                pass
    if values:
        return max(values), "process"

    # WDDM can hide per-process compute memory.  The global fallback is clearly
    # labelled in the report instead of silently dropping VRAM telemetry.
    output = _run_nvidia_smi(["--query-gpu=memory.used", "--format=csv,noheader,nounits"])
    values = []
    for line in output.splitlines():
        try:
            values.append(float(line.strip()))
        except ValueError:
            pass
    return (max(values), "global") if values else (None, "unavailable")


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return None


def _run_worker(config, requested_start, attempt):
    result_path = Path(config["result_path"])
    try:
        result_path.unlink()
    except FileNotFoundError:
        pass

    request_label = "auto" if requested_start <= 0 else f"{requested_start:06d}"
    log_path = Path(config["job_dir"]) / f"chunk_{request_label}_attempt_{attempt}.log"
    command = [
        config["blender_path"],
        "--background",
        "--factory-startup",
        config["blend_path"],
        "--python",
        config["worker_script"],
        "--",
        str(int(requested_start)),
        str(int(config["chunk_size"])),
        "1" if config.get("force_full_render") else "0",
    ]
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    started_at = time.time()
    peak_ram = 0
    peak_vram = 0.0
    vram_scope = "unavailable"
    with log_path.open("w", encoding="utf-8") as log_handle:
        process = subprocess.Popen(
            command,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            cwd=config["output_root"],
            creationflags=creationflags,
        )
        while process.poll() is None:
            ram = process_ram_bytes(process.pid)
            if ram is not None:
                peak_ram = max(peak_ram, ram)
            vram, scope = process_vram_mb(process.pid)
            if vram is not None:
                peak_vram = max(peak_vram, vram)
                vram_scope = scope
            time.sleep(1.0)
        return_code = process.returncode

    text = log_path.read_text(encoding="utf-8", errors="replace")
    print(f"\n[GS CHUNK LOG] {log_path.name}\n{text}", flush=True)
    fatal_signatures = detect_fatal_log(text)
    result = _read_json(result_path) or {}
    valid_result = bool(result.get("verified")) and result.get("state") in {"chunk_done", "done"}
    success = return_code == 0 and valid_result and not fatal_signatures
    return {
        "attempt": int(attempt),
        "pid": int(process.pid),
        "requested_start": int(requested_start),
        "start": int(result.get("start", 0) or 0),
        "end": int(result.get("end", 0) or 0),
        "total": int(result.get("total", 0) or 0),
        "rendered": int(result.get("rendered", 0) or 0),
        "complete": bool(result.get("complete", False)),
        "next_start": int(result.get("next_start", 0) or 0),
        "verified": bool(result.get("verified", False)),
        "return_code": int(return_code),
        "fatal_signatures": fatal_signatures,
        "peak_ram_bytes": int(peak_ram),
        "peak_vram_mb": round(float(peak_vram), 1),
        "vram_scope": vram_scope,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(started_at)),
        "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "duration_seconds": round(time.time() - started_at, 2),
        "log_path": str(log_path),
        "error": str(result.get("error", "")),
        "success": bool(success),
    }


def run_supervisor(config_path):
    config = _read_json(config_path)
    if not config:
        raise RuntimeError(f"Invalid background supervisor config: {config_path}")

    history_path = Path(config["history_path"])
    progress_path = Path(config["progress_path"])
    history = {
        "version": 1,
        "chunk_size": int(config["chunk_size"]),
        "retry_limit": 1,
        "force_full_render": bool(config.get("force_full_render")),
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "chunks": [],
    }
    requested_start = 1 if history["force_full_render"] else 0
    total = 0
    while True:
        display = "first missing frame" if requested_start <= 0 else f"frame {requested_start}"
        write_progress(progress_path, "chunk_start", 0, total, f"Starting fresh Blender process at {display}")
        successful = None
        for attempt in (1, 2):
            record = _run_worker(config, requested_start, attempt)
            history["chunks"].append(record)
            atomic_write_json(history_path, history)
            if record["success"]:
                successful = record
                break
            reason = ", ".join(record["fatal_signatures"]) or record["error"] or f"exit {record['return_code']}"
            if attempt == 1:
                write_progress(progress_path, "retrying", 0, record["total"], f"Chunk failed; retrying once: {reason}")
                print(f"[GS] chunk retry 1/1: {reason}", flush=True)
        if successful is None:
            message = "Background chunk failed after one retry"
            write_progress(progress_path, "error", 0, total, message, "See chunk history and log")
            history["state"] = "error"
            history["ended_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            atomic_write_json(history_path, history)
            raise RuntimeError(message)

        total = successful["total"]
        print(
            f"[GS] verified chunk {successful['start']}-{successful['end']}/{total}; "
            f"peak RAM={successful['peak_ram_bytes']} bytes, "
            f"peak VRAM={successful['peak_vram_mb']} MB ({successful['vram_scope']})",
            flush=True,
        )
        if successful["complete"]:
            history["state"] = "done"
            history["ended_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            history["peak_ram_bytes"] = max((item["peak_ram_bytes"] for item in history["chunks"]), default=0)
            history["peak_vram_mb"] = max((item["peak_vram_mb"] for item in history["chunks"]), default=0.0)
            atomic_write_json(history_path, history)
            return history
        requested_start = successful["next_start"]
        if requested_start <= 0 or requested_start > total:
            raise RuntimeError("Chunk worker returned an invalid continuation frame")


def main():
    if "--" not in sys.argv:
        raise RuntimeError("Missing supervisor config argument")
    arguments = sys.argv[sys.argv.index("--") + 1 :]
    if len(arguments) != 1:
        raise RuntimeError("Expected one supervisor config path")
    run_supervisor(arguments[0])


if __name__ == "__main__":
    main()
