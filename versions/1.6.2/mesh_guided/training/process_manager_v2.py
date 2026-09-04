import os
import subprocess
import time
from pathlib import Path

from ..utils.json_io import atomic_write_json, read_json
from .environment_v2 import check_training_environment


_PROCESSES = {}


def _device(value):
    value = str(value or "0").strip()
    return value if value.startswith("cuda:") else f"cuda:{value}"


def start_training(settings, root, environment=None):
    root = Path(root)
    environment = environment or check_training_environment(settings)
    if not environment["ok"]:
        raise RuntimeError(" ".join(environment["errors"]))
    command = [
        environment["python"], environment["script"],
        "--dataset", str(root),
        "--config", str(root / "config" / "training_config.yaml"),
        "--init-gaussians", str(root / "gaussians" / "init_gaussians.npz"),
        "--output", str(root / "training" / "output"),
        "--device", _device(settings.cuda_device),
        "--resume", "auto",
        "--status-dir", str(root / "training"),
    ]
    log_path = root / "training" / "logs" / "subprocess.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_path.open("a", encoding="utf-8")
    process_environment = os.environ.copy()
    process_environment.update({"PYTHONUTF8": "1", "PYTHONIOENCODING": "utf-8"})
    process = subprocess.Popen(
        command,
        cwd=environment["trainer"],
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        env=process_environment,
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    _PROCESSES[str(root)] = (process, log_handle)
    return process.pid, command, log_path


def poll_training(root):
    root = Path(root)
    item = _PROCESSES.get(str(root))
    status = read_json(root / "training" / "status.json", {}) or {}
    progress = read_json(root / "training" / "progress.json", {}) or {}
    metrics = read_json(root / "training" / "metrics.json", {}) or {}
    if item is None:
        running = status.get("state") in {"starting", "validating", "running", "paused"}
        return {"running": running, "return_code": None, "status": status, "progress": progress, "metrics": metrics}
    process, log_handle = item
    return_code = process.poll()
    if return_code is not None:
        log_handle.close()
        _PROCESSES.pop(str(root), None)
    return {"running": return_code is None, "return_code": return_code, "status": status, "progress": progress, "metrics": metrics}


def request_training_control(root, command):
    if command not in {"pause", "resume", "stop", "save_checkpoint"}:
        raise ValueError(f"Unsupported training command: {command}")
    atomic_write_json(Path(root) / "training" / "control.json", {
        "schema_version": "1.0", "command": command, "request_id": time.time_ns(),
    })
    return True


def pause_training(root):
    return request_training_control(root, "pause")


def resume_training(root):
    return request_training_control(root, "resume")


def cancel_training(root, force=False):
    root = Path(root)
    item = _PROCESSES.get(str(root))
    if item is None:
        return False
    process, log_handle = item
    if process.poll() is None and not force:
        request_training_control(root, "stop")
        return True
    if process.poll() is None:
        if os.name == "nt":
            subprocess.run(
                ["taskkill", "/PID", str(process.pid), "/T", "/F"], check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        else:
            process.terminate()
    log_handle.close()
    _PROCESSES.pop(str(root), None)
    return True
