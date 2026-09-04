import os
import subprocess
from pathlib import Path

from ..utils.json_io import read_json


_PROCESSES = {}


def start_training(settings, root):
    root = Path(root)
    command = [
        str(Path(settings.training_python).expanduser()),
        str(Path(settings.training_script).expanduser()),
        "--dataset", str(root),
        "--config", str(root / "config" / "training_config.yaml"),
        "--init-gaussians", str(root / "gaussians" / "init_gaussians.npz"),
        "--output", str(root / "training" / "output"),
    ]
    if str(settings.cuda_device).strip():
        command.extend(("--device", str(settings.cuda_device).strip()))
    log_path = root / "training" / "logs" / "trainer.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_path.open("a", encoding="utf-8")
    process = subprocess.Popen(
        command,
        cwd=str(Path(settings.trainer_path).expanduser()) if settings.trainer_path else str(root),
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    _PROCESSES[str(root)] = (process, log_handle)
    return process.pid, command, log_path


def poll_training(root):
    root = Path(root)
    item = _PROCESSES.get(str(root))
    progress = read_json(root / "training" / "output" / "progress.json", {}) or {}
    metrics = read_json(root / "training" / "output" / "metrics.json", {}) or {}
    if item is None:
        return {"running": False, "return_code": None, "progress": progress, "metrics": metrics}
    process, log_handle = item
    return_code = process.poll()
    if return_code is not None:
        log_handle.close()
        _PROCESSES.pop(str(root), None)
    return {"running": return_code is None, "return_code": return_code, "progress": progress, "metrics": metrics}


def cancel_training(root):
    root = Path(root)
    item = _PROCESSES.get(str(root))
    if item is None:
        return False
    process, log_handle = item
    if process.poll() is None:
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T"], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        else:
            process.terminate()
    log_handle.close()
    _PROCESSES.pop(str(root), None)
    return True

