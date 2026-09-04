import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .utils.json_io import atomic_write_json, read_json


STATE_FILE = "pipeline_state.json"


@dataclass
class PipelineState:
    plugin_version: str = "1.3.0"
    stage: str = "PREFLIGHT"
    task_root: str = ""
    current_index: int = 0
    total_items: int = 0
    completed_items: list = field(default_factory=list)
    completed_images: list = field(default_factory=list)
    initial_gaussians_complete: bool = False
    data_validation_complete: bool = False
    training_pid: int = 0
    last_checkpoint: str = ""
    status: str = "READY"
    message: str = ""
    error: str = ""
    updated_at: float = 0.0

    def save(self):
        self.updated_at = time.time()
        atomic_write_json(Path(self.task_root) / STATE_FILE, asdict(self))

    @classmethod
    def load(cls, root):
        data = read_json(Path(root) / STATE_FILE)
        if not isinstance(data, dict):
            return None
        values = {key: data[key] for key in cls.__dataclass_fields__ if key in data}
        return cls(**values)


def find_latest_state(output_root):
    output_root = Path(output_root).expanduser()
    if not output_root.exists():
        return None
    states = list(output_root.glob(f"*/{STATE_FILE}"))
    if not states:
        return None
    unfinished = []
    for path in states:
        state = PipelineState.load(path.parent)
        if state is not None and state.status != "DONE":
            unfinished.append(path)
    latest = max(unfinished or states, key=lambda item: item.stat().st_mtime)
    return PipelineState.load(latest.parent)


def remove_state(root):
    path = Path(root) / STATE_FILE
    if path.exists():
        os.unlink(path)

