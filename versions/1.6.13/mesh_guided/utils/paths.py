import re
from datetime import datetime
from pathlib import Path


TASK_DIRS = (
    "config",
    "mesh",
    "cameras",
    "images/train",
    "images/validation",
    "depth/train",
    "depth/validation",
    "normal/train",
    "normal/validation",
    "object_id/train",
    "object_id/validation",
    "triangle_id/train",
    "triangle_id/validation",
    "gaussians",
    "training/checkpoints",
    "training/logs",
    "training/output",
    "temp/mesh_parts",
    "temp/sample_parts",
    "temp/packed_textures",
)


def safe_component(value, fallback="mesh_guided_task"):
    value = re.sub(r"[^0-9A-Za-z._\-\u4e00-\u9fff]+", "_", str(value or "")).strip("._")
    return value or fallback


def resolve_task_root(settings):
    root = Path(settings.output_root).expanduser()
    name = safe_component(settings.task_name)
    if getattr(settings, "create_date_subdir", False):
        name = f"{datetime.now():%Y%m%d}_{name}"
    return root / name


def create_task_tree(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    for relative in TASK_DIRS:
        (root / relative).mkdir(parents=True, exist_ok=True)
    return root


def numbered_name(index, suffix):
    return f"{int(index):06d}{suffix}"

