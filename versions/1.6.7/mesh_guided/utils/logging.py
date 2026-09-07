import logging
from pathlib import Path


def task_logger(root):
    root = Path(root)
    logger = logging.getLogger(f"blender_gs_mesh_guided.{root}")
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    root.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(root / "pipeline.log", encoding="utf-8")
    handler.setFormatter(logging.Formatter(
        "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        "%Y-%m-%d %H:%M:%S",
    ))
    logger.addHandler(handler)
    logger.propagate = False
    return logger


def log_exception(logger, stage, subject, exc):
    logger.exception("stage=%s subject=%s error=%s", stage, subject or "-", exc)

