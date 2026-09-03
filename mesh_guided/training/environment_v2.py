import json
from pathlib import Path


def _discover_trainer(settings):
    if settings.trainer_path:
        trainer = Path(settings.trainer_path).expanduser()
    else:
        trainer = None
        for parent in Path(__file__).resolve().parents:
            candidate = parent / "mesh_guided_gs_trainer"
            if (candidate / "train.py").is_file():
                trainer = candidate
                break
    python_path = Path(settings.training_python).expanduser() if settings.training_python else None
    script_path = Path(settings.training_script).expanduser() if settings.training_script else None
    if trainer is not None:
        script_path = script_path or trainer / "train.py"
        python_path = python_path or trainer.parent / "mesh_guided_gs_trainer.venv" / "Scripts" / "python.exe"
    return trainer, python_path, script_path


def check_training_environment(settings):
    trainer, python_path, script_path = _discover_trainer(settings)
    errors = []
    if trainer is None or not trainer.is_dir():
        errors.append("未找到 mesh_guided_gs_trainer 目录，请先部署外部训练器。")
    if python_path is None or not python_path.is_file():
        errors.append("独立训练环境不存在，请运行 mesh_guided_gs_trainer/setup_env.ps1。")
    if script_path is None or not script_path.is_file():
        errors.append("训练入口 train.py 不存在。")
    report_path = trainer / "environment_report.json" if trainer else None
    report = {}
    if report_path and report_path.is_file():
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            report = {}
    if not report.get("success"):
        errors.append("environment_report.json 未显示真实 CUDA/gsplat 前向与反向验证成功。")
    return {
        "ok": not errors,
        "errors": errors,
        "trainer": str(trainer) if trainer else "",
        "python": str(python_path) if python_path else "",
        "script": str(script_path) if script_path else "",
        "environment_report": str(report_path) if report_path else "",
        "report": report,
    }
