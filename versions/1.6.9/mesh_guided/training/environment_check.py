from pathlib import Path


def check_training_environment(settings):
    python_path = Path(settings.training_python).expanduser() if settings.training_python else None
    script_path = Path(settings.training_script).expanduser() if settings.training_script else None
    errors = []
    if python_path is None or not python_path.is_file():
        errors.append("训练 Python 路径无效，请在“训练器设置”中选择独立环境的 python.exe。")
    if script_path is None or not script_path.is_file():
        errors.append("训练脚本路径无效，请选择支持 --dataset/--config/--init-gaussians/--output 的 train.py。")
    return {
        "ok": not errors,
        "errors": errors,
        "python": str(python_path) if python_path else "",
        "script": str(script_path) if script_path else "",
    }

