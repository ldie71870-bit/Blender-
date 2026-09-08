# 外部训练器接口

## 进程边界

训练器由独立 Python 环境启动。插件主进程不导入 PyTorch、CUDA 或 gsplat，避免 Blender Python 依赖、显存和模块版本冲突。

## 启动命令

    <training_python> <training_script>
      --dataset <任务目录>
      --config <任务目录>/config/training_config.yaml
      --init-gaussians <任务目录>/gaussians/init_gaussians.npz
      --output <任务目录>/training/output
      --device <CUDA设备>
      --resume auto
      --status-dir <任务目录>/training

命令使用参数数组直接传给 subprocess.Popen，不通过 shell。进程使用独立 cwd、UTF-8 和 Windows 无窗口标志。子进程 stdout/stderr 合并写入 training/logs/subprocess.log；训练器内部结构化日志写入 training/logs/trainer.log。

## 输入

- 数据集与坐标约定：见 DATA_FORMAT_MESH_GUIDED_CN.md。
- 训练配置：config/training_config.yaml。
- 初始高斯：gaussians/init_gaussians.npz。

默认配置包含四阶段：外观初始化、表面覆盖调整、误差驱动增密和外观精修。训练器负责解释配置并实现优化器、增密/裁剪和 checkpoint；插件不在 Blender 内实现 CUDA 训练循环。

## 状态与控制文件

训练器在任务目录 training/ 原子更新：

- status.json
- progress.json
- metrics.json
- error_report.json（失败时）
- resolved_config.yaml

插件向 training/control.json 原子写入 pause、resume、stop 或 save_checkpoint 请求。未知字段会被忽略。

progress.json 的主要字段包括 stage、current_step、total_steps、gaussian_count、elapsed_seconds 和 estimated_remaining_seconds。metrics.json 可包含总损失、各损失项、验证指标和显存使用量。

## Checkpoint 恢复

- resume=auto：恢复最新有效 checkpoint；没有候选时允许从头开始，损坏最新文件会尝试更早文件。
- resume=latest：必须找到并恢复有效 checkpoint，否则训练器明确失败。
- 显式 checkpoint：文件缺失、损坏或与配置/归一化不兼容时明确失败。
- checkpoint 位于 training/checkpoints/，保存模型、优化器、策略、RNG、Stage/Step、scheduler、配置兼容签名和场景归一化信息。

## 输出与退出

成功训练必须以退出码 0 结束，并在 training/output/point_cloud.ply 生成标准 3DGS PLY，同时输出 training/output/mesh_anchored_gaussians.npz。非零退出码被视为失败；数据集、日志和已有 checkpoint 保留。

取消优先通过 control.json 请求训练器保存 checkpoint 并正常停止；强制取消时，Windows 先向训练进程树发送正常终止，超时后才强制结束。重新打开 Blender 后，外部进程不会由 PID 自动接管，应从 checkpoint 恢复。

## Trainer 0.1.0 门禁

插件自动搜索 mesh_guided_gs_trainer/train.py 与同级 mesh_guided_gs_trainer.venv/Scripts/python.exe。只有 environment_report.json 的 success 为 true 时才启动真实训练。

当前开发机 CUDA 张量反向传播通过，但缺少 CUDA Toolkit/NVCC 与 MSVC，gsplat forward/backward 门禁保持失败；这不会被状态文件伪装为成功。Blender 实际导出的双相机数据集已通过训练器严格 dry-run。