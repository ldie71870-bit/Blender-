# Mesh 引导高斯测试结果

测试日期：`2026-07-22`

环境：Blender `5.1.0`，Windows，插件 `1.2.8`。

## 纯 Python 单元测试

命令：

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

结果：`10/10` 通过。

覆盖：

- Blender/OpenCV 相机矩阵互逆与投影中心。
- 三角形内部采样与重心坐标和为 1。
- 高斯最薄轴比例、尺度为正和四元数归一化。
- 小物体采样配额与目标数量精确守恒。
- 混合模式纹理变化权重会提高高变化三角形的采样概率。
- 关闭 Blender 后从输出根目录发现并优先恢复最近未完成状态。
- 2x2 纹理双线性采样。
- NPZ/PLY 导出与字段验证。
- 外部训练进程命令、PID、进度轮询、日志和 `point_cloud.ply` 输出契约。
- Pipeline 逐相机写入训练/验证 Triangle ID、复用渲染器并在完成时释放。

## Blender 5.1 注册测试

工作源码和发布 ZIP 解压后的 `register()`、`unregister()` 均成功，版本读数为 `(1, 2, 8)`。

## 10 万点端到端烟测

脚本：`tests/blender_mesh_guided_smoke.py`

实测结果：

- evaluated Mesh：1 个 Mesh 实例和 1 个带 bevel 的 Curve 实例；Cube 三角形 12 个、世界面积 24。
- evaluated Mesh：包含 Mesh 与带 bevel Curve；内部 Curve-to-Mesh 代理不会重复采样。
- 打包图像：`image_packed: true`，不依赖外部路径读取。
- 初始高斯：100,000。
- 核心浮点字段无 NaN/Inf。
- 四元数归一化。
- `scale_z / scale_x` 中位数：`0.0500000007`。
- `init_gaussians.npz`、`init_gaussians.ply`、`scene.glb` 和 Normal EXR 均为非空文件。
- `dataset_validation.json`：`valid: true`。

## 双相机数据集烟测

脚本：`tests/blender_mesh_guided_dataset_smoke.py`

实测结果：

- 相机数量：2；训练/验证各 1。
- 带非零 `shift_x`/`shift_y` 的 OpenCV 投影最大误差：小于 `0.000004` 像素。
- RGB：2/2。
- Depth EXR：2/2。
- Normal EXR：2/2。
- Object ID NPY：2/2，`int32`，64x64，背景 `-1`、对象 ID `1`。
- Triangle ID NPY：2/2，`int64`，64x64，背景 `-1`，非背景值全部小于 Mesh 三角形数量。
- 独立双三角形方向烟测：唯一值 `{-1, 0, 1}`，上方 ID 0、下方 ID 1，临时 EXR 已删除。
- 最终 `dataset_validation.json`：`valid: true`。
- 验证器逐帧检查尺寸、Depth NaN/Inf 与正值、Normal 单位长度、Object/Triangle ID 类型、形状和范围。

## 外部训练器协议联调

Blender 双相机烟测同时生成 config/scene_config.json 和 config/training_config.yaml。使用真实 Blender 输出执行 Trainer 0.1.0 严格 dry-run，结果为 valid=true：

- 初始高斯：32。
- Mesh 三角形：32。
- 训练相机：1。
- 验证相机：1。
- RGB、Depth、Normal、Object ID、Triangle ID 和全部 metadata 均通过训练器预检。

训练器源码测试为 59 passed, 1 skipped；唯一跳过是需要已构建 gsplat CUDA 扩展的真实梯度测试。
## 1.2.7 行为回归

原 1.2.7 测试源码加载 1.2.8 工作树后通过：

- 科学 Pose Sequence：`POSE_SEQUENCE_OK 14 6 2`。
- 旧帧恢复迁移：`done=1921 next=1922 recovered=1829`。
- 相机绿色金字塔 Mesh 外观：通过。
- Coverage Patch：`COVERAGE_PATCH_OK 8 7 SELECTED_OBJECTS`。
- 多内参 COLMAP：2 个离散 PINHOLE profile 通过。
- RGB/Depth/Object ID 导出和损坏帧恢复：通过。

## 未覆盖风险

- 未在真实大型室内项目上执行 1,000,000 到 10,000,000 点压力测试。
- 外部训练器已实现，但本机缺少 CUDA Toolkit/NVCC 与 MSVC，真实 gsplat forward/backward 和训练质量尚未验证。
- 未验证 V-Ray 自动渲染；按设计会明确阻止，而不是切换渲染器。
- UDIM、程序材质烘焙和毛发通用转换尚未实现，未纳入通过项。

