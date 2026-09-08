# Mesh 引导高斯使用说明

## 版本依据

- Blender：`5.1`
- 当前插件：`1.2.8`
- 实际基线：`blender_gs_colmap_exporter-1.2.7-chunked-background-render`

本模块是原 GS/COLMAP 插件中的新增折叠区，不替换原相机排线、RGB、Depth、Object ID、COLMAP、断点续渲或科学相机规划。

## 安装

1. 在 Blender 打开“编辑 > 偏好设置 > 插件”。
2. 选择“从磁盘安装”。
3. 选择 `blender_gs_colmap_exporter-1.2.8-mesh-guided-gs.zip`。
4. 启用 `Gaussian Splat COLMAP Dataset Generator`。

安装包内部只有一个顶层目录 `blender_gs_colmap_exporter/`，不需要手动复制 Python 文件。

## 工作模式

### 仅生成初始高斯

读取最终计算后的 Mesh 与材质，输出：

- `mesh/scene.glb`
- `mesh/training_mesh.npz`
- `mesh/mesh_metadata.json`
- `mesh/material_metadata.json`
- `gaussians/init_gaussians.npz`
- `gaussians/init_gaussians.ply`
- `dataset_validation.json`

此模式不要求场景中存在相机，也不启动渲染或训练。

### 生成训练数据集

在初始高斯基础上，直接从 Blender 相机导出 OpenCV 坐标相机参数，并生成 RGB、Depth、Normal、Object ID 和 Triangle ID。相机不足时调用原插件相机系统；已有相机少于“最少相机数量”时使用现有全部相机，不因此报错。

### 一键生成并训练

完成数据集验证后，通过独立 Python 进程启动外部训练脚本。PyTorch、CUDA 和 gsplat 不会被导入 Blender 主进程，也不会打包进插件 ZIP。

## 操作

1. 先保存 `.blend` 文件。
2. 在 3D 视图或渲染属性的 `GS Dataset` 面板展开“Mesh 引导高斯”。
3. 选择工作模式、输出根目录和任务名称。
4. 设置目标初始高斯数量；默认 `1,000,000`，范围 `100,000` 到 `10,000,000`。
5. 数据集模式下确认当前渲染器为 Cycles 或 EEVEE。
6. 点击“一键生成 Mesh 引导高斯”。

任务目录为 `输出根目录/任务名称`。开启“日期前缀”后，任务名称前会增加当天日期。

## Mesh 与采样

- 插件读取 dependency graph 的最终结果，包含已计算修改器、对象世界变换和实例矩阵。
- 非均匀缩放和负缩放通过逆转置法线矩阵处理。
- 每个采样点保存位置、插值法线、UV、切线、副切线、重心坐标、对象/材质/三角形 ID 和 PBR 基础属性。
- 打包图像直接从 `bpy.data.images[*].pixels` 读取，不依赖外部缓存路径。
- 最薄高斯轴沿表面法线，默认厚度为切向尺度的 `0.05`。

复杂程序节点目前只记录为 `procedural_base_color: true` 并回退到材质基础色；不会伪造烘焙结果。

## 相机与真值

- 坐标约定：OpenCV `+X` 向右、`+Y` 向下、`+Z` 向前。
- Depth：相机空间正向 Z，OpenEXR float32。
- Normal：相机空间法线，EXR 中编码为 `rgb = normal * 0.5 + 0.5`。
- Object ID：`int32` NPY，背景为 `-1`。
- Triangle ID：`int64` NPY，直接对应 `training_mesh.npz/triangles` 行号，背景为 `-1`。
- 自动相机筛选当前执行确定性的位姿去重。`visibility_report.json` 中没有实测值时，`measured_surface_coverage` 为 `null`。

V-Ray 或其他无法由 Blender Python 稳定自动渲染的引擎会在预检时中止，并提示改用 Cycles/EEVEE 或选择“仅生成初始高斯”。插件不会静默切换渲染器。

## 中断与恢复

`pipeline_state.json` 在每个阶段和对象完成后原子更新。取消任务会保留成功输出；使用“继续上次任务”从已记录阶段继续。旧版后台渲染仍按逐帧提交和分块检查点恢复。

“删除任务状态”只删除 `pipeline_state.json`，不会删除数据集。开启“覆盖已有任务”时，插件只清理当前任务目录下由本流水线管理的固定子目录。

## 训练与预览

训练模式会自动定位同工作区 `mesh_guided_gs_trainer` 和独立虚拟环境；只有 `environment_report.json` 成功后才启动。进程协议见 `TRAINER_INTERFACE_CN.md`。

任务完成后可：

- “加载初始高斯”：把 `init_gaussians.ply` 载入 Blender。
- “打开目录”：打开最近任务目录。
- 从 `training/output/point_cloud.ply` 使用兼容查看器载入最终训练结果。

