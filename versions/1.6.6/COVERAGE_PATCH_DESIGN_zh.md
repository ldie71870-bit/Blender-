# Coverage Patch / 局部样本补齐设计与实现

## 1. 整体设计方案

### 架构位置

Coverage Patch 是原科学相机规划之后的增量模块，不替换全局规划器：

1. `scientific_planner.py` 继续负责场景 BVH、射线命中、表面单元、观察计数和重叠单元等基础设施。
2. `coverage_patch.py` 负责目标区域提取、已有覆盖分析、局部候选生成、评分、停止条件、预览和正式补齐集合。
3. `__init__.py` 负责 Blender 属性、面板、操作器、稳定图片命名、仅补拍渲染、COLMAP/transforms/report 接线。

### 新增模块、类和核心函数

`coverage_patch.py`：

- `PatchRegion`：一个目标区域的表面单元、中心、范围和主要表面类型。
- `PatchPlan`：缓存、已有覆盖、候选、入选相机、补齐前后指标和报告。
- `plan_patch()`：完整局部补齐入口。
- `_existing_analysis()`：把已有相机转换为科学规划候选并重算观察次数。
- `_candidate_groups()`：围绕局部区域或已有路径生成安全候选。
- `_candidate_score()`：综合欠覆盖、新覆盖、方向多样性、视差、重叠和连通性评分。
- `_select()`：逐台增量选择，满足停止条件后立即结束。
- `create_preview()` / `apply_preview()` / `clear_preview()`：两阶段预览工作流。
- `report_data()`：为数据集报告提供补齐历史和正式相机清单。

### 复用的原科学规划能力

- `PlanningCache` 和场景级组合 BVH。
- `_surface_key()` 与 `_estimate_surface_cells()`。
- `_cast_candidate()` 与真实 FOV 射线网格。
- `_candidate_clear()` 的 Mesh 内部性和最近表面安全检测。
- `_quality_cell_map()` / `_apply_candidate()` 的观察次数、最佳入射角和方向桶统计。
- `_overlap_cell_keys()` 的共同可见内容估计。
- `Candidate`、相机 yaw/pitch 朝向与科学模式单位换算。

### 数据流

```text
用户目标
  -> 对象名 / 定向包围盒 / 自动欠覆盖区域
  -> 局部 target_surface_cells
  -> 已有相机射线覆盖统计
  -> under_observed_cells
  -> 局部安全候选（4x4 预筛）
  -> 少量高收益候选（完整射线复检）
  -> 增量评分与停止
  -> PatchCameras_Preview
  -> 用户删除/重生成/确认
  -> PatchCameras_Final
  -> frame_patch_* 图片 + 合并 COLMAP/transforms/report
```

## 2. 详细实现步骤

### UI

新增独立 `Coverage Patch / 样本补齐` 面板，包含：

- 补齐模式：选中对象、包围盒区域、欠覆盖自动检测。
- 最低/推荐观察次数、目标覆盖率、最大补拍数量。
- 候选半径、安全距离、最低重叠率。
- 是否允许极向关键帧、是否限制在已有路径附近、是否优先连接现有相机图。
- 最少相机、最高覆盖、最强连通性三种评分策略。
- 生成预览、应用补齐、清除预览、仅渲染补齐相机。

### 数据模型

- 预览集合：`PatchCameras_Preview`。
- 正式集合：`PatchCameras_Final`。
- 正式相机标记：`gs_patch_camera=True`。
- 图片 stem：`gs_dataset_image_stem=frame_patch_XXXX`。
- 场景补齐历史：`gs_patch_history_json`。

### 区域提取

- 选中对象：只对选中 Mesh 建立表面单元；其他大场景 Mesh 不做表面细分。
- 包围盒：Mesh 使用真实局部 bound box；Empty 使用其本地立方体显示范围，支持旋转和缩放。
- 自动检测：重算全场景已有覆盖，按欠观察单元数量选取主要对象区域，最多处理 8 个区域。
- 场景 BVH 始终包含真实场景，保证遮挡与安全判断正确；只有表面统计被局部化。

### 已有覆盖分析

- 原始相机和此前已应用的补齐相机都参与统计。
- 每个目标表面记录观察次数、最佳入射角、距离范围和 15 度方向桶。
- 统计可见覆盖率、达到最低/推荐观察次数的比例和至少两个方向桶的比例。

### 候选生成

- 普通模式围绕目标区域建立确定性环形候选，不使用随机撒点。
- 地面区域把相机放在目标上方，天花板放在下方，墙面在中低高自适应高度分布。
- 每个安全光心产生中心朝向、左右 yaw 和可选上下 pitch 变化。
- 路径限制开启时，只使用目标附近的已有路径点。
- 所有光心通过组合 BVH 的 Mesh 内部性和最近表面安全距离检测。
- 与已有相机小于 5 cm 的光心直接去重；每个光心最终最多选一个方向。

### 覆盖分析、评分与筛选

1. 全部局部候选先用 4x4 射线预筛。
2. 每个光心最多保留两个高分朝向，总量限制为最大补拍数的 8 倍（至少 24）。
3. 少量候选按科学模式完整射线质量复检。
4. 评分包含：
   - 未观察表面收益；
   - 低于最低/推荐次数的补偿收益；
   - 新方向桶收益；
   - 与已有相机共同可见内容；
   - 有效基线视差；
   - 可连接的已有/新增相机数量。
5. 每选一台就更新目标区域观察统计，然后重新评分。
6. 目标可见覆盖率和最低观察比例都达到阈值时立即停止；否则最多达到用户设置的补拍上限。

### 预览与应用

- 预览相机不参与渲染和正式数据集。
- 用户可在 3D 视图中删除不满意的单台预览相机。
- 重新生成会只清理旧预览，不动原相机或正式补齐相机。
- 应用时只转换仍存在的预览相机，并根据实际保留相机重新计算补齐后指标。

### 导出与报告

- 原始相机继续使用 `frame_0001...`，不重排。
- 补齐相机使用 `cam_patch_<region>_XXXX` 和 `frame_patch_XXXX`。
- COLMAP `images.txt`、`transforms.json` 和 `dataset_report.json` 合并原始与补齐相机。
- `patch_manifest.json` 单独列出补齐相机名、图片路径、矩阵和焦距。
- `dataset_report.json.coverage_patch` 包含目标要求的全部补齐前后字段和多次补齐历史。
- 仅补拍渲染只渲染 `PatchCameras_Final`，但结束后会重新写入合并后的位姿和报告。
- 补拍不会读写原始全量渲染的断点续渲状态。

## 3. 实现假设

- 原始数据集相机共用插件当前焦距和分辨率约定；补齐相机沿用相同内参。
- 补齐在原始相机已经生成后执行；没有原相机时操作会明确拒绝。
- “自动欠覆盖”基于当前 Blender 场景和相机重新计算，不依赖可能过期的磁盘报告。
- 表面单元是有限分辨率的保守采样；极细几何仍受原科学规划采样上限影响。
- 当前版本不支持 Edit Mode 面级选择；对象和定向包围盒已覆盖主要局部补拍流程。

## 4. 测试清单

- [x] 按选中对象补齐。
- [x] 按 Mesh/Empty 定向包围盒补齐。
- [x] 欠覆盖自动检测。
- [x] 小范围对象只采样目标对象表面。
- [x] 地面候选层与安全检测。
- [x] 天花板候选层与安全检测。
- [x] 墙面/墙角垂直目标候选。
- [x] 现有相机达到阈值时新增数量为 0。
- [x] 大场景额外 Mesh 不进入对象模式 surface cells。
- [x] 手动删除单台预览后按剩余相机应用。
- [x] 原始相机位置、旋转、命名不变。
- [x] 全局唯一补齐相机名和稳定 `frame_patch_*` 图片名。
- [x] COLMAP、transforms、dataset_report、patch_manifest 合并兼容。
- [x] 仅渲染补齐相机不生成/覆盖原始图片。
- [x] 独立后台 Blender 仅补拍流程。
- [x] 原科学规划、旧六面基线、单位换算、RGB/深度/ID/对象输出回归。

## 5. v1.2.2 Pose Sequence 迁移

当科学实现方式为 `SCIENTIFIC_POSE_SEQUENCE` 时：

1. 覆盖分析通过 `PoseCameraAdapter` 读取基础清单中的完整姿态，继续使用原射线、覆盖和重叠计算。
2. 预览阶段仍允许用户在 `PatchCameras_Preview` 中删除候选；确认时才把剩余矩阵转成 PoseSample。
3. 新样本追加到清单末尾的独立 `COVERAGE_PATCH` Segment，不重排基础帧。
4. logical_frame_id 和 IMAGE_ID 从现有最大值递增，`frame_patch_XXXXXX` 与旧 `frame_XXXXXX` 分离。
5. 转换完成即删除 `PatchCameras_Final` 中的真实相机，最终场景不积累 Patch CAMERA 对象。
6. `仅渲染补齐` 从清单筛选 `is_coverage_patch`，完成后用全清单重写合并后的 COLMAP、transforms 和报告。
7. `patch_manifest.json` 继续列出 Patch 图片、矩阵和内参；`dataset_report.json.coverage_patch.runs` 保留历史。

旧多相机科学模式和旧数据集仍沿用 `PatchCameras_Final`，原命名、渲染和导出路径不变。

