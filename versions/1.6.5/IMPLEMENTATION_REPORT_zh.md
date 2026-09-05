# 历史实施报告：Blender Gaussian Splatting Plugin v1.2.0

> 此文件记录旧版科学相机规划阶段，不代表当前发布版本。当前版本为 `1.2.8`，实际升级基线为
> `blender_gs_colmap_exporter-1.2.7-chunked-background-render`。Mesh 引导高斯实现状态见
> `MESH_GUIDED_IMPLEMENTATION_STATUS_CN.md`。

## 1. 当前架构审查结果

原插件的科学规划调用链为：

`Curve -> path_components() -> _adaptive_stations() -> _make_candidate_groups() -> 4x4 预筛 -> _select_candidates() -> 完整射线复检 -> _apply_scientific_plan() -> resolve_camera_clipping()`

关键复用点：

- `PlanningCache`：场景边界、单位换算、FOV、BVH、表面单元、射线缓存。
- `Candidate` / `RayObservation`：候选姿态、命中表面、方向分箱、覆盖足迹。
- `_candidate_clear()`：实体内部检测与最近表面安全距离。
- `_cast_candidate()`、`_apply_candidate()`：统一射线与覆盖累计。
- `_score_candidate()`、`_overlap_graph()`：覆盖、视差、重叠与连通性。
- `coverage_patch.py`：已有局部反向候选、增量相机和增量导出流程。
- `write_colmap()`、`write_report()`：COLMAP、transforms.json 和 dataset_report.json。

原预算只来自 `legacy_station_count * 6`。原覆盖分母来自路径候选可见面。碰撞推出发生在统计完成之后，因此移动或删除相机时报告可能过期。

## 2. 根因分析

1. 光心和方向候选耦合在 `_make_candidate_groups()` 内，所有光心都依赖 Curve 弧长、切线和六槽位偏移。
2. 覆盖分母由同一批路径候选建立，路径外表面无法进入漏拍统计。
3. 小空间没有独立的区域分类、层数、步长和安全距离策略。
4. Coverage Patch 是独立手动流程，没有进入初次规划的统一候选预算。
5. `resolve_camera_clipping()` 改变正式相机后没有重投覆盖与重叠。
6. 重叠图对大相机集合做全量两两比较，3600 张预算下存在平方级风险。
7. COLMAP 导出固定使用 camera_id 1，无法表达实际离散多内参。

## 3. 最小侵入式改造方案

- 保留旧 Curve 候选函数作为兼容适配器，不改候选 ID、六槽位顺序、预算和评分。
- 新增统一 `OriginProvider` / `OriginSeed`，新来源只生成光心描述。
- 新增独立 2.5D 自由空间模块，输出合法网格、净空场、连通域、中轴、门洞和有限候选。
- 自由空间、小空间、混合与 Coverage Driven 候选统一进入原射线、评分、选择和重叠系统。
- 新模式通过 `scientific_origin_mode` 显式启用；默认仍为 `MANUAL_CURVE`。
- 最终相机碰撞推出后只重投移动相机，删除相机从选择集合移除，再重建全部统计。
- 大于 1000 台相机时启用空间桶限制重叠配对；720 张及以下保持原算法。
- 按实际离散内参写多个 COLMAP camera_id；单内参输出保持原格式。

## 4. 新增和修改的文件

插件文件：

- 修改 `__init__.py`：设置、UI、无 Curve 入口、最终重投、一致性检查、报告、多内参导出。
- 修改 `scientific_planner.py`：Provider、自由空间/混合/Coverage Driven、全局覆盖、空间索引。
- 新增 `free_space_planner.py`：2.5D 栅格、连通域、净空、空间分类与候选分散。
- 修改 `blender_manifest.toml`：版本升为 1.2.0。
- 修改 `deploy.py`：发布包名更新为 free-space-planning。
- 新增本实施报告。

新增测试：

- `test_origin_provider.py`
- `test_free_space_planner.py`
- `test_small_space_hybrid.py`
- `test_global_coverage_driven.py`
- `test_post_clipping_recast.py`
- `test_intrinsics_consistency.py`
- `test_overlap_performance.py`

## 5. 数据结构设计

`OriginSeed` 包含：

- position / preferred_direction
- local_clearance / floor_z / ceiling_z
- layer_name / region_id / provider_type
- is_critical / source_reference
- 兼容候选构造所需的 origin_id、站点、步长、层索引和基础朝向

`FreeSpaceMap` 包含：

- 自适应 resolution / resolution_m
- 合法 `FreeSpaceCell` 集合
- `SpaceRegion` 连通域和分类
- boundary_mask / doorway_nodes / medial_axis_nodes
- candidate_origins / invalid_points

区域分类输出：

- NORMAL_ROOM
- SMALL_ROOM
- NARROW_CORRIDOR
- DOORWAY_TRANSITION
- CLUTTERED_SPACE

## 6. 关键算法伪代码

```text
build_free_space_map:
  choose adaptive grid resolution under max_grid_cells
  for each XY cell:
    cast down/up to find floor and ceiling
    reject insufficient headroom
    cast 16 horizontal rays for clearance
    reject below narrow-space minimum clearance
  split valid cells by 4-neighbor connectivity and floor-height tolerance
  mark boundaries, medial maxima and doorway bottlenecks
  classify each region using area, width, clearance percentiles,
    aspect ratio, safe origin count, doorway count and occupancy ratio

generate_free_space_origins:
  seed maximum-clearance, doorway, medial-axis and boundary cells
  allocate per-region quota by area
  deterministic farthest-point sampling with spacing and hard cap
  generate 2 layers for small/corridor regions, otherwise configured layers
  run the shared candidate-clearance check
  pass OriginSeed objects to the shared direction candidate builder

global_coverage_and_patch:
  compute provider-visible and path-visible sets
  use all legal free-space sources for globally reachable cells
  run initial selection
  find globally reachable cells below minimum observations
  project surface-driven proposals to nearest legal free-space cells
  reserve bounded budget slots for Coverage Driven candidates
  rerun the shared selector without exceeding image budget

post_clipping_recast:
  update Blender view layer
  remove dropped candidates
  copy actual camera position/orientation into moved candidates
  recast only moved candidates at final ray quality
  rebuild coverage and overlap graph
  regenerate report statistics from actual render poses
```

## 7. 分阶段实现结果

1. 架构审查与原版基线：完成。
2. Provider 抽象和 Curve 兼容适配：完成。
3. 2.5D 自由空间、门洞/中轴和调试数据：完成。
4. 小空间分类、两层、走廊交替方向、动态净空和混合来源：完成。
5. 全局可达分母、Coverage Driven 反向候选和预算内自动补齐：完成。
6. 最终姿态重投、训练一致性和离散多内参导出：完成。
7. 空间索引、完整回归、性能测试和发布：完成。

## 8. 直接代码修改摘要

- 默认 `scientific_origin_mode = MANUAL_CURVE`，旧项目结果不变。
- 可选 `AUTO_GRID_PATH`、`FREE_SPACE`、`SMALL_SPACE`、`HYBRID`。
- 新增旧路径、面积、自适应表面和用户固定四种预算模式。
- 小空间目标重叠按 0.825 计算，局部步长限制到 0.15-0.25 m。
- Coverage Driven 默认在新自由空间模式启用，候选数和目标表面数有硬上限。
- dataset_report.json 保留旧字段并新增：
  - origin_generation
  - space_classification
  - global_coverage
  - lens_profiles
  - training_consistency
- 调试 Collection 增加合法/非法区域、中轴、门洞和 Provider 光心。
- 一键推荐配置关闭 DOF 和 Motion Blur，并统一当前训练相机焦距。
- 多内参时 cameras.txt 写多个 camera_id，transforms.json 逐帧写准确内参。

## 9. 每阶段测试结果

- Provider 兼容：132 个 Curve 光心组、5416 个方向候选，签名逐项一致。
- 无 Curve 自由空间：18 台相机，84 个合法单元，重复运行结果一致。
- 小空间/混合：窄走廊识别为 NARROW_CORRIDOR，使用两层和 0.15-0.25 m 步长。
- 全局覆盖：8780 个全局可达表面单元，16 张预算内选入 4 个 Coverage Driven 光心。
- 最终重投：移动 1 台相机，覆盖率从 0.463577 重算为 0.463380。
- 离散内参：28/35 mm 正确导出 COLMAP camera_id 1/2。
- 旧科学规划：24 台、37355 个表面单元、2/4 层兼容测试通过。
- 旧六面：18 张固定基线完全一致。
- 10 个验收场景：全部保持 720 张并通过覆盖、重叠、固定内参和确定性断言。
- Coverage Patch、前台渲染、后台渲染、断点续渲、深度/ID/对象输出全部通过。

## 9.1 v1.2.1 近场采集保护补丁

新增行为：

- 区分 0.25 m 防碰撞安全距离与默认 0.60-1.00 m 推荐拍摄距离。
- 自由空间区域增加 NEAR_FIELD 标记，候选和最终相机增加 FAR/MID/NEAR 距离档位。
- 4 x 4 预筛统计 `dominant_near_surface_ratio`，按同一对象和同一法向表面双重聚合。
- 主导近表面达到 0.65 且共同环境不足时拒绝；欠覆盖收益、两台中远景强重叠和 0.20 m 真实基线共同构成例外条件。
- 近景默认最多占 15%，远景/中景先选；最终 8 x 8 射线与碰撞重投后再次执行近场淘汰。
- 门洞生成远、中、近机位，覆盖门外向内、门口桥接、门内回望、左右偏移、相邻空间回拍与门框细节。
- 近场步长使用 `nearest_target_distance * 0.20-0.30`，默认去重下限 0.12 m；小于 0.35 m 的常规光心优先拒绝。
- 调试显示 FAR/MID/NEAR 相机、距离过近拒绝点和主导表面拒绝点。
- 报告增加：`near_field_camera_count`、`rejected_too_close_candidate_count`、`dominant_surface_rejected_count`、`near_mid_far_camera_counts`、`near_field_average_overlap`、`near_field_under_observed_cells`。

新增/更新测试：

- `test_near_field_protection.py`：门洞七角色、选择顺序、主导表面拒绝、欠覆盖例外、真实步长。
- `test_free_space_planner.py`：近场报告字段、档位计数和近景比例。
- `test_small_space_hybrid.py`：近场最小去重步长边界。
- 后台回归改为读取最终 v1.2.1 ZIP，不再误测 v1.1.51 旧包。

完整回归保持 10 个 Curve 场景各 720 张；720/3600 重叠图为 0.0772 s / 0.3916 s。Coverage Patch、前后台渲染、深度/ID/Mask、旧六面、固定内参和多内参导出均通过。
## 10. 回归测试清单


已验证：

- 无 Curve 普通室内直接规划。
- Curve 默认兼容和手动多 Curve。
- 大空间候选硬上限。
- 小空间两层和窄走廊中轴/交替方向。
- L 形和多房间旧场景回归。
- Coverage Driven 预算内补齐。
- 路径候选与全局可达覆盖分母分离。
- 地面、天花板、垂直面覆盖。
- 固定单内参默认行为。
- 离散多内参 COLMAP camera_id。
- 碰撞移动后覆盖重算。
- 手动 Coverage Patch 单独渲染和增量导出。
- 取消规划不覆盖正式相机。
- 固定输入确定性。
- Cycles、深度、ID、对象 Mask、后台渲染和断点续渲。

## 11. 性能测试结果

Blender 5.1 / Windows：

- 720 台重叠图：0.0527 s，16408 条边。
- 3600 台重叠图：0.2891 s，93304 条边。
- 大于 1000 台时启用三维空间桶；小规模继续使用旧全量比较。
- 自由空间网格默认最多 200000 单元。
- 默认最多 600 个三维候选光心。
- Coverage Driven 默认最多 12 个光心、48 个欠覆盖目标、每目标 3 个候选。
- 每区域按面积分配配额，候选池超过 20000 单元时确定性降采样。

## 12. 尚未解决的风险和边界

- 当前自由空间是单探测高度的 2.5D 模型；同一 XY 上完全重叠的多层楼板需要按楼层分批规划，尚未做真正多层体素导航。
- 门洞检测是几何瓶颈启发式，不理解门的语义、开合动画或不可通行玻璃。
- 全局理论可达面是有限合法光心和粗射线的估计，不是连续自由空间的数学完备证明。
- 表面估计默认最多 200000 单元，超大型高细节场景会采用采样近似。
- Coverage Driven 受固定预算、遮挡和安全距离约束，无法保证所有理论可达表面都达到最低观察次数。
- 最终碰撞重投已准确修正报告；当前不会在碰撞后自动突破原预算新增相机，`post_clipping_repair_camera_count` 保持 0。
- REGION_PROFILES 支持正确导出已有离散焦距，但规划器不会自动为不同区域分配焦距；默认仍是 SINGLE_INTRINSICS。

## 13. v1.2.2 科学单相机位姿序列

新增 `pose_sequence.py`，以 `PoseSample`、`PoseSequence` 和 `PoseCameraAdapter` 解耦规划结果与 Blender 对象。多相机科学后端现在同样先生成 PoseSample，再从样本实现相机对象；序列后端复用完全相同的样本，只保留 `GS_CAPTURE_CAMERA`。

权威数据流：

```text
Curve / Free Space -> CandidatePose -> Coverage Selection
-> deterministic Segment ordering -> PoseSample[]
-> SCIENTIFIC_CAMERA_OBJECTS 或 SCIENTIFIC_POSE_SEQUENCE
-> render / resume / COLMAP / transforms / report / Patch
```

渲染器固定源场景帧，逐样本验证 evaluated matrix 和共享内参，关闭运动模糊与 DOF，在帧级暂存目录完成 RGB/Depth/ID/对象输出后生成提交清单。清单状态取代旧科学模式的独立帧计数器，并保存稳定 logical_frame_id/IMAGE_ID。

Coverage Patch 确认后追加 PoseSample 和独立 Segment，删除正式 Patch 相机对象；旧 ID 和文件名不变，只渲染新增 Patch。后台场景副本识别有效清单和主相机，直接恢复，不重新规划。

新增/扩展测试覆盖：

- 同一 PoseSample 的渲染矩阵、COLMAP 正逆转换和 transforms 一致性。
- 一台真实相机对应 N 个训练姿态。
- 多 Curve Segment 不产生插值训练帧。
- 动画场景固定源帧。
- manifest 状态恢复、提交文件检查和稳定编号。
- Patch 追加与只渲染 Patch。
- 焦距/传感器/分辨率变化立即失败。
- 真实 EEVEE RGB/Depth/ID 单相机双姿态渲染。
- 独立后台 Blender 复用清单且不污染前台场景。
- 全部旧科学规划、旧六面、Coverage Patch、前后台渲染和输出回归。

已知限制：当前没有多进程姿态分片；全量轻量调试模式仍会为每个样本创建 Empty，超大序列建议使用按步长抽样；冗余帧安全裁剪未默认启用，本版本始终保留完整规划视图数。

