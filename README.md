# Gaussian Splat COLMAP Dataset Generator

当前版本：`1.4.0`

实际升级基线：`blender_gs_colmap_exporter-1.2.7-chunked-background-render`

本版本新增 Mesh 引导高斯流水线。安装与使用见 `README_MESH_GUIDED_GS_CN.md`，数据契约见
`DATA_FORMAT_MESH_GUIDED_CN.md`，训练器接口见 `TRAINER_INTERFACE_CN.md`。

## Resume recovery (v1.2.7)

Legacy RGB/Depth/ID frames with valid file structure are recovered even when a v1.2.6
legacy_manifest.json exists but their per-frame commit is missing. Safe missing commits are
rebuilt before chunk selection, so a mixed old/new output directory resumes from the first truly
missing frame instead of rendering frame 1 again. Visibility-dependent per-object Depth/Mask
outputs still require their original commit marker.

Blender 5.1 扩展，用于在已知三维场景中生成训练相机、Cycles 图片、COLMAP `sparse/0`、`transforms.json` 和 `dataset_report.json`。

## Manual Walk Path（v1.4.0）

路径模式新增 `Manual Walk Path`。点击“开始录制”会启动 Blender Walk Navigation，并以 0.05 秒间隔持续记录当前视口的世界坐标 XYZ；确认行走导航后点击“结束录制”，插件会执行去重、0.10 m 弧长重采样和保留楼梯 Z 过渡的轻度平滑，生成独立且不会被后续操作修改的 `GS_WalkBasePath`。

BasePath 可生成 2/3/4 层路径，默认 3 层及 `+0.35 / 0.00 / -0.35 m`。修改 Layer Spacing 或任意 Layer Z Offset 会立即从 BasePath 重建，不必重新行走。每层按 0.075 m 对完整样条密集采样，使用 Evaluated Mesh 的共享 BVH、实体内部测试和可调安全球半径进行碰撞检查；连续碰撞样本会合并成一个区间，按安全半径向前后扩张后裁掉，保留的短于 0.50 m 的碎段自动删除。

蓝色 `GS_WalkBasePath` 只描述路线；合法的分层 Segment 保存在 `GS_MANUAL_WALK_VALID`，红色碰撞区间保存在不参与渲染的 `GS_MANUAL_WALK_COLLISION_DEBUG`。科学 Coverage 会直接读取合法 Segment 的准确 XYZ，不会再次增加高度层；弧长采样、Yaw/Pitch、Overlap、Polar/Bridge 和 Pose Selection 沿用原系统，最终仍只有一台 `GS_CAPTURE_CAMERA`。

## 3D 多楼层科学路径（v1.3.10）

科学模式的自动路径默认启用 `3D Multi-Level Planning`、`Curve Smoothing` 和 `Fragment Stitching`。规划器会在同一个 XY 保存多个合法楼层样本，建立 `FloorRegion + Connector` 拓扑，并把楼梯、平台或坡道作为连续 XYZ 路径接入上下楼层。挑空处没有连续地面支撑，因此不会生成悬空路径。

默认空间范围为“从种子可达空间”，起点可选 3D 光标或当前视角。规划器先校验相邻网格之间没有墙体阻挡，再只保留从起点沿地面、门洞、楼梯和坡道实际可达的三维连通分量；与起点隔离的模型空腔不会生成路径。“全部室内空间”仅在用户明确选择时启用。

同一高度相邻但由不同 Mesh 承载的表面会拆分成独立 FloorRegion，避免建筑外壳、夹层和地板边缘在几何缝隙处被误并入主房间。真正跨 Mesh 的楼梯仍通过经过宽度、单调性和主楼层验证的 Connector 连接。

走廊使用净空中心线并跟随 L/S/弧形空间；大房间在中心路径外仅补少量长偏移路径。平滑后的每段都会重新验证碰撞、水平净空、楼板和地面支撑，验证失败即保留安全的原始路径。PCA 平行线仍作为 3D 规划无法形成合法结果时的兼容回退。

科学模式的新默认实现后端是 `SCIENTIFIC_POSE_SEQUENCE`。所有训练视角写入 `camera_sequence.json`，场景只保留一台 `GS_CAPTURE_CAMERA`；`Create Preview` 把位置和四元数旋转烘焙到这台相机的时间轴，正式渲染仍逐条读取 PoseSample，不使用 Render Animation。

## 平衡吞吐量的分段独立进程渲染（v1.2.6）

后台渲染默认每 500 帧为一批，并在批次内默认开启 Cycles Persistent Data，复用 BVH、纹理和渲染数据。批次完成后工作进程仍会退出，释放 RAM、VRAM、Cycles 缓存和降噪器状态，再启动下一批。`每进程帧数` 和 Persistent Data 均可在渲染面板修改。

可靠性行为：

- Resume 会同时读取状态文件和磁盘实际输出，从首个缺失或损坏帧直接开始。
- 状态文件缺失但图片与逐帧提交记录完整时，已有帧仍会跳过。
- 每批结束验证 PNG/JPEG/EXR 结构及 RGB/Depth/ID/对象输出提交记录；未通过不会进入下一批。
- 工作进程异常自动重试一次；日志检测 out of memory、CUDA、OptiX 和 device error。
- `_gs_colmap_jobs/<输出路径哈希>/chunk_history.json` 记录每批范围、PID、尝试次数、峰值 RAM/VRAM 和独立日志路径。
- 取消后台任务会终止监督器及当前工作进程，已原子提交的帧保留供下次续渲。

## Blender 5.1 安装

在 Blender 中打开 `Edit > Preferences > Get Extensions`，使用右上角菜单选择 `Install from Disk...`，安装发布目录中的 zip。启用后，在 3D 视图按 `N`，打开 `GS` 页签。

## 路径曲线采集模式

路径曲线支持两种采集模式：

- `六面立方体兼容`：旧版同光心 `px/nx/py/ny/pz/nz` 六面输出。相机数量、顺序、命名、方向和 90 度透视参数保持不变。
- `科学三层覆盖（推荐）`：沿多条 Curve 建立分布式多高度机位，使用真实相机 FOV、自适应间距、射线覆盖评分、地面/天花板补拍和视图重叠图修复。

为避免旧 `.blend` 文件打开后改变行为，新属性默认值仍是 `六面立方体兼容`。新数据集建议手动切换到 `科学三层覆盖（推荐）`。

## 科学模式快速使用

1. 选择一条手动 Curve、一个包含多条 Curve 的 Collection，或点击 `生成井字排线`。
2. 将 `采集模式` 切换为 `科学三层覆盖（推荐）`。
3. 保持高度层数 3、目标重叠率 0.70、间距 0.25-0.65 m 和图片预算倍率 1.0。
4. 点击 `创建科学相机阵列`。规划期间显示阶段和进度，可按 Esc 或点击 `取消科学规划`。
5. 检查相机和可选调试显示，然后按原流程渲染或仅导出 COLMAP。

科学模式保持所有相机的焦距、传感器、分辨率和导出坐标约定一致，不会按位置改变内参。

## 科学单相机序列帧（v1.2.2，推荐）

科学模式提供三种实现后端：`LEGACY_CUBEMAP_OBJECTS`、`SCIENTIFIC_CAMERA_OBJECTS` 和默认的 `SCIENTIFIC_POSE_SEQUENCE`。旧 `.blend` 若已保存后端设置会继续遵循原值；新建科学设置默认只保留单台捕获相机。

选择 `单相机序列帧` 后，覆盖规划、近场保护、极向补拍、Bridge 和最终图片预算均不改变；插件把每个最终视角保存为 `PoseSample`，场景只保留一台 `GS_CAPTURE_CAMERA`。例如 720 个规划姿态仍输出 720 张训练图，只把 720 个 Blender 相机对象降为 1 个。

规划结果写入 `camera_sequence.json`，它是渲染、恢复、COLMAP、`transforms.json`、深度、ID、对象输出、报告和 Patch 的唯一姿态来源。每帧记录稳定的 `logical_frame_id`、`IMAGE_ID`、Segment、完整 `matrix_world`、共享内参哈希、输出路径和 `PENDING/RENDERING/COMPLETE/FAILED/SKIPPED` 状态。

正式渲染不会执行 Render Animation：场景始终停在 `Source Scene Frame`，只有主相机按清单逐姿态移动。每帧更新依赖图后校验活动相机、位置、旋转和内参；RGB/Depth/ID/对象输出先写暂存目录，全部成功并生成提交清单后才标为 `COMPLETE`。恢复时不重新规划、不重排，也不改变编号。

时间轴预览是独立功能：位置使用线性插值，旋转使用符号连续的四元数关键帧，Segment 起点带 Marker。预览关键帧从不参与正式渲染或导出。调试显示支持当前姿态、前后 N 帧、每 N 帧抽样和全量轻量 Empty。

Coverage Patch 在序列模式中把确认后的补拍姿态追加到独立 Patch Segment，使用 `frame_patch_000001...`，保留所有旧图片路径和 COLMAP ID，只渲染新增姿态；预览相机确认后会转成 PoseSample 并删除，不会在场景中累积正式 Patch 相机对象。

主要新增输出：

```text
camera_sequence.json
_gs_frame_commits/frame_000001.json
images/frame_000001.png
depth/frame_000001.png|exr
id/frame_000001.png
sparse/0/{cameras,images,points3D}.txt
transforms.json
dataset_report.json
```
## 近场采集保护（v1.2.1）

自由空间、小空间和混合光心模式默认启用近场采集保护：

- 防穿模安全距离继续只负责碰撞安全；推荐拍摄距离默认是 0.60-1.00 m。
- 玄关和门洞优先生成门外向内、门口桥接、门内回望、左右偏移和相邻空间回拍机位。
- 候选按远、中、近三档选择，远景和中景优先，近景默认不超过最终相机的 15%。
- 单一近距离对象或表面占据约 65% 以上视锥且共同环境不足时，候选会被淘汰。
- 只有目标确实欠覆盖，并与至少两台中远景相机形成足够重叠和真实基线时，近景才可作为少量补拍。
- 近场步长按最近目标距离的 20%-30% 收紧，同时保留最小光心去重距离和候选总数上限。
- `dataset_report.json` 新增 `near_field` 和完整近场规划统计；调试集合分别显示远、中、近与拒绝点。


## 绿色金字塔相机外观

- 新生成相机默认带绿色金字塔 Mesh 显示代理，正方形底面沿相机本地 `-Z` 指示镜头方向。
- 场景已有相机可一键全部转换，也可只转换选中相机；代理可随时移除。
- 真实对象始终保留为 `CAMERA`，渲染、深度/ID、COLMAP 和 `transforms.json` 行为不变。
- 代理共享同一个 Mesh 和材质，并从后台渲染、科学规划 BVH、稀疏点采样和对象分组中排除。

摄影棚、室外自动布光、自动曝光、灯具补光以及 `.ply/.splat` 高斯文件导入和 GPU 预览已从扩展中完整移除。

## Coverage Patch / 局部样本补齐

第一次科学规划和渲染后，可在保留原相机与原图片编号的前提下，对局部欠覆盖区域增量补拍：

- 支持选中 Mesh、Mesh/Empty 定向包围盒、全场景欠覆盖自动检测。
- 重算已有相机观察次数、方向多样性和重叠图，只在目标附近生成安全候选。
- 4 x 4 快速预筛后只对少量候选做完整射线复检；达到目标覆盖和最低观察比例后自动停止。
- `PatchCameras_Preview` 支持预览、手动删除和重生成；确认后进入 `PatchCameras_Final`。
- 补齐相机使用 `cam_patch_*` / `frame_patch_*`，不打乱 `frame_0001...` 原序列。
- “仅渲染补齐相机”只产出新增图片，随后合并更新 COLMAP、`transforms.json`、`dataset_report.json` 和 `patch_manifest.json`。

详细设计、数据流、假设和测试清单见 `COVERAGE_PATCH_DESIGN_zh.md`。

## 后台渲染准备优化（v1.1.51）

后台场景副本已经包含正式相机时，渲染进程会直接复用这些相机，不再因为 `自动创建阵列` 开启而重复执行一次完整科学规划。只有场景中完全没有相机时才自动规划。后台状态会明确显示配置渲染器、复用相机或创建阵列。

## 自动室内路径

自动路径复用现有地面、天花板、净空、连通域和防穿模检测：

- 普通房间按局部 PCA 主轴生成平行主线，每约两条主线增加一条垂直横线。
- 狭窄走廊和玄关保留基于双侧净空的中轴线，在门洞、转角和连通处保留节点。
- 不强制把独立 Curve 首尾连接成蛇形。
- 不同地面高度生成独立水平 Curve，不生成 Z 方向斜坡。
- 科学模式先生成一套 `Base` 空间网络，再按局部 floor/ceiling 投影为 2、3 或 4 个相机高度层，避免源路径重复分层。

## 默认科学参数

| 参数 | 默认值 |
|---|---:|
| 高度层数 | 3 |
| 目标重叠率 | 0.70 |
| 最低邻居重叠率 | 0.50 |
| 最小路径间距 | 0.25 m |
| 最大路径间距 | 0.65 m |
| 相机安全距离 | 0.25 m |
| 图片预算倍率 | 1.0 |
| 表面最低观察次数 | 3 |
| 表面推荐观察次数 | 5 |
| 最大朝向变化 | 15 度 |
| 最大高质量入射角 | 75 度 |
| 射线质量 | 普通 12 x 12 |
| 自动覆盖优化 | 开 |
| 自动补地面和天花板 | 开 |

所有米制参数通过 `scene.unit_settings.scale_length` 换算为场景单位。

规划采用两阶段射线检测：全部 Yaw/Pitch 候选先以 4 x 4 网格快速预筛，只对最终入选相机按“射线质量”的 8 x 8、12 x 12 或 16 x 16 网格复检。可见面分母来自全部合法候选；最终相机使用完整质量结果做总体 Yaw 修复、地面/天花板极向修复和重叠图修复。

候选光心先通过缓存场景 BVH 的精确最近表面查询和三方向奇偶检测，排除安全距离不足或位于 Mesh 内部的位置，再保留原有碰撞推出作为最终防线。最终覆盖明细只在全部确定性替换完成后构建一次，避免大图片预算下重复展开同一批表面单元。

Blender 5.1 参考测试中，普通 12 x 12 质量、720 张图的客厅用例总体/地面/天花板覆盖率为 98.097% / 95.383% / 95.312%，重复光心 0、单连通分量、弱邻居 0。

## 调试和报告

开启 `显示调试结果` 后，插件只管理 `GS_SCIENTIFIC_PATH_DEBUG` Collection。分层路径使用不同颜色，欠覆盖表面使用红色点，极向关键帧使用单独颜色，可显示视图重叠边。重新生成不会删除用户对象。

`dataset_report.json` 保留全部旧字段，并增加 `camera_planning`：模式、旧预算、最终数量、分层数量、自适应间距、重复光心、覆盖单元、地面/天花板/竖直面覆盖、逐对象覆盖、欠覆盖单元、不可达单元、图连通分量、极向/冗余/桥接相机数量等。

## 兼容性

- 旧六面模式由原 `expand_to_cubemap` 逻辑执行，并有 Blender 5.1 固定基线测试。
- 其他相机阵列模式不进入科学规划分支。
- Cycles、图片/深度/ID/单物品输出、断点续渲、后台渲染、ETA、COLMAP 和 `transforms.json` 逻辑未改动。
- 新属性均有默认值，旧 `.blend` 文件可继续打开。

详细功能和输出说明见 `MANUAL_zh.md`。
