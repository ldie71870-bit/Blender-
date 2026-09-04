# Changelog

## 1.6.3 - 2026-09-04

- 修复连续低位空隙已有安全候选，但在最终表面收益排序中落选的问题。
- 每层约一半的细部预算优先按实际空隙覆盖选择连续路线；不同缺口之间适度均衡。
- 提取缺口路线后继续搜索剩余支路，避免一条最长线代表整片复杂缺口。
- 普通细部短线也计入空间覆盖去重，空间收益与表面双视角收益继续独立统计。
- 保持主线算法、层高、障碍净距和默认 48 条总预算；增加分支和收益竞争回归测试。

## 1.6.2 - 2026-09-04

- 跨相机高度检查安全可达性，保留家具分隔的低位空隙。
- 各层独立评估表面可见性与视差，均衡分配细部补线预算。
- 新增 0.15 m 细部通行网格、0.10 m 细部净距与沿缺口的连续补线。
- 楼梯顶层随顶棚净空调整，并保留最低顶层高度要求。
- 开放主线从端点顺接，减少插入支路后的原路折返。
- 同高度交叉采用经过安全检查的访问顺序调整；空间空隙覆盖与表面观察收益分别记录。
- 按层和用途分组，提供彩色细线与分层查看，隐藏旧自动预览并保留原对象。
- 净距检测复用场景几何树；表面归属和法线继续使用 Blender 原生射线结果。

## 1.6.1 - 2026-09-04

- 主线连接加入闭环双入口/出口拼接，减少同点接入和立即折返。
- 主线不再强接局部缺口的补线；连接后再做受障碍约束的平滑，将细部覆盖交给独立短线。
- 新增可达候选视点的表面采样与遮挡、基线、观察夹角评估，对小物件增加定向射线采样。
- 按新增有效观察收益选择独立细部短线；每条线记录表面单元收益和目标物体。
- 为细部短线预留至少 5 个相机采样站点，并在固定预算不足时明确提示。
- 面板新增主线/细部短线选择按钮与采样表面观察报告。
- 指标明确限定为候选可观察的采样表面；最终相机 FOV、筛选及渲染质量仍需后续验证。

## 1.6.0 - 2026-09-04

- 自动排线按钮新增并默认调用轮廓排线流程，接入原有 3D 连通图模块；旧井字模式保留。
- 每个 XY 保存多组地面支持，加入大面积地面与高差连接筛选、分层障碍净空检查。
- 距离轮廓、窄区补线与图上最短连接生成少量长路径；简化和平滑后复核净空与地面支持。
- 新结果事务性创建到独立预览集合，保留现有曲线；支持单层检查。
- 界面生成使用独立后台进程和场景快照，支持取消，完成后追加预览集合。
- 科学相机采样尊重新曲线的显式高度，避免二次分层导致机位偏离路径。
- 通过障碍物、重叠楼层、起点楼层、楼梯、单位换算、后台往返与取消测试，并对 Luna 已保存场景完成实测。
- 本版未新增表面可见性覆盖求解；路径覆盖诊断与重建覆盖分开报告。详见 CONTOUR_GUIDE_zh.md。

## 1.5.5 - 2026-09-04

- HIP 每进程帧数默认改为 100，并在所有 HIP 后台渲染配置中直接暴露。
- HIP worker 始终遵守该帧数上限；Persistent Data 开启时在一个有界 worker 内复用，达到上限后重启释放缓存。
- HIP 每进程帧数不再依赖 HIP Memory-safe 模式，避免“开启持久数据时没有可设置的重启边界”。
- 新建设置默认允许 HIP 使用 Persistent Data；需要保守内存策略时再手动开启 HIP Memory-safe。

## 1.5.4 - 2026-09-04

- 修复 HIP 后台进程的硬件光追配置：每个以 `--factory-startup` 启动的 worker 现在会显式开启 Cycles `use_hiprt`。此前后台进程会丢失用户偏好中的 HIP-RT 设置，RX 7900 XTX 会回退到 HIP 软件 BVH，并可能在单帧申请约 92 GB。
- HIP 默认要求 HIP-RT 可成功启用；若 Blender 构建或 AMD 驱动不可用，会在进入首次渲染前给出明确错误，不再尝试软件 BVH 的超大分配。可手动选择 Auto 或 Disabled 作为兼容模式。
- 新增首次渲染前诊断：HIP-RT 状态、实际相机数、原始/依赖图实例的顶点和面数、Geometry Nodes 数量、分辨率和缺失 Image Texture 路径都会写入 worker 日志。
- 单帧 HIP OOM 不再由后台监督器机械重试；CPU 兜底改为默认关闭的显式应急选项。

## 1.5.3 - 2026-09-04

- 新增 Cycles 后端选择：可显式选择 HIP（AMD），并在设置不可用时明确回退到 CPU，不会悄悄切换到其他 GPU 后端。
- 新增 HIP 内存安全模式：默认关闭跨帧 Persistent Data，后台进程按可调的小批次渲染，批次结束重启 Blender 释放 HIP/RAM 缓存。
- 后台监督器检测到显存/内存不足时自动减半批次后重试，并把实际后端、批次大小写入 chunk 历史。
- 每帧渲染完成后清理 Blender Render Result 和 Python 临时对象，降低长任务中的主机内存增长。
- 对单帧 HIP 分配失败增加 CPU 兜底重试；若 CPU 也无法完成，会保留原始 OOM 错误，不会伪造成功输出。

## 1.3.5 - 2026-08-20

- 场景 Normal 固定保存为 Raw 16-bit RGBA PNG（RGB = `(normal + 1) / 2`，Alpha = 可见表面），不再为训练数据长期保存大型 Normal EXR。
- Depth 继续保存为 float32 OpenEXR；断点提交、输出清单和科学序列元数据均使用 `normal/*.png`。
- 物品级 Normal（Physical Files）同样使用 16-bit PNG，并保证写入后读回校验。

## 1.3.4 - 2026-08-20

- 修复长时间连续后台渲染后 Cycles Persistent Data 缓存临时实例材质，导致后续帧大量未知 Object ID 的问题。
- Depth、Normal、Object ID、Material ID 和物品切分相关辅助 Pass 强制关闭 Persistent Data，并在每个 Pass 后恢复原设置。
- 启用 Object-aware 输出时后台进程自动最多处理 25 帧后重启，释放代理网格、材质和显存；断点续渲仍按逐帧提交记录从首个未完成帧继续。

## 1.3.3 - 2026-08-19

- 修复路径 Collection 在后台保存/重载后包含空对象引用时，数据集渲染报 `'NoneType' object has no attribute 'type'` 的问题。
- 路径辅助几何隐藏与恢复现在会安全跳过空引用和已失效的 Blender RNA 对象。

## 1.3.2 - 2026-08-19

- 修复 Collection Instance（集合实例）可见几何未被 Object ID 分组覆盖，导致物品级 Depth、Normal、Mask 在部分帧报 `unknown Object ID pixels` 的问题。
- Object ID 渲染会按依赖图中的实际实例矩阵临时展开链接集合网格，渲染后完整清理并恢复实例可见性。
- Depth 与 Normal 改用 View Layer 材质覆盖，确保链接集合和实例化几何使用同一数据材质。
- 数据集渲染期间自动隐藏相机路径 Curve，避免辅助线进入训练数据。

## 1.3.1 - 2026-08-19

- 修复外部链接只读 Mesh 在 Depth、Normal、Object ID 和 Material ID 渲染后恢复材质时报错的问题。
- 完整恢复原始 DATA/OBJECT 材质链接、材质引用和原本为空的材质槽，不修改外部链接材质。
- 辅助通道渲染时临时禁用场景合成器，保证 ID/Mask 颜色可精确解码，并在结束后恢复合成设置。
- 球壳 12 相机改为模态分批生成，提供实时进度、预计剩余时间、取消按钮和 Esc 取消。
- 取消或失败时保留原相机阵列，完成后再原子替换，并分批创建绿色相机外观以保持前台响应。

## 1.3.0 - 2026-08-19

- 将 RGB、Scene/Object Depth、Scene/Object Normal、Object Mask、Object ID 与 Material ID 改为可独立选择的输出。
- 新增无 Blender 依赖的输出依赖解析器，严格区分内部 Pass 与用户保存项。
- 新增 World-space float32 EXR Normal，并通过同一 Object ID 可见区域切分 Object Normal/Mask/Depth。
- 新增默认 Virtual Split 与兼容旧流程的 Physical Files 模式。
- 新增稳定 `autogs_object_id` / `autogs_material_id`、渲染清单、逐帧数据校验和 Blender 5.1.1 烟雾测试。

## 1.2.8 - 2026-07-22

实际升级基线：`blender_gs_colmap_exporter-1.2.7-chunked-background-render`。

### 新增

- 新增默认折叠的“Mesh 引导高斯”模块，保留原相机规划、COLMAP 导出和分块后台渲染入口。
- 新增三种工作模式：仅生成初始高斯、生成训练数据集、一键生成并训练。
- 通过 evaluated dependency graph 按对象和实例提取最终三角 Mesh，记录世界坐标、平滑法线、UV、对象 ID、材质 ID、三角形 ID 与面积。
- 按三角形面积和自适应权重采样表面，不使用“Mesh 顶点直接转高斯”的简化方案。
- 从 Principled BSDF 和 `bpy.data.images` 读取材质与图像像素；Blender Pack Resources 图像无需先解包。
- 生成贴合表面的扁平高斯，输出 `init_gaussians.npz` 和二进制标准 3DGS PLY。
- 集中实现 Blender 到 OpenCV 相机坐标转换，直接导出内参、外参和训练/验证划分。
- 复用 1.2.7 的 RGB、线性 Depth、Object ID、COLMAP 和分块恢复渲染器；新增相机空间 Normal EXR。
- Object ID 从无损 ID 渲染图转换为 `int32` NPY；背景值为 `-1`。
- 从 `training_mesh.npz` 精确渲染逐像素 Triangle ID，原子输出 `int64` NPY；背景值为 `-1`。
- 新增原子 `pipeline_state.json`、逐阶段日志、继续上次任务、取消和中间文件保留。
- 新增独立训练进程接口、四阶段训练配置、进度/指标读取和进程取消。
- 新增初始高斯预览和输出目录入口。

### 修复

- 将发布号从错误的 PDF 目标号 `1.2.0` 顺延为高于真实基线的 `1.2.8`。
- 修正 Blender 5.1 EEVEE 标识为 `BLENDER_EEVEE`。
- 修正刚创建相机后依赖图尚未刷新的姿态去重问题。
- 修正带 `shift_x`/`shift_y` 相机的内参计算；与 Blender 投影实测误差远低于 1 像素。
- 修正 EEVEE 法线渲染兼容性和材质 Emission Strength 读取。
- 修正 Blender 像素缓冲转 Object ID NPY 时的垂直方向。
- 对 V-Ray 等不能稳定自动渲染的引擎给出明确错误，不再静默切换到 Cycles。

### 明确边界

- 自动相机筛选当前完成姿态去重；Mesh 三角形可见性贪心覆盖仍为实验方向，不报告虚构覆盖率。
- 程序化材质自动烘焙、完整 UDIM 和 V-Ray 特殊材质节点解析尚未完成。

### 外部训练器协议补充

- 新增 schema 1.0 稳定训练拓扑 mesh/training_mesh.npz，Triangle ID 直接对应 triangles 行号。
- 修正过滤零面积三角形后局部 Triangle ID 不连续的问题。
- 初始高斯 NPZ 与侧车 JSON 明确记录 Quaternion、Scale、Opacity、颜色、SH、法线轴和世界单位编码。
- 相机 JSON 新增 camera_id、file_id、真值路径、颜色/深度/法线语义与单位。
- 自动定位同工作区 mesh_guided_gs_trainer 和独立虚拟环境，以 environment_report.json 作为训练门禁。
- 训练状态改读 training 根目录的 status/progress/metrics JSON，暂停、继续和停止通过 control.json 发送。
