# Changelog

## 1.4.1 - 2026-09-03

- 修复真实开门在可达域栅格化阶段被相机安全半径侵蚀成封闭墙的问题；门洞拓扑使用局部窄半径和经真实墙体/地面验证的两格补桥，最终样条仍执行完整安全半径复核。
- 新增基于距离场窄颈的 `ROOM / CORRIDOR / PORTAL` 分类，真实门洞会连接两侧可达房间，隔离空洞仍保持不可达。
- 房间路径改为多条平行 Coverage Lane，并按可达网格覆盖率自动补线；每个房间至少保留一条合法路径。
- 自动可达路径实际生成 2/3/4 个局部楼板—天花板高度层；每层独立执行 0.05–0.10 m 密集采样、BVH 安全球与薄墙碰撞裁切。
- 自动预分层 Segment 直接接入现有科学 Coverage/Overlap/Pose 管线，防止再次扩展高度层；新增房间、门洞、覆盖率和每层 Segment 统计。
- Blender 5.1.1 回归覆盖两房真实门洞、三房串联门洞、2/3/4 层、大房间多路径、楼梯多层、人工行走路径及旧封闭空洞隔离。

## 1.4.0 - 2026-09-03

- 新增 Manual Walk Path：调用 Blender Walk Navigation，以定时器持续记录视口世界坐标 XYZ。
- 原始样本经过距离去重、0.10 m 弧长重采样和 Z 过渡保护平滑，楼梯与多楼层轨迹不会被压平成水平曲线。
- BasePath、2/3/4 层 Offset 配置和生成的 ValidSplineSegment 分离；修改层间距或单层偏移会自动重建，无需重新行走。
- 使用 Evaluated Mesh 共享 BVH、实体内部判断、0.075 m 密集样条采样和可调相机安全球执行碰撞裁切。
- 连续碰撞区间自动合并并按安全半径扩张，碰撞处拆分 Segment；短于 0.50 m 的碎段自动删除。
- 新增 Base、各高度层、合法 Segment 与红色碰撞区间调试显示，生成路径时不创建相机对象。
- ValidSplineSegment 直接接入现有科学弧长采样、Coverage、Overlap、Polar/Bridge 和 Pose Selection，并防止预分层人工路径被再次增加高度。
- Blender 5.1.1 验收覆盖普通房间、真实 Z 楼梯、Upper/Lower 独立碰撞断开、调高度后实时重裁切以及单一 `GS_CAPTURE_CAMERA`。

## 1.3.10 - 2026-09-03

- 修复旧全景路径模式绕过“种子可达空间”规划器的问题；开启多楼层规划后，所有自动路径模式统一使用三维可达域。
- 将种子可达域升级为路径生成、片段拼接与曲线平滑的最终硬掩膜，逐段重采样，禁止平滑后重新穿入墙体、无支撑空洞或不可达区域。
- 增加相机安全半径侵蚀与地面边缘多点支撑检测，拒绝悬空及过窄候选单元。
- 种子优先吸附到光标正下方的实际地面与支撑对象；找不到合法地面时给出明确错误，不再静默回退旧规划器。
- 三维网格改为八邻域连通，并加入可达域边界距离场，使覆盖路径优先经过空间中部。
- 调试视图新增全部候选、可达、拒绝及种子四类可视对象。
- 新增旧全景模式、门洞连通、无楼梯楼层隔离、空洞禁止跨越和最终曲线掩膜回归测试。

## 1.3.9 - 2026-09-03

- 修复同一高度的客厅地面与建筑外壳/空洞表面在边界相邻时被合并为同一 FloorRegion 的问题。
- 可行走采样记录实际承载 Mesh；平层区域默认不跨承载 Mesh 合并，真正楼梯仍通过验证后的 Connector 跨对象连接。
- Luna 当前场景实测最终 174 个路径点全部落在游标地面 `StaticMeshActor190`，其他 5 个外壳与装饰 Mesh 的路径点归零。
- 有效路径范围由 `X=-1.29～12.80, Y=-5.51～2.68` 收敛到客厅 `X=0.07～10.53, Y=-1.41～3.59`。

## 1.3.8 - 2026-09-03

- 使用用户当前打开的 Luna 场景自动保存文件进行 Blender 5.1.1 后台复现与修复。
- 修复窄斜面、家具表面和模型外壳形成“假楼梯”，把游标所在客厅与地下封闭空腔错误连通的问题。
- 楼梯/坡道连接器新增最小升高、楼层高差、垂直单调性、横向网格覆盖和两端主楼层校验。
- 可达性筛选改为先验证 Connector，再进行楼层级连通裁剪；被拒绝的假楼梯不会再把空腔楼层带入最终路径。
- Luna 实测由 5 个混杂楼层、3 条假楼梯收敛为 1 个有效客厅楼层、0 条假楼梯；最终路径高度稳定在 1.497–1.501 m。

## 1.3.7 - 2026-09-03

- 修复 1.3.6 的 3D 多楼层规划绕过“可达空间/游标种子”筛选，导致路径生成在封闭模型空腔中的问题。
- 相邻可行走网格现在必须通过真实墙体碰撞和连续地面支撑校验，连通搜索不再穿墙。
- 默认只生成从 3D 光标所在空间沿门洞、地面、楼梯和坡道可达的完整三维连通分量；无法定位合法起点时给出明确提示，不再静默回退到全场景排线。
- 新增“当前视角”可达性起点，并在路径面板中显示起点选择与种子对象设置。
- 新增上下重叠封闭空腔、墙体阻断和游标连通回归测试；通过 Blender 5.1.1 单层、多楼层楼梯和空腔隔离实测。

## 1.3.6 - 2026-09-03

- 自动科学路径升级为真正的 3D 多楼层规划：同一 XY 可保留多个合法楼层，并显式建立 `FloorRegion + Connector` 拓扑。
- 识别楼梯、平台和坡道；楼梯路径沿实际踏步连续改变 XYZ，不再拆成上下两层后直线硬连。
- 新增净空优先中心线、碰撞验证的曲线平滑、带楼层/切线/安全检查的 Fragment Stitching，以及大空间少量偏移覆盖路径。
- 科学路径按真实弧长采样，并在弯道、楼梯入口、平台和出口自动加密 Pose。
- Scientific 新工程默认使用 `SCIENTIFIC_POSE_SEQUENCE`，正式姿态只写入 `camera_sequence.json`，场景仅保留 `GS_CAPTURE_CAMERA`。
- `GS_SCIENTIFIC_PATH_DEBUG` 新增楼层区域、原始碎片、最终样条和楼梯连接显示；全部调试对象排除在 BVH、Coverage 和渲染之外。
- 新增两层直楼梯、同 XY 双层、S 形走廊、碎片拼接、挑空、单相机和弧长采样回归，并通过 Blender 5.1.1 后台实测。

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
