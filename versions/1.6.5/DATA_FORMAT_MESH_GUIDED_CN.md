# Mesh 引导高斯数据格式

## 目录

```text
任务目录/
├── config/
│   ├── scene_config.json
│   ├── training_config.yaml
│   ├── plugin_version.json
│   └── ground_truth_format.json
├── mesh/
│   ├── scene.glb
│   ├── training_mesh.npz
│   ├── mesh_metadata.json
│   └── material_metadata.json
├── cameras/
│   ├── cameras.json
│   ├── train.json
│   ├── validation.json
│   └── visibility_report.json
├── images/{train,validation}/
├── depth/{train,validation}/
├── normal/{train,validation}/
├── object_id/{train,validation}/
├── triangle_id/{train,validation}/
├── gaussians/
│   ├── init_gaussians.npz
│   └── init_gaussians.ply
├── training/{checkpoints,logs,output}/
├── temp/
├── pipeline.log
├── pipeline_state.json
└── dataset_validation.json
```

训练/验证真值文件使用六位编号，如 `000001.png`、`000001.exr` 和 `000001.npy`。为兼容原 1.2.7 渲染器，任务根目录可能同时保留扁平的 `images/`、`depth/`、`id/` 和 COLMAP 输出；训练器应读取划分子目录与 `cameras/*.json`。

## `init_gaussians.npz`

设高斯数量为 `N`：

| 字段 | 形状 | 类型 | 语义 |
| --- | --- | --- | --- |
| `means` | `N x 3` | float32 | 世界空间中心，米 |
| `quats` | `N x 4` | float32 | 归一化 WXYZ 四元数 |
| `scales` | `N x 3` | float32 | 线性轴尺度，第三轴沿表面法线 |
| `log_scales` | `N x 3` | float32 | `log(scales)`，供标准 3DGS 初始化 |
| `opacities` | `N` | float32 | Alpha 的 logit |
| `alpha` | `N` | float32 | `[0,1]` 线性透明度 |
| `sh0` | `N x 3` | float32 | 0 阶球谐系数，`(rgb-0.5)/C0` |
| `shN` | `N x 0` | float32 | 预留高阶球谐 |
| `colors` | `N x 3` | float32 | 线性基础色 |
| `triangle_ids` | `N` | int64 | 全局三角形锚点 |
| `object_ids` | `N` | int32 | 对象/实例 ID |
| `material_ids` | `N` | int32 | 对象材质槽 ID |
| `barycentrics` | `N x 3` | float32 | 三角形重心坐标，和为 1 |
| `surface_normals` | `N x 3` | float32 | 世界空间单位法线 |
| `original_positions` | `N x 3` | float32 | 训练前表面位置 |
| `normal_offsets` | `N` | float32 | 初始法线偏移，值为 0 |
| `uvs` | `N x 2` | float32 | 插值 UV |
| `roughness` | `N` | float32 | 粗糙度 |
| `metallic` | `N` | float32 | 金属度 |
| `emission` | `N x 3` | float32 | Emission Color 乘 Emission Strength |

## PLY

`init_gaussians.ply` 为 binary little-endian。字段兼容常见 3DGS PLY：位置、法线、`f_dc_0..2`、logit opacity、log scale 和 WXYZ rotation。Triangle/Object/Material 锚点以 NPZ 为权威来源。

## 相机

每条相机记录包含：

- `camera_to_world`、`world_to_camera`
- `fx`、`fy`、`cx`、`cy`
- 图像宽高、传感器尺寸、焦距
- 近远裁剪面、相机名、帧号和六位图像名

矩阵使用 OpenCV 相机坐标。`blender_camera_to_world` 同时保留用于审计。

## 真值

- RGB：PNG，颜色空间由场景当前渲染设置决定。
- Depth：EXR float32，相机空间正向 Z；背景由 Alpha 区分。
- Normal：EXR float32，相机空间；解码公式 `normal = rgb * 2 - 1`。
- Object ID：NPY int32；有效对象从 1 开始，背景为 `-1`。
- Triangle ID：NPY int64；值直接对应 `training_mesh.npz/triangles` 行号，背景为 `-1`。

## Schema 1.0 训练器补充

mesh/training_mesh.npz 是训练器唯一可信的 Triangle ID 拓扑。文件包含世界空间顶点、三角形索引、面法线、对象/材质 ID、切线、副切线、UV 和面积。scene.glb 仅供查看，不参与锚点 ID 推断。

gaussians/init_gaussians.npz 新增 schema_version、world_unit_to_meters、quaternion_order、scale_encoding、opacity_encoding、color_encoding、sh_encoding、surface_normal_space 和 gaussian_normal_axis。相同信息另写入 init_gaussians.metadata.json。

cameras 三个 JSON 记录五类真值相对路径和 OpenCV +Z 前向语义。Triangle ID 由稳定训练拓扑直接构建临时 Mesh，以 Raw 32-bit EXR 中转并原子写入 NPY；不经过 GLB，因此不会重排 ID。
