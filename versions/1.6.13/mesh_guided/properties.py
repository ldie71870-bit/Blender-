import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import PropertyGroup


class GSCOLMAP_MeshGuidedSettings(PropertyGroup):
    show_panel: BoolProperty(name="Mesh 引导高斯", default=False)
    show_mesh_settings: BoolProperty(name="Mesh 与采样", default=True)
    show_camera_settings: BoolProperty(name="相机与真值", default=False)
    show_training_settings: BoolProperty(name="训练器", default=False)
    work_mode: EnumProperty(
        name="工作模式",
        items=(
            ("INIT_ONLY", "仅生成初始高斯", "生成 NPZ、PLY、Mesh 元数据与 GLB"),
            ("DATASET", "生成训练数据集", "生成初始高斯和训练真值，不启动训练"),
            ("GENERATE_AND_TRAIN", "一键生成并训练", "生成完整数据并启动外部训练器"),
        ),
        default="GENERATE_AND_TRAIN",
    )
    output_root: StringProperty(name="输出根目录", subtype="DIR_PATH")
    task_name: StringProperty(name="任务名称", default="MeshGuided_GS")
    overwrite_task: BoolProperty(name="覆盖已有任务", default=False)
    create_date_subdir: BoolProperty(name="日期前缀", default=False)
    keep_intermediates: BoolProperty(name="保留中间文件", default=False)
    last_task_dir: StringProperty(name="最近任务目录", subtype="DIR_PATH")

    mesh_collection: PointerProperty(name="Mesh 集合", type=bpy.types.Collection)
    exclude_hidden: BoolProperty(name="排除隐藏对象", default=True)
    exclude_render_disabled: BoolProperty(name="排除渲染禁用对象", default=True)
    preserve_uv: BoolProperty(name="保留 UV", default=True)
    preserve_normals: BoolProperty(name="保留平滑法线", default=True)
    preserve_ids: BoolProperty(name="保留对象/材质/三角形 ID", default=True)
    export_glb: BoolProperty(name="导出 GLB 参考 Mesh", default=True)
    target_gaussian_count: IntProperty(
        name="目标初始高斯数量", default=1_000_000, min=100_000, max=10_000_000,
    )
    sampling_mode: EnumProperty(
        name="采样模式",
        items=(
            ("UNIFORM", "均匀面积采样", "按三角形世界空间面积采样"),
            ("ADAPTIVE", "自适应采样", "按面积、曲率与小物体权重采样"),
            ("MIXED", "混合采样", "组合几何与材质权重"),
        ),
        default="MIXED",
    )
    curvature_weight: FloatProperty(name="曲率权重", default=1.0, min=0.0, max=10.0)
    texture_weight: FloatProperty(name="纹理变化权重", default=1.0, min=0.0, max=10.0)
    small_object_weight: FloatProperty(name="小物体增强权重", default=1.0, min=0.0, max=3.0)
    normal_thickness_ratio: FloatProperty(name="法线厚度比例", default=0.05, min=0.005, max=0.2)
    random_seed: IntProperty(name="随机种子", default=20260720, min=0)
    memory_limit_mb: IntProperty(name="内存上限 (MB)", default=8192, min=512, max=262144)
    compress_npz: BoolProperty(name="压缩 NPZ", default=True)

    automatic_camera_selection: BoolProperty(name="自动相机筛选", default=True)
    target_surface_coverage: FloatProperty(name="目标表面覆盖率", default=0.98, min=0.1, max=1.0, subtype="FACTOR")
    minimum_camera_count: IntProperty(name="最少相机数量", default=80, min=1, max=10000)
    maximum_camera_count: IntProperty(name="最多相机数量", default=200, min=1, max=10000)
    validation_ratio: FloatProperty(name="验证集比例", default=0.1, min=0.0, max=0.5, subtype="FACTOR")
    render_resolution_x: IntProperty(name="宽度", default=1920, min=64, max=16384)
    render_resolution_y: IntProperty(name="高度", default=1080, min=64, max=16384)
    transparent_background: BoolProperty(name="透明背景", default=False)

    trainer_path: StringProperty(name="训练器目录", subtype="DIR_PATH")
    training_python: StringProperty(name="Python 环境", subtype="FILE_PATH")
    training_script: StringProperty(name="训练脚本", subtype="FILE_PATH")
    cuda_device: StringProperty(name="CUDA 设备", default="0")

    pipeline_progress: FloatProperty(name="进度", default=0.0, min=0.0, max=1.0, subtype="FACTOR")
    pipeline_status: StringProperty(name="状态", default="就绪")


def register():
    bpy.utils.register_class(GSCOLMAP_MeshGuidedSettings)
    bpy.types.Scene.gs_mesh_guided_settings = PointerProperty(type=GSCOLMAP_MeshGuidedSettings)


def unregister():
    if hasattr(bpy.types.Scene, "gs_mesh_guided_settings"):
        del bpy.types.Scene.gs_mesh_guided_settings
    bpy.utils.unregister_class(GSCOLMAP_MeshGuidedSettings)

