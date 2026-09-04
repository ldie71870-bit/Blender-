bl_info = {
    "name": "Gaussian Splat COLMAP Dataset Generator",
    "author": "Codex",
    "version": (1, 6, 3),
    "blender": (5, 1, 0),
    "location": "View3D/Shader Editor > Sidebar > GS; Render Properties > GS Dataset; Render Menu > GS Dataset",
    "description": "Automated Blender dataset renderer for Gaussian Splatting and COLMAP-style sparse models.",
    "category": "Render",
}

import hashlib
import json
import math
import os
import random
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Operator, Panel, PropertyGroup
from mathutils import Matrix, Vector


from . import scientific_planner
from . import coverage_patch
from . import pose_sequence
from . import mesh_guided
from . import output_pipeline
from . import contour_blender
from . import contour_jobs

I18N = {
    "en": {
        "panel": "GS Dataset",
        "language": "Language",
        "output_dir": "Output Directory",
        "render": "Render Dataset",
        "background_render": "Background Render",
        "cancel_render": "Cancel Background Render",
        "progress": "Progress",
        "create_cameras": "Create Camera Rig",
        "q_draft": "Draft",
        "q_std": "Standard",
        "q_ultra": "Photoreal",
        "quality": "Render Quality",
        "export_only": "Export COLMAP Only",
        "rig": "Rig",
        "target": "Look-at Target",
        "path": "Path Curve",
        "path_collection": "Path Curves Collection",
        "count": "Cameras",
        "rings": "Rings",
        "radius": "Radius",
        "height": "Height",
        "fov": "Focal Length",
        "resolution": "Resolution",
        "format": "Image Format",
        "camera_model": "Camera Model",
        "panorama_fov": "Panorama FOV",
        "live_update": "Live Camera Update",
        "path_look": "Path Aim",
        "path_samples": "Curve Detail",
        "path_count_mode": "Path Count Mode",
        "path_density": "Path Density",
        "engine": "Renderer",
        "cycles_samples": "Cycles Samples",
        "cycles_denoise": "Cycles Denoise",
        "cycles_device": "Cycles Device",
        "incremental": "Incremental",
        "depth": "Depth EXR",
        "mask": "Masks",
        "mask_collection": "Mask Collection",
        "point_samples": "Point Samples/View",
        "dedup": "Point Dedup Size",
        "volume": "Volume",
        "volume_count": "Volume Grid",
        "jitter": "Jitter",
        "exclude": "Exclusion Collection",
        "status_ready": "Ready.",
        "msg_no_output": "Please choose an output directory.",
        "msg_done": "Dataset generated",
        "msg_cameras_done": "Camera rig created",
        "msg_export_done": "COLMAP files exported",
    },
    "zh": {
        "panel": "高斯泼溅数据集",
        "language": "语言",
        "output_dir": "输出目录",
        "render": "渲染数据集",
        "background_render": "后台渲染",
        "cancel_render": "取消后台渲染",
        "progress": "进度",
        "create_cameras": "创建相机阵列",
        "q_draft": "草稿",
        "q_std": "标准",
        "q_ultra": "超写实",
        "quality": "渲染质量",
        "export_only": "仅导出 COLMAP",
        "rig": "阵列方式",
        "target": "注视目标",
        "path": "路径曲线",
        "path_collection": "路径曲线集合",
        "count": "相机数量",
        "rings": "环数",
        "radius": "半径",
        "height": "高度",
        "fov": "焦距",
        "resolution": "分辨率",
        "format": "图像格式",
        "camera_model": "相机模型",
        "panorama_fov": "全景视场角",
        "live_update": "实时更新相机",
        "path_look": "路径朝向",
        "path_samples": "曲线细分",
        "path_count_mode": "路径数量模式",
        "path_density": "路径密度",
        "engine": "渲染器",
        "cycles_samples": "Cycles 采样数",
        "cycles_denoise": "Cycles 降噪",
        "cycles_device": "Cycles 设备",
        "incremental": "增量渲染",
        "depth": "深度 EXR",
        "mask": "遮罩",
        "mask_collection": "遮罩集合",
        "point_samples": "每视角点云采样",
        "dedup": "点云去重尺寸",
        "volume": "体积采集",
        "volume_count": "体积网格",
        "jitter": "随机抖动",
        "exclude": "排除集合",
        "status_ready": "就绪。",
        "msg_no_output": "请选择输出目录。",
        "msg_done": "数据集已生成",
        "msg_cameras_done": "相机阵列已创建",
        "msg_export_done": "COLMAP 文件已导出",
    },
}


RIG_ITEMS = (
    ("SPHERE", "Product Sphere / 产品球面", "Orbit cameras around the target."),
    ("HEMISPHERE", "Product Hemisphere / 产品半球", "Upper hemisphere orbit."),
    ("CYLINDER", "Product Cylinder / 产品圆柱", "Cylindrical camera rings."),
    ("HALF_CYLINDER", "Half Cylinder / 半圆柱", "Half cylinder sweep."),
    ("PATH", "Path Curve / 路径曲线", "Use curve control points as camera positions."),
    ("VOLUME", "Volume Grid / 体积网格", "Sample cameras inside a box."),
    ("EXISTING", "Existing Cameras / 现有相机", "Use cameras already in the GS collection."),
)

RENDER_ENGINE_ITEMS = (
    ("CYCLES", "Cycles / Cycles", "Use the Cycles path tracer for training images."),
    ("BLENDER_EEVEE", "EEVEE / EEVEE", "Use EEVEE only if you intentionally need realtime-style output."),
)

CYCLES_DEVICE_ITEMS = (
    ("AUTO", "Auto / 自动", "Keep Blender's current Cycles device setting."),
    ("GPU", "GPU / GPU", "Use GPU rendering when available."),
    ("CPU", "CPU / CPU", "Use CPU rendering."),
)

CYCLES_BACKEND_ITEMS = (
    ("AUTO", "Auto / 自动", "Use the best available Cycles backend."),
    ("OPTIX", "OptiX / OptiX", "Use NVIDIA OptiX when available."),
    ("CUDA", "CUDA / CUDA", "Use NVIDIA CUDA when available."),
    ("HIP", "HIP / HIP", "Use AMD HIP when available."),
    ("ONEAPI", "oneAPI / oneAPI", "Use Intel oneAPI when available."),
    ("METAL", "Metal / Metal", "Use Apple Metal when available."),
)

HIP_RT_MODE_ITEMS = (
    ("REQUIRE", "Required / 必须启用", "Require HIP-RT before rendering on HIP. Recommended for large scenes."),
    ("AUTO", "Auto / 自动", "Try to enable HIP-RT, but allow Blender's software fallback if unavailable."),
    ("DISABLED", "Disabled / 关闭", "Deliberately use the HIP software BVH path. Not suitable for large GS scenes."),
)

CAMERA_MODEL_ITEMS = (
    ("PERSPECTIVE", "Perspective / 透视", "Standard perspective camera, exported as COLMAP PINHOLE. Directly usable by 3DGS."),
    ("PANORAMA_CUBE", "Panorama Cube (6× Perspective) / 全景立方体(可训练)",
     "360° coverage by splitting each position into 6 perspective cube faces (PINHOLE). "
     "Fully compatible with standard 3DGS, unlike equirectangular/fisheye."),
)

PATH_LOOK_ITEMS = (
    ("TARGET", "Look at Target / 注视目标", "Each path camera looks at the selected target or 3D cursor."),
    ("FORWARD", "Follow Curve / 沿曲线前进", "Each path camera aims along the sampled curve tangent."),
    ("CURVE_ROTATION", "Curve Rotation / 曲线旋转", "Use the path object's world rotation instead of requiring a look-at target."),
)

PATH_COUNT_MODE_ITEMS = (
    ("COUNT", "Fixed Count / 固定数量", "Use the camera count slider for path cameras."),
    ("DENSITY", "Density / 按密度", "Compute path camera count from curve length and camera density."),
)

PATH_CAPTURE_MODE_ITEMS = (
    ("LEGACY_PANORAMA_CUBE", "路径站点阵列", "沿原有 Curve 站点生成球壳12相机或旧六相机兼容阵列。"),
    ("SCIENTIFIC_THREE_LAYER", "科学三层覆盖（推荐）", "使用分布式多高度机位、射线覆盖优化和重叠图修复。"),
)

PATH_STATION_ARRAY_MODE_ITEMS = (
    ("SPHERICAL_SHELL_12", "球壳12相机", "每个原路径站点生成12个不同光心、径向向外的球壳相机。"),
    ("LEGACY_SIX", "旧六相机兼容", "保留旧版同光心 px/nx/py/ny/pz/nz 六相机行为。"),
)

SHELL_RADIUS_MODE_ITEMS = (
    ("FIXED", "固定半径", "始终尝试指定球壳半径；不安全时按失败策略处理。"),
    ("CLEARANCE_ADAPTIVE", "净空自适应", "12个位置必须共用半径；不安全时整体等比缩小。"),
)

SHELL_FAILURE_POLICY_ITEMS = (
    ("LEGACY_SIX_FALLBACK", "回退旧六相机", "最小球壳仍不安全时，在安全站点中心生成旧六相机。"),
    ("SKIP_STATION", "跳过站点", "无法安全放置完整球壳时跳过该站点。"),
)

SCIENTIFIC_REALIZATION_MODE_ITEMS = (
    ("LEGACY_CUBEMAP_OBJECTS", "Legacy Cubemap Objects", "Keep one Blender camera object for every cubemap face."),
    ("SCIENTIFIC_CAMERA_OBJECTS", "Scientific Camera Objects", "Keep one Blender camera object for every scientific training frame."),
    ("SCIENTIFIC_POSE_SEQUENCE", "Scientific Pose Sequence (Recommended)", "Keep every planned frame in camera_sequence.json and render it through one GS_CAPTURE_CAMERA."),
)

SEQUENCE_DEBUG_MODE_ITEMS = (
    ("OFF", "Off", "Hide pose-sequence debug markers."),
    ("CURRENT", "Current Pose", "Show only the selected logical pose."),
    ("NEIGHBORHOOD", "Current +/- N", "Show the selected pose and its neighbors in the same segment."),
    ("SAMPLED", "Every N Poses", "Show a deterministic stride sample."),
    ("ALL_LIGHTWEIGHT", "All Lightweight", "Show every pose as a lightweight Empty."),
)

SCIENTIFIC_RAY_QUALITY_ITEMS = (
    ("FAST", "快速 8 x 8", "最终入选相机使用 8 x 8 完整复检；全部候选先用 4 x 4 预筛。"),
    ("NORMAL", "普通 12 x 12", "最终入选相机使用 12 x 12 完整复检；全部候选先用 4 x 4 预筛。"),
    ("HIGH", "高质量 16 x 16", "最终入选相机使用 16 x 16 完整复检；全部候选先用 4 x 4 预筛。"),
)

SCIENTIFIC_ORIGIN_MODE_ITEMS = (
    ("MANUAL_CURVE", "Manual Curve", "Use the existing path-compatible scientific planner."),
    ("AUTO_GRID_PATH", "Automatic Grid Path", "Generate an indoor grid path when no Curve exists."),
    ("FREE_SPACE", "Automatic Free Space", "Generate origins directly from legal 2.5D free space."),
    ("SMALL_SPACE", "Small Space Adaptive", "Adapt layers and clearance for small rooms and corridors."),
    ("HYBRID", "Hybrid", "Combine optional Curve and free-space origins."),
)

SCIENTIFIC_BUDGET_MODE_ITEMS = (
    ("LEGACY_PATH_BUDGET", "Legacy Path Budget", "Keep the old six-view path budget when a Curve is used."),
    ("AREA_ADAPTIVE_BUDGET", "Free-space Area", "Use free-space area and component count as the image cap."),
    ("SURFACE_ADAPTIVE_BUDGET", "Reachable Surface", "Use estimated surface-cell count as the image cap."),
    ("USER_FIXED_BUDGET", "User Fixed", "Use a fixed image cap."),
)

SCIENTIFIC_PROGRESS_RANGES = {
    "path_sampling": (0.00, 0.05),
    "free_space_grid": (0.00, 0.15),
    "candidate_prefilter": (0.05, 0.65),
    "greedy_selection": (0.70, 0.05),
    "final_quality_rays": (0.75, 0.13),
    "final_coverage_repair": (0.88, 0.05),
    "final_category_repair": (0.93, 0.05),
    "final_overlap_repair": (0.98, 0.02),
}


def _scientific_progress_fraction(stage, current, total):
    offset, span = SCIENTIFIC_PROGRESS_RANGES.get(stage, (0.0, 1.0))
    return min(1.0, max(0.0, offset + span * current / max(1, total)))

FLOORPLAN_LAYER_MODE_ITEMS = (
    ("ONE", "单层", "仅生成中层高度，用于局部检查"),
    ("TWO", "二层：低层 + 顶层", "Create low and top routes."),
    ("THREE", "三层：低层 + 中层 + 顶层", "Create low, middle and top routes."),
    ("FOUR", "四层：低层 + 中层 + 高层 + 顶层", "Create low, middle, high and top routes."),
)

FLOORPLAN_SPACE_MODE_ITEMS = (
    ("REACHABLE", "从3D光标可达空间", "Only create curves in the island reachable from the 3D cursor."),
    ("ALL", "全部室内空间", "Create curves in every detected indoor island."),
)

FLOORPLAN_SEED_MODE_ITEMS = (
    ("CURSOR", "3D Cursor", "Keep only the island reachable from the 3D cursor."),
    ("OBJECT", "Seed Object", "Keep only the island reachable from the selected seed object."),
    ("CAMERA", "Scene Camera", "Keep only the island reachable from the scene camera."),
    ("ALL", "All Islands", "Create grid curves for every detected indoor island."),
)


# Outdoor daylight for interiors: a physical (Nishita) sky drives the look; presets only pick
# sun height/azimuth, haze and strength. OVERCAST/BLUE_HOUR turn the visible sun disc off.





def tr(settings, key):
    lang = getattr(settings, "language", "zh")
    return I18N.get(lang, I18N["zh"]).get(key, key)


def safe_name(value):
    keep = []
    for char in value:
        keep.append(char if char.isalnum() or char in ("-", "_", ".") else "_")
    return "".join(keep)


def ensure_dir(path):
    Path(path).mkdir(parents=True, exist_ok=True)


def write_progress(progress_path, state, current=0, total=0, message="", error="", elapsed=None, eta=None):
    if not progress_path:
        return
    payload = {
        "state": state,
        "current": current,
        "total": total,
        "progress": 0.0 if total <= 0 else min(1.0, max(0.0, current / total)),
        "message": message,
        "error": error,
        "elapsed": elapsed,
        "eta": eta,
    }
    path = Path(progress_path)
    try:
        ensure_dir(path.parent)
    except Exception:
        return
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    data = json.dumps(payload, ensure_ascii=False)
    # Progress is NON-critical. On Windows the foreground reader briefly holds
    # progress.json open, so os.replace() can raise WinError 5 / sharing violations.
    # That must NEVER abort the render -> retry, then fall back, then give up silently.
    for attempt in range(12):
        try:
            with open(tmp_path, "w", encoding="utf-8") as handle:
                handle.write(data)
            os.replace(tmp_path, path)
            return
        except (PermissionError, OSError):
            time.sleep(0.04 * (attempt + 1))
    try:
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(data)
    except Exception:
        pass
    finally:
        try:
            if tmp_path.exists():
                tmp_path.unlink()
        except Exception:
            pass


# --------------------------------------------------------------------------------------
# Resumable-render checkpoint + render-time helpers
# --------------------------------------------------------------------------------------
def _fmt_time(seconds):
    seconds = int(max(0, round(seconds)))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    if h:
        return f"{h}h{m:02d}m"
    if m:
        return f"{m}m{s:02d}s"
    return f"{s}s"


def _addon_version_str():
    return ".".join(str(v) for v in (globals().get("bl_info") or {}).get("version", (1, 3, 4)))


RENDER_STATE_NAME = "_gs_render_state.json"
_RENDER_SIG_KEYS = (
    "image_prefix", "image_format", "resolution_x", "resolution_y", "render_engine",
    "cycles_samples", "cycles_denoise", "cycles_device", "cycles_backend", "hip_rt_mode",
    "hip_memory_safe_mode", "hip_chunk_size", "hip_oom_fallback", "cycles_persistent_data",
    "camera_model", "color_look",
    "color_exposure", "render_rgb", "export_depth", "depth_format", "export_object_depth",
    "export_normal", "export_object_normal", "export_object_mask", "export_id",
    "export_material_id", "object_split_mode",
    "object_group_mode", "transparent_background", "rig_mode", "camera_count",
    "path_count_mode", "path_camera_density", "auto_create_rig",
)


def render_signature(settings, total):
    """A fingerprint of every setting that changes the rendered frames. If it differs from the
    checkpoint, resume is invalid and the render starts fresh (e.g. you enabled a new pass or
    changed resolution / camera count)."""
    sig = {k: getattr(settings, k, None) for k in _RENDER_SIG_KEYS}
    if (
        getattr(settings, "rig_mode", "") == "PATH"
        and getattr(settings, "path_capture_mode", "") == "LEGACY_PANORAMA_CUBE"
    ):
        array_mode = getattr(settings, "path_station_array_mode", "SPHERICAL_SHELL_12")
        sig["path_station_array_mode"] = array_mode
        if array_mode == "SPHERICAL_SHELL_12":
            for key in (
                "shell_radius", "shell_radius_mode", "shell_min_radius", "shell_failure_policy"
            ):
                sig[key] = getattr(settings, key, None)
    sig["total"] = int(total)
    return hashlib.sha1(json.dumps(sig, default=str, sort_keys=True).encode("utf-8")).hexdigest()[:16]


def _render_state_path(settings):
    return Path(settings.output_dir) / RENDER_STATE_NAME


def load_render_state(settings, total):
    """Return the checkpoint dict iff it matches the current settings signature, else None."""
    p = _render_state_path(settings)
    if not p.exists():
        return None
    try:
        with open(p, "r", encoding="utf-8") as handle:
            st = json.load(handle)
    except Exception:
        return None
    if st.get("signature") != render_signature(settings, total):
        return None
    return st


def save_render_state(settings, total, done, render_seconds, state="rendering"):
    """Persist resume progress + accumulated render time after each frame (atomic write)."""
    p = _render_state_path(settings)
    payload = {
        "version": _addon_version_str(),
        "total": int(total),
        "done": sorted(int(i) for i in done),
        "signature": render_signature(settings, total),
        "render_seconds": round(float(render_seconds), 2),
        "state": state,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    try:
        ensure_dir(p.parent)
        tmp = p.with_suffix(".json.tmp")
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=1)
        os.replace(tmp, p)
    except Exception:
        pass


def clear_render_state(settings):
    try:
        _render_state_path(settings).unlink()
    except Exception:
        pass


def collection_get(name):
    collection = bpy.data.collections.get(name)
    if not collection:
        collection = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(collection)
    return collection



CAMERA_VISUAL_COLLECTION = "GS_CAMERA_MESH_STYLE"
CAMERA_VISUAL_MESH = "GS_Camera_Green_Pyramid_Mesh"
CAMERA_VISUAL_MATERIAL = "GS_Camera_Green_Material"
CAMERA_VISUAL_PREFIX = "GS_CameraVisual_"


def is_camera_mesh_visual(obj):
    return bool(obj and obj.type == "MESH" and obj.get("gs_camera_mesh_visual"))


def _camera_visual_mesh():
    mesh = bpy.data.meshes.get(CAMERA_VISUAL_MESH)
    if mesh is not None:
        return mesh
    mesh = bpy.data.meshes.new(CAMERA_VISUAL_MESH)
    # Blender cameras look along local -Z. The apex is the optical centre and the
    # square base is in front of the lens, so the pyramid shows the view direction.
    vertices = (
        (0.0, 0.0, 0.0),
        (-0.5, -0.5, -1.0),
        (0.5, -0.5, -1.0),
        (0.5, 0.5, -1.0),
        (-0.5, 0.5, -1.0),
    )
    faces = ((0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1), (1, 4, 3, 2))
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    mesh["gs_camera_mesh_visual"] = True
    return mesh


def _camera_visual_material():
    material = bpy.data.materials.get(CAMERA_VISUAL_MATERIAL)
    if material is None:
        material = bpy.data.materials.new(CAMERA_VISUAL_MATERIAL)
    material.diffuse_color = (0.03, 0.80, 0.12, 1.0)
    return material


def camera_mesh_visual(camera):
    if camera is None:
        return None
    name = camera.get("gs_camera_mesh_visual_name", "")
    visual = bpy.data.objects.get(name) if name else None
    if is_camera_mesh_visual(visual) and visual.parent is camera:
        return visual
    for child in camera.children:
        if is_camera_mesh_visual(child):
            return child
    return None


def remove_camera_mesh_visual(camera):
    visual = camera_mesh_visual(camera)
    if visual is not None:
        bpy.data.objects.remove(visual, do_unlink=True)
    if camera is not None:
        original_size = camera.get("gs_camera_original_display_size")
        if original_size is not None and camera.type == "CAMERA":
            camera.data.display_size = float(original_size)
        for key in ("gs_camera_mesh_visual_name", "gs_camera_original_display_size"):
            if key in camera:
                del camera[key]


def ensure_camera_mesh_visual(scene, settings, camera):
    if camera is None or camera.type != "CAMERA":
        return None
    visual = camera_mesh_visual(camera)
    collection = collection_get(CAMERA_VISUAL_COLLECTION)
    collection.hide_render = True
    if visual is None:
        visual = bpy.data.objects.new(CAMERA_VISUAL_PREFIX + camera.name, _camera_visual_mesh())
        collection.objects.link(visual)
        visual.parent = camera
        visual.location = (0.0, 0.0, 0.0)
        visual.rotation_euler = (0.0, 0.0, 0.0)
        camera["gs_camera_mesh_visual_name"] = visual.name
    elif collection not in tuple(visual.users_collection):
        collection.objects.link(visual)
    material = _camera_visual_material()
    if not visual.data.materials:
        visual.data.materials.append(material)
    else:
        visual.data.materials[0] = material
    scale_length = float(getattr(scene.unit_settings, "scale_length", 1.0) or 1.0)
    size = max(0.01, float(getattr(settings, "camera_mesh_size", 0.25))) / max(1e-9, scale_length)
    visual.scale = (size, size, size)
    visual.color = (0.03, 0.80, 0.12, 1.0)
    visual.display_type = "SOLID"
    visual.show_in_front = True
    visual.hide_render = True
    visual["gs_camera_mesh_visual"] = True
    visual["gs_camera_source"] = camera.name
    for attribute in ("visible_camera", "visible_diffuse", "visible_glossy", "visible_shadow"):
        if hasattr(visual, attribute):
            setattr(visual, attribute, False)
    if "gs_camera_original_display_size" not in camera:
        camera["gs_camera_original_display_size"] = float(camera.data.display_size)
    camera.data.display_size = 0.01
    return visual


def sync_camera_mesh_visuals(scene, settings, cameras):
    cameras = [camera for camera in cameras if camera and camera.type == "CAMERA"]
    enabled = bool(getattr(settings, "camera_mesh_style", True))
    if bpy.app.background:
        enabled = False
    for camera in cameras:
        if enabled:
            ensure_camera_mesh_visual(scene, settings, camera)
        else:
            remove_camera_mesh_visual(camera)
    return sum(camera_mesh_visual(camera) is not None for camera in cameras)


def hide_camera_mesh_visuals(scene):
    states = []
    for obj in scene.objects:
        if not is_camera_mesh_visual(obj):
            continue
        states.append((obj.name, bool(obj.hide_viewport)))
        obj.hide_viewport = True
    if states:
        try:
            bpy.context.view_layer.update()
        except Exception:
            pass
    return states


def restore_camera_mesh_visuals(states):
    for name, hidden in states or ():
        obj = bpy.data.objects.get(name)
        if obj is not None:
            obj.hide_viewport = hidden
    if states:
        try:
            bpy.context.view_layer.update()
        except Exception:
            pass














def apply_view_transform(scene, transform="AgX", exposure=0.0, look="None", set_gamma=True):
    """Set colour management for one render pass.
      - COLOUR training images: AgX (default) / Filmic roll off highlights so a bright scene
        never overexposes; 'Standard' is faithful sRGB but can clip (use a negative exposure
        to pull it back). These three are the user-facing 'Color Look'.
      - DEPTH / ID passes: pass transform='Raw' so the stored values stay linear and exact
        (no tone curve baked into the depth EXR or the id colours).
    Always restore around a pass with _save_render_state/_restore_render_state so passes never
    contaminate each other (this is what previously made depth+color renders look wrong)."""
    vs = getattr(scene, "view_settings", None)
    if vs is None:
        return
    try:
        vs.view_transform = transform
    except (TypeError, AttributeError):
        try:
            vs.view_transform = "Standard"
        except Exception:
            return
    try:
        vs.look = look
    except (TypeError, AttributeError):
        pass
    try:
        vs.exposure = float(exposure)
        if set_gamma:
            vs.gamma = 1.0       # raw/colour passes want neutral gamma; interactive paths pass
                                 # set_gamma=False so the user's own Gamma is never reset.
    except Exception:
        pass


















































# type -> power multiplier on the base wattage












RENDER_QUALITY_PRESETS = {
    "DRAFT":    dict(samples=64,   thresh=0.05,  clamp=6.0,  bounces=8,  trans=10, transparent=12),
    "STANDARD": dict(samples=256,  thresh=0.01,  clamp=10.0, bounces=12, trans=12, transparent=16),
    "ULTRA":    dict(samples=1024, thresh=0.003, clamp=15.0, bounces=16, trans=16, transparent=24),
}


def apply_render_quality(scene, settings, mode):
    """Apply the quality preset without enabling long-lived Cycles caches."""
    preset = RENDER_QUALITY_PRESETS.get(mode) or RENDER_QUALITY_PRESETS["STANDARD"]
    settings.cycles_samples = preset["samples"]
    settings.cycles_denoise = True
    cyc = getattr(scene, "cycles", None)
    if cyc is not None:
        pairs = (
            ("samples", preset["samples"]),
            ("use_adaptive_sampling", True),
            ("adaptive_threshold", preset["thresh"]),
            ("use_denoising", True),
            ("denoiser", "OPENIMAGEDENOISE"),
            ("sample_clamp_indirect", preset["clamp"]),
            ("blur_glossy", 1.0),
            ("max_bounces", preset["bounces"]),
            ("diffuse_bounces", max(4, preset["bounces"] // 2)),
            ("glossy_bounces", max(4, preset["bounces"] // 2)),
            ("transmission_bounces", preset["trans"]),
            ("transparent_max_bounces", preset["transparent"]),
            ("caustics_reflective", False),
            ("caustics_refractive", False),
            ("use_light_tree", True),
        )
        for attr, value in pairs:
            try:
                setattr(cyc, attr, value)
            except Exception:
                pass
    try:
        scene.render.use_persistent_data = bool(getattr(settings, "cycles_persistent_data", False))
    except Exception:
        pass
    return preset["samples"]


def look_at(obj, target, roll=0.0):
    direction = Vector(target) - obj.location
    if direction.length < 1e-6:
        return
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    if roll:
        obj.rotation_euler.rotate_axis("Z", roll)


def aim_along(obj, direction, roll=0.0):
    if direction.length < 1e-6:
        return
    obj.rotation_euler = direction.normalized().to_track_quat("-Z", "Y").to_euler()
    if roll:
        obj.rotation_euler.rotate_axis("Z", roll)


def target_location(settings):
    if settings.target_object:
        return settings.target_object.matrix_world.translation.copy()
    return bpy.context.scene.cursor.location.copy()


def active_dataset_cameras(scene, settings, include_patch=True):
    collection = bpy.data.collections.get(settings.camera_collection)
    cameras = []
    if collection:
        cameras = [
            obj for obj in collection.objects
            if obj.type == "CAMERA"
            and not obj.get("gs_patch_preview")
            and not obj.get("gs_patch_camera")
        ]
    if not cameras:
        cameras = [
            obj for obj in scene.objects
            if obj.type == "CAMERA"
            and not obj.get("gs_patch_preview")
            and not obj.get("gs_patch_camera")
        ]
    cameras = sorted(cameras, key=lambda item: item.name)
    if include_patch:
        known = {camera.as_pointer() for camera in cameras}
        cameras.extend(
            camera for camera in coverage_patch.final_cameras(scene)
            if camera.as_pointer() not in known
        )
    return cameras


def scientific_realization_mode(settings):
    if (
        getattr(settings, "rig_mode", "") != "PATH"
        or getattr(settings, "path_capture_mode", "LEGACY_PANORAMA_CUBE") != "SCIENTIFIC_THREE_LAYER"
    ):
        return "LEGACY_CUBEMAP_OBJECTS"
    mode = getattr(settings, "scientific_realization_mode", "SCIENTIFIC_CAMERA_OBJECTS")
    if mode not in {
        "LEGACY_CUBEMAP_OBJECTS", "SCIENTIFIC_CAMERA_OBJECTS", "SCIENTIFIC_POSE_SEQUENCE"
    }:
        return "SCIENTIFIC_CAMERA_OBJECTS"
    return mode


def is_pose_sequence_mode(settings):
    return scientific_realization_mode(settings) == "SCIENTIFIC_POSE_SEQUENCE"


def active_pose_sequence(scene, settings, prefer_disk=True):
    if not is_pose_sequence_mode(settings):
        return None
    return pose_sequence.load_sequence(scene, settings, prefer_disk=prefer_disk)


def dataset_export_cameras(scene, settings, patch_only=False, prefer_disk=True):
    sequence = active_pose_sequence(scene, settings, prefer_disk=prefer_disk)
    if sequence is not None:
        return pose_sequence.adapters(sequence, patch_only=patch_only)
    if patch_only:
        return coverage_patch.final_cameras(scene)
    return active_dataset_cameras(scene, settings)


def clear_pose_sequence_debug(scene):
    collection = bpy.data.collections.get("GS_SCIENTIFIC_PATH_DEBUG")
    if collection is None:
        return 0
    removed = 0
    for obj in list(collection.objects):
        if obj.get("gs_pose_sequence_debug"):
            bpy.data.objects.remove(obj, do_unlink=True)
            removed += 1
    return removed


def build_pose_sequence_debug(scene, settings, sequence):
    clear_pose_sequence_debug(scene)
    mode = getattr(settings, "sequence_debug_mode", "OFF")
    if mode == "OFF" or not sequence.frames:
        return []
    collection = bpy.data.collections.get("GS_SCIENTIFIC_PATH_DEBUG")
    if collection is None:
        collection = bpy.data.collections.new("GS_SCIENTIFIC_PATH_DEBUG")
        scene.collection.children.link(collection)
    enabled = [sample for sample in sequence.frames if sample.render_enabled]
    current_id = int(getattr(settings, "sequence_debug_frame", 1))
    current_index = min(range(len(enabled)), key=lambda index: abs(enabled[index].logical_frame_id - current_id))
    if mode == "CURRENT":
        indices = [current_index]
    elif mode == "NEIGHBORHOOD":
        radius = max(0, int(getattr(settings, "sequence_debug_neighbor_count", 2)))
        segment = enabled[current_index].segment_id
        indices = [
            index for index in range(max(0, current_index - radius), min(len(enabled), current_index + radius + 1))
            if enabled[index].segment_id == segment
        ]
    elif mode == "SAMPLED":
        stride = max(1, int(getattr(settings, "sequence_debug_stride", 10)))
        indices = list(range(0, len(enabled), stride))
        if current_index not in indices:
            indices.append(current_index)
            indices.sort()
    else:
        indices = list(range(len(enabled)))
    result = []
    for index in indices:
        sample = enabled[index]
        marker = bpy.data.objects.new(f"GS_Pose_{sample.logical_frame_id:06d}", None)
        marker.empty_display_type = "ARROWS"
        marker.empty_display_size = 0.12
        marker.show_in_front = True
        marker.matrix_world = sample.matrix_world.copy()
        hue = (sample.segment_id * 0.173) % 1.0
        marker.color = (
            0.25 + 0.65 * abs(math.sin(math.tau * hue)),
            0.25 + 0.65 * abs(math.sin(math.tau * (hue + 0.333))),
            0.25 + 0.65 * abs(math.sin(math.tau * (hue + 0.666))),
            1.0,
        )
        marker["gs_scientific_debug"] = True
        marker["gs_pose_sequence_debug"] = True
        marker["gs_logical_frame_id"] = sample.logical_frame_id
        marker["gs_segment_id"] = sample.segment_id
        collection.objects.link(marker)
        result.append(marker)
    capture = bpy.data.objects.get(pose_sequence.CAPTURE_CAMERA_NAME)
    if capture is not None and mode in {"CURRENT", "NEIGHBORHOOD"}:
        capture.matrix_world = enabled[current_index].matrix_world.copy()
        scene.camera = capture
    return result


def dataset_camera_items(scene, settings, cameras=None):
    cameras = list(cameras if cameras is not None else active_dataset_cameras(scene, settings))
    extension = ".png" if settings.image_format == "PNG" else ".jpg"
    items = []
    base_index = 0
    for camera in cameras:
        if camera.get("gs_patch_camera"):
            stem = str(camera.get("gs_dataset_image_stem", ""))
            if not stem:
                stem = safe_name(f"{settings.image_prefix}_patch_{len(items) + 1:04d}")
        else:
            base_index += 1
            stem = safe_name(f"{settings.image_prefix}_{base_index:04d}")
        items.append((camera, stem, stem + extension))
    return items


def create_camera(name, location, target, collection, settings):
    data = bpy.data.cameras.new(name + "_Data")
    configure_camera_data(data, settings)
    obj = bpy.data.objects.new(name, data)
    obj.location = location
    collection.objects.link(obj)
    look_at(obj, target)
    return obj


def create_camera_at(name, location, collection, settings):
    data = bpy.data.cameras.new(name + "_Data")
    configure_camera_data(data, settings)
    obj = bpy.data.objects.new(name, data)
    obj.location = location
    collection.objects.link(obj)
    return obj


def configure_camera_data(data, settings):
    data.lens = settings.focal_length
    data.clip_start = 0.01  # tiny near plane so close geometry isn't clipped into the view
    data.clip_end = settings.ray_distance
    # All training cameras are perspective (PINHOLE). The "Panorama Cube" model is
    # produced by splitting each position into 6 perspective cube faces -> see
    # expand_to_cubemap. Equirectangular / fisheye are removed because standard 3DGS
    # cannot train on them.
    data.type = "PERSP"


CUBE_FACES = (
    ("px", Vector((1.0, 0.0, 0.0)), Vector((0.0, 0.0, 1.0))),
    ("nx", Vector((-1.0, 0.0, 0.0)), Vector((0.0, 0.0, 1.0))),
    ("py", Vector((0.0, 1.0, 0.0)), Vector((0.0, 0.0, 1.0))),
    ("ny", Vector((0.0, -1.0, 0.0)), Vector((0.0, 0.0, 1.0))),
    ("pz", Vector((0.0, 0.0, 1.0)), Vector((0.0, 1.0, 0.0))),
    ("nz", Vector((0.0, 0.0, -1.0)), Vector((0.0, -1.0, 0.0))),
)


def orient_camera(obj, forward, up):
    """Aim a camera so its view direction (local -Z) points along `forward`, with `up`."""
    z = (-forward).normalized()
    x = up.normalized().cross(z)
    if x.length < 1e-6:
        x = Vector((1.0, 0.0, 0.0))
    x.normalize()
    y = z.cross(x)
    rot = Matrix(((x.x, y.x, z.x), (x.y, y.y, z.y), (x.z, y.z, z.z)))
    obj.rotation_euler = rot.to_euler()


def expand_to_cubemap(scene, settings, cameras, collection):
    """Replace each placeholder camera with 6 perspective cube-face cameras (90° FOV)
    at the same position -> full 360° coverage as PINHOLE training views."""
    result = []
    for station_index, cam in enumerate(cameras, 1):
        pos = cam.matrix_world.translation.copy()
        base = cam.name
        source_curve = cam.get("source_curve", "")
        result.extend(_generate_legacy_six_camera_group(
            settings,
            collection,
            pos,
            station_index,
            source_curve,
            name_base=base,
            fallback_used=False,
        ))
        bpy.data.objects.remove(cam, do_unlink=True)
    return result


_ICOSAHEDRON_UNIT_DIRECTIONS = None
_SHELL_CLEARANCE_DIRECTIONS = None
SHELL_DEBUG_COLLECTION = "GS_SPHERICAL_SHELL_DEBUG"
SHELL_DEBUG_MESH = "GS_Spherical_Shell_Icosahedron"


def _generate_icosahedron_unit_directions():
    """Return the cached, deterministic 12-vertex icosahedron order."""
    global _ICOSAHEDRON_UNIT_DIRECTIONS
    if _ICOSAHEDRON_UNIT_DIRECTIONS is None:
        phi = (1.0 + math.sqrt(5.0)) / 2.0
        vertices = (
            (0.0, 1.0, phi),
            (0.0, 1.0, -phi),
            (0.0, -1.0, phi),
            (0.0, -1.0, -phi),
            (1.0, phi, 0.0),
            (1.0, -phi, 0.0),
            (-1.0, phi, 0.0),
            (-1.0, -phi, 0.0),
            (phi, 0.0, 1.0),
            (phi, 0.0, -1.0),
            (-phi, 0.0, 1.0),
            (-phi, 0.0, -1.0),
        )
        _ICOSAHEDRON_UNIT_DIRECTIONS = tuple(Vector(vertex).normalized() for vertex in vertices)
    return _ICOSAHEDRON_UNIT_DIRECTIONS


def _stable_path_forward(sampled, index, tangent):
    """Build the stable horizontal Local Y axis without changing station sampling."""
    try:
        forward = Vector(tangent)
    except Exception:
        forward = Vector((0.0, 0.0, 0.0))
    forward.z = 0.0
    if forward.length >= 1e-8:
        return forward.normalized()

    center = Vector(sampled[index][0])
    source = sampled[index][2]
    previous = None
    following = None
    if index > 0 and sampled[index - 1][2] is source:
        previous = Vector(sampled[index - 1][0])
    if index + 1 < len(sampled) and sampled[index + 1][2] is source:
        following = Vector(sampled[index + 1][0])
    if previous is not None and following is not None:
        forward = following - previous
    elif following is not None:
        forward = following - center
    elif previous is not None:
        forward = center - previous
    forward.z = 0.0
    if forward.length < 1e-8:
        return Vector((0.0, 1.0, 0.0))
    return forward.normalized()


def _path_shell_frame(path_forward):
    up = Vector((0.0, 0.0, 1.0))
    forward = Vector(path_forward)
    forward.z = 0.0
    if forward.length < 1e-8:
        forward = Vector((0.0, 1.0, 0.0))
    else:
        forward.normalize()
    right = forward.cross(up)
    if right.length < 1e-8:
        right = Vector((1.0, 0.0, 0.0))
    else:
        right.normalize()
    return right, forward, up


def _world_shell_directions(path_forward):
    right, forward, up = _path_shell_frame(path_forward)
    directions = tuple(
        (right * local.x + forward * local.y + up * local.z).normalized()
        for local in _generate_icosahedron_unit_directions()
    )
    return directions, (right, forward, up)


def _orient_camera_radially(camera, radial_direction, path_forward):
    """Set a deterministic quaternion with local -Z pointing radially outward."""
    forward = Vector(radial_direction).normalized()
    preferred_up = Vector((0.0, 0.0, 1.0))
    if abs(forward.dot(preferred_up)) > 0.98:
        preferred_up = Vector(path_forward).normalized()
    z_axis = -forward
    x_axis = preferred_up.cross(z_axis)
    if x_axis.length < 1e-8:
        fallback_up = Vector((0.0, 1.0, 0.0)) if abs(forward.y) < 0.98 else Vector((1.0, 0.0, 0.0))
        x_axis = fallback_up.cross(z_axis)
    x_axis.normalize()
    y_axis = z_axis.cross(x_axis).normalized()
    rotation = Matrix((
        (x_axis.x, y_axis.x, z_axis.x),
        (x_axis.y, y_axis.y, z_axis.y),
        (x_axis.z, y_axis.z, z_axis.z),
    )).to_quaternion()
    camera.rotation_mode = "QUATERNION"
    camera.rotation_quaternion = rotation
    actual_forward = rotation @ Vector((0.0, 0.0, -1.0))
    if actual_forward.dot(forward) <= 0.999:
        raise RuntimeError("Failed to orient spherical-shell camera radially outward")


def _shell_clearance_directions():
    global _SHELL_CLEARANCE_DIRECTIONS
    if _SHELL_CLEARANCE_DIRECTIONS is None:
        _SHELL_CLEARANCE_DIRECTIONS = tuple(_dome_directions(30))
    return _SHELL_CLEARANCE_DIRECTIONS


def _scene_units_per_meter(scene):
    scale = float(getattr(scene.unit_settings, "scale_length", 1.0) or 1.0)
    return 1.0 / max(1e-9, scale)


def _shell_position_is_safe(scene, depsgraph, point, clearance):
    search = max(clearance * 6.0, 2.0 * _scene_units_per_meter(scene))
    nearest, _open_direction = _nearest_surface(
        scene, depsgraph, Vector(point), _shell_clearance_directions(), search
    )
    return nearest + 1e-8 >= clearance


def _shell_radius_is_safe(scene, depsgraph, shell_center, directions, radius, clearance):
    for direction in directions:
        position = shell_center + direction * radius
        if not _shell_position_is_safe(scene, depsgraph, position, clearance):
            return False
        if radius > 2e-4:
            hit, _location, _normal, _face, obj, _matrix = scene.ray_cast(
                depsgraph,
                shell_center + direction * 1e-4,
                direction,
                distance=max(0.0, radius - 2e-4),
            )
            if hit and obj and not is_camera_mesh_visual(obj):
                return False
    return True


def _shell_radius_candidates(scene, settings):
    requested_m = max(0.0, float(getattr(settings, "shell_radius", 0.18)))
    minimum_m = max(0.0, float(getattr(settings, "shell_min_radius", 0.06)))
    minimum_m = min(requested_m, minimum_m)
    if getattr(settings, "shell_radius_mode", "CLEARANCE_ADAPTIVE") == "FIXED":
        values_m = [requested_m]
    else:
        values_m = [requested_m * factor for factor in (1.0, 0.85, 0.70, 0.55, 0.40)]
        values_m = [value for value in values_m if value + 1e-9 >= minimum_m]
        if not values_m or abs(values_m[-1] - minimum_m) > 1e-9:
            values_m.append(minimum_m)
    units_per_meter = _scene_units_per_meter(scene)
    result = []
    for value_m in values_m:
        if not result or abs(result[-1][0] - value_m) > 1e-9:
            result.append((value_m, value_m * units_per_meter))
    return result


def _apply_path_camera_metadata(
    camera,
    station_index,
    shell_camera_index,
    shell_center,
    shell_radius,
    radial_direction,
    rig_type,
    fallback_used,
    source_curve,
):
    camera["station_index"] = int(station_index)
    camera["shell_camera_index"] = int(shell_camera_index)
    camera["shell_center"] = [float(value) for value in shell_center]
    camera["shell_radius"] = float(shell_radius)
    camera["radial_direction"] = [float(value) for value in radial_direction]
    camera["rig_type"] = str(rig_type)
    camera["fallback_used"] = bool(fallback_used)
    camera["source_curve"] = str(source_curve or "")


def _generate_legacy_six_camera_group(
    settings,
    collection,
    shell_center,
    station_index,
    source_curve,
    name_base=None,
    fallback_used=False,
):
    """Generate the retained co-located px/nx/py/ny/pz/nz compatibility group."""
    cameras = []
    base = name_base or f"GS_CAM_S{station_index:04d}"
    for face_index, (suffix, forward, up) in enumerate(CUBE_FACES):
        data = bpy.data.cameras.new(f"{base}_{suffix}_Data")
        data.type = "PERSP"
        data.sensor_fit = "HORIZONTAL"
        data.lens = data.sensor_width / 2.0
        data.clip_start = 0.01
        data.clip_end = settings.ray_distance
        camera = bpy.data.objects.new(f"{base}_{suffix}", data)
        camera.location = shell_center
        collection.objects.link(camera)
        orient_camera(camera, forward, up)
        _apply_path_camera_metadata(
            camera,
            station_index,
            face_index,
            shell_center,
            0.0,
            forward,
            "LEGACY_SIX",
            fallback_used,
            source_curve,
        )
        cameras.append(camera)
    return cameras


def _generate_spherical_shell_12_group(
    scene,
    settings,
    collection,
    shell_center,
    path_forward,
    station_index,
    source_curve,
    depsgraph,
):
    """Generate one safe 12-camera shell or apply the configured station failure policy."""
    shell_center = Vector(shell_center)
    directions, frame = _world_shell_directions(path_forward)
    clearance = max(1e-6, float(getattr(settings, "min_clearance", 0.30))) * _scene_units_per_meter(scene)
    actual_radius_m = None
    actual_radius_scene = None
    for candidate_m, candidate_scene in _shell_radius_candidates(scene, settings):
        if _shell_radius_is_safe(
            scene, depsgraph, shell_center, directions, candidate_scene, clearance
        ):
            actual_radius_m = candidate_m
            actual_radius_scene = candidate_scene
            break

    curve_name = getattr(source_curve, "name", "") if source_curve else ""
    if actual_radius_m is None:
        use_fallback = getattr(settings, "shell_failure_policy", "LEGACY_SIX_FALLBACK") == "LEGACY_SIX_FALLBACK"
        if use_fallback and _shell_position_is_safe(scene, depsgraph, shell_center, clearance):
            cameras = _generate_legacy_six_camera_group(
                settings,
                collection,
                shell_center,
                station_index,
                curve_name,
                fallback_used=True,
            )
            return cameras, 0.0, True, False, None
        return [], 0.0, False, True, None

    cameras = []
    for shell_index, direction in enumerate(directions):
        position = shell_center + direction * actual_radius_scene
        camera = create_camera_at(
            f"GS_CAM_S{station_index:04d}_SH{shell_index:02d}",
            position,
            collection,
            settings,
        )
        _orient_camera_radially(camera, direction, path_forward)
        _apply_path_camera_metadata(
            camera,
            station_index,
            shell_index,
            shell_center,
            actual_radius_m,
            direction,
            "SPHERICAL_SHELL_12",
            False,
            curve_name,
        )
        cameras.append(camera)
    return cameras, actual_radius_m, False, False, (shell_center.copy(), frame, actual_radius_scene)


def _shell_debug_mesh():
    mesh = bpy.data.meshes.get(SHELL_DEBUG_MESH)
    if mesh is not None:
        return mesh
    directions = _generate_icosahedron_unit_directions()
    edge_length = min(
        (directions[left] - directions[right]).length
        for left in range(len(directions))
        for right in range(left + 1, len(directions))
    )
    edges = [
        (left, right)
        for left in range(len(directions))
        for right in range(left + 1, len(directions))
        if abs((directions[left] - directions[right]).length - edge_length) < 1e-6
    ]
    mesh = bpy.data.meshes.new(SHELL_DEBUG_MESH)
    mesh.from_pydata([tuple(direction) for direction in directions], edges, [])
    mesh.update()
    mesh["gs_shell_debug_proxy"] = True
    mesh["gs_camera_mesh_visual"] = True
    return mesh


def _clear_shell_debug_proxies():
    removed = 0
    for obj in list(bpy.data.objects):
        if obj.get("gs_shell_debug_proxy"):
            bpy.data.objects.remove(obj, do_unlink=True)
            removed += 1
    return removed


def _create_shell_debug_proxy(scene, station_index, shell_center, frame, radius_scene):
    collection = bpy.data.collections.get(SHELL_DEBUG_COLLECTION)
    if collection is None:
        collection = bpy.data.collections.new(SHELL_DEBUG_COLLECTION)
        scene.collection.children.link(collection)
    proxy = bpy.data.objects.new(f"GS_SHELL_S{station_index:04d}", _shell_debug_mesh())
    collection.objects.link(proxy)
    right, forward, up = frame
    rotation = Matrix((
        (right.x, forward.x, up.x),
        (right.y, forward.y, up.y),
        (right.z, forward.z, up.z),
    )).to_quaternion()
    proxy.location = shell_center
    proxy.rotation_mode = "QUATERNION"
    proxy.rotation_quaternion = rotation
    proxy.scale = (radius_scene, radius_scene, radius_scene)
    proxy.display_type = "WIRE"
    proxy.show_in_front = True
    proxy.hide_render = True
    proxy["gs_shell_debug_proxy"] = True
    proxy["gs_camera_mesh_visual"] = True
    proxy["station_index"] = int(station_index)
    for attribute in ("visible_camera", "visible_diffuse", "visible_glossy", "visible_shadow"):
        if hasattr(proxy, attribute):
            setattr(proxy, attribute, False)
    return proxy


def remove_old_rig(collection):
    _clear_shell_debug_proxies()
    for obj in list(collection.objects):
        if obj.type == "CAMERA":
            remove_camera_mesh_visual(obj)
            bpy.data.objects.remove(obj, do_unlink=True)


def cubic_bezier(p0, h0, h1, p1, t):
    inv = 1.0 - t
    return (inv ** 3) * p0 + 3.0 * (inv ** 2) * t * h0 + 3.0 * inv * (t ** 2) * h1 + (t ** 3) * p1


def curve_polyline_points(path_object, detail):
    if not path_object or path_object.type != "CURVE":
        return []
    detail = max(2, detail)
    points = []
    for spline in path_object.data.splines:
        spline_points = []
        if spline.type == "BEZIER":
            bezier_points = list(spline.bezier_points)
            if len(bezier_points) < 2:
                continue
            segment_count = len(bezier_points) if spline.use_cyclic_u else len(bezier_points) - 1
            for index in range(segment_count):
                a = bezier_points[index]
                b = bezier_points[(index + 1) % len(bezier_points)]
                for step in range(detail):
                    if index > 0 and step == 0:
                        continue
                    t = step / detail
                    spline_points.append(path_object.matrix_world @ cubic_bezier(a.co, a.handle_right, b.handle_left, b.co, t))
            spline_points.append(path_object.matrix_world @ bezier_points[0 if spline.use_cyclic_u else -1].co)
        else:
            raw_points = [path_object.matrix_world @ point.co.xyz for point in spline.points]
            if spline.use_cyclic_u and raw_points:
                raw_points.append(raw_points[0].copy())
            spline_points.extend(raw_points)
        if points and spline_points:
            points.append(spline_points[0])
        points.extend(spline_points)
    return points


def sample_polyline_evenly(points, count):
    if not points:
        return []
    if len(points) == 1 or count <= 1:
        return [(points[0].copy(), Vector((0, 0, -1)))]
    lengths = [0.0]
    for index in range(1, len(points)):
        lengths.append(lengths[-1] + (points[index] - points[index - 1]).length)
    total = lengths[-1]
    if total <= 1e-6:
        return [(points[0].copy(), Vector((0, 0, -1))) for _ in range(count)]
    sampled = []
    segment = 1
    for item in range(count):
        distance = total * item / max(1, count - 1)
        while segment < len(lengths) - 1 and lengths[segment] < distance:
            segment += 1
        start = points[segment - 1]
        end = points[segment]
        span = max(1e-6, lengths[segment] - lengths[segment - 1])
        factor = (distance - lengths[segment - 1]) / span
        location = start.lerp(end, factor)
        tangent = (end - start).normalized() if (end - start).length > 1e-6 else Vector((0, 0, -1))
        sampled.append((location, tangent))
    return sampled


def polyline_length(points):
    if len(points) < 2:
        return 0.0
    return sum((points[index] - points[index - 1]).length for index in range(1, len(points)))


def path_camera_count(settings, points):
    if settings.path_count_mode != "DENSITY":
        return max(1, settings.camera_count)
    length = polyline_length(points)
    if length <= 1e-6:
        return max(1, settings.camera_count)
    return max(1, min(settings.max_path_cameras, int(math.ceil(length * settings.path_camera_density))))


def collection_curve_objects(collection):
    if not collection:
        return []
    seen = set()
    objects = []

    def visit(item):
        for obj in item.objects:
            if obj.type == "CURVE" and obj.name not in seen:
                seen.add(obj.name)
                objects.append(obj)
        for child in item.children:
            visit(child)

    visit(collection)
    return sorted(objects, key=lambda obj: obj.name.lower())


def path_curve_objects(settings):
    if settings.path_collection:
        objects = collection_curve_objects(settings.path_collection)
        if objects:
            return objects
    if settings.path_object and settings.path_object.type == "CURVE":
        return [settings.path_object]
    return []


def path_components(settings):
    components = []
    for path_object in path_curve_objects(settings):
        points = curve_polyline_points(path_object, settings.path_samples_per_segment)
        if not points:
            continue
        components.append({
            "object": path_object,
            "points": points,
            "length": polyline_length(points),
        })
    return components


def distribute_counts_by_length(total_count, components):
    total_count = max(1, int(total_count))
    if not components:
        return []
    if len(components) == 1:
        return [total_count]

    lengths = [max(0.0, item["length"]) for item in components]
    if total_count < len(components):
        counts = [0] * len(components)
        ranked = sorted(range(len(components)), key=lambda index: lengths[index], reverse=True)
        for index in ranked[:total_count]:
            counts[index] = 1
        return counts

    counts = [1] * len(components)
    remaining = total_count - len(components)
    total_length = sum(lengths)
    if total_length <= 1e-6:
        for index in range(remaining):
            counts[index % len(counts)] += 1
        return counts

    exact = [(remaining * length / total_length) for length in lengths]
    floors = [int(math.floor(value)) for value in exact]
    counts = [count + floor for count, floor in zip(counts, floors)]
    left = total_count - sum(counts)
    remainders = sorted(range(len(exact)), key=lambda index: exact[index] - floors[index], reverse=True)
    for index in remainders[:left]:
        counts[index] += 1
    return counts


def _base_path_component_counts(settings, components):
    if settings.path_count_mode != "DENSITY":
        return distribute_counts_by_length(settings.camera_count, components)

    desired = []
    for item in components:
        length = item["length"]
        if length <= 1e-6:
            desired.append(1)
        else:
            desired.append(max(1, int(math.ceil(length * settings.path_camera_density))))
    total = sum(desired)
    cap = max(1, settings.max_path_cameras)
    if total <= cap:
        return desired
    return distribute_counts_by_length(cap, components)


def path_component_counts(settings, components):
    counts = _base_path_component_counts(settings, components)
    minimums = [max(1, int(item["object"].get("gs_detail_min_samples", 1))) for item in components]
    if not any(value > 1 for value in minimums):
        return counts
    if sum(counts) < sum(minimums):
        raise ValueError(f"细部短线需要至少 {sum(minimums)} 个路径采样点；请增加相机数量或路径采样密度/上限")
    result = [max(count, minimum) for count, minimum in zip(counts, minimums)]
    excess = sum(result) - sum(counts)
    while excess > 0:
        index = max(range(len(result)), key=lambda i: result[i] - minimums[i])
        take = min(excess, result[index] - minimums[index])
        result[index] -= take
        excess -= take
    return result


def sample_path_components(settings):
    sampled = []
    components = path_components(settings)
    for item, count in zip(components, path_component_counts(settings, components)):
        if count <= 0:
            continue
        for point, tangent in sample_polyline_evenly(item["points"], count):
            sampled.append((point, tangent, item["object"]))
    return sampled


def _dome_directions(count=30):
    """Evenly distributed directions on a sphere (Fibonacci) for clearance probing."""
    dirs = []
    ga = math.pi * (3.0 - math.sqrt(5.0))
    for i in range(count):
        z = 1.0 - 2.0 * (i + 0.5) / count
        r = math.sqrt(max(0.0, 1.0 - z * z))
        a = i * ga
        dirs.append(Vector((r * math.cos(a), r * math.sin(a), z)))
    return dirs


def _nearest_surface(scene, depsgraph, point, dirs, search):
    """Return (nearest hit distance over all dirs, most-open direction)."""
    nearest = search
    open_dir = Vector((0.0, 0.0, 1.0))
    farthest = -1.0
    for d in dirs:
        hit, loc, _n, _fi, _ob, _mx = scene.ray_cast(depsgraph, point + d * 1e-4, d, distance=search)
        dist = (loc - point).length if hit else search
        if dist < nearest:
            nearest = dist
        if dist > farthest:
            farthest = dist
            open_dir = d
    return nearest, open_dir


def resolve_camera_clipping(scene, settings, cameras, center, keep_rotation, clearance_override=None):
    """Fix cameras that are embedded in / too close to geometry (穿模):
    push each toward the most-open direction until it has min_clearance from any
    surface; drop the few that cannot be freed (truly enclosed)."""
    if not getattr(settings, "avoid_clipping", False) or not cameras:
        return cameras
    depsgraph = bpy.context.evaluated_depsgraph_get()
    dirs = _dome_directions(30)
    clear = max(1e-3, settings.min_clearance if clearance_override is None else clearance_override)
    search = max(clear * 6.0, 2.0)
    kept = []
    for cam in cameras:
        point = cam.matrix_world.translation.copy()
        nearest, open_dir = _nearest_surface(scene, depsgraph, point, dirs, search)
        if nearest >= clear:
            kept.append(cam)
            continue
        moved = point.copy()
        ok = False
        for _ in range(10):
            moved = moved + open_dir * ((clear - nearest) + clear * 0.5)
            nearest, open_dir = _nearest_surface(scene, depsgraph, moved, dirs, search)
            if nearest >= clear:
                ok = True
                break
        if ok:
            cam.location = moved
            if not keep_rotation:
                look_at(cam, center)
            kept.append(cam)
        else:
            bpy.data.objects.remove(cam, do_unlink=True)
    return kept


def scene_mesh_bounds(scene):
    mins = Vector((1e18, 1e18, 1e18))
    maxs = Vector((-1e18, -1e18, -1e18))
    found = False
    for obj in scene.objects:
        if obj.type != "MESH" or is_camera_mesh_visual(obj):
            continue
        try:
            if not obj.visible_get():
                continue
        except Exception:
            pass
        for corner in obj.bound_box:
            w = obj.matrix_world @ Vector(corner)
            mins.x, mins.y, mins.z = min(mins.x, w.x), min(mins.y, w.y), min(mins.z, w.z)
            maxs.x, maxs.y, maxs.z = max(maxs.x, w.x), max(maxs.y, w.y), max(maxs.z, w.z)
            found = True
    return (mins, maxs) if found else None


@dataclass
class FloorplanCell:
    key: tuple
    x: float
    y: float
    floor_z: float
    ceiling_z: float


def _floorplan_horizontal_clearance(scene, depsgraph, point, margin, hdirs):
    nearest = margin
    for d in hdirs:
        hit, loc, _n, _fi, _o, _m = scene.ray_cast(depsgraph, point + d * 1e-4, d, distance=margin)
        if hit:
            nearest = min(nearest, (loc - point).length)
    return nearest


def _floorplan_layer_specs(settings):
    mode = getattr(settings, "floorplan_layer_mode", "THREE")
    if mode == "ONE":
        return [("Mid", "FLOOR", max(0.10, settings.floorplan_mid_height))]
    specs = [
        ("Low", "FLOOR", min(0.50, max(0.10, settings.floorplan_low_height))),
    ]
    if mode in {"THREE", "FOUR"}:
        specs.append(("Mid", "FLOOR", max(0.10, settings.floorplan_mid_height)))
    if mode == "FOUR":
        specs.append(("High", "FLOOR", max(0.10, settings.floorplan_high_height)))
    specs.append(("Top", "FLOOR", max(0.10, settings.floorplan_top_height)))
    return specs


def _floorplan_layer_point(cell, spec):
    label, mode, value = spec
    if mode == "CEILING":
        z = cell.ceiling_z - value
    else:
        z = cell.floor_z + value
    if z <= cell.floor_z + 0.05 or z >= cell.ceiling_z - 0.05:
        return None
    return Vector((cell.x, cell.y, z))


def _floorplan_probe_cells(scene, settings, bounds):
    """Recast-inspired raster pass: sample XY cells, keep only indoor walkable cells
    with floor below, ceiling above, enough headroom and horizontal clearance."""
    mins, maxs = bounds
    depsgraph = bpy.context.evaluated_depsgraph_get()
    density_spacing = 1.0 / max(0.05, getattr(settings, "floorplan_curve_density", 1.6))
    spacing = max(0.05, min(density_spacing, getattr(settings, "floorplan_probe_spacing", 0.6)))
    margin = max(0.03, min(settings.floorplan_wall_margin, getattr(settings, "floorplan_narrow_margin", 0.12)))
    headroom = max(0.2, getattr(settings, "floorplan_min_headroom", 1.0))
    probe_z = mins.z + max(0.05, settings.eye_height)
    hdirs = [Vector((math.cos(i * math.tau / 20.0), math.sin(i * math.tau / 20.0), 0.0)) for i in range(20)]
    nx = max(1, int(math.ceil((maxs.x - mins.x) / spacing)))
    ny = max(1, int(math.ceil((maxs.y - mins.y) / spacing)))
    cells = {}
    floor_dist = max(0.2, (probe_z - mins.z) + 1.0)
    ceiling_dist = max(0.2, (maxs.z - probe_z) + 1.0)
    for iy in range(ny + 1):
        y = min(maxs.y, mins.y + iy * spacing)
        for ix in range(nx + 1):
            x = min(maxs.x, mins.x + ix * spacing)
            probe = Vector((x, y, probe_z))
            floor_hit, floor_loc, *_ = scene.ray_cast(depsgraph, probe, Vector((0, 0, -1)), distance=floor_dist)
            if not floor_hit:
                continue
            ceil_hit, ceil_loc, *_ = scene.ray_cast(depsgraph, probe, Vector((0, 0, 1)), distance=ceiling_dist)
            if not ceil_hit:
                continue
            if ceil_loc.z - floor_loc.z < headroom:
                continue
            base_z = min(max(floor_loc.z + settings.eye_height, floor_loc.z + 0.10), ceil_loc.z - 0.10)
            base = Vector((x, y, base_z))
            if _floorplan_horizontal_clearance(scene, depsgraph, base, margin, hdirs) < margin:
                continue
            cells[(ix, iy)] = FloorplanCell((ix, iy), x, y, floor_loc.z, ceil_loc.z)
    return cells, spacing, margin, hdirs


def _floorplan_neighbor_keys(key, cells):
    ix, iy = key
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nk = (ix + dx, iy + dy)
        if nk not in cells:
            continue
        yield nk


def _floorplan_segment_clear(scene, depsgraph, a, b, margin, hdirs):
    delta = b - a
    dist = delta.length
    if dist <= 1e-6:
        return True
    direction = delta / dist
    hit, _loc, _n, _fi, _o, _m = scene.ray_cast(depsgraph, a + direction * 1e-4, direction, distance=max(0.0, dist - 2e-4))
    if hit:
        return False
    mid = a.lerp(b, 0.5)
    return _floorplan_horizontal_clearance(scene, depsgraph, mid, margin, hdirs) >= margin


def _floorplan_layer_graph(scene, depsgraph, cells, spec, margin, hdirs):
    points = {}
    for key, cell in cells.items():
        point = _floorplan_layer_point(cell, spec)
        if point is None:
            continue
        if _floorplan_horizontal_clearance(scene, depsgraph, point, margin, hdirs) < margin:
            continue
        points[key] = point

    graph = {key: [] for key in points}
    for key, point in points.items():
        for nk in _floorplan_neighbor_keys(key, points):
            if nk <= key:
                continue
            other = points[nk]
            if not _floorplan_segment_clear(scene, depsgraph, point, other, margin, hdirs):
                continue
            cost = (other - point).length
            graph[key].append((nk, cost))
            graph[nk].append((key, cost))
    return points, graph


def _floorplan_components(graph):
    seen = set()
    components = []
    for key in graph:
        if key in seen:
            continue
        stack = [key]
        seen.add(key)
        comp = []
        while stack:
            item = stack.pop()
            comp.append(item)
            for neighbor, _cost in graph[item]:
                if neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
        components.append(comp)
    return components


def _floorplan_seed_location(scene, settings):
    mode = getattr(settings, "floorplan_seed_mode", "CURSOR")
    if mode == "OBJECT" and getattr(settings, "floorplan_seed_object", None):
        return settings.floorplan_seed_object.matrix_world.translation.copy()
    if mode == "CAMERA" and scene.camera:
        return scene.camera.matrix_world.translation.copy()
    return scene.cursor.location.copy()


def _floorplan_filter_components(scene, settings, components, layer_points):
    if not components:
        return components
    mode = getattr(settings, "floorplan_space_mode", "REACHABLE")
    if mode == "ALL":
        return components

    seed = _floorplan_seed_location(scene, settings)
    best_component = None
    best_dist = 1e30
    for component in components:
        for key in component:
            point = layer_points.get(key)
            if point is None:
                continue
            dist = (Vector((point.x, point.y, 0.0)) - Vector((seed.x, seed.y, 0.0))).length
            if dist < best_dist:
                best_dist = dist
                best_component = component
    return [best_component] if best_component else components


def _floorplan_has_edge(graph, a, b):
    return any(neighbor == b for neighbor, _cost in graph.get(a, ()))


def _floorplan_median(values):
    values = sorted(values)
    n = len(values)
    if not n:
        return None
    mid = n // 2
    return values[mid] if n % 2 else (values[mid - 1] + values[mid]) * 0.5


def _floorplan_region_z(points):
    """Single representative floor height for one connected region so every line
    in it stays flat and parallel to the ground. Median is robust to a handful
    of furniture-top or step cells that would otherwise tilt the lines."""
    return _floorplan_median([p.z for p in points])


def _floorplan_flat_graph(graph, points, tolerance):
    """Split scientific source paths at floor-height changes."""
    flat = {key: [] for key in graph}
    for key, neighbors in graph.items():
        point = points[key]
        for neighbor, cost in neighbors:
            if abs(point.z - points[neighbor].z) <= tolerance:
                flat[key].append((neighbor, cost))
    return flat


def _floorplan_cell_width(scene, depsgraph, point, axis, limit):
    """Free width and centre along one axis, found by ray casting to the walls
    on both sides. When a side is open (a doorway), the width is reported as wide
    so junctions are treated as rooms, not corridors."""
    if axis == "X":
        pos, neg = Vector((1.0, 0.0, 0.0)), Vector((-1.0, 0.0, 0.0))
        base = point.x
    else:
        pos, neg = Vector((0.0, 1.0, 0.0)), Vector((0.0, -1.0, 0.0))
        base = point.y
    hit_p, loc_p, *_ = scene.ray_cast(depsgraph, point + pos * 1e-3, pos, distance=limit)
    hit_n, loc_n, *_ = scene.ray_cast(depsgraph, point + neg * 1e-3, neg, distance=limit)
    if not (hit_p and hit_n):
        return limit * 2.0, base
    dp = (loc_p - point).length
    dn = (loc_n - point).length
    return dp + dn, base + (dp - dn) * 0.5


def _floorplan_classify_cells(scene, depsgraph, keys, layer_points, region_z, corridor_width, spacing):
    """Decide, per cell, which axis lines may pass through and at which centred
    coordinate. Wide cells host a full grid; cells narrow in one direction host a
    single centred spine, so corridors and small rooms collapse to a tidy centre
    line instead of a ragged ladder of off-centre grid segments."""
    threshold = max(0.3, corridor_width)
    limit = threshold + 1.0
    info = {}
    # phase 1: raw free width and centre on each axis
    for key in keys:
        probe = layer_points[key].copy()
        probe.z = region_z
        wx, cx = _floorplan_cell_width(scene, depsgraph, probe, "X", limit)
        wy, cy = _floorplan_cell_width(scene, depsgraph, probe, "Y", limit)
        info[key] = {
            "cx": cx, "cy": cy,
            "narrow_x": wx <= threshold,
            "narrow_y": wy <= threshold,
            "wx": wx, "wy": wy,
        }
    # phase 2: bridge short openings (doorways) so a corridor spine stays whole
    max_gap = max(1, int(math.ceil(1.1 / max(0.05, spacing))))
    _floorplan_close_narrow_gaps(info, "narrow_x", max_gap)  # vertical corridors (along Y)
    _floorplan_close_narrow_gaps(info, "narrow_y", max_gap)  # horizontal corridors (along X)
    # phase 3: pick the line type per cell
    for data in info.values():
        narrow_x, narrow_y = data["narrow_x"], data["narrow_y"]
        if narrow_x and narrow_y:
            # tight both ways (small room / closet): one spine along the longer axis
            data["allow_x"], data["allow_y"] = (True, False) if data["wx"] >= data["wy"] else (False, True)
        elif narrow_x:
            data["allow_x"], data["allow_y"] = False, True   # vertical corridor -> spine along Y
        elif narrow_y:
            data["allow_x"], data["allow_y"] = True, False   # horizontal corridor -> spine along X
        else:
            data["allow_x"], data["allow_y"] = True, True    # open room -> full grid
    return info


def _floorplan_close_narrow_gaps(info, flag, max_gap):
    """Morphological closing of a narrow-corridor mask along the corridor axis.
    A short stretch (<= max_gap cells) of non-narrow cells flanked by narrow cells
    is a doorway interrupting a corridor, so it is marked narrow too. The run
    builder still honours walls/edges, so this never bridges a real obstacle."""
    # narrow_x corridors run along Y (fixed ix, vary iy); narrow_y along X
    fixed, vary = (0, 1) if flag == "narrow_x" else (1, 0)
    lines = {}
    for key in info:
        lines.setdefault(key[fixed], []).append(key)
    for _line, keys in lines.items():
        keys.sort(key=lambda k: k[vary])
        narrow = [k[vary] for k in keys if info[k][flag]]
        if len(narrow) < 2:
            continue
        narrow_set = set(narrow)
        by_pos = {k[vary]: k for k in keys}
        lo, hi = min(narrow), max(narrow)
        pos = lo
        while pos <= hi:
            if pos in narrow_set:
                pos += 1
                continue
            run_end = pos
            while run_end <= hi and run_end not in narrow_set:
                run_end += 1
            gap = run_end - pos  # contiguous non-narrow positions [pos, run_end)
            if gap <= max_gap:
                for p in range(pos, run_end):
                    if p in by_pos:
                        info[by_pos[p]][flag] = True
            pos = run_end


def _floorplan_typed_runs(keys_set, graph, info, axis):
    """Contiguous, edge-connected runs of cells along one axis, split wherever the
    line type changes (open-room grid vs centred corridor) so every emitted run is
    a single straight, flat line."""
    allow_key = "allow_x" if axis == "X" else "allow_y"
    perp_flag = "narrow_y" if axis == "X" else "narrow_x"
    grouped = {}
    for key in keys_set:
        if not info[key][allow_key]:
            continue
        group_key = key[1] if axis == "X" else key[0]
        grouped.setdefault(group_key, []).append(key)

    runs = []
    for _group_key, keys in sorted(grouped.items()):
        keys = sorted(keys, key=lambda item: item[0] if axis == "X" else item[1])
        current = []
        previous = None
        current_flag = None
        for key in keys:
            flag = info[key][perp_flag]
            if previous is None:
                current = [key]
                current_flag = flag
            else:
                expected = (previous[0] + 1, previous[1]) if axis == "X" else (previous[0], previous[1] + 1)
                if key == expected and flag == current_flag and _floorplan_has_edge(graph, previous, key):
                    current.append(key)
                else:
                    if current:
                        runs.append((current, current_flag))
                    current = [key]
                    current_flag = flag
            previous = key
        if current:
            runs.append((current, current_flag))
    return runs


def _clear_auto_floorplan_paths(scene):
    for obj in list(bpy.data.objects):
        if obj.name == "GS_FloorPath" or obj.name.startswith("GS_FloorPath_"):
            bpy.data.objects.remove(obj, do_unlink=True)
    old = bpy.data.collections.get("GS_FloorPath_Auto")
    if old:
        for obj in list(old.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
        for parent in list(scene.collection.children):
            if parent == old:
                scene.collection.children.unlink(old)
        bpy.data.collections.remove(old)


def _create_floorplan_curve(collection, name, points):
    if len(points) < 2:
        return None
    for a, b in zip(points, points[1:]):
        dz = abs(b.z - a.z)
        if dz > 1e-4:
            return None
    curve_data = bpy.data.curves.new(name, "CURVE")
    curve_data.dimensions = "3D"
    spline = curve_data.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for i, p in enumerate(points):
        spline.points[i].co = (p.x, p.y, p.z, 1.0)
    curve_obj = bpy.data.objects.new(name, curve_data)
    collection.objects.link(curve_obj)
    return curve_obj


def _floorplan_line_points(axis, perp, z, lo, hi, step):
    """Evenly spaced vertices along one straight, axis-aligned, flat segment."""
    span = hi - lo
    count = max(1, int(round(span / max(0.05, step))))
    points = []
    for i in range(count + 1):
        value = lo + span * i / count
        if axis == "X":
            points.append(Vector((value, perp, z)))
        else:
            points.append(Vector((perp, value, z)))
    return points


def _floorplan_segment_endpoint(axis, perp, z, value):
    return Vector((value, perp, z)) if axis == "X" else Vector((perp, value, z))


def _floorplan_merge_segments(segments, scene, depsgraph, margin, hdirs, bridge_gap):
    """Merge collinear segments that share an axis, perpendicular coordinate and
    height into single continuous lines.

    - Overlapping or touching ranges always merge (kills duplicate corridor
      columns that all snap to the same centre).
    - A gap up to ``bridge_gap`` is also merged, but only when the straight bridge
      is actually clear at the line's height. So a line flies over a low table or
      a slim object as one continuous curve, yet never jumps a wall or a tall
      cabinet (the ray hits those) and never leaps a wide blocked span. This is
      what removes the little disconnected stubs around furniture."""
    groups = {}
    for seg in segments:
        gk = (seg["axis"], round(seg["perp"], 2), round(seg["z"], 3))
        groups.setdefault(gk, []).append(seg)
    merged = []
    for (axis, _perp, _z), segs in groups.items():
        segs.sort(key=lambda s: s["lo"])
        current = None
        for seg in segs:
            if current is None:
                current = dict(seg)
                continue
            gap = seg["lo"] - current["hi"]
            if gap <= 1e-6:                                  # overlapping / touching
                current["hi"] = max(current["hi"], seg["hi"])
                current["narrow"] = current["narrow"] and seg["narrow"]
                continue
            if gap <= bridge_gap:
                a = _floorplan_segment_endpoint(axis, current["perp"], current["z"], current["hi"])
                b = _floorplan_segment_endpoint(axis, current["perp"], current["z"], seg["lo"])
                if _floorplan_segment_clear(scene, depsgraph, a, b, margin, hdirs):
                    current["hi"] = max(current["hi"], seg["hi"])
                    current["narrow"] = current["narrow"] and seg["narrow"]
                    continue
            merged.append(current)
            current = dict(seg)
        if current:
            merged.append(current)
    return merged


def _floorplan_pca_axes(keys, layer_points):
    points = [layer_points[key] for key in keys]
    center = Vector((
        sum(point.x for point in points) / len(points),
        sum(point.y for point in points) / len(points),
        0.0,
    ))
    xx = sum((point.x - center.x) ** 2 for point in points) / len(points)
    yy = sum((point.y - center.y) ** 2 for point in points) / len(points)
    xy = sum((point.x - center.x) * (point.y - center.y) for point in points) / len(points)
    angle = 0.5 * math.atan2(2.0 * xy, xx - yy) if abs(xy) + abs(xx - yy) > 1e-12 else 0.0
    main = Vector((math.cos(angle), math.sin(angle), 0.0))
    cross = Vector((-main.y, main.x, 0.0))
    return center, main, cross


def _floorplan_axis_values(low, high, step):
    if high <= low + 1e-6:
        return [(low + high) * 0.5]
    count = max(1, int(math.floor((high - low) / max(0.05, step))))
    start = (low + high - count * step) * 0.5
    return [start + index * step for index in range(count + 1)]


def _floorplan_oriented_runs(
    scene, depsgraph, kept, cells, layer_points, center, along, across,
    fixed_value, along_low, along_high, sample_step, margin, hdirs, spacing, region_z,
):
    if along_high <= along_low + 1e-6:
        return []
    origin_x = _floorplan_median([cell.x - key[0] * spacing for key, cell in cells.items()])
    origin_y = _floorplan_median([cell.y - key[1] * spacing for key, cell in cells.items()])
    count = max(1, int(math.ceil((along_high - along_low) / max(0.05, sample_step))))
    runs = []
    current = []
    for index in range(count + 1):
        value = along_low + (along_high - along_low) * index / count
        point = center + along * value + across * fixed_value
        key = (int(round((point.x - origin_x) / spacing)), int(round((point.y - origin_y) / spacing)))
        cell_point = layer_points.get(key)
        valid = key in kept and cell_point is not None
        if valid:
            distance_xy = math.hypot(cell_point.x - point.x, cell_point.y - point.y)
            valid = distance_xy <= spacing * 0.85
        candidate = Vector((point.x, point.y, region_z))
        if valid and current:
            valid = _floorplan_segment_clear(scene, depsgraph, current[-1], candidate, margin, hdirs)
        if valid:
            current.append(candidate)
        else:
            if len(current) >= 2:
                runs.append(current)
            current = []
    if len(current) >= 2:
        runs.append(current)
    return runs


def _build_scientific_floorplan_network(scene, settings, cells, spacing, margin, hdirs):
    depsgraph = bpy.context.evaluated_depsgraph_get()
    base_spec = ("Base", "FLOOR", max(0.10, settings.eye_height))
    layer_points, graph = _floorplan_layer_graph(scene, depsgraph, cells, base_spec, margin, hdirs)
    scale = float(getattr(scene.unit_settings, "scale_length", 1.0) or 1.0)
    graph = _floorplan_flat_graph(graph, layer_points, 0.12 / max(1e-9, scale))
    components = sorted(_floorplan_components(graph), key=len, reverse=True)
    components = _floorplan_filter_components(scene, settings, components, layer_points)
    if not components:
        return None

    _clear_auto_floorplan_paths(scene)
    collection = bpy.data.collections.new("GS_FloorPath_Auto")
    scene.collection.children.link(collection)
    line_spacing = max(0.10, settings.floorplan_spacing / max(1e-9, scale))
    sample_step = min(spacing, line_spacing * 0.5)
    corridor_width = getattr(settings, "floorplan_corridor_center_width", 1.6) / max(1e-9, scale)
    bridge_gap = max(0.0, getattr(settings, "floorplan_bridge_gap", 2.5) / max(1e-9, scale))
    min_line = max(0.0, getattr(settings, "floorplan_min_line", 1.0) / max(1e-9, scale))
    min_region = max(1, getattr(settings, "floorplan_min_region_cells", 1))
    objects = []
    total_points = 0
    region_index = 1

    for component in components:
        keys = [key for key in component if key in layer_points]
        if len(keys) < min_region:
            continue
        region_z = _floorplan_region_z([layer_points[key] for key in keys])
        if region_z is None:
            continue
        kept = {key for key in keys if abs(layer_points[key].z - region_z) <= 0.12 / max(1e-9, scale)}
        if len(kept) < min_region:
            continue
        info = _floorplan_classify_cells(
            scene, depsgraph, kept, layer_points, region_z, corridor_width, spacing
        )
        emitted = []

        corridor_segments = []
        for axis in ("X", "Y"):
            for run_keys, is_narrow in _floorplan_typed_runs(kept, graph, info, axis):
                if not is_narrow:
                    continue
                if axis == "X":
                    along_values = [layer_points[key].x for key in run_keys]
                    perpendicular = _floorplan_median([info[key]["cy"] for key in run_keys])
                else:
                    along_values = [layer_points[key].y for key in run_keys]
                    perpendicular = _floorplan_median([info[key]["cx"] for key in run_keys])
                corridor_segments.append({
                    "axis": axis, "perp": perpendicular, "z": region_z,
                    "lo": min(along_values), "hi": max(along_values), "narrow": True,
                })
        for segment in _floorplan_merge_segments(
            corridor_segments, scene, depsgraph, margin, hdirs, bridge_gap
        ):
            points = _floorplan_line_points(
                segment["axis"], segment["perp"], region_z, segment["lo"], segment["hi"], sample_step
            )
            if len(points) >= 2:
                emitted.append((f"Corridor_{segment['axis']}", points))

        open_keys = {
            key for key in kept
            if not info[key]["narrow_x"] and not info[key]["narrow_y"]
        }
        if open_keys:
            center, main_axis, cross_axis = _floorplan_pca_axes(open_keys, layer_points)
            main_values = [(layer_points[key] - center).dot(main_axis) for key in open_keys]
            cross_values = [(layer_points[key] - center).dot(cross_axis) for key in open_keys]
            main_low, main_high = min(main_values), max(main_values)
            cross_low, cross_high = min(cross_values), max(cross_values)
            for fixed in _floorplan_axis_values(cross_low, cross_high, line_spacing):
                for points in _floorplan_oriented_runs(
                    scene, depsgraph, open_keys, cells, layer_points, center, main_axis, cross_axis,
                    fixed, main_low, main_high, sample_step, margin, hdirs, spacing, region_z,
                ):
                    if (points[-1] - points[0]).length >= min_line:
                        emitted.append(("PCA_Main", points))
            for fixed in _floorplan_axis_values(main_low, main_high, line_spacing * 2.0):
                for points in _floorplan_oriented_runs(
                    scene, depsgraph, open_keys, cells, layer_points, center, cross_axis, main_axis,
                    fixed, cross_low, cross_high, sample_step, margin, hdirs, spacing, region_z,
                ):
                    if (points[-1] - points[0]).length >= min_line:
                        emitted.append(("PCA_Cross", points))

        seen = set()
        line_index = 1
        for axis_name, points in emitted:
            signature = (
                tuple(round(value, 3) for value in points[0]),
                tuple(round(value, 3) for value in points[-1]),
            )
            signature = tuple(sorted(signature))
            if signature in seen:
                continue
            seen.add(signature)
            name = f"GS_FloorPath_SCI_{region_index:02d}_{line_index:03d}"
            curve_obj = _create_floorplan_curve(collection, name, points)
            if curve_obj is None:
                continue
            curve_obj["gs_floorplan_layer"] = "Base"
            curve_obj["gs_floorplan_region"] = region_index
            curve_obj["gs_floorplan_axis"] = axis_name
            objects.append(curve_obj)
            total_points += len(points)
            line_index += 1
        region_index += 1

    if not objects:
        return None
    return {"primary": objects[0], "collection": collection, "objects": objects, "points": total_points}


def build_floorplan_path(scene, settings):
    """Build indoor, ground-parallel GS capture lines (Recast-inspired).

    Pipeline: rasterize walkable indoor cells (floor/ceiling/clearance tests),
    split them into connected regions, then per region and per layer:
      1. pin one flat height (median floor) so every line is parallel to the
         ground, dropping furniture-top / step cells that deviate too much;
      2. classify each cell by free width -> open rooms get a full grid, narrow
         corridors and small rooms get a single centred spine;
      3. build straight axis-aligned runs and merge overlapping ones so each tidy
         line is a single continuous curve (no break at crossings, no duplicates).
    """
    if getattr(settings, "floorplan_method", "CONTOUR") == "CONTOUR":
        wm = getattr(bpy.context, "window_manager", None)
        if wm:
            wm.progress_begin(0, 100)
        def contour_progress(stage, current, total):
            if wm:
                wm.progress_update(100 * current / max(1, total))
        try:
            return contour_blender.generate(scene, settings, progress=contour_progress)
        finally:
            if wm:
                wm.progress_end()
    bounds = scene_mesh_bounds(scene)
    if not bounds:
        return None
    depsgraph = bpy.context.evaluated_depsgraph_get()
    cells, spacing, margin, hdirs = _floorplan_probe_cells(scene, settings, bounds)
    if not cells:
        return None

    if getattr(settings, "path_capture_mode", "LEGACY_PANORAMA_CUBE") == "SCIENTIFIC_THREE_LAYER":
        return _build_scientific_floorplan_network(scene, settings, cells, spacing, margin, hdirs)

    _clear_auto_floorplan_paths(scene)
    collection = bpy.data.collections.new("GS_FloorPath_Auto")
    scene.collection.children.link(collection)
    layer_specs = _floorplan_layer_specs(settings)
    min_region = max(1, getattr(settings, "floorplan_min_region_cells", 1))
    corridor_width = getattr(settings, "floorplan_corridor_center_width", 1.6)
    # gaps up to this length are bridged into one continuous line when the straight
    # span is clear at the line height (flies over low furniture, not through walls).
    bridge_gap = max(0.0, getattr(settings, "floorplan_bridge_gap", 2.5))
    # wide-room grid fragments shorter than this are dropped as confetti; centred
    # corridor / small-room spines are kept whatever their length.
    min_line = max(0.0, getattr(settings, "floorplan_min_line", 1.0))
    # cells whose floor sits more than this above/below the region floor (furniture
    # tops, half-steps) are dropped so lines stay flat and avoid those surfaces.
    z_tol = 0.12
    objects = []
    total_points = 0
    for layer_index, spec in enumerate(layer_specs, 1):
        label = spec[0]
        layer_points, graph = _floorplan_layer_graph(scene, depsgraph, cells, spec, margin, hdirs)
        components = sorted(_floorplan_components(graph), key=len, reverse=True)
        components = _floorplan_filter_components(scene, settings, components, layer_points)
        region_index = 1
        for component in components:
            if len(component) < min_region:
                continue
            comp_keys = [k for k in component if k in layer_points]
            if not comp_keys:
                continue
            # 1) one flat height per region; drop furniture-top / step outliers
            region_z = _floorplan_region_z([layer_points[k] for k in comp_keys])
            if region_z is None:
                continue
            kept = {k for k in comp_keys if abs(layer_points[k].z - region_z) <= z_tol}
            if len(kept) < min_region:
                continue
            # 2) classify cells: wide -> grid, narrow -> centred spine
            info = _floorplan_classify_cells(scene, depsgraph, kept, layer_points, region_z, corridor_width, spacing)
            # 3) build straight runs per axis, collect as flat segments
            segments = []
            for axis in ("X", "Y"):
                for run_keys, perp_narrow in _floorplan_typed_runs(kept, graph, info, axis):
                    if axis == "X":
                        along = [layer_points[k].x for k in run_keys]
                        perp = (_floorplan_median([info[k]["cy"] for k in run_keys])
                                if perp_narrow else layer_points[run_keys[0]].y)
                    else:
                        along = [layer_points[k].y for k in run_keys]
                        perp = (_floorplan_median([info[k]["cx"] for k in run_keys])
                                if perp_narrow else layer_points[run_keys[0]].x)
                    lo, hi = min(along), max(along)
                    if hi - lo < 1e-6:
                        # isolated cell on this axis -> short centred stub
                        half = max(0.05, spacing * 0.35)
                        lo, hi = lo - half, hi + half
                    segments.append({"axis": axis, "perp": perp, "z": region_z,
                                     "lo": lo, "hi": hi, "narrow": perp_narrow})
            # 4) bridge gaps over low furniture, merge duplicates, drop confetti
            line_index = 1
            for seg in _floorplan_merge_segments(segments, scene, depsgraph, margin, hdirs, bridge_gap):
                length = seg["hi"] - seg["lo"]
                floor_len = spacing * 0.9 if seg["narrow"] else min_line
                if length < floor_len:
                    continue
                points = _floorplan_line_points(seg["axis"], seg["perp"], seg["z"], seg["lo"], seg["hi"], spacing)
                if len(points) < 2:
                    continue
                obj_name = f"GS_FloorPath_{layer_index:02d}_{label}_{region_index:02d}_{line_index:03d}"
                curve_obj = _create_floorplan_curve(collection, obj_name, points)
                if curve_obj is None:
                    continue
                curve_obj["gs_floorplan_layer"] = label
                curve_obj["gs_floorplan_region"] = region_index
                curve_obj["gs_floorplan_axis"] = seg["axis"]
                objects.append(curve_obj)
                total_points += len(points)
                line_index += 1
            region_index += 1
    if not objects:
        return None
    return {
        "primary": objects[0],
        "collection": collection,
        "objects": objects,
        "points": total_points,
    }


_LAST_CAMERA_PLANNING = {}
_LAST_PATH_CAMERA_RIG = {}
PATH_CAMERA_RIG_SCENE_KEY = "_gs_path_camera_rig"


def _clear_path_camera_rig_report(scene):
    _LAST_PATH_CAMERA_RIG.pop(scene.as_pointer(), None)
    if PATH_CAMERA_RIG_SCENE_KEY in scene:
        del scene[PATH_CAMERA_RIG_SCENE_KEY]


def _store_path_camera_rig_report(scene, data):
    data = dict(data)
    _LAST_PATH_CAMERA_RIG[scene.as_pointer()] = data
    scene[PATH_CAMERA_RIG_SCENE_KEY] = json.dumps(data, sort_keys=True)
    return data


def _path_camera_rig_report(scene):
    data = _LAST_PATH_CAMERA_RIG.get(scene.as_pointer())
    if data is not None:
        return dict(data)
    try:
        return dict(json.loads(scene.get(PATH_CAMERA_RIG_SCENE_KEY, "{}")))
    except (TypeError, ValueError):
        return {}


def _duplicate_camera_origin_count(scene, cameras):
    tolerance = 0.001 * _scene_units_per_meter(scene)
    origins = {
        tuple(int(round(axis / tolerance)) for axis in camera.location)
        for camera in cameras
    }
    return max(0, len(cameras) - len(origins))


def _remove_camera(camera):
    data = getattr(camera, "data", None)
    remove_camera_mesh_visual(camera)
    bpy.data.objects.remove(camera, do_unlink=True)
    if data is not None and data.users == 0:
        bpy.data.cameras.remove(data)


def _apply_pose_metadata(camera, sample, candidate=None):
    camera["gs_scientific_layer"] = sample.layer_type
    camera["gs_scientific_kind"] = sample.sample_type.lower()
    camera["gs_scientific_candidate_id"] = sample.candidate_id
    camera["gs_scientific_provider"] = sample.provider_type
    camera["gs_scientific_region"] = sample.region_id
    camera["gs_scientific_distance_band"] = sample.near_field_class
    if candidate is not None:
        camera["gs_dominant_near_surface_ratio"] = candidate.dominant_near_surface_ratio
        camera["gs_near_field_overlap"] = candidate.near_field_average_overlap


def _realize_scientific_cameras(sequence, plan, collection, settings):
    by_id = {candidate.candidate_id: candidate for candidate in plan["selected"]}
    cameras = []
    for index, sample in enumerate(sequence.frames, 1):
        camera = create_camera_at(f"GS_Camera_{index:04d}", sample.matrix_world.translation, collection, settings)
        camera.matrix_world = sample.matrix_world.copy()
        _apply_pose_metadata(camera, sample, by_id.get(sample.candidate_id))
        cameras.append(camera)
    return cameras


def _filter_plan_to_realized_cameras(plan, cameras):
    old = list(plan["selected"])
    old_index = {candidate.candidate_id: index for index, candidate in enumerate(old)}
    surviving_ids = {
        str(camera.get("gs_scientific_candidate_id", "")) for camera in cameras
    }
    selected = [candidate for candidate in old if candidate.candidate_id in surviving_ids]
    new_index = {candidate.candidate_id: index for index, candidate in enumerate(selected)}
    edges = []
    for left, right, ratio in plan.get("edges", ()):
        if left >= len(old) or right >= len(old):
            continue
        left_id = old[left].candidate_id
        right_id = old[right].candidate_id
        if left_id in new_index and right_id in new_index:
            edges.append((new_index[left_id], new_index[right_id], ratio))
    plan["selected"] = selected
    plan["edges"] = edges
    return old_index


def _capture_camera(collection, settings, sequence, data=None):
    camera = bpy.data.objects.get(pose_sequence.CAPTURE_CAMERA_NAME)
    if camera is not None and camera.type != "CAMERA":
        bpy.data.objects.remove(camera, do_unlink=True)
        camera = None
    if camera is None:
        data = data or bpy.data.cameras.new(pose_sequence.CAPTURE_CAMERA_NAME + "_Data")
        configure_camera_data(data, settings)
        camera = bpy.data.objects.new(pose_sequence.CAPTURE_CAMERA_NAME, data)
        collection.objects.link(camera)
    elif collection not in camera.users_collection:
        collection.objects.link(camera)
    configure_camera_data(camera.data, settings)
    camera["gs_pose_sequence_capture"] = True
    camera["gs_scientific_realization"] = "SCIENTIFIC_POSE_SEQUENCE"
    if sequence.frames:
        camera.matrix_world = sequence.frames[0].matrix_world.copy()
    return camera


def _apply_scientific_plan(scene, settings, collection, plan, replace_old=False, components=()):
    if replace_old:
        remove_old_rig(collection)
    prototype_data = bpy.data.cameras.new("GS_Scientific_Shared_Intrinsics")
    configure_camera_data(prototype_data, settings)
    matrices = {
        candidate.candidate_id: scientific_planner.camera_matrix(
            candidate.position, candidate.yaw, candidate.pitch
        )
        for candidate in plan["selected"]
    }
    preliminary = pose_sequence.build_sequence(
        scene, settings, plan, matrices, prototype_data, components=components, ordered=True
    )
    cameras = _realize_scientific_cameras(preliminary, plan, collection, settings)
    planned_positions = {
        str(camera.get("gs_scientific_candidate_id", camera.name)): camera.location.copy()
        for camera in cameras
    }
    planned_count = len(cameras)
    scale = float(getattr(scene.unit_settings, "scale_length", 1.0) or 1.0)
    clearance = settings.scientific_camera_clearance / max(1e-9, scale)
    cameras = resolve_camera_clipping(
        scene, settings, cameras, target_location(settings), True, clearance_override=clearance
    )
    stats = plan["stats"]
    dropped_count = planned_count - len(cameras)
    moved_candidate_ids = {
        str(camera.get("gs_scientific_candidate_id", camera.name))
        for camera in cameras
        if str(camera.get("gs_scientific_candidate_id", camera.name)) in planned_positions
        and (
            camera.location
            - planned_positions[str(camera.get("gs_scientific_candidate_id", camera.name))]
        ).length > 1e-6
    }
    adjusted_count = len(moved_candidate_ids)
    stats["post_plan_dropped_camera_count"] = dropped_count
    stats["post_plan_adjusted_camera_count"] = adjusted_count
    if dropped_count:
        stats["unsafe_candidate_count"] = stats.get("unsafe_candidate_count", 0) + dropped_count
        reasons = stats.setdefault("unsafe_candidate_reasons", {})
        reasons["post_plan_clearance_failed"] = dropped_count
    if bool(getattr(settings, "scientific_post_clipping_recast", True)):
        # Recast consumes the realized matrices and updates the selected candidate set.
        # PoseSamples are rebuilt immediately afterwards and remain authoritative.
        recast_stats = scientific_planner.recast_final_cameras(
            plan,
            cameras,
            affected_candidate_ids=moved_candidate_ids,
        )
        near_field_removed_ids = set(recast_stats.get("near_field_removed_candidate_ids", ()))
        if near_field_removed_ids:
            kept_cameras = []
            for camera in cameras:
                candidate_id = str(camera.get("gs_scientific_candidate_id", ""))
                if candidate_id not in near_field_removed_ids:
                    kept_cameras.append(camera)
                    continue
                _remove_camera(camera)
            cameras = kept_cameras
        selected_by_id = {
            candidate.candidate_id: candidate for candidate in plan["selected"]
        }
        for camera in cameras:
            candidate = selected_by_id.get(str(camera.get("gs_scientific_candidate_id", "")))
            if candidate is None:
                continue
            camera["gs_scientific_distance_band"] = candidate.distance_band
            camera["gs_dominant_near_surface_ratio"] = candidate.dominant_near_surface_ratio
            camera["gs_near_field_overlap"] = candidate.near_field_average_overlap
    else:
        _filter_plan_to_realized_cameras(plan, cameras)

    matrices = {
        str(camera.get("gs_scientific_candidate_id", camera.name)): camera.matrix_world.copy()
        for camera in cameras
    }
    sequence = pose_sequence.build_sequence(
        scene, settings, plan, matrices, prototype_data, components=components, ordered=True
    )
    by_id = {
        str(camera.get("gs_scientific_candidate_id", camera.name)): camera
        for camera in cameras
    }
    cameras = [by_id[sample.candidate_id] for sample in sequence.frames if sample.candidate_id in by_id]
    for index, camera in enumerate(cameras, 1):
        camera.name = f"GS_Camera_{index:04d}"
        camera.data.name = f"GS_Camera_{index:04d}_Data"

    realization = scientific_realization_mode(settings)
    if realization == "SCIENTIFIC_POSE_SEQUENCE":
        for camera in list(cameras):
            _remove_camera(camera)
        capture = _capture_camera(collection, settings, sequence, data=prototype_data)
        prototype_data = None
        cameras = [capture]
        pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
        if bool(getattr(settings, "sequence_create_preview_keyframes", False)):
            pose_sequence.create_preview_keyframes(scene, capture, sequence)
            pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
    else:
        pose_sequence.clear_sequence(scene, settings, remove_disk=True)

    if prototype_data is not None and prototype_data.users == 0:
        bpy.data.cameras.remove(prototype_data)

    report_cameras = pose_sequence.adapters(sequence) if realization == "SCIENTIFIC_POSE_SEQUENCE" else cameras
    if bool(getattr(settings, "scientific_validate_training_consistency", True)):
        stats["training_consistency"] = training_consistency_report(scene, settings, report_cameras)
    stats["realization_mode"] = realization
    stats["blender_camera_object_count"] = len(cameras)
    stats["planned_pose_count"] = len(sequence.frames)
    stats["final_training_frame_count"] = len(sequence.frames)
    stats["final_camera_count"] = len(sequence.frames)
    tolerance = 0.001 / max(1e-9, scale)
    origins = {
        tuple(int(round(axis / tolerance)) for axis in sample.matrix_world.translation)
        for sample in sequence.frames
    }
    stats["duplicated_origin_count"] = len(sequence.frames) - len(origins)
    if realization == "SCIENTIFIC_POSE_SEQUENCE":
        stats.update(pose_sequence.sequence_report(sequence))
    _LAST_CAMERA_PLANNING[scene.as_pointer()] = stats
    if settings.scientific_show_debug:
        scientific_planner.build_debug_display(scene, settings, plan, report_cameras)
    else:
        scientific_planner.clear_debug_display(scene)
    sync_camera_mesh_visuals(scene, settings, cameras)
    settings.scientific_planning_progress = 1.0
    if realization == "SCIENTIFIC_POSE_SEQUENCE":
        settings.scientific_planning_status = (
            f"科学位姿序列规划完成：{len(sequence.frames)} 个姿态，1 个 Blender 相机对象"
        )
    else:
        settings.scientific_planning_status = f"科学规划完成：{len(cameras)} 台相机"
    return cameras


def create_scientific_path_rig(scene, settings, collection):
    scientific_planner.clear_debug_display(scene)
    origin_mode = getattr(settings, "scientific_origin_mode", "MANUAL_CURVE")
    components = path_components(settings)
    if origin_mode == "AUTO_GRID_PATH" and not components:
        generated = build_floorplan_path(scene, settings)
        if generated:
            settings.path_object = generated["primary"]
            settings.path_collection = generated["collection"]
            components = path_components(settings)
    if origin_mode in {"MANUAL_CURVE", "AUTO_GRID_PATH"} and not components:
        return [], None
    component_counts = path_component_counts(settings, components)
    wm = getattr(bpy.context, "window_manager", None)
    settings.scientific_cancel_requested = False
    original_frame = scene.frame_current
    original_subframe = scene.frame_subframe
    if is_pose_sequence_mode(settings):
        scene.frame_set(int(getattr(settings, "sequence_source_scene_frame", original_frame)))
    if wm:
        wm.progress_begin(0, 1000)

    def progress(stage, current, total):
        fraction = _scientific_progress_fraction(stage, current, total)
        settings.scientific_planning_progress = fraction
        settings.scientific_planning_status = f"{stage}: {current}/{total}"
        if wm:
            wm.progress_update(int(fraction * 1000))

    try:
        plan = scientific_planner.plan_scientific(
            scene,
            settings,
            components,
            component_counts,
            progress=progress,
            cancel=lambda: bool(settings.scientific_cancel_requested),
        )
    except Exception:
        scene.frame_set(original_frame, subframe=original_subframe)
        raise
    finally:
        if wm:
            wm.progress_end()
    if not plan:
        settings.scientific_planning_status = "未找到合法科学机位"
        scene.frame_set(original_frame, subframe=original_subframe)
        return [], None

    try:
        cameras = _apply_scientific_plan(scene, settings, collection, plan, components=components)
    finally:
        scene.frame_set(original_frame, subframe=original_subframe)
    return cameras, plan


def create_rig(scene, settings):
    if settings.rig_mode == "PATH":
        path_component_counts(settings, path_components(settings))
    collection = collection_get(settings.camera_collection)
    _clear_path_camera_rig_report(scene)
    if settings.rig_mode != "EXISTING":
        remove_old_rig(collection)

    center = target_location(settings)
    count = max(1, settings.camera_count)
    rings = max(1, settings.rings)
    radius = max(0.01, settings.radius)
    height = settings.height

    if settings.rig_mode == "EXISTING":
        cameras = active_dataset_cameras(scene, settings)
        sync_camera_mesh_visuals(scene, settings, cameras)
        return cameras

    cameras = []
    scientific_plan = None
    shell_path = False
    path_station_count = 0
    if settings.rig_mode in {"SPHERE", "HEMISPHERE"}:
        upper_only = settings.rig_mode == "HEMISPHERE"
        total = count
        for index in range(total):
            t = index / max(1, total - 1)
            if upper_only:
                polar = math.radians(12.0) + t * math.radians(78.0)
            else:
                polar = math.radians(12.0) + t * math.radians(156.0)
            azimuth = index * math.radians(137.508)
            x = radius * math.sin(polar) * math.cos(azimuth)
            y = radius * math.sin(polar) * math.sin(azimuth)
            z = radius * math.cos(polar)
            cameras.append(create_camera(f"GS_Camera_{index + 1:04d}", center + Vector((x, y, z)), center, collection, settings))

    elif settings.rig_mode in {"CYLINDER", "HALF_CYLINDER"}:
        per_ring = max(1, math.ceil(count / rings))
        arc = math.pi if settings.rig_mode == "HALF_CYLINDER" else math.tau
        start = -arc / 2.0 if settings.rig_mode == "HALF_CYLINDER" else 0.0
        made = 0
        for ring in range(rings):
            z = 0.0 if rings == 1 else -height / 2.0 + height * ring / (rings - 1)
            for slot in range(per_ring):
                if made >= count:
                    break
                angle = start + arc * slot / max(1, per_ring - (0 if settings.rig_mode == "CYLINDER" else 1))
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                cameras.append(create_camera(f"GS_Camera_{made + 1:04d}", center + Vector((x, y, z)), center, collection, settings))
                made += 1

    elif settings.rig_mode == "PATH":
        if settings.path_capture_mode == "SCIENTIFIC_THREE_LAYER":
            cameras, scientific_plan = create_scientific_path_rig(scene, settings, collection)
        else:
            count = path_camera_count(settings, curve_polyline_points(settings.path_object, settings.path_samples_per_segment))
            sampled = sample_path_components(settings)
            if not sampled:
                fallback = [center + Vector((radius * math.cos(i * math.tau / count), radius * math.sin(i * math.tau / count), 0)) for i in range(count)]
                sampled = [(point, center - point, settings.path_object) for point in fallback]
            path_station_count = len(sampled)
            shell_path = getattr(settings, "path_station_array_mode", "SPHERICAL_SHELL_12") == "SPHERICAL_SHELL_12"
            if shell_path:
                depsgraph = bpy.context.evaluated_depsgraph_get()
                requested_radius = float(getattr(settings, "shell_radius", 0.18))
                actual_radii = []
                adaptive_shrunk = 0
                fallback_count = 0
                skipped_count = 0
                debug_specs = []
                for index, (point, tangent, path_object) in enumerate(sampled):
                    path_forward = _stable_path_forward(sampled, index, tangent)
                    group, actual_radius, used_fallback, skipped, debug_spec = _generate_spherical_shell_12_group(
                        scene,
                        settings,
                        collection,
                        point,
                        path_forward,
                        index + 1,
                        path_object,
                        depsgraph,
                    )
                    cameras.extend(group)
                    if actual_radius > 0.0:
                        actual_radii.append(actual_radius)
                        if actual_radius + 1e-9 < requested_radius:
                            adaptive_shrunk += 1
                    fallback_count += int(used_fallback)
                    skipped_count += int(skipped)
                    if debug_spec is not None:
                        debug_specs.append((index + 1, debug_spec))
                if getattr(settings, "show_shell_debug_mesh", False):
                    for station_index, (shell_center, frame, radius_scene) in debug_specs:
                        _create_shell_debug_proxy(scene, station_index, shell_center, frame, radius_scene)
                _store_path_camera_rig_report(scene, {
                    "mode": "SPHERICAL_SHELL_12",
                    "station_count": path_station_count,
                    "cameras_per_station": 12,
                    "requested_shell_radius": requested_radius,
                    "average_actual_shell_radius": sum(actual_radii) / len(actual_radii) if actual_radii else 0.0,
                    "minimum_actual_shell_radius": min(actual_radii) if actual_radii else 0.0,
                    "maximum_actual_shell_radius": max(actual_radii) if actual_radii else 0.0,
                    "adaptive_shrunk_station_count": adaptive_shrunk,
                    "legacy_fallback_station_count": fallback_count,
                    "skipped_station_count": skipped_count,
                    "duplicate_camera_origin_count": _duplicate_camera_origin_count(scene, cameras),
                })
            else:
                for index, (point, tangent, path_object) in enumerate(sampled):
                    camera = create_camera_at(f"GS_Camera_{index + 1:04d}", point, collection, settings)
                    camera["station_index"] = index + 1
                    camera["source_curve"] = getattr(path_object, "name", "") if path_object else ""
                    if settings.path_look_mode == "TARGET":
                        look_at(camera, center)
                    elif settings.path_look_mode == "FORWARD":
                        aim_along(camera, tangent)
                    elif path_object:
                        camera.rotation_euler = path_object.matrix_world.to_euler()
                    cameras.append(camera)

    elif settings.rig_mode == "VOLUME":
        dims = settings.volume_size
        gx, gy, gz = max(1, settings.volume_x), max(1, settings.volume_y), max(1, settings.volume_z)
        rng = random.Random(settings.random_seed)
        made = 0
        for ix in range(gx):
            for iy in range(gy):
                for iz in range(gz):
                    if made >= count:
                        break
                    frac = Vector((
                        0.0 if gx == 1 else ix / (gx - 1) - 0.5,
                        0.0 if gy == 1 else iy / (gy - 1) - 0.5,
                        0.0 if gz == 1 else iz / (gz - 1) - 0.5,
                    ))
                    jitter = Vector((rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5))) * settings.volume_jitter
                    loc = center + Vector((frac.x * dims.x, frac.y * dims.y, frac.z * dims.z)) + jitter
                    if point_inside_exclusions(loc, settings.exclude_collection):
                        continue
                    cameras.append(create_camera(f"GS_Camera_{made + 1:04d}", loc, center, collection, settings))
                    made += 1

    keep_rotation = settings.rig_mode == "PATH" and settings.path_look_mode != "TARGET"
    if scientific_plan is None and not shell_path:
        cameras = resolve_camera_clipping(scene, settings, cameras, center, keep_rotation)
    legacy_path = settings.rig_mode == "PATH" and settings.path_capture_mode == "LEGACY_PANORAMA_CUBE"
    legacy_compatibility = (
        legacy_path
        and getattr(settings, "path_station_array_mode", "SPHERICAL_SHELL_12") == "LEGACY_SIX"
    )
    if legacy_compatibility or (settings.rig_mode != "PATH" and settings.camera_model == "PANORAMA_CUBE"):
        cameras = expand_to_cubemap(scene, settings, cameras, collection)
    if legacy_compatibility:
        _store_path_camera_rig_report(scene, {
            "mode": "LEGACY_SIX",
            "station_count": path_station_count,
            "cameras_per_station": 6,
            "requested_shell_radius": float(getattr(settings, "shell_radius", 0.18)),
            "average_actual_shell_radius": 0.0,
            "minimum_actual_shell_radius": 0.0,
            "maximum_actual_shell_radius": 0.0,
            "adaptive_shrunk_station_count": 0,
            "legacy_fallback_station_count": 0,
            "skipped_station_count": 0,
            "duplicate_camera_origin_count": _duplicate_camera_origin_count(scene, cameras),
        })
    sync_camera_mesh_visuals(scene, settings, cameras)
    return cameras


def point_inside_exclusions(point, collection):
    if not collection:
        return False
    for obj in collection.objects:
        if obj.type != "MESH" or is_camera_mesh_visual(obj):
            continue
        local = obj.matrix_world.inverted() @ point
        xs = [corner[0] for corner in obj.bound_box]
        ys = [corner[1] for corner in obj.bound_box]
        zs = [corner[2] for corner in obj.bound_box]
        if min(xs) <= local.x <= max(xs) and min(ys) <= local.y <= max(ys) and min(zs) <= local.z <= max(zs):
            return True
    return False


def sensor_fit(camera_data):
    return camera_data.sensor_fit if camera_data.sensor_fit != "AUTO" else "HORIZONTAL"


def camera_params(scene, camera):
    width = int(scene.render.resolution_x * scene.render.resolution_percentage / 100)
    height = int(scene.render.resolution_y * scene.render.resolution_percentage / 100)
    data = camera.data
    if data.type == "PANO":
        if data.panorama_type == "EQUIRECTANGULAR":
            fx = width / math.tau
            fy = height / math.pi
            return width, height, fx, fy, width * 0.5, height * 0.5
        fov = getattr(data, "fisheye_fov", math.pi)
        fx = width / max(1e-6, fov)
        fy = height / max(1e-6, fov)
        return width, height, fx, fy, width * 0.5, height * 0.5
    if sensor_fit(data) == "VERTICAL":
        fy = data.lens / data.sensor_height * height
        fx = fy
    else:
        fx = data.lens / data.sensor_width * width
        fy = fx
    cx = width * 0.5 - data.shift_x * width
    cy = height * 0.5 + data.shift_y * height
    return width, height, fx, fy, cx, cy


def _has_animated_property(owner, token):
    animation = getattr(owner, "animation_data", None)
    action = getattr(animation, "action", None) if animation else None
    return bool(action and any(token in curve.data_path for curve in action.fcurves))


def training_consistency_report(scene, settings, cameras):
    lenses = sorted({round(float(camera.data.lens), 6) for camera in cameras})
    sensor_widths = sorted({round(float(camera.data.sensor_width), 6) for camera in cameras})
    sensor_heights = sorted({round(float(camera.data.sensor_height), 6) for camera in cameras})
    dof_cameras = sorted(
        camera.name for camera in cameras
        if bool(getattr(getattr(camera.data, "dof", None), "use_dof", False))
    )
    animated_lens_cameras = sorted(
        camera.name for camera in cameras if _has_animated_property(camera.data, "lens")
    )
    motion_blur = False
    for owner in (getattr(scene, "render", None), getattr(scene, "eevee", None)):
        if owner is not None and bool(getattr(owner, "use_motion_blur", False)):
            motion_blur = True
    exposure_animated = _has_animated_property(scene, "exposure")
    warnings = []
    if len(lenses) != 1:
        warnings.append("Training cameras use multiple focal lengths.")
    if len(sensor_widths) != 1 or len(sensor_heights) != 1:
        warnings.append("Training cameras use multiple sensor sizes.")
    if dof_cameras:
        warnings.append("Depth of field is enabled on training cameras.")
    if motion_blur:
        warnings.append("Motion blur is enabled.")
    if animated_lens_cameras:
        warnings.append("Focal length is animated.")
    if exposure_animated:
        warnings.append("Exposure is animated.")
    return {
        "is_consistent": not warnings,
        "warning_count": len(warnings),
        "warnings": warnings,
        "focal_lengths_mm": lenses,
        "sensor_widths_mm": sensor_widths,
        "sensor_heights_mm": sensor_heights,
        "resolution": [int(settings.resolution_x), int(settings.resolution_y)],
        "depth_of_field_enabled_camera_count": len(dof_cameras),
        "motion_blur_enabled": motion_blur,
        "animated_focal_length_camera_count": len(animated_lens_cameras),
        "animated_exposure": exposure_animated,
        "fixed_exposure": float(getattr(settings, "color_exposure", 0.0)),
        "stable_color_management": True,
    }


def apply_training_recommendations(scene, settings, cameras):
    for camera in cameras:
        dof = getattr(camera.data, "dof", None)
        if dof is not None:
            dof.use_dof = False
        camera.data.lens = float(settings.focal_length)
    for owner in (getattr(scene, "render", None), getattr(scene, "eevee", None)):
        if owner is not None and hasattr(owner, "use_motion_blur"):
            owner.use_motion_blur = False
    if getattr(scene, "view_settings", None) is not None:
        scene.view_settings.exposure = float(getattr(settings, "color_exposure", 0.0))
    return training_consistency_report(scene, settings, cameras)


def camera_export_model(camera):
    data = camera.data
    if data.type != "PANO":
        return "PINHOLE"
    if data.panorama_type == "EQUIRECTANGULAR":
        return "EQUIRECTANGULAR"
    if data.panorama_type in {"FISHEYE_EQUIDISTANT", "FISHEYE_EQUISOLID"}:
        return "OPENCV_FISHEYE"
    return data.panorama_type


def blender_to_colmap(camera):
    r_bcam_to_cv = Matrix(((1, 0, 0), (0, -1, 0), (0, 0, -1)))
    location, rotation = camera.matrix_world.decompose()[0:2]
    r_world_to_bcam = rotation.to_matrix().transposed()
    t_world_to_bcam = -(r_world_to_bcam @ location)
    r_world_to_cv = r_bcam_to_cv @ r_world_to_bcam
    t_world_to_cv = r_bcam_to_cv @ t_world_to_bcam
    quat = r_world_to_cv.to_quaternion()
    return quat, t_world_to_cv


def colmap_to_blender(quaternion, translation):
    """Inverse of blender_to_colmap, used by round-trip regression tests."""
    r_bcam_to_cv = Matrix(((1, 0, 0), (0, -1, 0), (0, 0, -1)))
    r_world_to_cv = quaternion.to_matrix()
    r_world_to_bcam = r_bcam_to_cv.transposed() @ r_world_to_cv
    t_world_to_bcam = r_bcam_to_cv.transposed() @ Vector(translation)
    rotation_world = r_world_to_bcam.transposed()
    location = -(rotation_world @ t_world_to_bcam)
    matrix = rotation_world.to_4x4()
    matrix.translation = location
    return matrix


def write_colmap(scene, settings, cameras, image_names, points):
    sparse_dir = Path(settings.output_dir) / "sparse" / "0"
    ensure_dir(sparse_dir)
    width, height, fx, fy, cx, cy = camera_params(scene, cameras[0])
    export_model = camera_export_model(cameras[0])
    profiles = []
    profile_lookup = {}
    camera_ids = []
    for camera in cameras:
        params = camera_params(scene, camera)
        model = camera_export_model(camera)
        key = (model,) + tuple(round(float(value), 8) for value in params)
        camera_id = profile_lookup.get(key)
        if camera_id is None:
            camera_id = len(profiles) + 1
            profile_lookup[key] = camera_id
            profiles.append((model, params))
        camera_ids.append(camera_id)
    with open(sparse_dir / "cameras.txt", "w", encoding="utf-8") as handle:
        handle.write("# Camera list with one line of data per camera:\n")
        handle.write("# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        for camera_id, (model, params) in enumerate(profiles, 1):
            profile_width, profile_height, profile_fx, profile_fy, profile_cx, profile_cy = params
            if model == "PINHOLE":
                handle.write(f"{camera_id} PINHOLE {profile_width} {profile_height} {profile_fx:.8f} {profile_fy:.8f} {profile_cx:.8f} {profile_cy:.8f}\n")
            elif model == "OPENCV_FISHEYE":
                handle.write(f"{camera_id} OPENCV_FISHEYE {profile_width} {profile_height} {profile_fx:.8f} {profile_fy:.8f} {profile_cx:.8f} {profile_cy:.8f} 0 0 0 0\n")
            else:
                handle.write(f"{camera_id} EQUIRECTANGULAR {profile_width} {profile_height} {profile_fx:.8f} {profile_fy:.8f} {profile_cx:.8f} {profile_cy:.8f}\n")

    with open(sparse_dir / "images.txt", "w", encoding="utf-8") as handle:
        handle.write("# Image list with two lines of data per image:\n")
        handle.write("# IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
        handle.write("# POINTS2D[] as (X, Y, POINT3D_ID)\n")
        for index, camera in enumerate(cameras, start=1):
            quat, trans = blender_to_colmap(camera)
            name = image_names[index - 1]
            camera_id = camera_ids[index - 1]
            image_id = int(getattr(getattr(camera, "sample", None), "image_id", index))
            handle.write(
                f"{image_id} {quat.w:.10f} {quat.x:.10f} {quat.y:.10f} {quat.z:.10f} "
                f"{trans.x:.10f} {trans.y:.10f} {trans.z:.10f} {camera_id} {name}\n\n"
            )

    with open(sparse_dir / "points3D.txt", "w", encoding="utf-8") as handle:
        handle.write("# 3D point list with one line of data per point:\n")
        handle.write("# POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[]\n")
        for index, point in enumerate(points, start=1):
            handle.write(
                f"{index} {point.location.x:.8f} {point.location.y:.8f} {point.location.z:.8f} "
                f"{point.color[0]} {point.color[1]} {point.color[2]} 0.0\n"
            )

    transforms = {
        "camera_model": export_model,
        "blender_camera_model": settings.camera_model,
        "w": width,
        "h": height,
        "fl_x": fx,
        "fl_y": fy,
        "cx": cx,
        "cy": cy,
        "frames": [],
    }
    output_plan = _resolved_output_plan(settings)
    for index, (camera, name) in enumerate(zip(cameras, image_names)):
        stem = Path(name).stem
        frame = {
            "file_path": f"images/{name}",
            "rgb_available": output_plan.config.rgb,
            "transform_matrix": [[float(value) for value in row] for row in camera.matrix_world],
        }
        if output_plan.saves(output_pipeline.SCENE_DEPTH):
            depth_suffix = ".exr" if settings.depth_format == "EXR" else ".png"
            frame["depth_file_path"] = f"depth/{stem}{depth_suffix}"
        if output_plan.saves(output_pipeline.SCENE_NORMAL):
            frame["normal_file_path"] = f"normal/{stem}.png"
        if output_plan.saves(output_pipeline.OBJECT_ID):
            frame["id_file_path"] = f"id/{stem}.png"
        if output_plan.config.material_id:
            frame["material_id_file_path"] = f"material_id/{stem}.png"
        sample = getattr(camera, "sample", None)
        if sample is not None:
            frame.update({
                "logical_frame_id": int(sample.logical_frame_id),
                "image_id": int(sample.image_id),
                "segment_id": int(sample.segment_id),
                "sample_type": sample.sample_type,
            })
            if sample.depth_path:
                frame["depth_file_path"] = sample.depth_path
            if sample.normal_path:
                frame["normal_file_path"] = sample.normal_path
            if sample.id_path:
                frame["id_file_path"] = sample.id_path
            if sample.material_id_path:
                frame["material_id_file_path"] = sample.material_id_path
            if sample.object_depth_path:
                frame["object_depth_file_pattern"] = sample.object_depth_path
            if sample.object_normal_path:
                frame["object_normal_file_pattern"] = sample.object_normal_path
            if sample.mask_path:
                frame["object_mask_file_pattern"] = sample.mask_path
        if len(profiles) > 1:
            frame_width, frame_height, frame_fx, frame_fy, frame_cx, frame_cy = camera_params(scene, camera)
            frame.update({
                "camera_id": camera_ids[index],
                "w": frame_width, "h": frame_height,
                "fl_x": frame_fx, "fl_y": frame_fy,
                "cx": frame_cx, "cy": frame_cy,
            })
        transforms["frames"].append(frame)
    with open(Path(settings.output_dir) / "transforms.json", "w", encoding="utf-8") as handle:
        json.dump(transforms, handle, indent=2)


def write_transforms_only(scene, settings, cameras, image_names):
    """Write transforms.json from the same camera adapters used by COLMAP export."""
    width, height, fx, fy, cx, cy = camera_params(scene, cameras[0])
    profiles = [camera_params(scene, camera) for camera in cameras]
    transforms = {
        "camera_model": camera_export_model(cameras[0]),
        "blender_camera_model": settings.camera_model,
        "w": width, "h": height, "fl_x": fx, "fl_y": fy, "cx": cx, "cy": cy,
        "frames": [],
    }
    distinct_profiles = {
        tuple(round(float(value), 8) for value in params) for params in profiles
    }
    output_plan = _resolved_output_plan(settings)
    for camera, name, params in zip(cameras, image_names, profiles):
        stem = Path(name).stem
        frame = {
            "file_path": f"images/{name}",
            "rgb_available": output_plan.config.rgb,
            "transform_matrix": [[float(value) for value in row] for row in camera.matrix_world],
        }
        if output_plan.saves(output_pipeline.SCENE_DEPTH):
            depth_suffix = ".exr" if settings.depth_format == "EXR" else ".png"
            frame["depth_file_path"] = f"depth/{stem}{depth_suffix}"
        if output_plan.saves(output_pipeline.SCENE_NORMAL):
            frame["normal_file_path"] = f"normal/{stem}.png"
        if output_plan.saves(output_pipeline.OBJECT_ID):
            frame["id_file_path"] = f"id/{stem}.png"
        if output_plan.config.material_id:
            frame["material_id_file_path"] = f"material_id/{stem}.png"
        sample = getattr(camera, "sample", None)
        if sample is not None:
            frame.update({
                "logical_frame_id": int(sample.logical_frame_id),
                "image_id": int(sample.image_id),
                "segment_id": int(sample.segment_id),
                "sample_type": sample.sample_type,
            })
            if sample.depth_path:
                frame["depth_file_path"] = sample.depth_path
            if sample.normal_path:
                frame["normal_file_path"] = sample.normal_path
            if sample.id_path:
                frame["id_file_path"] = sample.id_path
            if sample.material_id_path:
                frame["material_id_file_path"] = sample.material_id_path
            if sample.object_depth_path:
                frame["object_depth_file_pattern"] = sample.object_depth_path
            if sample.object_normal_path:
                frame["object_normal_file_pattern"] = sample.object_normal_path
            if sample.mask_path:
                frame["object_mask_file_pattern"] = sample.mask_path
        if len(distinct_profiles) > 1:
            frame.update({
                "w": params[0], "h": params[1], "fl_x": params[2],
                "fl_y": params[3], "cx": params[4], "cy": params[5],
            })
        transforms["frames"].append(frame)
    pose_sequence.atomic_write_json(Path(settings.output_dir) / "transforms.json", transforms)
    return transforms


@dataclass
class CloudPoint:
    location: Vector
    color: tuple


def material_color(obj):
    mat = obj.active_material
    if mat and hasattr(mat, "diffuse_color"):
        col = mat.diffuse_color
        return tuple(max(0, min(255, int(channel * 255))) for channel in col[:3])
    return (190, 190, 190)


def _sample_sparse_points_impl(scene, cameras, settings):
    if settings.point_samples_per_view <= 0:
        return []
    depsgraph = bpy.context.evaluated_depsgraph_get()
    points = {}
    per_side = max(2, int(math.sqrt(settings.point_samples_per_view)))
    dedup = max(1e-5, settings.point_dedup_size)
    for camera in cameras:
        origin = camera.matrix_world.translation
        rotation = camera.matrix_world.to_quaternion()
        for ix in range(per_side):
            u = (ix + 0.5) / per_side
            for iy in range(per_side):
                v = (iy + 0.5) / per_side
                if camera.data.type == "PANO":
                    theta = (u - 0.5) * math.tau
                    phi = (0.5 - v) * math.pi
                    direction_local = Vector((
                        math.sin(theta) * math.cos(phi),
                        math.sin(phi),
                        -math.cos(theta) * math.cos(phi),
                    )).normalized()
                else:
                    frame = [corner.copy() for corner in camera.data.view_frame(scene=scene)]
                    # Blender returns left-bottom, right-bottom, right-top, left-top.
                    lb, rb, rt, lt = frame[0], frame[1], frame[2], frame[3]
                    bottom = lb.lerp(rb, u)
                    top = lt.lerp(rt, u)
                    direction_local = bottom.lerp(top, v).normalized()
                direction = (rotation @ direction_local).normalized()
                hit, location, normal, face_index, obj, matrix = scene.ray_cast(depsgraph, origin, direction, distance=settings.ray_distance)
                if not hit or not obj or obj.type != "MESH" or is_camera_mesh_visual(obj):
                    continue
                key = (round(location.x / dedup), round(location.y / dedup), round(location.z / dedup))
                if key not in points:
                    points[key] = CloudPoint(location.copy(), material_color(obj))
    return list(points.values())


def sample_sparse_points(scene, cameras, settings):
    visual_states = hide_camera_mesh_visuals(scene)
    try:
        return _sample_sparse_points_impl(scene, cameras, settings)
    finally:
        restore_camera_mesh_visuals(visual_states)


def _physical_gpu_names():
    """Return the set of GPU NAMES that are PHYSICALLY present on this machine, by asking
    the OS (NVIDIA: nvidia-smi; otherwise WMI on Windows / lspci on Linux). Used to filter
    out STALE device entries Blender keeps cached in userpref.blend from old GPUs you no
    longer have (the most common cause of "phantom" multi-GPU detection)."""
    names = set()
    # 1) NVIDIA — nvidia-smi is authoritative
    nvsmi_candidates = [
        "nvidia-smi",
        os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32", "nvidia-smi.exe"),
        r"C:\Program Files\NVIDIA Corporation\NVSMI\nvidia-smi.exe",
    ]
    for exe in nvsmi_candidates:
        try:
            r = subprocess.run([exe, "--query-gpu=name", "--format=csv,noheader"],
                               capture_output=True, text=True, timeout=5)
            if r.returncode == 0 and r.stdout.strip():
                for line in r.stdout.splitlines():
                    line = line.strip()
                    if line:
                        names.add(line)
                break
        except Exception:
            pass
    # 2) Windows: WMI Win32_VideoController (covers AMD/Intel and is a useful sanity check)
    if os.name == "nt":
        try:
            ps = subprocess.run(
                ["powershell", "-NoProfile", "-Command",
                 "Get-CimInstance Win32_VideoController | Select-Object -ExpandProperty Name"],
                capture_output=True, text=True, timeout=8)
            if ps.returncode == 0:
                for line in ps.stdout.splitlines():
                    line = line.strip()
                    # skip virtual / display-only adapters
                    if line and "iddriver" not in line.lower() and "remote" not in line.lower():
                        names.add(line)
        except Exception:
            pass
    return names


def _name_matches_physical(blender_name, physical_names):
    """Blender device names sometimes differ slightly from OS names (e.g. extra spaces,
    'NVIDIA GeForce' prefix). Match loosely: substring match either way, case-insensitive."""
    if not physical_names:
        return True  # can't verify -> trust Blender
    bn = blender_name.lower().strip()
    for pn in physical_names:
        pl = pn.lower().strip()
        if bn == pl or bn in pl or pl in bn:
            return True
    return False


def list_cycles_gpus():
    """Return a list of (backend, device_name) for every GPU Cycles can see, preferring
    OPTIX over CUDA on the SAME card (so we never count a card twice). Also filters out
    STALE cached devices that no longer exist on the machine (e.g. Blender remembers an
    RTX 2080 SUPER you replaced with an RTX 3080 — we don't want to spawn a worker for
    the ghost)."""
    try:
        cp = bpy.context.preferences.addons["cycles"].preferences
    except Exception:
        return []
    physical = _physical_gpu_names()
    gpus = []  # (backend, name)
    by_name = {}
    for backend in ("OPTIX", "HIP", "ONEAPI", "METAL", "CUDA"):
        try:
            cp.compute_device_type = backend
            cp.get_devices()
        except Exception:
            continue
        for dev in cp.devices:
            try:
                if dev.type != backend or dev.type == "CPU":
                    continue
                if dev.name in by_name:
                    continue
                if not _name_matches_physical(dev.name, physical):
                    # stale device — Blender remembers it but the GPU is no longer here.
                    print(f"[GS] Skipping stale Cycles device entry: '{dev.name}' "
                          f"(not present on this machine). Physical GPUs: {sorted(physical)}")
                    continue
                by_name[dev.name] = backend
                gpus.append((backend, dev.name))
            except Exception:
                pass
    return gpus


def activate_cycles_devices(restrict_names=None, prefer_backend=None):
    """Enable Cycles GPU devices in user prefs (the official `cycles.device = "GPU"` on a
    scene does NOTHING unless the user has already ticked devices in Edit > Preferences).
    `restrict_names`: when given, ONLY enable devices whose name is in this set (GPU pin
    for one worker). `prefer_backend`: e.g. "OPTIX" to force a backend across all workers."""
    try:
        cp = bpy.context.preferences.addons["cycles"].preferences
    except Exception:
        return ("", [])
    gpus = list_cycles_gpus()
    if not gpus:
        return ("CPU", [])
    if prefer_backend:
        # An explicit backend must never silently turn into another GPU backend. This is
        # particularly important for HIP troubleshooting: falling back to CUDA/OptiX can
        # hide a missing AMD runtime and reintroduce the original memory failure.
        if not any(b == prefer_backend for b, _ in gpus):
            return (prefer_backend, [])
        backend = prefer_backend
    elif restrict_names:
        backends_for_pinned = [b for b, n in gpus if n in restrict_names]
        backend = backends_for_pinned[0] if backends_for_pinned else gpus[0][0]
    else:
        # OPTIX > CUDA > HIP > ONEAPI > METAL; first one with any device wins
        order = ["OPTIX", "CUDA", "HIP", "ONEAPI", "METAL"]
        present = {b for b, _ in gpus}
        backend = next((b for b in order if b in present), gpus[0][0])
    try:
        cp.compute_device_type = backend
        cp.get_devices()
    except Exception:
        return ("CPU", [])
    enabled = []
    for dev in cp.devices:
        try:
            if dev.type == backend:
                if restrict_names is None or dev.name in restrict_names:
                    dev.use = True
                    enabled.append(dev.name)
                else:
                    dev.use = False
            else:
                dev.use = False  # CPU + other-backend duplicates stay OFF
        except Exception:
            pass
    return (backend, enabled)


def _configure_cycles_hiprt(settings):
    """Enable HIP-RT explicitly after a factory-startup worker has selected HIP.

    Background jobs intentionally start Blender with ``--factory-startup``. That resets
    Cycles preferences, including ``use_hiprt``, so an interactive HIP-RT checkbox does not
    carry into the worker. On a large scene the resulting HIP software-BVH path can request
    tens of gigabytes for a single frame. Keep the accelerator selection and this preference
    together so a HIP worker cannot silently take a different ray-tracing path.
    """
    mode = str(getattr(settings, "hip_rt_mode", "REQUIRE") or "REQUIRE").upper()
    if mode not in {"REQUIRE", "AUTO", "DISABLED"}:
        mode = "REQUIRE"
    try:
        preferences = bpy.context.preferences.addons["cycles"].preferences
    except Exception as exc:
        if mode == "REQUIRE":
            raise RuntimeError("HIP-RT is required, but Cycles preferences are unavailable") from exc
        print(f"[GS] HIP-RT unavailable: Cycles preferences could not be read ({exc})", flush=True)
        return {"mode": mode, "supported": False, "enabled": False, "runtime": "unknown"}

    binary_path = Path(getattr(bpy.app, "binary_path", "") or "")
    runtime = "unknown"
    if binary_path:
        runtime = "present" if (binary_path.parent / "hiprt64.dll").is_file() else "not_found"

    if not hasattr(preferences, "use_hiprt"):
        message = "This Blender build exposes no Cycles use_hiprt preference"
        if mode == "REQUIRE":
            raise RuntimeError(
                "HIP-RT is required for this HIP render, but " + message + ". "
                "Use an official Blender 5.1 build with HIP-RT and a current AMD driver, "
                "or select CPU instead of the HIP software BVH path."
            )
        print(f"[GS] HIP-RT unavailable: {message}", flush=True)
        return {"mode": mode, "supported": False, "enabled": False, "runtime": runtime}

    if mode == "DISABLED":
        try:
            preferences.use_hiprt = False
        except Exception:
            pass
        print("[GS] HIP-RT deliberately disabled; Cycles will use HIP software BVH", flush=True)
        return {"mode": mode, "supported": True, "enabled": False, "runtime": runtime}

    try:
        preferences.use_hiprt = True
        enabled = bool(preferences.use_hiprt)
    except Exception as exc:
        enabled = False
        message = str(exc)
    else:
        message = ""

    if not enabled and mode == "REQUIRE":
        detail = f" ({message})" if message else ""
        raise RuntimeError(
            "HIP-RT is required for this HIP render but Blender could not enable it" + detail + ". "
            "The background worker starts with --factory-startup, so the interactive Preferences "
            "checkbox is not inherited. Update Blender/AMD Adrenalin, then retry; do not continue "
            "with the HIP software BVH path for this large scene."
        )
    print(
        f"[GS] HIP-RT: mode={mode.lower()} enabled={enabled} runtime={runtime}",
        flush=True,
    )
    return {"mode": mode, "supported": True, "enabled": enabled, "runtime": runtime}


def configure_render(scene, settings, gpu_restrict=None, apply_color=True):
    active_backend = "CPU"
    hiprt_state = {"mode": "n/a", "supported": False, "enabled": False, "runtime": "n/a"}
    try:
        scene.render.engine = settings.render_engine
    except TypeError:
        scene.render.engine = "CYCLES"
    if scene.render.engine == "CYCLES" and hasattr(scene, "cycles"):
        scene.cycles.samples = settings.cycles_samples
        scene.cycles.use_denoising = settings.cycles_denoise
        requested_backend = getattr(settings, "cycles_backend", "AUTO")
        # Selecting a concrete backend is an explicit request for GPU rendering, even if
        # the older GPU/CPU selector is still set to AUTO or CPU in a saved scene.
        desired = settings.cycles_device
        if requested_backend not in {"", "AUTO"}:
            desired = "GPU"
        if desired == "AUTO":
            desired = "GPU" if list_cycles_gpus() else "CPU"
        if desired == "GPU":
            backend, enabled = activate_cycles_devices(
                restrict_names=gpu_restrict,
                prefer_backend=(requested_backend if requested_backend not in {"", "AUTO"} else None),
            )
            scene.cycles.device = "GPU" if enabled else "CPU"
            if enabled:
                print(f"[GS] Cycles GPU active: backend={backend} devices={enabled}")
            else:
                print(f"[GS] Cycles fell back to CPU (requested backend {backend} is unavailable).")
                backend = "CPU"
        else:
            backend = "CPU"
            scene.cycles.device = "CPU"
        active_backend = backend
        if backend == "HIP":
            hiprt_state = _configure_cycles_hiprt(settings)
        # HIP allocators are more sensitive to long-lived Cycles scene caches. A fresh
        # background worker still provides the strongest isolation, while this also keeps
        # foreground renders bounded when the user intentionally disables background mode.
        hip_safe = bool(getattr(settings, "hip_memory_safe_mode", True))
        if hip_safe and backend == "HIP":
            try:
                scene.render.use_persistent_data = False
            except Exception:
                pass
            print("[GS] HIP memory-safe mode: persistent Cycles data disabled", flush=True)
    scene.render.resolution_x = settings.resolution_x
    scene.render.resolution_y = settings.resolution_y
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = settings.image_format
    if settings.image_format == "PNG":
        scene.render.image_settings.color_mode = "RGBA" if settings.transparent_background else "RGB"
    scene.render.film_transparent = settings.transparent_background
    try:
        scene.render.use_persistent_data = bool(
            getattr(settings, "cycles_persistent_data", False)
        )
        if (
            scene.render.engine == "CYCLES"
            and active_backend == "HIP"
            and bool(getattr(settings, "hip_memory_safe_mode", True))
        ):
            scene.render.use_persistent_data = False
    except Exception:
        pass
    try:
        scene["gs_cycles_backend"] = active_backend
        scene["gs_hiprt_mode"] = hiprt_state["mode"]
        scene["gs_hiprt_enabled"] = bool(hiprt_state["enabled"])
        scene["gs_hiprt_runtime"] = hiprt_state["runtime"]
        scene["gs_hip_oom_fallback"] = bool(getattr(settings, "hip_oom_fallback", False))
        scene["gs_hip_oom_recovered"] = False
    except Exception:
        pass
    # faithful, reproducible colour for the dataset render. apply_color=False (viewport preview)
    # leaves the user's OWN Colour Management untouched -- preview = exactly what they see.
    if apply_color:
        apply_view_transform(scene, getattr(settings, "color_look", "AgX"),
                             getattr(settings, "color_exposure", 0.0))


def _material_slot_counts(objects):
    object_counts = {obj.as_pointer(): len(obj.material_slots) for obj in objects}
    data_counts = {}
    for obj in objects:
        data_counts.setdefault(obj.data.as_pointer(), (obj.data, len(obj.data.materials)))
    return object_counts, data_counts


def _override_object_materials(scene, material):
    """Object-level material override for every mesh (shared-mesh safe: does NOT touch the
    mesh datablock, so linked/instanced geometry is not corrupted). Returns a restore record."""
    objects = [
        obj for obj in scene.objects
        if obj.type == "MESH" and not is_camera_mesh_visual(obj)
    ]
    object_counts, data_counts = _material_slot_counts(objects)
    saved = []
    for obj in objects:
        original_slot_count = object_counts[obj.as_pointer()]
        original_data_slot_count = data_counts[obj.data.as_pointer()][1]
        if not obj.material_slots:
            obj.data.materials.append(None)
        record = []
        for slot in obj.material_slots:
            record.append((slot.link, slot.material))
            slot.link = "OBJECT"
            slot.material = material
        saved.append((obj, record, original_slot_count, original_data_slot_count))
    return saved


def _override_view_layer_materials(scene, material):
    """Override every renderable surface, including linked Collection Instances."""
    saved = []
    for view_layer in scene.view_layers:
        try:
            saved.append((view_layer, view_layer.material_override))
            view_layer.material_override = material
        except Exception as exc:
            print(f"[GS] could not override view layer {view_layer.name}: {exc}", flush=True)
    return saved


def _restore_view_layer_materials(saved):
    for view_layer, material in saved:
        try:
            view_layer.material_override = material
        except Exception:
            pass


def _restore_object_materials(saved):
    data_counts = {}
    for item in saved:
        obj, record = item[:2]
        original_slot_count = item[2] if len(item) > 2 else len(record)
        original_data_slot_count = item[3] if len(item) > 3 else original_slot_count
        data_counts.setdefault(obj.data.as_pointer(), (obj.data, original_data_slot_count))
        for index, (link, material) in enumerate(record):
            if index < len(obj.material_slots):
                if index >= original_slot_count:
                    obj.material_slots[index].link = "OBJECT"
                    obj.material_slots[index].material = None
                    continue
                obj.material_slots[index].link = link
                # DATA slots expose their material from the mesh datablock. The override
                # never changed that datablock, and linked meshes can be read-only.
                if link == "OBJECT":
                    obj.material_slots[index].material = material
    for data, original_count in data_counts.values():
        if original_count == 0 and data.materials:
            data.materials.clear()
        else:
            while len(data.materials) > original_count:
                data.materials.pop(index=len(data.materials) - 1)


def _save_render_state(scene):
    """Snapshot every render/colour setting a pass might touch, so each pass (color / depth /
    id / mask) is fully isolated and can NEVER contaminate the next one."""
    ims = scene.render.image_settings
    vs = getattr(scene, "view_settings", None)
    st = {
        "camera": scene.camera,
        "filepath": scene.render.filepath,
        "file_format": ims.file_format,
        "color_mode": ims.color_mode,
        "color_depth": ims.color_depth,
        "filter_size": scene.render.filter_size,
        "film_transparent": scene.render.film_transparent,
        "use_compositing": scene.render.use_compositing,
        "use_persistent_data": getattr(scene.render, "use_persistent_data", False),
    }
    if vs is not None:
        st["view_transform"] = vs.view_transform
        st["look"] = vs.look
        st["exposure"] = vs.exposure
        st["gamma"] = vs.gamma
    return st


def _restore_render_state(scene, st):
    ims = scene.render.image_settings
    vs = getattr(scene, "view_settings", None)
    scene.camera = st["camera"]
    scene.render.filepath = st["filepath"]
    ims.file_format = st["file_format"]
    ims.color_mode = st["color_mode"]
    ims.color_depth = st["color_depth"]
    scene.render.filter_size = st["filter_size"]
    scene.render.film_transparent = st["film_transparent"]
    scene.render.use_compositing = st["use_compositing"]
    if "use_persistent_data" in st:
        try:
            scene.render.use_persistent_data = st["use_persistent_data"]
        except Exception:
            pass
    if vs is not None and "view_transform" in st:
        try:
            vs.view_transform = st["view_transform"]
            vs.look = st["look"]
            vs.exposure = st["exposure"]
            vs.gamma = st["gamma"]
        except Exception:
            pass


def _is_render_oom(exc):
    message = str(exc).lower()
    return any(
        marker in message
        for marker in ("out of memory", "malloc returns null", "failed to allocate")
    )


def _render_still_with_hip_fallback(scene):
    """Render one still and fall back to CPU when HIP cannot allocate this frame.

    A chunk of one frame cannot reduce an allocation made by a single scene. In that case
    continuing to retry HIP only wastes time; the CPU path can still finish the dataset when
    host RAM is sufficient. The marker is consumed by the background supervisor so the
    recovered frame is not rejected merely because Blender logged the original HIP OOM.
    """
    try:
        bpy.ops.render.render(write_still=True)
        return scene.get("gs_cycles_backend", "CPU")
    except Exception as exc:
        cycles = getattr(scene, "cycles", None)
        can_fallback = bool(scene.get("gs_hip_oom_fallback", False))
        if not _is_render_oom(exc) or cycles is None or not can_fallback:
            raise
        if scene.get("gs_cycles_backend") != "HIP" or getattr(cycles, "device", "CPU") != "GPU":
            raise
        print(
            "[GS] HIP single-frame OOM; releasing buffers and retrying this frame on CPU",
            flush=True,
        )
        _release_render_buffers()
        cycles.device = "CPU"
        scene.render.use_persistent_data = False
        try:
            bpy.ops.render.render(write_still=True)
        except Exception:
            raise
        scene["gs_cycles_backend"] = "CPU_FALLBACK"
        scene["gs_hip_oom_recovered"] = True
        print("[GS] HIP_OOM_RECOVERED_CPU", flush=True)
        return "CPU_FALLBACK"


def render_depth(scene, camera, target_path):
    """Per-pixel linear depth EXR (camera View-Z) via a temporary emission material, rendered
    with the *Raw* view transform so the stored values are true linear metres (no AgX/Filmic
    curve baked into the depth). Background is transparent (alpha 0) so only geometry carries
    depth. Fully state-isolated -> never disturbs the colour pass."""
    target_path = Path(target_path)
    st = _save_render_state(scene)
    ims = scene.render.image_settings
    mat = bpy.data.materials.new("GS_TEMP_DEPTH")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    cam_node = nt.nodes.new("ShaderNodeCameraData")
    emission = nt.nodes.new("ShaderNodeEmission")
    output = nt.nodes.new("ShaderNodeOutputMaterial")
    depth_socket = cam_node.outputs.get("View Z Depth") or cam_node.outputs.get("View Distance")
    nt.links.new(depth_socket, emission.inputs["Color"])
    emission.inputs["Strength"].default_value = 1.0
    nt.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    saved = _override_view_layer_materials(scene, mat)
    try:
        scene.camera = camera
        # This pass changes the material on every render and must not enter Cycles' persistent
        # scene cache. Persistent cache entries retain temporary linked-instance materials.
        scene.render.use_persistent_data = False
        scene.render.use_compositing = False
        scene.render.film_transparent = True
        apply_view_transform(scene, "Raw")
        ims.file_format = "OPEN_EXR"
        ims.color_mode = "RGBA"
        ims.color_depth = "32"
        scene.render.filepath = str(target_path.with_suffix(""))  # Blender appends .exr
        _render_still_with_hip_fallback(scene)
        produced = Path(str(target_path.with_suffix("")) + ".exr")
        if produced.exists() and produced != target_path:
            if target_path.exists():
                target_path.unlink()
            produced.replace(target_path)
    finally:
        _restore_view_layer_materials(saved)
        _restore_render_state(scene, st)
        try:
            bpy.data.materials.remove(mat)
        except Exception:
            pass


def render_mask(scene, camera, collection, target_path):
    """Binary foreground mask: collection objects = white, everything else = black, rendered
    Raw so the values are exact 0/1 (no tone curve). State-isolated."""
    if not collection:
        return
    st = _save_render_state(scene)
    ims = scene.render.image_settings
    white = bpy.data.materials.new("GS_TEMP_MASK_WHITE")
    black = bpy.data.materials.new("GS_TEMP_MASK_BLACK")
    for mat, color in ((white, (1, 1, 1, 1)), (black, (0, 0, 0, 1))):
        mat.diffuse_color = color
        mat.use_nodes = True
        mat.node_tree.nodes.clear()
        emission = mat.node_tree.nodes.new("ShaderNodeEmission")
        output = mat.node_tree.nodes.new("ShaderNodeOutputMaterial")
        emission.inputs["Color"].default_value = color
        emission.inputs["Strength"].default_value = 1.0
        mat.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    mask_names = {obj.name for obj in collection.objects}
    objects = [
        obj for obj in scene.objects
        if obj.type == "MESH" and not is_camera_mesh_visual(obj)
    ]
    object_counts, data_counts = _material_slot_counts(objects)
    saved = []
    for obj in objects:
        original_slot_count = object_counts[obj.as_pointer()]
        original_data_slot_count = data_counts[obj.data.as_pointer()][1]
        if not obj.material_slots:
            obj.data.materials.append(None)
        target_mat = white if obj.name in mask_names else black
        record = []
        for slot in obj.material_slots:
            record.append((slot.link, slot.material))
            slot.link = "OBJECT"
            slot.material = target_mat
        saved.append((obj, record, original_slot_count, original_data_slot_count))
    try:
        scene.camera = camera
        scene.render.use_persistent_data = False
        scene.render.use_compositing = False
        scene.render.filepath = str(target_path)
        scene.render.film_transparent = False
        apply_view_transform(scene, "Raw")
        ims.file_format = "PNG"
        ims.color_mode = "RGB"
        _render_still_with_hip_fallback(scene)
    finally:
        _restore_object_materials(saved)
        _restore_render_state(scene, st)
        try:
            bpy.data.materials.remove(white)
            bpy.data.materials.remove(black)
        except Exception:
            pass


def _is_collection_instance(obj):
    return (
        obj.type == "EMPTY"
        and obj.instance_type == "COLLECTION"
        and obj.instance_collection is not None
    )


def _object_groups(scene, mode="GROUP"):
    """Ordered list of (item_name, [scene objects]) at the chosen granularity:
      GROUP      -> all meshes under one top-most parent (Empty) are one item;
      MESH       -> every mesh object is its own item;
      COLLECTION -> every Blender collection is one item.
    Render-enabled Collection Instance empties are included because their evaluated meshes
    are visible even though those meshes are not members of scene.objects. Viewport visibility
    is intentionally ignored because a viewport-hidden object may still be rendered."""
    groups = {}
    order = []
    for obj in scene.objects:
        if obj.type != "MESH" and not _is_collection_instance(obj):
            continue
        if obj.type == "MESH" and is_camera_mesh_visual(obj):
            continue
        if obj.hide_render:
            continue
        if mode == "MESH":
            key = obj.name
        elif mode == "COLLECTION":
            key = obj.users_collection[0].name if obj.users_collection else "Ungrouped"
        else:  # GROUP: walk up to the top-most ancestor (usually an Empty)
            root = obj
            while root.parent is not None:
                root = root.parent
            key = root.name
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(obj)
    return [(k, groups[k]) for k in order]


def _create_collection_instance_id_proxies(scene, objmat):
    """Realize grouped Collection Instances as temporary ID-material mesh objects.

    The dependency graph supplies the evaluated mesh and exact world matrix used by the
    renderer. The original instancer is hidden only after all of its proxies are created.
    """
    depsgraph = bpy.context.evaluated_depsgraph_get()
    depsgraph.update()
    created = []
    visibility = []
    targets = {}
    counts = {}
    for instancer_name, material in objmat.items():
        instancer = scene.objects.get(instancer_name)
        if instancer is not None and _is_collection_instance(instancer):
            targets[instancer.as_pointer()] = (instancer, material)
            counts[instancer.as_pointer()] = 0
    try:
        # DepsgraphObjectInstance RNA handles expire as iteration advances, so copy each
        # evaluated mesh and matrix while the current handle is still valid.
        for instance in depsgraph.object_instances:
            if not getattr(instance, "is_instance", False):
                continue
            parent = getattr(instance, "parent", None)
            try:
                parent_original = parent.original if parent is not None else None
                target = targets.get(parent_original.as_pointer()) if parent_original else None
            except Exception:
                target = None
            if target is None:
                continue
            source = getattr(instance, "object", None)
            if source is None or source.type != "MESH" or not getattr(instance, "show_self", True):
                continue
            instancer, material = target
            pointer = instancer.as_pointer()
            index = counts[pointer]
            mesh = bpy.data.meshes.new_from_object(
                source,
                preserve_all_data_layers=False,
                depsgraph=depsgraph,
            )
            proxy = bpy.data.objects.new(
                f"GS_TEMP_ID_INSTANCE_{safe_name(instancer.name)}_{index:04d}",
                mesh,
            )
            proxy.matrix_world = instance.matrix_world.copy()
            mesh.materials.clear()
            mesh.materials.append(material)
            scene.collection.objects.link(proxy)
            created.append((proxy, mesh))
            counts[pointer] += 1
        for pointer, (instancer, _material) in targets.items():
            if counts[pointer] == 0:
                print(
                    f"[GS] warning: Collection Instance {instancer.name} has no evaluated meshes",
                    flush=True,
                )
                continue
            visibility.append((instancer, bool(instancer.hide_render)))
            instancer.hide_render = True
        return created, visibility
    except Exception:
        _remove_collection_instance_id_proxies(created, visibility)
        raise


def _remove_collection_instance_id_proxies(created, visibility):
    for instancer, hidden in visibility:
        try:
            if instancer.name in bpy.data.objects:
                instancer.hide_render = hidden
        except Exception:
            pass
    for proxy, mesh in reversed(created):
        try:
            bpy.data.objects.remove(proxy, do_unlink=True)
        except Exception:
            pass
        try:
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)
        except Exception:
            pass


def _group_color(index):
    """Distinct, well-separated flat colour for an item (golden-ratio hue spacing)."""
    import colorsys
    h = (index * 0.6180339887498949) % 1.0
    s = 0.7 + 0.3 * ((index // 7) % 2)   # vary saturation a little for very large counts
    r, g, b = colorsys.hsv_to_rgb(h, min(1.0, s), 1.0)
    return (r, g, b)


def build_id_groups(scene, settings):
    """Stable object groups: (name, numeric id, colour, objects).

    The ID is persisted on every member object so Outliner reordering cannot change a
    dataset mapping. New groups receive an ID above the current maximum.
    """
    raw = _object_groups(scene, getattr(settings, "object_group_mode", "GROUP"))
    used = {
        int(obj.get("autogs_object_id"))
        for _name, objects in raw for obj in objects
        if int(obj.get("autogs_object_id", 0) or 0) > 0
    }
    next_id = max(used, default=0) + 1
    groups = []
    assigned = set()
    for name, objects in raw:
        ids = sorted({
            int(obj.get("autogs_object_id")) for obj in objects
            if int(obj.get("autogs_object_id", 0) or 0) > 0
        })
        object_id = ids[0] if ids and ids[0] not in assigned else next_id
        if not ids or object_id == next_id:
            next_id += 1
        assigned.add(object_id)
        for obj in objects:
            obj["autogs_object_id"] = object_id
        groups.append((name, object_id, _group_color(object_id), objects))
    return sorted(groups, key=lambda item: (item[1], item[0]))


def render_id(scene, camera, target_path, groups):
    """Object-ID (segmentation) map: each item is rendered as its flat emission colour with the
    *Raw* view transform and a crisp pixel filter, so each item is an exact, decodable colour.
    Background transparent. State-isolated."""
    target_path = Path(target_path)
    st = _save_render_state(scene)
    ims = scene.render.image_settings
    mats = []
    objmat = {}
    for name, _object_id, color, objs in groups:
        m = bpy.data.materials.new(f"GS_TEMP_ID_{safe_name(name)}")
        m.use_nodes = True
        m.node_tree.nodes.clear()
        em = m.node_tree.nodes.new("ShaderNodeEmission")
        out = m.node_tree.nodes.new("ShaderNodeOutputMaterial")
        em.inputs["Color"].default_value = (color[0], color[1], color[2], 1.0)
        em.inputs["Strength"].default_value = 1.0
        m.node_tree.links.new(em.outputs["Emission"], out.inputs["Surface"])
        mats.append(m)
        for o in objs:
            objmat[o.name] = m
    saved = []
    instance_proxies = []
    instance_visibility = []
    objects = [
        obj for obj in scene.objects
        if obj.type == "MESH" and not is_camera_mesh_visual(obj) and obj.name in objmat
    ]
    object_counts, data_counts = _material_slot_counts(objects)
    for obj in objects:
        m = objmat.get(obj.name)
        if m is None:
            continue
        original_slot_count = object_counts[obj.as_pointer()]
        original_data_slot_count = data_counts[obj.data.as_pointer()][1]
        if not obj.material_slots:
            obj.data.materials.append(None)
        record = []
        for slot in obj.material_slots:
            record.append((slot.link, slot.material))
            slot.link = "OBJECT"
            slot.material = m
        saved.append((obj, record, original_slot_count, original_data_slot_count))
    cyc = None
    saved_cyc = None
    try:
        instance_proxies, instance_visibility = _create_collection_instance_id_proxies(
            scene, objmat
        )
        scene.camera = camera
        # ID rendering creates and removes evaluated proxy meshes every frame. Reusing the
        # persistent Cycles scene here eventually produces stale/unknown ID pixels and leaks VRAM.
        scene.render.use_persistent_data = False
        scene.render.use_compositing = False
        scene.render.film_transparent = True
        scene.render.filter_size = 0.01  # crisp edges
        # EXACT, decodable colours: 1 sample + BOX filter -> NO anti-alias blending between
        # items, so every opaque pixel is precisely one item's colour (kills stray pixels in
        # the per-object split). Restored with the rest of the cycles state below.
        cyc = getattr(scene, "cycles", None)
        if cyc is not None:
            try: saved_cyc = (cyc.samples, cyc.pixel_filter_type, cyc.use_denoising)
            except Exception: saved_cyc = None
            try: cyc.samples = 1
            except Exception: pass
            try: cyc.pixel_filter_type = "BOX"
            except Exception: pass
            try: cyc.use_denoising = False
            except Exception: pass
        apply_view_transform(scene, "Raw")
        ims.file_format = "PNG"
        ims.color_mode = "RGBA"
        ims.color_depth = "8"
        scene.render.filepath = str(target_path.with_suffix(""))
        _render_still_with_hip_fallback(scene)
        produced = Path(str(target_path.with_suffix("")) + ".png")
        if produced.exists() and produced != target_path:
            if target_path.exists():
                target_path.unlink()
            produced.replace(target_path)
    finally:
        _remove_collection_instance_id_proxies(instance_proxies, instance_visibility)
        _restore_object_materials(saved)
        _restore_render_state(scene, st)
        if cyc is not None and saved_cyc is not None:
            try: cyc.samples, cyc.pixel_filter_type, cyc.use_denoising = saved_cyc
            except Exception: pass
        for m in mats:
            try:
                bpy.data.materials.remove(m)
            except Exception:
                pass


def render_normal(scene, camera, target_path):
    """Render visible world-space surface normals as float32 XYZ in an EXR.

    Blender renders an encoded 0..1 emission first; the file is immediately decoded to
    true [-1, 1] values. Alpha remains the authoritative valid-surface mask.
    """
    import numpy as np

    target_path = Path(target_path)
    st = _save_render_state(scene)
    ims = scene.render.image_settings
    mat = bpy.data.materials.new("GS_TEMP_WORLD_NORMAL")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    geometry = nt.nodes.new("ShaderNodeNewGeometry")
    scale = nt.nodes.new("ShaderNodeVectorMath")
    scale.operation = "SCALE"
    scale.inputs[3].default_value = 0.5
    offset = nt.nodes.new("ShaderNodeVectorMath")
    offset.operation = "ADD"
    offset.inputs[1].default_value = (0.5, 0.5, 0.5)
    emission = nt.nodes.new("ShaderNodeEmission")
    output = nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(geometry.outputs["Normal"], scale.inputs[0])
    nt.links.new(scale.outputs["Vector"], offset.inputs[0])
    nt.links.new(offset.outputs["Vector"], emission.inputs["Color"])
    nt.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    saved = _override_view_layer_materials(scene, mat)
    try:
        scene.camera = camera
        scene.render.use_persistent_data = False
        scene.render.use_compositing = False
        scene.render.film_transparent = True
        apply_view_transform(scene, "Raw")
        ims.file_format = "OPEN_EXR"
        ims.color_mode = "RGBA"
        ims.color_depth = "32"
        scene.render.filepath = str(target_path.with_suffix(""))
        _render_still_with_hip_fallback(scene)
        produced = Path(str(target_path.with_suffix("")) + ".exr")
        if produced.exists() and produced != target_path:
            if target_path.exists():
                target_path.unlink()
            produced.replace(target_path)
        rgba, w, h = _read_exr_rgba(target_path)
        valid = rgba[:, :, 3] > 0.5
        normals = rgba[:, :, :3] * 2.0 - 1.0
        lengths = np.linalg.norm(normals, axis=2)
        valid &= np.isfinite(normals).all(axis=2) & (lengths > 1e-8)
        normals[valid] /= lengths[valid, None]
        normals[~valid] = 0.0
        rgba[:, :, :3] = normals
        rgba[:, :, 3] = valid.astype(np.float32)
        _write_exr(target_path, rgba.reshape(w * h * 4), w, h)
    finally:
        _restore_view_layer_materials(saved)
        _restore_render_state(scene, st)
        try:
            bpy.data.materials.remove(mat)
        except Exception:
            pass


def build_material_groups(scene):
    """Return stable (name, material_id, colour, material) records."""
    materials = sorted({
        slot.material for obj in scene.objects if obj.type == "MESH"
        for slot in obj.material_slots if slot.material is not None
    }, key=lambda material: material.name_full)
    used = {
        int(material.get("autogs_material_id")) for material in materials
        if int(material.get("autogs_material_id", 0) or 0) > 0
    }
    next_id = max(used, default=0) + 1
    groups = []
    assigned = set()
    for material in materials:
        material_id = int(material.get("autogs_material_id", 0) or 0)
        if material_id <= 0 or material_id in assigned:
            material_id = next_id
            next_id += 1
            material["autogs_material_id"] = material_id
        assigned.add(material_id)
        groups.append((material.name_full, material_id, _group_color(material_id), material))
    return sorted(groups, key=lambda item: (item[1], item[0]))


def render_material_id(scene, camera, target_path, material_groups):
    """Render visible material IDs once for the full scene, preserving face assignments."""
    target_path = Path(target_path)
    st = _save_render_state(scene)
    ims = scene.render.image_settings
    replacements = {}
    temporary = []
    for name, material_id, color, source in material_groups:
        mat = bpy.data.materials.new(f"GS_TEMP_MATERIAL_ID_{material_id}_{safe_name(name)}")
        mat.use_nodes = True
        mat.node_tree.nodes.clear()
        emission = mat.node_tree.nodes.new("ShaderNodeEmission")
        output = mat.node_tree.nodes.new("ShaderNodeOutputMaterial")
        emission.inputs["Color"].default_value = (*color, 1.0)
        mat.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
        replacements[source] = mat
        temporary.append(mat)
    objects = [
        obj for obj in scene.objects
        if obj.type == "MESH" and not is_camera_mesh_visual(obj)
    ]
    object_counts, data_counts = _material_slot_counts(objects)
    saved = []
    for obj in objects:
        original_slot_count = object_counts[obj.as_pointer()]
        original_data_slot_count = data_counts[obj.data.as_pointer()][1]
        record = []
        if not obj.material_slots:
            obj.data.materials.append(None)
        for slot in obj.material_slots:
            source = slot.material
            record.append((slot.link, source))
            replacement = replacements.get(source)
            if replacement is not None:
                slot.link = "OBJECT"
                slot.material = replacement
        saved.append((obj, record, original_slot_count, original_data_slot_count))
    cyc = getattr(scene, "cycles", None)
    saved_cyc = None
    try:
        scene.camera = camera
        scene.render.use_persistent_data = False
        scene.render.use_compositing = False
        scene.render.film_transparent = True
        scene.render.filter_size = 0.01
        if cyc is not None:
            try:
                saved_cyc = (cyc.samples, cyc.pixel_filter_type, cyc.use_denoising)
                cyc.samples, cyc.pixel_filter_type, cyc.use_denoising = 1, "BOX", False
            except Exception:
                pass
        apply_view_transform(scene, "Raw")
        ims.file_format = "PNG"
        ims.color_mode = "RGBA"
        ims.color_depth = "8"
        scene.render.filepath = str(target_path.with_suffix(""))
        _render_still_with_hip_fallback(scene)
        produced = Path(str(target_path.with_suffix("")) + ".png")
        if produced.exists() and produced != target_path:
            if target_path.exists():
                target_path.unlink()
            produced.replace(target_path)
    finally:
        _restore_object_materials(saved)
        _restore_render_state(scene, st)
        if cyc is not None and saved_cyc is not None:
            try:
                cyc.samples, cyc.pixel_filter_type, cyc.use_denoising = saved_cyc
            except Exception:
                pass
        for mat in temporary:
            try:
                bpy.data.materials.remove(mat)
            except Exception:
                pass


def _write_exr(path, rgba_flat, w, h):
    """Write a flat RGBA float buffer (len w*h*4, bottom-up like bpy pixels) to a 32-bit EXR."""
    img = bpy.data.images.new(Path(path).stem, width=w, height=h, float_buffer=True, alpha=True)
    try:
        img.colorspace_settings.name = "Non-Color"
    except Exception:
        pass
    img.pixels.foreach_set(rgba_flat)
    img.filepath_raw = str(path)
    img.file_format = "OPEN_EXR"
    try:
        img.save()
    finally:
        bpy.data.images.remove(img)


def _write_png(path, rgba_flat, w, h):
    """Write a flat RGBA float buffer (0..1) to an 8-bit PNG (sRGB-encoded for normal viewing)."""
    img = bpy.data.images.new(Path(path).stem, width=w, height=h, alpha=True)
    img.pixels.foreach_set(rgba_flat)
    img.filepath_raw = str(path)
    img.file_format = "PNG"
    try:
        img.save()
    finally:
        bpy.data.images.remove(img)


DEPTH_PNG_SCALE_MM = 1000.0  # depth(m) -> 16-bit PNG value in millimetres (max 65.535 m)


def _save_raw_png(path, rgba_hw4, w, h, depth_bits):
    """Write a flat 0..1 buffer to a grayscale PNG with NO colour management (Raw view transform +
    Non-Color), so the stored bytes are EXACTLY the input values (metric depth / binary mask)."""
    img = bpy.data.images.new(Path(path).stem, w, h, alpha=False, float_buffer=(depth_bits == "16"))
    try:
        img.colorspace_settings.name = "Non-Color"
    except Exception:
        pass
    img.pixels.foreach_set(rgba_hw4.reshape(-1))
    sc = bpy.context.scene
    ims = sc.render.image_settings
    sv = (ims.file_format, ims.color_mode, ims.color_depth, sc.view_settings.view_transform)
    ims.file_format = "PNG"; ims.color_mode = "BW"; ims.color_depth = depth_bits
    try:
        sc.view_settings.view_transform = "Raw"
    except Exception:
        pass
    try:
        img.save_render(str(path), scene=sc)
    finally:
        ims.file_format, ims.color_mode, ims.color_depth, sc.view_settings.view_transform = sv
        try:
            bpy.data.images.remove(img)
        except Exception:
            pass


def _write_png16_depth(path, depth_m, mask, w, h, scale=DEPTH_PNG_SCALE_MM):
    """16-bit grayscale PNG of depth in MILLIMETRES (value = depth_m*scale, clamped 65535), exact &
    metric-recoverable (depth_metres = pixel/65535*65535 / scale, i.e. raw16/scale). Background = 0."""
    import numpy as np
    mm = np.clip(np.where(mask, depth_m, 0.0) * scale, 0.0, 65535.0) / 65535.0
    rgba = np.zeros((h, w, 4), np.float32)
    rgba[:, :, 0] = rgba[:, :, 1] = rgba[:, :, 2] = mm
    rgba[:, :, 3] = 1.0
    _save_raw_png(path, rgba, w, h, "16")


def _write_png16_normal(path, normal_rgba, w, h):
    """Write world normals as a Raw 16-bit RGBA PNG.

    RGB stores the reversible encoding ``(normal + 1) / 2`` and alpha is the
    visible-surface mask.  This keeps the physical object output compact while
    retaining substantially more precision than an 8-bit normal map.
    """
    import numpy as np
    encoded = np.empty((h, w, 4), np.float32)
    encoded[:, :, :3] = np.clip(normal_rgba[:, :, :3] * 0.5 + 0.5, 0.0, 1.0)
    encoded[:, :, 3] = np.clip(normal_rgba[:, :, 3], 0.0, 1.0)
    img = bpy.data.images.new(Path(path).stem, w, h, alpha=True, float_buffer=True)
    try:
        try:
            img.colorspace_settings.name = "Non-Color"
        except Exception:
            pass
        img.pixels.foreach_set(encoded.reshape(-1))
        img.filepath_raw = str(path)
        img.file_format = "PNG"
        img.save()
    finally:
        try:
            bpy.data.images.remove(img)
        except Exception:
            pass


def _write_png_mask(path, mask, w, h):
    """8-bit white/black PNG for a per-object GS mask: mask True = white(255), else black(0)."""
    import numpy as np
    g = mask.astype(np.float32)
    rgba = np.zeros((h, w, 4), np.float32)
    rgba[:, :, 0] = rgba[:, :, 1] = rgba[:, :, 2] = g
    rgba[:, :, 3] = 1.0
    _save_raw_png(path, rgba, w, h, "8")


def _read_depth_exr(path):
    """(depth[h,w], mask[h,w]>0, w, h) from a float depth EXR (R = View-Z metres, A = geometry)."""
    import numpy as np
    img = bpy.data.images.load(str(path), check_existing=False)
    w, h = img.size
    arr = np.empty(len(img.pixels), np.float32); img.pixels.foreach_get(arr)
    try:
        bpy.data.images.remove(img)
    except Exception:
        pass
    arr = arr.reshape(h, w, 4)
    return arr[:, :, 0], (arr[:, :, 3] > 0.5), w, h


def _read_exr_rgba(path):
    """Read a float EXR into Blender's bottom-up (height, width, RGBA) orientation."""
    import numpy as np
    img = bpy.data.images.load(str(path), check_existing=False)
    w, h = img.size
    arr = np.empty(len(img.pixels), np.float32)
    img.pixels.foreach_get(arr)
    try:
        bpy.data.images.remove(img)
    except Exception:
        pass
    return arr.reshape(h, w, 4), w, h


def _read_id_rgba(path):
    """RGBA float array (h,w,4) of an id-map PNG, loaded the same way the colours were written so
    pixels round-trip to the linear item colours (default colorspace, as the legacy split did)."""
    import numpy as np
    img = bpy.data.images.load(str(path), check_existing=False)
    w, h = img.size
    arr = np.empty(len(img.pixels), np.float32); img.pixels.foreach_get(arr)
    try:
        bpy.data.images.remove(img)
    except Exception:
        pass
    return arr.reshape(h, w, 4)


# a pixel must match an item colour at least this tightly (linear RGB distance) to be kept --
# distinct item hues are >0.3 apart and the id render is exact, so this safely drops edge /
# ambiguous / background pixels instead of dumping them onto the nearest item.
_ID_MATCH_THRESHOLD2 = 0.13 ** 2
_ID_UNKNOWN_WARNING_RATIO = 0.0001


def _id_regions(idp, groups):
    """From an id RGBA array -> (nearest_item_index[h,w], matched_mask[h,w]) with a STRICT colour
    match so an item never picks up stray / blended / background pixels."""
    import numpy as np
    h, w, _ = idp.shape
    idrgb = idp[:, :, :3]
    idalpha = idp[:, :, 3] > 0.5
    nearest = np.full((h, w), -1, np.int32)
    best = np.full((h, w), 1e18, np.float32)
    for i, (_name, _object_id, color, _objs) in enumerate(groups):
        c = np.array(color, np.float32)
        d2 = ((idrgb - c[None, None, :]) ** 2).sum(axis=2)
        upd = d2 < best
        nearest[upd] = i
        best[upd] = d2[upd]
    matched = idalpha & (best < _ID_MATCH_THRESHOLD2)
    return nearest, matched


def split_object_depth(depth, dmask, idp, groups, objects_root, stem, depth_format):
    """Cut the full-scene depth into one file per item by masking with the id map (occlusion-aware,
    strict colour match). Writes objects/<item>/depth/<stem>.(exr|png) in the chosen format."""
    import numpy as np
    h, w = depth.shape
    nearest, matched = _id_regions(idp, groups)
    valid = matched & dmask
    written = 0
    for i, (name, _object_id, _color, _objs) in enumerate(groups):
        m = valid & (nearest == i)
        if not m.any():
            continue
        ddir = Path(objects_root) / safe_name(name) / "depth"
        ensure_dir(ddir)
        if depth_format == "PNG":
            _write_png16_depth(ddir / (stem + ".png"), depth, m, w, h)
        else:
            md = np.where(m, depth, 0.0)
            out = np.zeros((h, w, 4), np.float32)
            out[:, :, 0] = out[:, :, 1] = out[:, :, 2] = md
            out[:, :, 3] = m.astype(np.float32)
            _write_exr(ddir / (stem + ".exr"), out.reshape(w * h * 4), w, h)
        written += 1
    return written


def split_object_mask(idp, groups, objects_root, stem):
    """Per-object binary GS mask (white = this item, black = everything else) from the id map.
    Writes objects/<item>/mask/<stem>.png."""
    h, w, _ = idp.shape
    nearest, matched = _id_regions(idp, groups)
    written = 0
    for i, (name, _object_id, _color, _objs) in enumerate(groups):
        m = matched & (nearest == i)
        if not m.any():
            continue
        mdir = Path(objects_root) / safe_name(name) / "mask"
        ensure_dir(mdir)
        _write_png_mask(mdir / (stem + ".png"), m, w, h)
        written += 1
    return written


def split_object_normal(normal_rgba, idp, groups, objects_root, stem):
    """Split a full-scene world-normal buffer into encoded 16-bit PNG files."""
    import numpy as np
    h, w, _ = normal_rgba.shape
    nearest, matched = _id_regions(idp, groups)
    normal_valid = normal_rgba[:, :, 3] > 0.5
    written = 0
    for i, (name, _object_id, _color, _objs) in enumerate(groups):
        mask = matched & normal_valid & (nearest == i)
        if not mask.any():
            continue
        output = np.zeros((h, w, 4), np.float32)
        output[:, :, :3][mask] = normal_rgba[:, :, :3][mask]
        output[:, :, 3] = mask.astype(np.float32)
        directory = Path(objects_root) / safe_name(name) / "normal"
        ensure_dir(directory)
        _write_png16_normal(directory / (stem + ".png"), output, w, h)
        written += 1
    return written


def _validate_frame_buffers(stem, depth_data=None, normal_rgba=None, idp=None, groups=None):
    """Fail fast on alignment, non-finite geometry data, or invalid unit normals."""
    import numpy as np
    shapes = []
    if depth_data is not None:
        depth, depth_mask = depth_data
        shapes.append(("depth", depth.shape))
        if not np.isfinite(depth[depth_mask]).all() or (depth[depth_mask] < 0).any():
            raise ValueError(f"{stem}: invalid depth value")
    if normal_rgba is not None:
        shapes.append(("normal", normal_rgba.shape[:2]))
        valid = normal_rgba[:, :, 3] > 0.5
        normals = normal_rgba[:, :, :3][valid]
        if not np.isfinite(normals).all():
            raise ValueError(f"{stem}: NaN/Inf normal")
        if normals.size:
            lengths = np.linalg.norm(normals, axis=1)
            if not np.allclose(lengths, 1.0, atol=2e-3):
                raise ValueError(f"{stem}: non-unit world normal")
    if idp is not None:
        shapes.append(("object_id", idp.shape[:2]))
        if groups:
            _nearest, matched = _id_regions(idp, groups)
            opaque = idp[:, :, 3] > 0.5
            unknown = opaque & ~matched
            unknown_count = int(np.count_nonzero(unknown))
            if unknown_count:
                opaque_count = int(np.count_nonzero(opaque))
                ratio = unknown_count / max(1, opaque_count)
                message = (
                    f"{stem}: {unknown_count}/{opaque_count} unknown Object ID pixels "
                    f"({ratio:.5%})"
                )
                if ratio > _ID_UNKNOWN_WARNING_RATIO:
                    raise ValueError(
                        message + "; render-visible geometry is not covered by Object ID groups"
                    )
                print(f"[GS] warning: {message}; treating strict edge pixels as background", flush=True)
    if shapes and any(shape != shapes[0][1] for _name, shape in shapes[1:]):
        raise ValueError(f"{stem}: pass resolution mismatch: {shapes}")


def _hide_dataset_path_geometry(settings):
    """Hide camera-layout curves during all dataset passes and return restore records."""
    collection = getattr(settings, "path_collection", None)
    if collection is None:
        return []
    saved = []
    for obj in collection.all_objects:
        # Linked collections can expose a stale empty RNA slot after a library reload.
        if obj is None:
            continue
        try:
            obj_type = obj.type
        except (AttributeError, ReferenceError):
            continue
        if obj_type not in {"CURVE", "SURFACE", "FONT", "CURVES"}:
            continue
        try:
            hidden = bool(obj.hide_render)
            saved.append((obj, hidden))
            obj.hide_render = True
        except Exception as exc:
            try:
                obj_name = obj.name
            except Exception:
                obj_name = "<stale object>"
            print(f"[GS] could not hide path geometry {obj_name}: {exc}", flush=True)
    return saved


def _restore_dataset_path_geometry(saved):
    for obj, hidden in saved:
        try:
            if obj.name in bpy.data.objects:
                obj.hide_render = hidden
        except Exception:
            pass


def _release_render_buffers():
    """Drop Blender's transient Render Result after each frame.

    The saved files are already on disk before this is called. Keeping the result image
    alive across thousands of HIP frames unnecessarily retains a large host-side buffer and
    can keep allocator blocks reachable until the worker exits.
    """
    try:
        result = bpy.data.images.get("Render Result")
        if result is not None:
            bpy.data.images.remove(result)
    except Exception:
        pass
    try:
        import gc
        gc.collect()
    except Exception:
        pass


def _render_one_view_impl(scene, settings, camera, stem, dirs, extension, plan, groups,
                          material_groups, depth_format):
    """Render exactly the resolved passes for one camera and save only planned products."""
    scene.camera = camera
    required = plan.required_internal_passes
    config = plan.config
    failures = {"depth": 0, "normal": 0, "mask": 0, "id": 0, "material_id": 0}
    if output_pipeline.PASS_BEAUTY in required:
        scene.render.filepath = str(dirs["images"] / (stem + extension))
        _render_still_with_hip_fallback(scene)

    keep_exr = plan.saves(output_pipeline.SCENE_DEPTH) and depth_format == "EXR"
    depth_render_path = (dirs["depth"] / (stem + ".exr")) if keep_exr else (dirs["_tmp"] / (stem + "_depth.exr"))
    depth = dmask = None
    dw = dh = 0
    if output_pipeline.PASS_DEPTH in required:
        try:
            ensure_dir(depth_render_path.parent)
            render_depth(scene, camera, depth_render_path)
            depth, dmask, dw, dh = _read_depth_exr(depth_render_path)
        except Exception as exc:
            failures["depth"] = 1
            print(f"[GS] depth render failed for {stem}: {exc}")
    if plan.saves(output_pipeline.SCENE_DEPTH) and depth is not None and depth_format == "PNG":
        try:
            _write_png16_depth(dirs["depth"] / (stem + ".png"), depth, dmask, dw, dh)
        except Exception as exc:
            failures["depth"] = 1
            print(f"[GS] depth PNG write failed for {stem}: {exc}")

    normal_rgba = None
    # Normals are rendered internally as EXR for lossless decode and validation,
    # then persisted as compact 16-bit PNG.  Depth remains independently EXR.
    normal_render_path = dirs["_tmp"] / (stem + "_normal.exr")
    if output_pipeline.PASS_NORMAL in required:
        try:
            ensure_dir(normal_render_path.parent)
            render_normal(scene, camera, normal_render_path)
            normal_rgba, nw, nh = _read_exr_rgba(normal_render_path)
        except Exception as exc:
            failures["normal"] = 1
            print(f"[GS] normal render failed for {stem}: {exc}")

    idp = None
    if output_pipeline.PASS_OBJECT_ID in required:
        keep_id = plan.saves(output_pipeline.OBJECT_ID)
        id_tmp = (dirs["id"] / (stem + ".png")) if keep_id else (dirs["_tmp"] / (stem + "_id.png"))
        try:
            ensure_dir(id_tmp.parent)
            render_id(scene, camera, id_tmp, groups)
            idp = _read_id_rgba(id_tmp)
        except Exception as exc:
            failures["id"] = 1
            print(f"[GS] id render failed for {stem}: {exc}")
        finally:
            try:
                if not keep_id and id_tmp.exists():
                    id_tmp.unlink()
            except Exception:
                pass

    if output_pipeline.PASS_MATERIAL_ID in required:
        try:
            render_material_id(scene, camera, dirs["material_id"] / (stem + ".png"), material_groups)
        except Exception as exc:
            failures["material_id"] = 1
            print(f"[GS] material ID render failed for {stem}: {exc}")

    try:
        _validate_frame_buffers(
            stem,
            (depth, dmask) if depth is not None else None,
            normal_rgba,
            idp,
            groups,
        )
    except Exception as exc:
        print(f"[GS] frame validation failed for {stem}: {exc}")
        raise

    if plan.saves(output_pipeline.SCENE_NORMAL) and normal_rgba is not None:
        try:
            _write_png16_normal(dirs["normal"] / (stem + ".png"), normal_rgba, nw, nh)
        except Exception as exc:
            failures["normal"] = 1
            print(f"[GS] normal PNG write failed for {stem}: {exc}")

    if config.object_depth and plan.physical_split and depth is not None and idp is not None:
        try:
            split_object_depth(depth, dmask, idp, groups, dirs["objects"], stem, depth_format)
        except Exception as exc:
            failures["depth"] = 1
            print(f"[GS] per-object depth split failed for {stem}: {exc}")
    if config.object_normal and plan.physical_split and normal_rgba is not None and idp is not None:
        try:
            split_object_normal(normal_rgba, idp, groups, dirs["objects"], stem)
        except Exception as exc:
            failures["normal"] = 1
            print(f"[GS] per-object normal split failed for {stem}: {exc}")
    if config.object_mask and plan.physical_split and idp is not None:
        try:
            split_object_mask(idp, groups, dirs["objects"], stem)
        except Exception as exc:
            failures["mask"] = 1
            print(f"[GS] per-object mask split failed for {stem}: {exc}")
    if not keep_exr:
        try:
            if depth_render_path.exists():
                depth_render_path.unlink()
        except Exception:
            pass
    try:
        normal_render_path.unlink(missing_ok=True)
    except Exception:
        pass
    return failures


def _render_one_view(scene, settings, camera, stem, dirs, extension, plan, groups,
                     material_groups, depth_format):
    """Render one aligned frame while excluding camera-layout path geometry."""
    path_visibility = _hide_dataset_path_geometry(settings)
    try:
        return _render_one_view_impl(
            scene, settings, camera, stem, dirs, extension, plan, groups,
            material_groups, depth_format,
        )
    finally:
        _restore_dataset_path_geometry(path_visibility)
        _release_render_buffers()


def _referenced_cycles_images():
    """Return image-texture issues with an owner name, without mutating the scene."""
    owners = []
    for material in bpy.data.materials:
        if material.use_nodes and material.node_tree is not None:
            owners.append((f"material:{material.name}", material.node_tree))
    world = getattr(bpy.context.scene, "world", None)
    if world and world.use_nodes and world.node_tree is not None:
        owners.append((f"world:{world.name}", world.node_tree))
    for group in bpy.data.node_groups:
        owners.append((f"node_group:{group.name}", group))

    issues = []
    seen = set()
    for owner, node_tree in owners:
        for node in getattr(node_tree, "nodes", ()):
            image = getattr(node, "image", None)
            if image is None:
                continue
            key = (owner, node.name, image.name)
            if key in seen:
                continue
            seen.add(key)
            if getattr(image, "packed_file", None) is not None or image.source != "FILE":
                continue
            try:
                raw_path = image.filepath_from_user()
            except Exception:
                raw_path = image.filepath
            if not raw_path:
                issues.append(f"{owner}/{node.name}: image '{image.name}' has an empty filepath")
                continue
            try:
                absolute_path = bpy.path.abspath(raw_path, library=image.library)
            except Exception:
                absolute_path = raw_path
            if not Path(absolute_path).is_file():
                issues.append(
                    f"{owner}/{node.name}: image '{image.name}' missing at '{absolute_path}'"
                )
    return issues


def _log_render_preflight(scene, settings):
    """Print render-relevant scene diagnostics before Cycles builds its device scene."""
    mesh_count = 0
    polygon_count = 0
    vertex_count = 0
    hidden_mesh_count = 0
    geometry_nodes = 0
    camera_count = 0
    for obj in scene.objects:
        if obj.type == "CAMERA":
            camera_count += 1
        if obj.type != "MESH" or is_camera_mesh_visual(obj):
            continue
        if obj.hide_render:
            hidden_mesh_count += 1
            continue
        mesh_count += 1
        try:
            polygon_count += len(obj.data.polygons)
            vertex_count += len(obj.data.vertices)
        except Exception:
            pass
        try:
            geometry_nodes += sum(modifier.type == "NODES" for modifier in obj.modifiers)
        except Exception:
            pass

    evaluated_instances = 0
    evaluated_triangles = 0
    evaluated_vertices = 0
    try:
        depsgraph = bpy.context.evaluated_depsgraph_get()
        for instance in depsgraph.object_instances:
            obj = getattr(instance, "object", None)
            if obj is None or obj.type != "MESH" or is_camera_mesh_visual(obj):
                continue
            data = getattr(obj, "data", None)
            if data is None:
                continue
            evaluated_instances += 1
            evaluated_triangles += len(data.polygons)
            evaluated_vertices += len(data.vertices)
    except Exception as exc:
        print(f"[GS] Render preflight: evaluated instance count unavailable ({exc})", flush=True)

    width = int(getattr(scene.render, "resolution_x", 0))
    height = int(getattr(scene.render, "resolution_y", 0))
    percentage = int(getattr(scene.render, "resolution_percentage", 100))
    image_issues = _referenced_cycles_images()
    print(
        f"[GS] Render preflight: backend={scene.get('gs_cycles_backend', 'CPU')} "
        f"hiprt={scene.get('gs_hiprt_enabled', False)} "
        f"resolution={width}x{height}@{percentage}% cameras={camera_count} "
        f"visible_meshes={mesh_count} hidden_meshes={hidden_mesh_count} "
        f"mesh_vertices={vertex_count} mesh_polygons={polygon_count} "
        f"geometry_nodes={geometry_nodes} evaluated_mesh_instances={evaluated_instances} "
        f"evaluated_vertices={evaluated_vertices} evaluated_polygons={evaluated_triangles} "
        f"image_issues={len(image_issues)}",
        flush=True,
    )
    for issue in image_issues[:20]:
        print(f"[GS] Render asset warning: {issue}", flush=True)
    if len(image_issues) > 20:
        print(f"[GS] Render asset warning: {len(image_issues) - 20} additional image issue(s)", flush=True)


def _valid_output_file(path):
    """Reject truncated render files, not merely missing or zero-byte files."""
    path = Path(path)
    try:
        size = path.stat().st_size
        if size <= 0:
            return False
        suffix = path.suffix.lower()
        with path.open("rb") as handle:
            head = handle.read(16)
            if suffix == ".png":
                if head[:8] != b"\x89PNG\r\n\x1a\n" or size < 20:
                    return False
                handle.seek(max(0, size - 32))
                return b"IEND" in handle.read(32)
            if suffix in {".jpg", ".jpeg"}:
                if head[:2] != b"\xff\xd8" or size < 4:
                    return False
                handle.seek(-2, os.SEEK_END)
                return handle.read(2) == b"\xff\xd9"
            if suffix == ".exr":
                return head[:4] == b"\x76\x2f\x31\x01"
        return True
    except (OSError, ValueError):
        return False


def _chunk_bounds(total, complete_flags, chunk_control):
    if chunk_control is None:
        return 1, total
    size = max(1, int(chunk_control.get("size", 250) or 250))
    requested = int(chunk_control.get("requested_start", 0) or 0)
    force_render = bool(chunk_control.get("force_render", False))
    search_start = max(1, requested if requested > 0 else 1)
    if force_render:
        start = search_start
    else:
        start = next(
            (index for index in range(search_start, total + 1) if not complete_flags[index - 1]),
            total + 1,
        )
    end = min(total, start + size - 1) if start <= total else total
    chunk_control.update({"start": start, "end": end, "total": total})
    return start, end


def _finish_chunk_control(chunk_control, complete_flags, rendered, verified, force_render=False):
    if chunk_control is None:
        return all(complete_flags)
    total = len(complete_flags)
    start = int(chunk_control.get("start", total + 1))
    end = int(chunk_control.get("end", total))
    complete = (end >= total) if force_render else all(complete_flags)
    chunk_control.update({
        "state": "done" if complete else "chunk_done",
        "complete": bool(complete),
        "next_start": 0 if complete else end + 1,
        "rendered": int(rendered),
        "verified": bool(verified),
        "start": start,
        "end": end,
        "total": total,
    })
    return complete


def _resolved_output_plan(settings):
    return output_pipeline.resolve_required_passes(
        output_pipeline.RenderOutputConfig.from_settings(settings)
    )


def _frame_expected_outputs(root, settings, stem, image_name):
    root = Path(root)
    plan = _resolved_output_plan(settings)
    expected = []
    if plan.saves(output_pipeline.RGB):
        expected.append(root / "images" / image_name)
    if plan.saves(output_pipeline.SCENE_DEPTH):
        suffix = ".exr" if getattr(settings, "depth_format", "EXR") == "EXR" else ".png"
        expected.append(root / "depth" / f"{stem}{suffix}")
    if plan.saves(output_pipeline.SCENE_NORMAL):
        expected.append(root / "normal" / f"{stem}.png")
    if plan.saves(output_pipeline.OBJECT_ID):
        expected.append(root / "id" / f"{stem}.png")
    if plan.saves(output_pipeline.MATERIAL_ID):
        expected.append(root / "material_id" / f"{stem}.png")
    return expected


def _sequence_expected_outputs(root, settings, sample):
    image_name = Path(sample.rgb_path).name
    stem = Path(sample.rgb_path).stem
    expected = _frame_expected_outputs(root, settings, stem, image_name)
    return expected


def _sequence_commit_path(root, sample):
    return Path(root) / "_gs_frame_commits" / f"frame_{sample.logical_frame_id:06d}.json"


def _sequence_output_complete(root, settings, sample):
    if not all(_valid_output_file(path) for path in _sequence_expected_outputs(root, settings, sample)):
        return False
    marker = _sequence_commit_path(root, sample)
    try:
        payload = json.loads(marker.read_text(encoding="utf-8"))
    except Exception:
        return False
    files = payload.get("files", ())
    return isinstance(files, (list, tuple)) and all(
        _valid_output_file(Path(root) / relative)
        for relative in files
    )


def _legacy_commit_path(root, index):
    return Path(root) / "_gs_frame_commits" / f"legacy_{int(index):06d}.json"


def _legacy_commit_manifest_path(root):
    return Path(root) / "_gs_frame_commits" / "legacy_manifest.json"


def _write_legacy_commit_manifest(root):
    pose_sequence.atomic_write_json(_legacy_commit_manifest_path(root), {"version": 1})


def _legacy_required_outputs(root, settings, stem, image_name):
    return _frame_expected_outputs(root, settings, stem, image_name)


def _legacy_output_complete(root, settings, index, stem, image_name):
    root = Path(root)
    required = _legacy_required_outputs(root, settings, stem, image_name)
    if not all(_valid_output_file(path) for path in required):
        return False

    # RGB, whole-scene depth and ID have deterministic paths, so structurally valid files can be
    # recovered from pre-commit versions. Per-object outputs are visibility-dependent and still
    # require their marker to prove that the whole frame finished.
    marker = _legacy_commit_path(root, index)
    plan = _resolved_output_plan(settings)
    if not marker.exists():
        return not (
            plan.physical_split and (
                plan.config.object_depth or plan.config.object_normal or plan.config.object_mask
            )
        )
    try:
        payload = json.loads(marker.read_text(encoding="utf-8"))
    except Exception:
        return False
    if payload.get("backend") != "CAMERA_OBJECTS" or payload.get("stem") != stem:
        return False
    files = payload.get("files", ())
    return isinstance(files, (list, tuple)) and all(
        _valid_output_file(root / relative)
        for relative in files
    )


def _write_legacy_commit(root, settings, index, stem, image_name):
    root = Path(root)
    files = []
    for path in _legacy_required_outputs(root, settings, stem, image_name):
        if not _valid_output_file(path):
            raise RuntimeError(f"Required frame output is missing: {path}")
        files.append(path.relative_to(root).as_posix())

    object_patterns = []
    depth_extension = ".exr" if getattr(settings, "depth_format", "EXR") == "EXR" else ".png"
    plan = _resolved_output_plan(settings)
    if plan.physical_split and plan.config.object_depth:
        object_patterns.append(f"*/depth/{stem}{depth_extension}")
    if plan.physical_split and plan.config.object_normal:
        object_patterns.append(f"*/normal/{stem}.png")
    if plan.physical_split and plan.config.object_mask:
        object_patterns.append(f"*/mask/{stem}.png")
    objects_root = root / "objects"
    for pattern in object_patterns:
        for path in sorted(objects_root.glob(pattern)):
            if _valid_output_file(path):
                files.append(path.relative_to(root).as_posix())

    pose_sequence.atomic_write_json(
        _legacy_commit_path(root, index),
        {
            "backend": "CAMERA_OBJECTS",
            "index": int(index),
            "stem": stem,
            "files": sorted(set(files)),
        },
    )


def _discover_legacy_done(root, settings, items):
    """Find complete frames on disk and migrate safe pre-commit outputs in place."""
    done = set()
    recovered = 0
    for index, (_camera, stem, image_name) in enumerate(items, start=1):
        if not _legacy_output_complete(root, settings, index, stem, image_name):
            continue
        done.add(index)
        marker = _legacy_commit_path(root, index)
        if marker.exists():
            continue
        try:
            _write_legacy_commit(root, settings, index, stem, image_name)
            recovered += 1
        except Exception as exc:
            print(f"[GS] could not rebuild frame commit {index}: {exc}", flush=True)
    return done, recovered


def _commit_staged_frame(stage_root, output_root):
    files = [
        path for path in Path(stage_root).rglob("*")
        if path.is_file() and "_tmp" not in path.relative_to(stage_root).parts
    ]
    committed = []
    for path in sorted(files, key=lambda item: item.as_posix()):
        relative = path.relative_to(stage_root)
        target = Path(output_root) / relative
        ensure_dir(target.parent)
        os.replace(path, target)
        committed.append(relative.as_posix())
    return committed


def _verify_capture_pose(scene, settings, capture_camera, sample, sequence, position_tolerance=1e-6, rotation_tolerance=1e-5):
    scene.camera = capture_camera
    capture_camera.matrix_world = sample.matrix_world.copy()
    bpy.context.view_layer.update()
    evaluated = capture_camera.evaluated_get(bpy.context.evaluated_depsgraph_get())
    actual = evaluated.matrix_world.copy()
    position_error = float((actual.translation - sample.matrix_world.translation).length)
    rotation_error = float(
        actual.to_quaternion().rotation_difference(sample.matrix_world.to_quaternion()).angle
    )
    sample.verification_position_error = position_error
    sample.verification_rotation_error = rotation_error
    if scene.camera is not capture_camera:
        raise RuntimeError("GS_CAPTURE_CAMERA is not the active render camera")
    current_hash = pose_sequence.intrinsics_hash(
        pose_sequence.shared_intrinsics(scene, settings, capture_camera.data)
    )
    if current_hash != sequence.sequence_intrinsics_hash:
        raise ValueError(f"Pose {sample.logical_frame_id} intrinsics changed during rendering")
    if position_error > position_tolerance or rotation_error > rotation_tolerance:
        raise RuntimeError(
            f"Evaluated pose mismatch for logical frame {sample.logical_frame_id}: "
            f"position={position_error:.9g}, rotation={rotation_error:.9g}"
        )
    return position_error, rotation_error


def _render_pose_sequence_dataset(
    scene, settings, sequence, progress_path=None, patch_only=False, chunk_control=None
):
    import shutil

    capture = bpy.data.objects.get(pose_sequence.CAPTURE_CAMERA_NAME)
    if capture is None or capture.type != "CAMERA":
        raise RuntimeError("GS_CAPTURE_CAMERA is missing; rebuild the scientific pose sequence.")
    pose_sequence.freeze_sequence(sequence)
    pose_sequence.validate_intrinsics(scene, settings, capture, sequence)
    if sequence.planning_settings_hash and sequence.planning_settings_hash != pose_sequence.planning_settings_signature(settings):
        raise ValueError("Scientific planning settings changed after camera_sequence.json was created")
    if sequence.output_settings_hash and sequence.output_settings_hash != pose_sequence.output_settings_signature(settings):
        raise ValueError("Render output settings changed after camera_sequence.json was created")
    root = Path(settings.output_dir)
    target_samples = [
        sample for sample in sequence.frames
        if sample.render_enabled and (not patch_only or sample.is_coverage_patch)
    ]
    if patch_only and not target_samples:
        raise RuntimeError("No appended Coverage Patch poses are available.")
    if not target_samples:
        raise RuntimeError("The camera sequence has no render-enabled poses.")
    incremental = bool(getattr(settings, "incremental", True)) or patch_only
    force_render = bool(chunk_control and chunk_control.get("force_render")) and not patch_only
    if not incremental and chunk_control is None:
        for sample in target_samples:
            sample.render_status = "PENDING"
            sample.error = ""
    for sample in sequence.frames:
        if not sample.render_enabled:
            sample.render_status = "SKIPPED"
        elif incremental:
            if _sequence_output_complete(root, settings, sample):
                sample.render_status = "COMPLETE"
                sample.error = ""
            elif sample.render_status == "COMPLETE":
                sample.render_status = "PENDING"
    pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
    total = len(target_samples)
    complete_flags = [
        sample.render_status == "COMPLETE" and _sequence_output_complete(root, settings, sample)
        for sample in target_samples
    ]
    chunk_start, chunk_end = _chunk_bounds(total, complete_flags, chunk_control)
    if force_render and chunk_start <= chunk_end:
        for sample in target_samples[chunk_start - 1:chunk_end]:
            sample.render_status = "PENDING"
            sample.error = ""
        complete_flags[chunk_start - 1:chunk_end] = [False] * (chunk_end - chunk_start + 1)
        if chunk_start == 1:
            shutil.rmtree(root / "_gs_frame_commits", ignore_errors=True)
    chunk_samples = target_samples[chunk_start - 1:chunk_end] if chunk_start <= chunk_end else []
    pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)

    plan = _resolved_output_plan(settings)
    if not plan.has_outputs:
        raise ValueError("Select at least one Render Output.")
    depth_format = getattr(settings, "depth_format", "EXR")
    groups = build_id_groups(scene, settings) if plan.needs_object_groups else []
    material_groups = build_material_groups(scene) if output_pipeline.PASS_MATERIAL_ID in plan.required_internal_passes else []
    extension = ".png" if settings.image_format == "PNG" else ".jpg"
    ensure_dir(root)
    if plan.needs_object_groups:
        ensure_dir(root / "metadata")
        pose_sequence.atomic_write_json(
            root / "metadata" / "object_map.json",
            {"items": [{"id": object_id, "name": name, "color": list(color), "objects": [obj.name for obj in objects]} for name, object_id, color, objects in groups]},
        )
        if plan.saves(output_pipeline.OBJECT_ID):
            ensure_dir(root / "id")
            pose_sequence.atomic_write_json(
                root / "id" / "id_map.json",
                {"items": [{"id": object_id, "name": name, "color": list(color), "objects": [obj.name for obj in objects]} for name, object_id, color, objects in groups]},
            )
        if plan.physical_split:
            ensure_dir(root / "objects")
            pose_sequence.atomic_write_json(
                root / "objects" / "items.json",
                {"items": [{"id": object_id, "name": name, "objects": [obj.name for obj in objects]} for name, object_id, _color, objects in groups]},
            )
    if material_groups:
        ensure_dir(root / "material_id")
        material_map = {"items": [{"id": material_id, "name": name, "color": list(color)} for name, material_id, color, _material in material_groups]}
        pose_sequence.atomic_write_json(root / "material_id" / "material_map.json", material_map)
        ensure_dir(root / "metadata")
        pose_sequence.atomic_write_json(root / "metadata" / "material_map.json", material_map)

    old_camera = scene.camera
    old_filepath = scene.render.filepath
    old_frame = scene.frame_current
    old_subframe = scene.frame_subframe
    old_matrix = capture.matrix_world.copy()
    old_dof = bool(getattr(getattr(capture.data, "dof", None), "use_dof", False))
    if getattr(capture.data, "dof", None) is not None:
        capture.data.dof.use_dof = False
    blur_states = []
    for owner in (getattr(scene, "render", None), getattr(scene, "eevee", None)):
        if owner is not None and hasattr(owner, "use_motion_blur"):
            blur_states.append((owner, bool(owner.use_motion_blur)))
            owner.use_motion_blur = False
    done = sum(complete_flags)
    rendered_this_chunk = 0
    render_seconds = 0.0
    try:
        scene.frame_set(sequence.source_scene_frame)
        current_scene_hash = pose_sequence.scene_signature(scene, settings, sequence.source_scene_frame)
        if sequence.scene_hash and current_scene_hash != sequence.scene_hash:
            raise ValueError(
                "Scene content at the sequence source frame changed after planning"
            )
        for sample in chunk_samples:
            if sample.render_status == "COMPLETE" and _sequence_output_complete(root, settings, sample):
                continue
            write_progress(
                progress_path, "rendering", done, total,
                f"logical frame {sample.logical_frame_id}/{len(sequence.frames)}", elapsed=render_seconds,
            )
            sample.render_status = "RENDERING"
            sample.error = ""
            pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
            stage = root / "_gs_tmp" / f"frame_{sample.logical_frame_id:06d}"
            shutil.rmtree(stage, ignore_errors=True)
            dirs = {
                "images": stage / "images", "depth": stage / "depth",
                "normal": stage / "normal", "id": stage / "id",
                "material_id": stage / "material_id", "objects": stage / "objects", "_tmp": stage / "_tmp",
            }
            for key in dirs:
                ensure_dir(dirs[key])
            stem = Path(sample.rgb_path).stem
            started = time.time()
            try:
                pos_error, rot_error = _verify_capture_pose(scene, settings, capture, sample, sequence)
                sequence.max_position_verification_error = max(sequence.max_position_verification_error, pos_error)
                sequence.max_rotation_verification_error = max(sequence.max_rotation_verification_error, rot_error)
                failures = _render_one_view(
                    scene, settings, capture, stem, dirs, extension,
                    plan, groups, material_groups, depth_format,
                )
                if any(failures.values()):
                    raise RuntimeError(f"output pass failure: {failures}")
                for expected in _frame_expected_outputs(stage, settings, stem, Path(sample.rgb_path).name):
                    if not _valid_output_file(expected):
                        raise RuntimeError(f"Required render output is missing: {expected}")
                committed = _commit_staged_frame(stage, root)
                pose_sequence.atomic_write_json(
                    _sequence_commit_path(root, sample),
                    {"logical_frame_id": sample.logical_frame_id, "image_id": sample.image_id, "files": committed},
                )
                sample.render_status = "COMPLETE"
                sample.error = ""
                done += 1
                render_seconds += time.time() - started
                pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
                rendered_this_chunk += 1
            except Exception as exc:
                sample.render_status = "FAILED"
                sample.error = str(exc)
                pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
                raise
            finally:
                shutil.rmtree(stage, ignore_errors=True)
    finally:
        scene.camera = old_camera
        scene.render.filepath = old_filepath
        capture.matrix_world = old_matrix
        if getattr(capture.data, "dof", None) is not None:
            capture.data.dof.use_dof = old_dof
        scene.frame_set(old_frame, subframe=old_subframe)
        for owner, value in blur_states:
            owner.use_motion_blur = value
        bpy.context.view_layer.update()

    complete_flags = [_sequence_output_complete(root, settings, sample) for sample in target_samples]
    chunk_verified = all(
        complete_flags[index - 1] for index in range(chunk_start, chunk_end + 1)
    )
    if not chunk_verified:
        raise RuntimeError(f"Chunk output verification failed for frames {chunk_start}-{chunk_end}")
    dataset_complete = _finish_chunk_control(
        chunk_control, complete_flags, rendered_this_chunk, True, force_render=force_render
    )
    if chunk_control is not None and not dataset_complete:
        message = f"Verified chunk {chunk_start}-{chunk_end}; next fresh process starts at {chunk_end + 1}"
        write_progress(progress_path, "chunk_done", sum(complete_flags), total, message, elapsed=render_seconds)
        settings.render_status = message
        return rendered_this_chunk, 0
    if not all(complete_flags):
        missing = [str(index) for index, complete in enumerate(complete_flags, 1) if not complete]
        raise RuntimeError("Dataset finalization blocked by incomplete frames: " + ", ".join(missing[:20]))

    export_cameras = pose_sequence.adapters(sequence)
    export_image_names = [Path(camera.sample.rgb_path).name for camera in export_cameras]
    write_progress(progress_path, "points", total, total, "Sampling sparse point cloud", elapsed=render_seconds)
    points = sample_sparse_points(scene, export_cameras, settings)
    write_progress(progress_path, "exporting", total, total, "Writing manifest-backed exports", elapsed=render_seconds)
    write_colmap(scene, settings, export_cameras, export_image_names, points)
    write_report(scene, settings, export_cameras, export_image_names, points)
    label = "Patch pose sequence generated" if patch_only else "Pose sequence dataset generated"
    message = f"{label}: {total} frames in {_fmt_time(render_seconds)}"
    write_progress(progress_path, "done", total, total, message, elapsed=render_seconds)
    settings.render_status = message
    return total, len(points)


def render_dataset(
    scene, settings, progress_path=None, patch_only=False, reuse_existing_cameras=False,
    chunk_control=None,
):
    write_progress(progress_path, "preparing", 0, 0, "Configuring render engine")
    configure_render(scene, settings)
    _log_render_preflight(scene, settings)
    sequence = active_pose_sequence(scene, settings, prefer_disk=True)
    if sequence is not None:
        return _render_pose_sequence_dataset(
            scene, settings, sequence, progress_path=progress_path, patch_only=patch_only,
            chunk_control=chunk_control,
        )
    if patch_only:
        cameras = coverage_patch.final_cameras(scene)
    else:
        cameras = active_dataset_cameras(scene, settings) if reuse_existing_cameras else []
        if cameras:
            message = f"Reusing {len(cameras)} existing cameras; camera planning skipped"
            write_progress(progress_path, "preparing", 0, len(cameras), message)
            print(f"[GS] {message}", flush=True)
        elif settings.auto_create_rig:
            message = "No existing cameras found; creating camera rig"
            write_progress(progress_path, "planning", 0, 0, message)
            print(f"[GS] {message}", flush=True)
            create_rig(scene, settings)
            cameras = active_dataset_cameras(scene, settings)
        else:
            cameras = active_dataset_cameras(scene, settings)
    if not cameras:
        raise RuntimeError("No patch cameras available." if patch_only else "No cameras available.")
    items = dataset_camera_items(scene, settings, cameras)
    total = len(items)

    # ---- resumable render checkpoint ----------------------------------------------------
    # 'Resume' (the incremental toggle) re-reads _gs_render_state.json: if its signature still
    # matches the current settings, we continue from the frames not yet marked done and keep the
    # accumulated render time. Any interruption (crash / cancel / close / power loss) is safe
    # because the checkpoint is rewritten after EVERY finished frame. To start over from frame 1,
    # use the "Restart Render" button (it clears this checkpoint) rather than the progress bar.
    resume = bool(getattr(settings, "incremental", True)) and not patch_only
    chunked = chunk_control is not None
    force_render = bool(chunk_control and chunk_control.get("force_render")) and not patch_only
    prev = load_render_state(settings, total) if (resume or chunked) and not patch_only else None
    done = set(int(i) for i in prev.get("done", [])) if prev else set()
    render_seconds = float(prev.get("render_seconds", 0.0)) if prev else 0.0
    root = Path(settings.output_dir)
    checkpoint_done = set(done)
    if resume or patch_only:
        valid_done, recovered_commits = _discover_legacy_done(root, settings, items)
        missing_done = checkpoint_done - valid_done
        done = valid_done
        if missing_done:
            first_missing = min(missing_done)
            print(
                f"[GS] resume checkpoint repaired: {len(missing_done)} completed frame(s) "
                f"have missing outputs; restarting at frame {first_missing}",
                flush=True,
            )
        discovered = done - checkpoint_done
        if discovered:
            print(
                f"[GS] disk resume discovered {len(discovered)} complete frame(s) "
                "not present in the checkpoint",
                flush=True,
            )
        if recovered_commits:
            print(
                f"[GS] disk resume rebuilt {recovered_commits} missing frame commit(s)",
                flush=True,
            )
        if missing_done or discovered:
            save_render_state(settings, total, done, render_seconds, state="rendering")
    complete_flags = [index in done for index in range(1, total + 1)]
    chunk_start, chunk_end = _chunk_bounds(total, complete_flags, chunk_control)
    if force_render and chunk_start <= chunk_end:
        if chunk_start == 1:
            import shutil
            shutil.rmtree(root / "_gs_frame_commits", ignore_errors=True)
            done.clear()
            complete_flags = [False] * total
        else:
            done.difference_update(range(chunk_start, chunk_end + 1))
            complete_flags[chunk_start - 1:chunk_end] = [False] * (chunk_end - chunk_start + 1)
    _write_legacy_commit_manifest(root)
    fresh = not done and not patch_only and (not chunked or chunk_start == 1)
    if done:
        print(f"[GS] resuming render: {len(done)}/{total} frames already done, "
              f"{_fmt_time(render_seconds)} so far", flush=True)
    plan = _resolved_output_plan(settings)
    if not plan.has_outputs:
        raise ValueError("Select at least one Render Output.")
    pass_summary = ", ".join(sorted(plan.required_internal_passes))
    save_summary = ", ".join(sorted(plan.requested_outputs))
    write_progress(progress_path, "rendering", len(done), total,
                   (("Resuming" if done else "Rendering") +
                    f" | Required: {pass_summary} | Saving: {save_summary} | RGB: " +
                    ("Enabled" if plan.config.rgb else "Skipped")),
                   elapsed=render_seconds)

    dirs = {
        "images": root / "images",
        "depth": root / "depth",
        "normal": root / "normal",
        "id": root / "id",
        "material_id": root / "material_id",
        "objects": root / "objects",
        "_tmp": root / "_gs_tmp",
    }
    depth_format = getattr(settings, "depth_format", "EXR")

    # Fresh run (no valid checkpoint) -> wipe per-object + temp scratch (and any stale id/depth_view/
    # masks folders from an older version) so nothing lingers. A resume keeps existing outputs.
    if fresh:
        import shutil
        for d in (dirs["objects"], dirs["_tmp"], root / "id", root / "material_id",
                  root / "normal", root / "depth_view", root / "masks"):
            try:
                shutil.rmtree(d, ignore_errors=True)
            except Exception:
                pass

    if plan.saves(output_pipeline.RGB):
        ensure_dir(dirs["images"])
    if plan.saves(output_pipeline.SCENE_DEPTH):
        ensure_dir(dirs["depth"])
    if plan.saves(output_pipeline.SCENE_NORMAL):
        ensure_dir(dirs["normal"])
    groups = build_id_groups(scene, settings) if plan.needs_object_groups else []
    material_groups = build_material_groups(scene) if output_pipeline.PASS_MATERIAL_ID in plan.required_internal_passes else []
    if plan.needs_object_groups:
        ensure_dir(dirs["_tmp"])
        ensure_dir(root / "metadata")
        pose_sequence.atomic_write_json(
            root / "metadata" / "object_map.json",
            {"items": [{"id": object_id, "name": name, "color": list(color), "objects": [obj.name for obj in objects]} for name, object_id, color, objects in groups]},
        )
        if plan.physical_split:
            ensure_dir(dirs["objects"])
        if plan.saves(output_pipeline.OBJECT_ID):
            ensure_dir(dirs["id"])
            pose_sequence.atomic_write_json(
                dirs["id"] / "id_map.json",
                {"items": [{"id": object_id, "name": name, "color": list(color), "objects": [obj.name for obj in objects]} for name, object_id, color, objects in groups]},
            )
        if plan.physical_split:
            legend = {"items": [{"id": object_id, "name": name, "objects": [o.name for o in objs]}
                                for name, object_id, _color, objs in groups]}
            pose_sequence.atomic_write_json(dirs["objects"] / "items.json", legend)
    if material_groups:
        ensure_dir(dirs["material_id"])
        material_map = {"items": [{"id": material_id, "name": name, "color": list(color)} for name, material_id, color, _material in material_groups]}
        pose_sequence.atomic_write_json(dirs["material_id"] / "material_map.json", material_map)
        ensure_dir(root / "metadata")
        pose_sequence.atomic_write_json(root / "metadata" / "material_map.json", material_map)

    image_names = [image_name for _camera, _stem, image_name in items]
    extension = ".png" if settings.image_format == "PNG" else ".jpg"
    old_camera = scene.camera
    old_filepath = scene.render.filepath
    failure_totals = {"depth": 0, "normal": 0, "mask": 0, "id": 0, "material_id": 0}
    session_frames = 0
    session_seconds = 0.0
    rendered_this_chunk = 0
    for index, (camera, stem, image_name) in enumerate(items, start=1):
        if index < chunk_start or index > chunk_end:
            continue
        if index in done:
            continue  # already rendered in an earlier session -> resume past it
        remaining = total - len(done)
        eta = (session_seconds / session_frames) * remaining if session_frames else 0.0
        write_progress(progress_path, "rendering", len(done), total,
                       f"{image_name} | elapsed {_fmt_time(render_seconds)} | ETA {_fmt_time(eta)}",
                       elapsed=render_seconds, eta=eta)
        t0 = time.time()
        failures = _render_one_view(
            scene, settings, camera, stem, dirs, extension, plan, groups, material_groups, depth_format
        )
        if any(failures.values()):
            raise RuntimeError(f"output pass failure for {image_name}: {failures}")
        _write_legacy_commit(root, settings, index, stem, image_name)
        dt = time.time() - t0
        for key, value in failures.items():
            failure_totals[key] += value
        done.add(index)
        render_seconds += dt
        session_frames += 1
        session_seconds += dt
        rendered_this_chunk += 1
        # Patch-only rendering never mutates the original full-dataset resume checkpoint.
        if not patch_only:
            save_render_state(settings, total, done, render_seconds, state="rendering")
        write_progress(progress_path, "rendering", len(done), total,
                       f"done {image_name} ({dt:.1f}s) | elapsed {_fmt_time(render_seconds)}",
                       elapsed=render_seconds)
    for name, count in failure_totals.items():
        if count:
            print(f"[GS] {name} failed on {count}/{total} cameras.")

    # the id/depth temp scratch is never part of the dataset -> remove it
    try:
        import shutil
        shutil.rmtree(dirs["_tmp"], ignore_errors=True)
    except Exception:
        pass

    scene.camera = old_camera
    scene.render.filepath = old_filepath
    complete_flags = []
    for index, (_camera, stem, image_name) in enumerate(items, start=1):
        complete_flags.append(_legacy_output_complete(root, settings, index, stem, image_name))
    chunk_verified = all(
        complete_flags[index - 1] for index in range(chunk_start, chunk_end + 1)
    )
    if not chunk_verified:
        raise RuntimeError(f"Chunk output verification failed for frames {chunk_start}-{chunk_end}")
    dataset_complete = _finish_chunk_control(
        chunk_control, complete_flags, rendered_this_chunk, True, force_render=force_render
    )
    if chunk_control is not None and not dataset_complete:
        if not patch_only:
            save_render_state(settings, total, done, render_seconds, state="chunk_done")
        message = f"Verified chunk {chunk_start}-{chunk_end}; next fresh process starts at {chunk_end + 1}"
        write_progress(progress_path, "chunk_done", len(done), total, message, elapsed=render_seconds)
        settings.render_status = message
        return rendered_this_chunk, 0
    if not all(complete_flags):
        missing = [str(index) for index, complete in enumerate(complete_flags, 1) if not complete]
        raise RuntimeError("Dataset finalization blocked by incomplete frames: " + ", ".join(missing[:20]))
    if not patch_only:
        save_render_state(settings, total, done, render_seconds, state="finalizing")
    export_cameras = active_dataset_cameras(scene, settings)
    export_items = dataset_camera_items(scene, settings, export_cameras)
    export_image_names = [image_name for _camera, _stem, image_name in export_items]
    write_progress(progress_path, "points", total, total, "Sampling sparse point cloud", elapsed=render_seconds)
    points = sample_sparse_points(scene, export_cameras, settings)
    write_progress(progress_path, "exporting", total, total, "Writing COLMAP files", elapsed=render_seconds)
    write_colmap(scene, settings, export_cameras, export_image_names, points)
    write_report(scene, settings, export_cameras, export_image_names, points)
    if not patch_only:
        save_render_state(settings, total, done, render_seconds, state="done")
    label = "Patch dataset generated" if patch_only else "Dataset generated"
    final_msg = f"{label}: {total} frames in {_fmt_time(render_seconds)}"
    write_progress(progress_path, "done", total, total, final_msg, elapsed=render_seconds)
    try:
        settings.render_status = f"Done: {total} frames in {_fmt_time(render_seconds)}"
    except Exception:
        pass
    print(f"[GS] {final_msg}", flush=True)
    return len(cameras), len(points)


def _camera_planning_report(scene, settings, cameras):
    if settings.rig_mode != "PATH":
        return None
    mode = getattr(settings, "path_capture_mode", "LEGACY_PANORAMA_CUBE")
    if mode == "SCIENTIFIC_THREE_LAYER":
        data = dict(_LAST_CAMERA_PLANNING.get(scene.as_pointer(), {}))
        data.setdefault("mode", mode)
        sequence = active_pose_sequence(scene, settings)
        if sequence is not None:
            data.update(pose_sequence.sequence_report(sequence))
            data["final_camera_count"] = len([item for item in sequence.frames if item.render_enabled])
            data["blender_camera_object_count"] = 1
            matrices = [item.matrix_world for item in sequence.frames if item.render_enabled]
        else:
            data["realization_mode"] = "SCIENTIFIC_CAMERA_OBJECTS"
            data["final_camera_count"] = len(cameras)
            data["blender_camera_object_count"] = len(cameras)
            matrices = [camera.matrix_world for camera in cameras]
        scale = float(getattr(scene.unit_settings, "scale_length", 1.0) or 1.0)
        tolerance = 0.001 / max(1e-9, scale)
        origins = {tuple(int(round(axis / tolerance)) for axis in matrix.translation) for matrix in matrices}
        data["duplicated_origin_count"] = len(matrices) - len(origins)
        return data
    rig_data = _path_camera_rig_report(scene)
    cameras_per_station = max(1, int(rig_data.get("cameras_per_station", 6)))
    station_count = int(rig_data.get("station_count", len(cameras) // cameras_per_station))
    rig_mode = rig_data.get("mode", "LEGACY_SIX")
    return {
        "mode": rig_mode,
        "legacy_station_count": station_count,
        "legacy_view_budget": station_count * 6,
        "final_camera_count": len(cameras),
        "layer_count": 1,
        "layer_camera_counts": {"spherical_shell" if rig_mode == "SPHERICAL_SHELL_12" else "shared_origin": len(cameras)},
        "target_overlap": None, "minimum_step": None, "maximum_step": None,
        "actual_step_min": None, "actual_step_max": None, "actual_step_mean": None,
        "duplicated_origin_count": int(rig_data.get("duplicate_camera_origin_count", max(0, len(cameras) - station_count))),
        "coverage_cell_size": None, "visible_surface_cell_count": 0,
        "cells_observed_at_least_1": 0, "cells_observed_at_least_3": 0, "cells_observed_at_least_5": 0,
        "floor_coverage_ratio": None, "ceiling_coverage_ratio": None, "vertical_surface_coverage_ratio": None,
        "under_observed_cell_count": 0, "overlap_graph_component_count": None,
        "polar_keyframe_count": station_count * 2, "removed_redundant_camera_count": 0, "bridge_camera_count": 0,
        "unreachable_surface_cells": 0,
    }


def write_report(scene, settings, cameras, image_names, points):
    plan = _resolved_output_plan(settings)
    width, height, fx, fy, cx, cy = camera_params(scene, cameras[0])
    export_model = camera_export_model(cameras[0])
    patch_data = coverage_patch.report_data(scene)
    planning_report = _camera_planning_report(scene, settings, cameras)
    planning_data = planning_report or {}
    profile_values = sorted({
        (
            round(float(camera.data.lens), 6),
            round(float(camera.data.sensor_width), 6),
            round(float(camera.data.sensor_height), 6),
        )
        for camera in cameras
    })
    lens_profiles = {
        "mode": "SINGLE_INTRINSICS" if len(profile_values) <= 1 else "REGION_PROFILES",
        "profile_count": len(profile_values),
        "profiles": [
            {"profile_id": index, "lens_mm": value[0], "sensor_width_mm": value[1], "sensor_height_mm": value[2]}
            for index, value in enumerate(profile_values, 1)
        ],
    }
    report = {
        # Blender 4.2+ extensions strip bl_info from the module — fall back to constants so
        # write_report works whether loaded as a legacy addon or as a manifest extension.
        "addon": (globals().get("bl_info") or {}).get("name", "Gaussian Splat COLMAP Dataset Generator"),
        "version": _addon_version_str(),
        "camera_count": len(cameras),
        "blender_camera_object_count": planning_data.get("blender_camera_object_count", len(cameras)),
        "image_count": len(image_names),
        "sparse_point_count": len(points),
        "resolution": [width, height],
        "render_engine": settings.render_engine,
        "cycles": {
            "samples": settings.cycles_samples if settings.render_engine == "CYCLES" else None,
            "denoise": settings.cycles_denoise if settings.render_engine == "CYCLES" else None,
            "device": settings.cycles_device if settings.render_engine == "CYCLES" else None,
            "backend": scene.get("gs_cycles_backend", getattr(settings, "cycles_backend", "AUTO")) if settings.render_engine == "CYCLES" else None,
            "requested_backend": getattr(settings, "cycles_backend", "AUTO") if settings.render_engine == "CYCLES" else None,
            "hip_rt_mode": getattr(settings, "hip_rt_mode", "REQUIRE") if settings.render_engine == "CYCLES" else None,
            "hip_rt_enabled": bool(scene.get("gs_hiprt_enabled", False)) if settings.render_engine == "CYCLES" else None,
            "hip_rt_runtime": scene.get("gs_hiprt_runtime", "n/a") if settings.render_engine == "CYCLES" else None,
            "hip_memory_safe_mode": bool(getattr(settings, "hip_memory_safe_mode", True)) if settings.render_engine == "CYCLES" else None,
            "hip_chunk_size": int(getattr(settings, "hip_chunk_size", 100)) if settings.render_engine == "CYCLES" else None,
            "hip_oom_fallback": bool(getattr(settings, "hip_oom_fallback", False)) if settings.render_engine == "CYCLES" else None,
        },
        "camera_model": export_model,
        "blender_camera_model": "PERSPECTIVE" if (
            settings.rig_mode == "PATH"
            and (
                settings.path_capture_mode == "SCIENTIFIC_THREE_LAYER"
                or getattr(settings, "path_station_array_mode", "SPHERICAL_SHELL_12") == "SPHERICAL_SHELL_12"
            )
        ) else settings.camera_model,
        "rig_mode": settings.rig_mode,
        "path_capture_mode": settings.path_capture_mode if settings.rig_mode == "PATH" else None,
        "realization_mode": planning_data.get("realization_mode", scientific_realization_mode(settings)),
        "camera_planning": planning_report,
        "camera_rig": _path_camera_rig_report(scene) if settings.rig_mode == "PATH" and settings.path_capture_mode == "LEGACY_PANORAMA_CUBE" else None,
        "origin_generation": planning_data.get("origin_generation", {}),
        "space_classification": planning_data.get("space_classification", {}),
        "global_coverage": planning_data.get("global_coverage", {}),
        "near_field": {
            key: planning_data.get(key)
            for key in (
                "near_field_camera_count", "rejected_too_close_candidate_count",
                "dominant_surface_rejected_count", "near_mid_far_camera_counts",
                "near_field_average_overlap", "near_field_under_observed_cells",
            )
            if key in planning_data
        },
        "lens_profiles": lens_profiles,
        "training_consistency": planning_data.get("training_consistency", training_consistency_report(scene, settings, cameras)),
        "coverage_patch": patch_data,
        "path_count_mode": settings.path_count_mode if settings.rig_mode == "PATH" else None,
        "path_camera_density": settings.path_camera_density if settings.rig_mode == "PATH" else None,
        "path_collection": settings.path_collection.name if settings.rig_mode == "PATH" and settings.path_collection else None,
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
        "color_look": settings.color_look,
        "color_exposure": settings.color_exposure,
        "depth_format": settings.depth_format if output_pipeline.PASS_DEPTH in plan.required_internal_passes else None,
        "depth_png_scale_mm": DEPTH_PNG_SCALE_MM if settings.depth_format == "PNG" else None,
        "outputs": {
            "rgb": "images/" if plan.config.rgb else None,
            "id": "id/" if plan.saves(output_pipeline.OBJECT_ID) else None,
            "material_id": "material_id/" if plan.config.material_id else None,
            "camera_sequence": pose_sequence.SEQUENCE_FILENAME if planning_data.get("realization_mode") == "SCIENTIFIC_POSE_SEQUENCE" else None,
            "colmap": "sparse/0/",
            "transforms": "transforms.json",
            "depth": ("depth/ (%s)" % settings.depth_format) if plan.saves(output_pipeline.SCENE_DEPTH) else None,
        "normal": "normal/ (PNG 16-bit encoded world XYZ: rgb = (normal + 1) / 2)" if plan.saves(output_pipeline.SCENE_NORMAL) else None,
            "object_depth": ("objects/<item>/depth/ (%s)" % settings.depth_format) if plan.physical_split and plan.config.object_depth else ("virtual: depth/ + id/" if plan.config.object_depth else None),
            "object_normal": "objects/<item>/normal/ (PNG 16-bit encoded world XYZ: rgb = (normal + 1) / 2)" if plan.physical_split and plan.config.object_normal else ("virtual: normal/ + id/" if plan.config.object_normal else None),
            "object_mask": "objects/<item>/mask/ (PNG white/black)" if plan.physical_split and plan.config.object_mask else ("virtual: id/" if plan.config.object_mask else None),
            "items_legend": "objects/items.json" if plan.physical_split and plan.needs_object_groups else None,
            "coverage_patch_manifest": "patch_manifest.json" if patch_data else None,
        },
    }
    root = Path(settings.output_dir)
    with open(root / "dataset_report.json", "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
    manifest = output_pipeline.output_manifest(plan)
    manifest.update({
        "schema_version": "1.0",
        "addon_version": _addon_version_str(),
        "frame_count": len(cameras),
        "resolution": [width, height],
        "object_map": "metadata/object_map.json" if plan.needs_object_groups else None,
        "material_map": "metadata/material_map.json" if plan.config.material_id else None,
    })
    metadata_dir = root / "metadata"
    ensure_dir(metadata_dir)
    pose_sequence.atomic_write_json(metadata_dir / "render_manifest.json", manifest)
    pose_sequence.atomic_write_json(root / "render_manifest.json", manifest)
    if patch_data:
        patch_frames = []
        for camera, image_name in zip(cameras, image_names):
            if not camera.get("gs_patch_camera"):
                continue
            patch_frames.append({
                "camera_name": camera.name,
                "image_path": f"images/{image_name}",
                "transform_matrix": [[float(value) for value in row] for row in camera.matrix_world],
                "lens_mm": float(camera.data.lens),
            })
        manifest = {
            "version": 1,
            "patch_camera_count": len(patch_frames),
            "frames": patch_frames,
            "coverage_patch": patch_data,
        }
        with open(root / "patch_manifest.json", "w", encoding="utf-8") as handle:
            json.dump(manifest, handle, indent=2, ensure_ascii=False)


_LIVE_UPDATE_LOCK = False
_BACKGROUND_RENDER = None


SETTINGS_VALUE_KEYS = (
    "language",
    "output_dir",
    "camera_collection",
    "camera_mesh_style",
    "camera_mesh_size",
    "patch_mode",
    "patch_min_observation_count",
    "patch_recommended_observation_count",
    "patch_target_coverage_ratio",
    "patch_max_camera_count",
    "patch_candidate_radius",
    "patch_camera_safety_distance",
    "patch_min_overlap_ratio",
    "patch_allow_polar",
    "patch_limit_to_path",
    "patch_prefer_existing_connect",
    "patch_priority",
    "live_update_cameras",
    "rig_mode",
    "path_look_mode",
    "path_count_mode",
    "path_capture_mode",
    "path_station_array_mode",
    "shell_radius",
    "shell_radius_mode",
    "shell_min_radius",
    "shell_failure_policy",
    "show_shell_debug_mesh",
    "scientific_realization_mode",
    "sequence_source_scene_frame",
    "sequence_create_preview_keyframes",
    "scientific_origin_mode",
    "scientific_budget_mode",
    "scientific_fixed_budget",
    "scientific_minimum_budget",
    "scientific_maximum_budget",
    "scientific_auto_small_space",
    "free_space_grid_resolution",
    "free_space_max_grid_cells",
    "free_space_candidate_spacing",
    "free_space_max_origin_count",
    "free_space_boundary_bias",
    "free_space_doorway_priority",
    "free_space_medial_axis_priority",
    "free_space_narrow_clearance",
    "free_space_probe_height",
    "free_space_min_headroom",
    "free_space_views_per_m2",
    "near_field_protection",
    "near_field_recommended_distance_min",
    "near_field_recommended_distance_max",
    "near_field_unsuitable_distance",
    "near_field_dominant_surface_ratio",
    "near_field_minimum_environment_ratio",
    "near_field_step_distance_ratio",
    "near_field_minimum_origin_spacing",
    "near_field_doorway_target_overlap",
    "near_field_minimum_mid_overlap",
    "near_field_required_mid_neighbors",
    "near_field_maximum_camera_ratio",
    "near_field_minimum_baseline",
    "near_field_minimum_under_observed_gain",
    "scientific_global_reachable_coverage",
    "scientific_coverage_driven",
    "coverage_driven_max_cameras",
    "coverage_driven_max_surface_targets",
    "coverage_driven_candidates_per_surface",
    "scientific_post_clipping_recast",
    "scientific_validate_training_consistency",
    "scientific_layer_count",
    "scientific_target_overlap",
    "scientific_minimum_overlap",
    "scientific_minimum_step",
    "scientific_maximum_step",
    "scientific_camera_clearance",
    "scientific_view_budget_multiplier",
    "scientific_minimum_observations",
    "scientific_preferred_observations",
    "scientific_ray_quality",
    "scientific_auto_coverage",
    "scientific_auto_floor_ceiling",
    "scientific_show_debug",
    "scientific_keep_candidates",
    "scientific_advanced_expand",
    "scientific_maximum_heading_change",
    "scientific_maximum_incidence_angle",
    "scientific_show_overlap_graph",
    "path_camera_density",
    "max_path_cameras",
    "path_samples_per_segment",
    "camera_count",
    "rings",
    "radius",
    "height",
    "focal_length",
    "camera_model",
    "panorama_fov",
    "resolution_x",
    "resolution_y",
    "image_format",
    "render_engine",
    "cycles_samples",
    "cycles_denoise",
    "cycles_device",
    "cycles_backend",
    "hip_rt_mode",
    "hip_memory_safe_mode",
    "hip_chunk_size",
    "hip_oom_fallback",
    "cycles_persistent_data",
    "background_chunk_size",
    "color_look",
    "color_exposure",
    "image_prefix",
    "auto_create_rig",
    "incremental",
    "transparent_background",
    "render_rgb",
    "export_id",
    "export_depth",
    "depth_format",
    "export_object_depth",
    "export_normal",
    "export_object_normal",
    "export_object_mask",
    "export_material_id",
    "object_split_mode",
    "object_group_mode",
    "point_samples_per_view",
    "point_dedup_size",
    "ray_distance",
    "volume_size",
    "volume_x",
    "volume_y",
    "volume_z",
    "volume_jitter",
    "random_seed",
)


def settings_snapshot(settings):
    values = {}
    for key in SETTINGS_VALUE_KEYS:
        value = getattr(settings, key)
        if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
            try:
                value = list(value)
            except TypeError:
                pass
        values[key] = value
    return {
        "values": values,
        "target_object": settings.target_object.name if settings.target_object else "",
        "path_object": settings.path_object.name if settings.path_object else "",
        "path_collection": settings.path_collection.name if settings.path_collection else "",
        "mask_collection": settings.mask_collection.name if settings.mask_collection else "",
        "exclude_collection": settings.exclude_collection.name if settings.exclude_collection else "",
        "patch_bounds_object": settings.patch_bounds_object.name if settings.patch_bounds_object else "",
    }


def build_background_worker_script(addon_parent, snapshot_path, progress_path, result_path):
    return f"""
import json
import sys
import traceback
from pathlib import Path

sys.path.insert(0, {addon_parent!r})
import bpy
import blender_gs_colmap_exporter as addon

arguments = sys.argv[sys.argv.index("--") + 1:]
requested_start = int(arguments[0])
chunk_size = int(arguments[1])
force_render = arguments[2] == "1"
control = dict(requested_start=requested_start, size=chunk_size, force_render=force_render)

def fail(message):
    addon.write_progress({progress_path!r}, "error", 0, 0, "Background render failed", message)

try:
    try:
        addon.register()
    except Exception:
        pass
    with open({snapshot_path!r}, "r", encoding="utf-8") as handle:
        snapshot = json.load(handle)
    settings = bpy.context.scene.gs_colmap_settings
    settings.live_update_cameras = False
    for key, value in snapshot["values"].items():
        try:
            setattr(settings, key, value)
        except Exception:
            pass
    settings.live_update_cameras = False
    target_name = snapshot.get("target_object") or ""
    path_name = snapshot.get("path_object") or ""
    path_collection_name = snapshot.get("path_collection") or ""
    mask_name = snapshot.get("mask_collection") or ""
    exclude_name = snapshot.get("exclude_collection") or ""
    patch_bounds_name = snapshot.get("patch_bounds_object") or ""
    settings.target_object = bpy.data.objects.get(target_name) if target_name else None
    settings.path_object = bpy.data.objects.get(path_name) if path_name else None
    settings.path_collection = bpy.data.collections.get(path_collection_name) if path_collection_name else None
    settings.mask_collection = bpy.data.collections.get(mask_name) if mask_name else None
    settings.exclude_collection = bpy.data.collections.get(exclude_name) if exclude_name else None
    settings.patch_bounds_object = bpy.data.objects.get(patch_bounds_name) if patch_bounds_name else None
    count, point_count = addon.render_dataset(
        bpy.context.scene, settings, {progress_path!r},
        patch_only=bool(snapshot.get("patch_only", False)),
        reuse_existing_cameras=True,
        chunk_control=control,
    )
    result = dict(control)
    result.update(
        count=count,
        point_count=point_count,
        error="",
        cycles_backend=str(bpy.context.scene.get("gs_cycles_backend", getattr(settings, "cycles_backend", "AUTO"))),
        hip_oom_recovered=bool(bpy.context.scene.get("gs_hip_oom_recovered", False)),
    )
    addon.pose_sequence.atomic_write_json({result_path!r}, result)
except Exception as exc:
    fail(str(exc))
    result = dict(control)
    result.update(state="error", verified=False, complete=False, error=str(exc))
    try:
        addon.pose_sequence.atomic_write_json({result_path!r}, result)
    except Exception:
        pass
    traceback.print_exc()
    raise
"""


def build_background_supervisor_script(module_path):
    return f"""
import importlib.util

spec = importlib.util.spec_from_file_location("gs_background_chunks", {module_path!r})
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.main()
"""


def _configured_cycles_backend(settings):
    """Resolve the backend that a worker is expected to use.

    The parent process uses the same device probe as ``configure_render`` so it can choose a
    safe process lifetime before a worker opens the .blend file. ``AUTO`` follows the normal
    Cycles preference order; an explicit backend remains explicit even when unavailable so the
    worker reports a clear CPU fallback instead of silently selecting a different accelerator.
    """
    if getattr(settings, "render_engine", "CYCLES") != "CYCLES":
        return "CPU"
    requested = getattr(settings, "cycles_backend", "AUTO")
    if requested not in {"", "AUTO"}:
        return requested
    if getattr(settings, "cycles_device", "AUTO") == "CPU":
        return "CPU"
    gpus = list_cycles_gpus()
    if not gpus:
        return "CPU"
    present = {backend for backend, _name in gpus}
    for backend in ("OPTIX", "CUDA", "HIP", "ONEAPI", "METAL"):
        if backend in present:
            return backend
    return gpus[0][0]


def launch_background_render(context, settings, patch_only=False):
    root = Path(settings.output_dir)
    ensure_dir(root)
    # Keep the job/scratch files (progress.json, settings.json, scene copy, log) OUT of the
    # dataset folder. Otherwise tools like Postshot scan the dataset, find progress.json and
    # fail with "missing frames member". Put it in a per-output temp dir instead.
    key = hashlib.sha1(str(root.resolve()).encode("utf-8", "ignore")).hexdigest()[:12]
    job_dir = Path(tempfile.gettempdir()) / "gs_colmap_jobs" / key
    # Remove a stale in-dataset job dir from older versions so existing datasets import cleanly.
    legacy = root / "_gs_background_job"
    if legacy.exists():
        try:
            import shutil
            shutil.rmtree(legacy, ignore_errors=True)
        except Exception:
            pass
    ensure_dir(job_dir)
    blend_path = job_dir / "scene_for_render.blend"
    snapshot_path = job_dir / "settings.json"
    worker_script = job_dir / "run_background_worker.py"
    supervisor_script = job_dir / "run_background_supervisor.py"
    supervisor_config = job_dir / "supervisor_config.json"
    result_path = job_dir / "worker_result.json"
    history_path = job_dir / "chunk_history.json"
    progress_path = job_dir / "progress.json"
    log_path = job_dir / "background_render.log"

    snapshot = settings_snapshot(settings)
    snapshot["values"]["live_update_cameras"] = False
    snapshot["patch_only"] = bool(patch_only)
    pose_sequence.atomic_write_json(snapshot_path, snapshot)

    addon_parent = str(Path(__file__).resolve().parent.parent)
    addon_module = str(Path(__file__).resolve().parent / "background_chunks.py")
    with open(worker_script, "w", encoding="utf-8") as handle:
        handle.write(build_background_worker_script(
            addon_parent, str(snapshot_path), str(progress_path), str(result_path)
        ))
    with open(supervisor_script, "w", encoding="utf-8") as handle:
        handle.write(build_background_supervisor_script(addon_module))

    write_progress(progress_path, "preparing", 0, 0, "Saving scene copy")
    bpy.ops.wm.save_as_mainfile(filepath=str(blend_path), copy=True)

    blender_path = bpy.app.binary_path
    selected_backend = _configured_cycles_backend(settings)
    hip_persistent = (
        selected_backend == "HIP"
        and bool(getattr(settings, "cycles_persistent_data", False))
        and not bool(getattr(settings, "hip_memory_safe_mode", True))
    )
    configured_chunk_size = max(1, int(getattr(settings, "background_chunk_size", 500)))
    # Object-aware passes create per-frame proxy meshes/materials. Keep these jobs short so a
    # failed frame can resume quickly and Blender releases evaluated geometry before VRAM grows.
    # HIP + Persistent Data still obeys the explicit HIP frame limit below; the auxiliary
    # passes disable persistence while they run and restore it before the next beauty frame.
    try:
        output_plan = _resolved_output_plan(settings)
        if output_plan.needs_object_groups and not hip_persistent:
            configured_chunk_size = min(configured_chunk_size, 25)
    except Exception:
        pass
    hip_backend = selected_backend == "HIP"
    hip_safe = hip_backend and bool(getattr(settings, "hip_memory_safe_mode", True))
    if hip_backend:
        # The HIP frame limit is independent from the memory-safe switch.  When Persistent
        # Data is enabled the cache is useful within this bounded worker, then the process
        # restart releases all HIP/HIP-RT allocations before the next batch.
        hip_chunk_size = max(1, int(getattr(settings, "hip_chunk_size", 100)))
        configured_chunk_size = min(configured_chunk_size, hip_chunk_size)
        persistent = bool(getattr(settings, "cycles_persistent_data", False)) and not hip_safe
        print(
            f"[GS] HIP worker chunk size={configured_chunk_size}; "
            f"persistent_data={persistent}; worker restarts after each chunk",
            flush=True,
        )
    config = {
        "blender_path": blender_path,
        "blend_path": str(blend_path),
        "worker_script": str(worker_script),
        "snapshot_path": str(snapshot_path),
        "progress_path": str(progress_path),
        "result_path": str(result_path),
        "history_path": str(history_path),
        "job_dir": str(job_dir),
        "output_root": str(root),
        "chunk_size": configured_chunk_size,
        "cycles_backend": selected_backend,
        "hip_memory_safe_mode": hip_safe,
        "hip_chunk_size": max(1, int(getattr(settings, "hip_chunk_size", 100))),
        "force_full_render": not bool(getattr(settings, "incremental", True)) and not patch_only,
    }
    pose_sequence.atomic_write_json(supervisor_config, config)
    log_handle = open(log_path, "w", encoding="utf-8")
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    process = subprocess.Popen(
        [blender_path, "--background", "--factory-startup", "--python", str(supervisor_script),
         "--", str(supervisor_config)],
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        cwd=str(root),
        creationflags=creationflags,
    )
    return {
        "process": process,
        "log_handle": log_handle,
        "progress_path": str(progress_path),
        "log_path": str(log_path),
        "job_dir": str(job_dir),
        "history_path": str(history_path),
    }


def _terminate_process_tree(process):
    if process is None or process.poll() is not None:
        return
    if os.name == "nt":
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=creationflags,
            check=False,
        )
    else:
        process.terminate()


def live_update_camera_rig(self, context):
    global _LIVE_UPDATE_LOCK
    if _LIVE_UPDATE_LOCK:
        return
    if not getattr(self, "live_update_cameras", False):
        return
    if getattr(self, "rig_mode", "EXISTING") == "EXISTING":
        return
    if not context or not getattr(context, "scene", None):
        return
    if (
        getattr(self, "rig_mode", "") == "PATH"
        and getattr(self, "path_capture_mode", "LEGACY_PANORAMA_CUBE") == "LEGACY_PANORAMA_CUBE"
        and getattr(self, "path_station_array_mode", "SPHERICAL_SHELL_12") == "SPHERICAL_SHELL_12"
    ):
        if not getattr(self, "camera_build_active", False):
            self.camera_build_progress = 0.0
            self.camera_build_status = "球壳参数已更新"
        return
    _LIVE_UPDATE_LOCK = True
    try:
        create_rig(context.scene, self)
    except Exception:
        pass
    finally:
        _LIVE_UPDATE_LOCK = False


def update_camera_mesh_style(self, context):
    if not context or not getattr(context, "scene", None):
        return
    scene = context.scene
    cameras = sorted(
        (obj for obj in scene.objects if obj.type == "CAMERA"),
        key=lambda obj: obj.name,
    )
    try:
        sync_camera_mesh_visuals(scene, self, cameras)
    except Exception:
        pass



class GSCOLMAP_Settings(PropertyGroup):
    language: EnumProperty(
        name="Language",
        items=(("zh", "中文", "精准中文界面"), ("en", "English", "English interface")),
        default="zh",
    )
    output_dir: StringProperty(name="Output Directory", subtype="DIR_PATH")
    camera_collection: StringProperty(name="Camera Collection", default="GS_COLMAP_Cameras")
    camera_mesh_style: BoolProperty(
        name="绿色金字塔相机样式", default=True,
        description="保留真实相机用于渲染，并用绿色金字塔 Mesh 显示镜头方向。",
        update=update_camera_mesh_style,
    )
    camera_mesh_size: FloatProperty(
        name="金字塔尺寸 (m)", default=0.25, min=0.02, soft_max=2.0,
        description="绿色相机 Mesh 的真实米制长度。",
        update=update_camera_mesh_style,
    )
    patch_mode: EnumProperty(
        name="补齐模式",
        items=(
            ("SELECTED_OBJECTS", "选中对象", "分析当前选中的 Mesh 对象"),
            ("BOUNDS", "包围盒区域", "分析指定 Mesh 或 Empty 定义的空间范围"),
            ("AUTO_UNDEROBSERVED", "欠覆盖自动检测", "从全场景中自动提取观察次数不足的主要区域"),
        ),
        default="SELECTED_OBJECTS",
    )
    patch_bounds_object: PointerProperty(name="补齐包围盒", type=bpy.types.Object)
    patch_min_observation_count: IntProperty(name="最低观察次数", default=3, min=1, max=20)
    patch_recommended_observation_count: IntProperty(name="推荐观察次数", default=5, min=1, max=30)
    patch_target_coverage_ratio: FloatProperty(name="目标区域覆盖率", default=0.95, min=0.10, max=1.0, subtype="FACTOR")
    patch_max_camera_count: IntProperty(name="最大补拍数量", default=24, min=1, max=500)
    patch_candidate_radius: FloatProperty(name="候选半径 (m)", default=1.5, min=0.10, soft_max=10.0)
    patch_camera_safety_distance: FloatProperty(name="安全距离 (m)", default=0.25, min=0.05, soft_max=2.0)
    patch_min_overlap_ratio: FloatProperty(name="最低重叠率", default=0.30, min=0.05, max=0.90, subtype="FACTOR")
    patch_allow_polar: BoolProperty(name="允许极向关键帧", default=True)
    patch_limit_to_path: BoolProperty(name="限制在已有路径附近", default=False)
    patch_prefer_existing_connect: BoolProperty(name="优先连接现有相机图", default=True)
    patch_priority: EnumProperty(
        name="补齐策略",
        items=(
            ("MINIMAL", "最少相机优先", "达到目标后立即停止"),
            ("COVERAGE", "最高覆盖优先", "提高新覆盖与欠覆盖收益权重"),
            ("CONNECTIVITY", "最强连通性优先", "提高与已有相机重叠的权重"),
        ),
        default="MINIMAL",
    )
    patch_progress: FloatProperty(name="补齐进度", default=0.0, min=0.0, max=1.0, subtype="FACTOR")
    patch_status: StringProperty(name="补齐状态", default="请选择目标区域并生成预览")
    live_update_cameras: BoolProperty(name="Live Camera Update", default=True)
    scientific_origin_mode: EnumProperty(name="Origin Source", items=SCIENTIFIC_ORIGIN_MODE_ITEMS, default="MANUAL_CURVE")
    scientific_budget_mode: EnumProperty(name="Image Budget Mode", items=SCIENTIFIC_BUDGET_MODE_ITEMS, default="LEGACY_PATH_BUDGET")
    scientific_fixed_budget: IntProperty(name="Fixed Image Budget", default=720, min=1, max=10000)
    scientific_minimum_budget: IntProperty(name="Minimum Images", default=24, min=1, max=10000)
    scientific_maximum_budget: IntProperty(name="Maximum Images", default=3600, min=1, max=20000)
    scientific_auto_small_space: BoolProperty(name="Detect Small Spaces", default=True)
    free_space_grid_resolution: FloatProperty(name="Free-space Grid Resolution (m)", default=0.35, min=0.10, max=1.0)
    free_space_max_grid_cells: IntProperty(name="Maximum Free-space Grid Cells", default=200000, min=1000, max=1000000)
    free_space_candidate_spacing: FloatProperty(name="Free-space Origin Spacing (m)", default=0.75, min=0.10, soft_max=5.0)
    free_space_max_origin_count: IntProperty(name="Maximum Candidate Origins", default=600, min=1, max=10000)
    free_space_boundary_bias: FloatProperty(name="Boundary Candidate Ratio", default=0.25, min=0.0, max=1.0, subtype="FACTOR")
    free_space_doorway_priority: BoolProperty(name="Prioritize Doorways", default=True)
    free_space_medial_axis_priority: BoolProperty(name="Prioritize Medial Axis", default=True)
    free_space_narrow_clearance: FloatProperty(name="Narrow-space Minimum Clearance (m)", default=0.12, min=0.05, soft_max=1.0)
    free_space_probe_height: FloatProperty(name="Free-space Probe Height (m)", default=1.40, min=0.20, soft_max=3.0)
    free_space_min_headroom: FloatProperty(name="Minimum Headroom (m)", default=1.20, min=0.50, soft_max=5.0)
    free_space_views_per_m2: FloatProperty(name="Images per Square Meter", default=6.0, min=0.1, soft_max=30.0)
    near_field_protection: BoolProperty(name="Near-field Capture Protection", default=True)
    near_field_recommended_distance_min: FloatProperty(name="Recommended Distance Minimum (m)", default=0.60, min=0.10, soft_max=5.0)
    near_field_recommended_distance_max: FloatProperty(name="Recommended Distance Maximum (m)", default=1.00, min=0.10, soft_max=8.0)
    near_field_unsuitable_distance: FloatProperty(name="Unsuitable Optical-center Distance (m)", default=0.35, min=0.05, soft_max=2.0)
    near_field_dominant_surface_ratio: FloatProperty(name="Dominant Near-surface Limit", default=0.65, min=0.50, max=0.90, subtype="FACTOR")
    near_field_minimum_environment_ratio: FloatProperty(name="Minimum Shared Environment", default=0.35, min=0.10, max=0.90, subtype="FACTOR")
    near_field_step_distance_ratio: FloatProperty(name="Near-field Step / Target Distance", default=0.25, min=0.20, max=0.30, subtype="FACTOR")
    near_field_minimum_origin_spacing: FloatProperty(name="Minimum Near-field Origin Spacing (m)", default=0.12, min=0.05, soft_max=1.0)
    near_field_doorway_target_overlap: FloatProperty(name="Entrance Target Overlap", default=0.85, min=0.70, max=0.95, subtype="FACTOR")
    near_field_minimum_mid_overlap: FloatProperty(name="Near/Mid Minimum Overlap", default=0.35, min=0.10, max=0.90, subtype="FACTOR")
    near_field_required_mid_neighbors: IntProperty(name="Required Mid/Far Neighbors", default=2, min=1, max=8)
    near_field_maximum_camera_ratio: FloatProperty(name="Maximum Near-camera Ratio", default=0.15, min=0.0, max=0.50, subtype="FACTOR")
    near_field_minimum_baseline: FloatProperty(name="Minimum Near/Mid Baseline (m)", default=0.20, min=0.05, soft_max=2.0)
    near_field_minimum_under_observed_gain: FloatProperty(name="Minimum Under-observed Gain", default=0.05, min=0.0, max=0.50, subtype="FACTOR")
    scientific_global_reachable_coverage: BoolProperty(name="Global Reachable Coverage", default=False)
    scientific_coverage_driven: BoolProperty(name="Coverage-driven Origins", default=True)
    coverage_driven_max_cameras: IntProperty(name="Maximum Coverage-driven Origins", default=12, min=0, max=500)
    coverage_driven_max_surface_targets: IntProperty(name="Maximum Under-covered Targets", default=48, min=1, max=1000)
    coverage_driven_candidates_per_surface: IntProperty(name="Candidates per Under-covered Target", default=3, min=1, max=12)
    scientific_post_clipping_recast: BoolProperty(name="Recast after Collision Push-out", default=True)
    scientific_validate_training_consistency: BoolProperty(name="Validate Training Consistency", default=True)
    rig_mode: EnumProperty(name="Rig", items=RIG_ITEMS, default="HEMISPHERE", update=live_update_camera_rig)
    target_object: PointerProperty(name="Look-at Target", type=bpy.types.Object, update=live_update_camera_rig)
    path_object: PointerProperty(name="Path Curve", type=bpy.types.Object, update=live_update_camera_rig)
    path_collection: PointerProperty(name="Path Curves Collection", type=bpy.types.Collection, update=live_update_camera_rig)
    path_look_mode: EnumProperty(name="Path Aim", items=PATH_LOOK_ITEMS, default="TARGET", update=live_update_camera_rig)
    path_count_mode: EnumProperty(name="Path Count Mode", items=PATH_COUNT_MODE_ITEMS, default="COUNT", update=live_update_camera_rig)
    path_capture_mode: EnumProperty(name="采集模式", items=PATH_CAPTURE_MODE_ITEMS, default="LEGACY_PANORAMA_CUBE", description="旧项目使用六面兼容；新数据集推荐科学三层覆盖。")
    path_station_array_mode: EnumProperty(
        name="采集阵列",
        items=PATH_STATION_ARRAY_MODE_ITEMS,
        default="SPHERICAL_SHELL_12",
        update=live_update_camera_rig,
    )
    shell_radius: FloatProperty(
        name="球壳半径 (m)",
        default=0.18,
        min=0.03,
        max=0.50,
        precision=3,
        update=live_update_camera_rig,
    )
    shell_radius_mode: EnumProperty(
        name="半径模式",
        items=SHELL_RADIUS_MODE_ITEMS,
        default="CLEARANCE_ADAPTIVE",
        update=live_update_camera_rig,
    )
    shell_min_radius: FloatProperty(
        name="最小球壳半径 (m)",
        default=0.06,
        min=0.03,
        max=0.50,
        precision=3,
        update=live_update_camera_rig,
    )
    shell_failure_policy: EnumProperty(
        name="失败处理",
        items=SHELL_FAILURE_POLICY_ITEMS,
        default="LEGACY_SIX_FALLBACK",
        update=live_update_camera_rig,
    )
    show_shell_debug_mesh: BoolProperty(
        name="显示球壳调试代理",
        default=False,
        update=live_update_camera_rig,
    )
    scientific_realization_mode: EnumProperty(name="Realization Backend", items=SCIENTIFIC_REALIZATION_MODE_ITEMS, default="SCIENTIFIC_CAMERA_OBJECTS")
    sequence_source_scene_frame: IntProperty(name="Source Scene Frame", default=1, min=-1048574, max=1048574, description="Hold the scene at this frame while every pose is rendered.")
    sequence_create_preview_keyframes: BoolProperty(name="Create Preview Keyframes", default=False, description="Create optional quaternion preview keyframes; rendering never consumes them.")
    sequence_debug_mode: EnumProperty(name="Pose Display", items=SEQUENCE_DEBUG_MODE_ITEMS, default="OFF")
    sequence_debug_frame: IntProperty(name="Logical Frame", default=1, min=1, max=10000000)
    sequence_debug_neighbor_count: IntProperty(name="Neighbor Count", default=2, min=0, max=100)
    sequence_debug_stride: IntProperty(name="Display Stride", default=10, min=1, max=10000)
    scientific_layer_count: IntProperty(name="高度层数", default=3, min=2, max=4, description="科学模式使用 2、3 或 4 个安全高度层，推荐 3 层。")
    scientific_minimum_overlap: FloatProperty(name="最低邻居重叠率", default=0.50, min=0.10, max=0.90, subtype="FACTOR", description="重叠图中认定高质量邻居所需的共同可见表面比例。")
    scientific_target_overlap: FloatProperty(name="目标重叠率", default=0.70, min=0.50, max=0.95, subtype="FACTOR", description="用于按真实视场角和局部深度计算机位间距。")
    scientific_minimum_step: FloatProperty(name="最小路径间距 (m)", default=0.25, min=0.05, soft_max=2.0, description="真实米制最小间距；规划时按场景 scale_length 自动换算。")
    scientific_maximum_step: FloatProperty(name="最大路径间距 (m)", default=0.65, min=0.10, soft_max=5.0, description="真实米制最大间距；规划时按场景 scale_length 自动换算。")
    scientific_camera_clearance: FloatProperty(name="相机安全距离 (m)", default=0.25, min=0.05, soft_max=2.0, description="真实米制安全距离；规划时按场景 scale_length 自动换算。")
    scientific_view_budget_multiplier: FloatProperty(name="图片预算倍率", default=1.0, min=0.1, max=3.0, description="以旧六面方案图片数为基准缩放科学模式最终图片预算。")
    scientific_minimum_observations: IntProperty(name="表面最低观察次数", default=3, min=1, max=20, description="低于此次数的可见表面单元视为欠覆盖。")
    scientific_preferred_observations: IntProperty(name="表面推荐观察次数", default=5, min=1, max=30, description="达到此次数后降低继续添加重复视图的收益。")
    scientific_ray_quality: EnumProperty(name="射线质量", items=SCIENTIFIC_RAY_QUALITY_ITEMS, default="NORMAL", description="控制最终入选相机完整复检的射线网格密度；全部候选固定使用 4x4 快速预筛。")
    scientific_auto_coverage: BoolProperty(name="自动覆盖优化", default=True, description="用确定性覆盖评分选择视角并删除冗余候选。")
    scientific_auto_floor_ceiling: BoolProperty(name="自动补地面和天花板", default=True, description="覆盖不足时在关键节点补充正负 55 度斜视关键帧。")
    scientific_show_debug: BoolProperty(name="显示调试结果", default=False, description="在独立调试集合中显示分层路径、欠覆盖点和重叠图。")
    scientific_keep_candidates: BoolProperty(name="保留候选相机", default=False, description="仅调试：以灰色标记未入选候选机位。")
    scientific_advanced_expand: BoolProperty(name="科学模式高级参数", default=False, description="展开低频使用的科学规划参数。")
    scientific_maximum_heading_change: FloatProperty(name="最大朝向变化", default=15.0, min=1.0, max=90.0, description="路径转向超过此角度时保留并细分关键节点。")
    scientific_maximum_incidence_angle: FloatProperty(name="最大入射角", default=75.0, min=30.0, max=89.0, description="超过此角度的表面命中不计为高质量观察。")
    scientific_show_overlap_graph: BoolProperty(name="显示重叠图", default=True, description="调试显示最终相机之间的共同可见表面连接。")
    scientific_planning_progress: FloatProperty(name="规划进度", default=0.0, min=0.0, max=1.0, subtype="FACTOR")
    scientific_planning_status: StringProperty(name="规划状态", default="就绪")
    scientific_cancel_requested: BoolProperty(name="取消科学规划", default=False)
    scientific_planning_active: BoolProperty(name="科学规划进行中", default=False)
    camera_build_progress: FloatProperty(name="生成进度", default=0.0, min=0.0, max=1.0, subtype="FACTOR")
    camera_build_status: StringProperty(name="生成状态", default="就绪")
    camera_build_cancel_requested: BoolProperty(name="取消相机生成", default=False)
    camera_build_active: BoolProperty(name="相机生成进行中", default=False)
    camera_build_cancelable: BoolProperty(name="相机生成可取消", default=False)
    path_camera_density: FloatProperty(name="Path Density", default=2.0, min=0.01, soft_max=20.0, update=live_update_camera_rig)
    max_path_cameras: IntProperty(name="Max Path Cameras", default=1000, min=1, max=10000, update=live_update_camera_rig)
    path_samples_per_segment: IntProperty(name="Curve Detail", default=24, min=2, max=256, update=live_update_camera_rig)
    camera_count: IntProperty(name="Cameras", default=120, min=1, max=10000, update=live_update_camera_rig)
    rings: IntProperty(name="Rings", default=4, min=1, max=128, update=live_update_camera_rig)
    radius: FloatProperty(name="Radius", default=5.0, min=0.01, soft_max=100.0, update=live_update_camera_rig)
    height: FloatProperty(name="Height", default=3.0, soft_min=0.0, soft_max=100.0, update=live_update_camera_rig)
    focal_length: FloatProperty(name="Focal Length", default=35.0, min=1.0, soft_max=300.0, update=live_update_camera_rig)
    camera_model: EnumProperty(name="Camera Model", items=CAMERA_MODEL_ITEMS, default="PERSPECTIVE", update=live_update_camera_rig)
    panorama_fov: FloatProperty(name="Panorama FOV", default=180.0, min=1.0, max=360.0, update=live_update_camera_rig)
    resolution_x: IntProperty(name="Resolution X", default=1920, min=64, max=16384)
    resolution_y: IntProperty(name="Resolution Y", default=1080, min=64, max=16384)
    image_format: EnumProperty(name="Image Format", items=(("PNG", "PNG", ""), ("JPEG", "JPEG", "")), default="PNG")
    render_engine: EnumProperty(name="Renderer", items=RENDER_ENGINE_ITEMS, default="CYCLES")
    cycles_samples: IntProperty(name="Cycles Samples", default=128, min=1, max=4096)
    cycles_denoise: BoolProperty(name="Cycles Denoise", default=True)
    cycles_device: EnumProperty(name="Cycles Device", items=CYCLES_DEVICE_ITEMS, default="AUTO")
    cycles_backend: EnumProperty(name="Cycles Backend", items=CYCLES_BACKEND_ITEMS, default="AUTO")
    hip_rt_mode: EnumProperty(
        name="HIP-RT",
        items=HIP_RT_MODE_ITEMS,
        default="REQUIRE",
        description=(
            "Background workers start with factory preferences. Require HIP-RT for large HIP "
            "scenes so Cycles cannot silently use the high-memory software BVH path."
        ),
    )
    hip_memory_safe_mode: BoolProperty(
        name="HIP Memory-safe Mode", default=False,
        description=(
            "Disable Persistent Data and use short background batches for HIP. Enable this "
            "only when the scene shows VRAM growth or HIP-RT instability."
        ),
    )
    hip_chunk_size: IntProperty(
        name="HIP Frames per Process", default=100, min=1, max=5000,
        description=(
            "Maximum HIP frames rendered by one Blender worker before it exits and restarts. "
            "This limit applies whether Persistent Data is enabled or not."
        ),
    )
    hip_oom_fallback: BoolProperty(
        name="HIP OOM CPU Fallback", default=False,
        description=(
            "If a single HIP frame cannot allocate its Cycles buffers, retry that frame on CPU "
            "instead of failing the whole dataset."
        ),
    )
    cycles_persistent_data: BoolProperty(
        name="Cycles Persistent Data", default=True,
        description=(
            "Reuse Cycles BVH, textures and render data between frames inside one chunk. "
            "The worker still exits after the chunk to release all caches."
        ),
    )
    color_look: EnumProperty(
        name="Color Look",
        description="View transform for the COLOR training images. AgX (recommended) rolls off "
                    "highlights so a bright scene never overexposes; Filmic is a classic film curve; "
                    "Standard is faithful sRGB but can clip a bright scene (use a negative Exposure). "
                    "Depth and ID passes always render linear (Raw) regardless of this.",
        items=(("AgX", "AgX (recommended)", "No highlight clipping, natural colour"),
               ("Filmic", "Filmic", "Classic film curve, no clipping"),
               ("Standard", "Standard (faithful sRGB)", "Faithful, but a bright scene can overexpose")),
        default="AgX",
    )
    color_exposure: FloatProperty(
        name="Exposure",
        description="Exposure offset (stops) on the COLOR render only. Use a negative value to pull "
                    "a too-bright scene back into range (especially with the Standard look).",
        default=0.0, min=-10.0, max=10.0, soft_min=-5.0, soft_max=5.0,
    )
    export_id: BoolProperty(
        name="Object ID",
        description="Render an object-ID (segmentation) map per view: each item gets a distinct flat "
                    "colour (id/<frame>.png) + id/id_map.json. Used to cut elements out together with depth.",
        default=False,
    )
    export_object_depth: BoolProperty(
        name="Per-object Depth",
        description="Split the full-scene depth into one depth file per item (objects/<name>/depth/...) "
                    "by masking the full-scene depth with the same visible-surface Object ID buffer.",
        default=False,
    )
    export_normal: BoolProperty(
        name="Scene Normal",
        description="Full-scene visible-surface world-space XYZ normals in 16-bit PNG (RGB = (normal + 1) / 2).",
        default=False,
    )
    export_object_normal: BoolProperty(
        name="Object Normal",
        description="Object-aware visible world normals as 16-bit PNG, derived from Scene Normal and Object ID without per-object rendering.",
        default=False,
    )
    export_object_mask: BoolProperty(
        name="Per-object Mask",
        description="Per-item binary GS mask (white = this item, black = everything else), split from the "
                    "full-scene Object ID buffer. Occluded pixels remain excluded.",
        default=False,
    )
    export_material_id: BoolProperty(
        name="Material ID",
        description="Stable visible material segmentation map plus material_map.json.",
        default=False,
    )
    object_split_mode: EnumProperty(
        name="Object Split Storage",
        description="Store runtime scene buffers once, or write a visible non-empty file per object.",
        items=(
            ("VIRTUAL_SPLIT", "Virtual / Runtime", "Save scene buffers and Object ID; split at training time"),
            ("PHYSICAL_FILES", "Physical Files", "Write per-object Depth, Normal and Mask files"),
        ),
        default="VIRTUAL_SPLIT",
    )
    depth_format: EnumProperty(
        name="Depth Format",
        description="File format for the depth output (whole-scene and per-object).",
        items=(("EXR", "EXR (float, linear m)", "32-bit OpenEXR storing true linear depth in metres"),
               ("PNG", "PNG (16-bit, mm)", "16-bit grayscale PNG storing depth in millimetres "
                                           "(depth_m*1000, max 65.5 m); recover metres = pixel16/1000")),
        default="EXR",
    )
    object_group_mode: EnumProperty(
        name="Item Granularity",
        description="What counts as one 'item' for the ID map and per-object depth.",
        items=(("GROUP", "Top group / Empty", "All meshes under one Empty = one item (a house = 1, each prop = 1)"),
               ("MESH", "Each mesh object", "Every mesh object is its own item"),
               ("COLLECTION", "Each collection", "Every Blender collection is one item")),
        default="GROUP",
    )
    background_render: BoolProperty(name="Background Render", default=True)
    background_chunk_size: IntProperty(
        name="Frames per Process", default=500, min=1, max=5000,
        description=(
            "Balanced default: reuse one warm Blender process for up to 500 frames, then exit "
            "to release RAM, VRAM and Cycles caches."
        ),
    )
    render_progress: FloatProperty(name="Render Progress", default=0.0, min=0.0, max=1.0, subtype="FACTOR")
    render_status: StringProperty(name="Render Status", default="Ready")
    render_log_path: StringProperty(name="Render Log Path", default="")
    image_prefix: StringProperty(name="Image Prefix", default="frame")
    auto_create_rig: BoolProperty(name="Auto-create Rig", default=True)
    incremental: BoolProperty(name="Incremental", default=True)
    transparent_background: BoolProperty(name="Transparent Background", default=False)
    render_rgb: BoolProperty(name="RGB Beauty", default=True)
    export_depth: BoolProperty(name="Depth", default=False)
    export_masks: BoolProperty(name="Masks", default=False)
    mask_collection: PointerProperty(name="Mask Collection", type=bpy.types.Collection)
    point_samples_per_view: IntProperty(name="Point Samples/View", default=400, min=0, max=4096)
    point_dedup_size: FloatProperty(name="Point Dedup Size", default=0.03, min=0.0001, soft_max=1.0)
    ray_distance: FloatProperty(name="Ray Distance", default=1000.0, min=0.1, soft_max=100000.0)
    volume_size: FloatVectorProperty(name="Volume Size", default=(4.0, 4.0, 2.0), size=3, min=0.01, update=live_update_camera_rig)
    volume_x: IntProperty(name="Volume X", default=5, min=1, max=128, update=live_update_camera_rig)
    volume_y: IntProperty(name="Volume Y", default=5, min=1, max=128, update=live_update_camera_rig)
    volume_z: IntProperty(name="Volume Z", default=3, min=1, max=128, update=live_update_camera_rig)
    volume_jitter: FloatProperty(name="Jitter", default=0.0, min=0.0, soft_max=10.0, update=live_update_camera_rig)
    random_seed: IntProperty(name="Random Seed", default=42, update=live_update_camera_rig)
    exclude_collection: PointerProperty(name="Exclusion Collection", type=bpy.types.Collection, update=live_update_camera_rig)
    avoid_clipping: BoolProperty(name="Avoid Wall Clipping", default=True, update=live_update_camera_rig)
    min_clearance: FloatProperty(name="Min Clearance (m)", default=0.30, min=0.01, soft_max=5.0, update=live_update_camera_rig)
    eye_height: FloatProperty(name="Eye Height (m)", default=1.5, min=0.05, soft_max=10.0)
    floorplan_method: EnumProperty(name="排线方法", items=[
        ("CONTOUR", "轮廓排线（多层）", "沿自由空间轮廓生成长线，使用真实通行连接，保留独立预览"),
        ("GRID", "井字排线（旧版）", "保留旧的直线扫描流程，用于对照"),
    ], default="CONTOUR")
    contour_probe_spacing: FloatProperty(name="采样间距（m）", default=0.25, min=0.05, max=1.0)
    contour_clearance: FloatProperty(name="障碍净距（m）", default=0.20, min=0.05, max=2.0)
    contour_floor_collection: PointerProperty(name="地面集合（可选）", type=bpy.types.Collection,
        description="仅从此集合提取地面；全部可见网格仍参与避障。指定地面可减少家具被误认成楼层")
    contour_max_step: FloatProperty(name="最大相邻地面高差（m）", default=0.38, min=0.05, max=0.6)
    contour_max_bridge: FloatProperty(name="最长连接路程（m）", default=15.0, min=0.0, max=100.0)
    contour_min_area: FloatProperty(name="最小连通区域（㎡）", default=1.5, min=0.1, max=100.0)
    contour_min_floor_area: FloatProperty(name="地面最小面积（㎡）", default=4.0, min=0.5, max=100.0,
        description="自动识别时排除小型家具顶面；楼梯连接另行保留。小平台可指定地面集合")
    contour_smoothing: IntProperty(name="转角平滑", default=2, min=0, max=4)
    contour_adapt_top: BoolProperty(name="顶层随净空调整", default=True,
        description="楼梯和低顶棚处保持安全顶距，必要时降低顶层机位；不会低于中层附近")
    contour_detail_enabled: BoolProperty(name="细部表面补线", default=True,
        description="检查可观察表面的多视角与视差，为主线没有充分观察的细节添加独立短线")
    contour_detail_precision: FloatProperty(name="表面采样尺度（m）", default=0.10, min=0.05, max=0.5)
    contour_detail_grid: FloatProperty(name="缝隙采样间距（m）", default=0.15, min=0.05, max=0.3,
        description="细部通行图单独细化，用于家具间隙和楼梯各高度的补线")
    contour_detail_clearance: FloatProperty(name="细部障碍净距（m）", default=0.10, min=0.05, max=0.5,
        description="细部短线的相机中心净距，独立于主线；小于此宽度两倍的缝隙仍不会生成路径")
    contour_detail_distance: FloatProperty(name="细部观察距离上限（m）", default=2.5, min=0.5, max=5.0)
    contour_detail_baseline: FloatProperty(name="最小观察基线（m）", default=0.12, min=0.05, max=0.5)
    contour_detail_angle: FloatProperty(name="最小观察夹角（度）", default=12.0, min=3.0, max=35.0)
    contour_detail_budget: IntProperty(name="细部短线数量上限", default=48, min=1, max=100,
        description="总预算在各个高度层之间均衡分配，避免高层抢占低层和楼梯补线")
    contour_running: BoolProperty(default=False, options={"SKIP_SAVE"})
    contour_status: StringProperty(default="", options={"SKIP_SAVE"})
    floorplan_spacing: FloatProperty(name="Path Spacing (m)", default=0.6, min=0.1, soft_max=10.0)
    floorplan_curve_density: FloatProperty(
        name="曲线密度（点/米）",
        description="每米生成多少个排线采样点；数值越大，曲线越密。",
        default=2.2, min=0.1, soft_max=10.0,
    )
    floorplan_probe_spacing: FloatProperty(
        name="Narrow Probe Grid (m)",
        description="Extra raster spacing cap for hallways and door openings. Lower values find narrower corridors.",
        default=10.0, min=0.05, soft_max=10.0,
    )
    floorplan_wall_margin: FloatProperty(name="Wall Margin (m)", default=0.5, min=0.05, soft_max=5.0)
    floorplan_narrow_margin: FloatProperty(
        name="Narrow Margin (m)",
        description="Minimum horizontal clearance used to keep corridor cells alive. Closed doors still block connectivity.",
        default=0.12, min=0.03, soft_max=1.0,
    )
    floorplan_corridor_center_width: FloatProperty(
        name="Corridor Center Width (m)",
        description="Spaces narrower than this become a single centred line (corridors, entryways, closets); wider spaces get a full grid.",
        default=1.6, min=0.2, soft_max=5.0,
    )
    floorplan_bridge_gap: FloatProperty(
        name="Bridge Gap (m)",
        description="Internal: a gap up to this length is bridged into one continuous line when the straight span is clear at the line height (flies over low furniture, never through walls).",
        default=2.5, min=0.0, soft_max=6.0,
    )
    floorplan_min_line: FloatProperty(
        name="Min Grid Line (m)",
        description="Internal: wide-room grid lines shorter than this are dropped as confetti; centred corridor/small-room spines are always kept.",
        default=1.0, min=0.0, soft_max=5.0,
    )
    floorplan_space_mode: EnumProperty(name="排线空间范围", items=FLOORPLAN_SPACE_MODE_ITEMS, default="REACHABLE")
    floorplan_expand: BoolProperty(name="室内井字排线", default=True)
    floorplan_seed_mode: EnumProperty(name="Reachable From", items=FLOORPLAN_SEED_MODE_ITEMS, default="CURSOR")
    floorplan_seed_object: PointerProperty(name="Seed Object", type=bpy.types.Object)
    floorplan_layer_mode: EnumProperty(name="Route Layers", items=FLOORPLAN_LAYER_MODE_ITEMS, default="THREE")
    floorplan_low_height: FloatProperty(
        name="Low Height from Floor (m)",
        description="Low route height above local floor. Clamped to the safe threshold 0.10-0.50 m.",
        default=0.30, min=0.10, max=0.50,
    )
    floorplan_mid_height: FloatProperty(name="Mid Height from Floor (m)", default=1.20, min=0.10, soft_max=5.0)
    floorplan_high_height: FloatProperty(name="High Height from Floor (m)", default=1.70, min=0.10, soft_max=8.0)
    floorplan_top_height: FloatProperty(name="Top Height from Floor (m)", default=2.00, min=0.10, soft_max=8.0)
    floorplan_ceiling_offset: FloatProperty(
        name="Top Offset from Ceiling (m)",
        description="Top route is measured downward from the local ceiling so uneven indoor ceilings stay safe.",
        default=0.50, min=0.05, soft_max=3.0,
    )
    floorplan_min_headroom: FloatProperty(name="Min Headroom (m)", default=1.0, min=0.20, soft_max=5.0)
    floorplan_min_region_cells: IntProperty(name="Min Island Cells", default=1, min=1, max=1000)


class GSCOLMAP_OT_create_cameras(Operator):
    bl_idname = "gs_colmap.create_cameras"
    bl_label = "Create Camera Rig"
    bl_options = {"REGISTER", "UNDO"}

    _timer = None
    _phase = "idle"
    _staging = None
    _collection = None
    _sampled = None
    _cameras = None
    _cleanup_objects = None
    _debug_specs = None
    _actual_radii = None
    _index = 0
    _move_index = 0
    _debug_index = 0
    _style_index = 0
    _adaptive_shrunk = 0
    _fallback_count = 0
    _skipped_count = 0
    _requested_radius = 0.0
    _generation_started = 0.0
    _commit_started = False
    _cancel_error = None
    _station_state = None
    _visual_states = None

    @staticmethod
    def _uses_incremental_shell(settings):
        return (
            settings.rig_mode == "PATH"
            and settings.path_capture_mode == "LEGACY_PANORAMA_CUBE"
            and getattr(settings, "path_station_array_mode", "SPHERICAL_SHELL_12") == "SPHERICAL_SHELL_12"
        )

    @staticmethod
    def _tag_redraw(context):
        screen = getattr(context, "screen", None)
        for area in getattr(screen, "areas", ()):
            area.tag_redraw()

    def _set_progress(self, context, fraction, status):
        settings = context.scene.gs_colmap_settings
        fraction = min(1.0, max(0.0, float(fraction)))
        settings.camera_build_progress = fraction
        settings.camera_build_status = status
        context.window_manager.progress_update(int(round(fraction * 1000.0)))
        self._tag_redraw(context)

    @staticmethod
    def _remove_build_object(obj):
        try:
            current = bpy.data.objects.get(obj.name)
        except ReferenceError:
            return
        if current is None:
            return
        if current.type == "CAMERA":
            _remove_camera(current)
        else:
            bpy.data.objects.remove(current, do_unlink=True)

    def _remove_batch(self, objects, max_items=24, time_budget=0.04):
        started = time.monotonic()
        removed = 0
        while objects and removed < max_items and time.monotonic() - started < time_budget:
            self._remove_build_object(objects.pop())
            removed += 1
        return removed

    def _remove_empty_staging(self):
        staging = self._staging
        if staging is None:
            return
        try:
            if len(staging.objects) == 0:
                bpy.data.collections.remove(staging)
        except ReferenceError:
            pass
        self._staging = None

    def _finish(self, context):
        global _LIVE_UPDATE_LOCK
        if self._timer is not None:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        context.window_manager.progress_end()
        settings = context.scene.gs_colmap_settings
        settings.camera_build_active = False
        settings.camera_build_cancelable = False
        settings.camera_build_cancel_requested = False
        restore_camera_mesh_visuals(self._visual_states)
        self._visual_states = None
        _LIVE_UPDATE_LOCK = False
        self._tag_redraw(context)

    def _begin_cancel(self, context, error=None):
        settings = context.scene.gs_colmap_settings
        settings.camera_build_cancelable = False
        settings.camera_build_cancel_requested = False
        self._cancel_error = error
        self._cleanup_objects = list(getattr(self._staging, "objects", ()))
        self._phase = "cancel_cleanup"
        label = "发生错误，正在清理临时相机" if error else "正在取消并清理临时相机"
        self._set_progress(context, settings.camera_build_progress, label)

    def _prepare(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        self._collection = collection_get(settings.camera_collection)
        self._visual_states = hide_camera_mesh_visuals(scene)
        staging_name = f"{settings.camera_collection}__BUILDING"
        self._staging = bpy.data.collections.get(staging_name)
        if self._staging is None:
            self._staging = bpy.data.collections.new(staging_name)
        if self._staging not in tuple(scene.collection.children):
            scene.collection.children.link(self._staging)
        self._staging.hide_render = True
        self._cleanup_objects = list(self._staging.objects)

        center = target_location(settings)
        radius = max(0.01, settings.radius)
        count = path_camera_count(
            settings,
            curve_polyline_points(settings.path_object, settings.path_samples_per_segment),
        )
        self._sampled = sample_path_components(settings)
        if not self._sampled:
            fallback = [
                center + Vector((radius * math.cos(i * math.tau / count), radius * math.sin(i * math.tau / count), 0))
                for i in range(count)
            ]
            self._sampled = [(point, center - point, settings.path_object) for point in fallback]

        self._cameras = []
        self._debug_specs = []
        self._actual_radii = []
        self._index = 0
        self._move_index = 0
        self._debug_index = 0
        self._style_index = 0
        self._adaptive_shrunk = 0
        self._fallback_count = 0
        self._skipped_count = 0
        self._requested_radius = float(getattr(settings, "shell_radius", 0.18))
        self._depsgraph = context.evaluated_depsgraph_get()
        self._generation_started = time.monotonic()
        self._station_state = None
        self._phase = "staging_cleanup" if self._cleanup_objects else "generate"
        self._set_progress(context, 0.02, f"准备完成：{len(self._sampled)} 个路径站点")

    def _mark_final_names(self, group):
        for camera in group:
            station = int(camera.get("station_index", 0))
            camera_index = int(camera.get("shell_camera_index", 0))
            if camera.get("rig_type") == "SPHERICAL_SHELL_12":
                name = f"GS_CAM_S{station:04d}_SH{camera_index:02d}"
            else:
                suffix = CUBE_FACES[camera_index][0]
                name = f"GS_CAM_S{station:04d}_{suffix}"
            camera["_gs_build_final_object_name"] = name
            camera["_gs_build_final_data_name"] = f"{name}_Data"

    def _start_position_probe(self, state, position, mode):
        state["position"] = Vector(position)
        state["position_mode"] = mode
        state["clearance_ray_index"] = 0

    def _begin_station_probe(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        point, tangent, path_object = self._sampled[self._index]
        path_forward = _stable_path_forward(self._sampled, self._index, tangent)
        directions, frame = _world_shell_directions(path_forward)
        clearance = max(1e-6, float(getattr(settings, "min_clearance", 0.30))) * _scene_units_per_meter(scene)
        state = {
            "center": Vector(point),
            "path_forward": path_forward,
            "source_curve": path_object,
            "directions": directions,
            "frame": frame,
            "clearance": clearance,
            "clearance_directions": _shell_clearance_directions(),
            "search": max(clearance * 6.0, 2.0 * _scene_units_per_meter(scene)),
            "candidates": _shell_radius_candidates(scene, settings),
            "candidate_index": 0,
            "direction_index": 0,
        }
        self._station_state = state
        candidate_scene = state["candidates"][0][1]
        self._start_position_probe(state, state["center"] + directions[0] * candidate_scene, "shell")

    def _record_station_result(
        self, context, group, actual_radius, used_fallback, skipped, debug_spec
    ):
        total = len(self._sampled)
        self._mark_final_names(group)
        self._cameras.extend(group)
        if actual_radius > 0.0:
            self._actual_radii.append(actual_radius)
            if actual_radius + 1e-9 < self._requested_radius:
                self._adaptive_shrunk += 1
        self._fallback_count += int(used_fallback)
        self._skipped_count += int(skipped)
        if debug_spec is not None:
            self._debug_specs.append((self._index + 1, debug_spec))
        self._index += 1
        self._station_state = None

        elapsed = max(0.001, time.monotonic() - self._generation_started)
        eta = elapsed * max(0, total - self._index) / max(1, self._index)
        fraction = 0.05 + 0.70 * self._index / max(1, total)
        self._set_progress(
            context,
            fraction,
            f"生成站点 {self._index}/{total} · 相机 {len(self._cameras)} · 剩余约 {_fmt_time(eta)}",
        )

    def _create_safe_shell_group(self, context, state, actual_radius_m, actual_radius_scene):
        settings = context.scene.gs_colmap_settings
        curve = state["source_curve"]
        curve_name = getattr(curve, "name", "") if curve else ""
        group = []
        for shell_index, direction in enumerate(state["directions"]):
            camera = create_camera_at(
                f"GS_BUILD_S{self._index + 1:04d}_SH{shell_index:02d}",
                state["center"] + direction * actual_radius_scene,
                self._staging,
                settings,
            )
            _orient_camera_radially(camera, direction, state["path_forward"])
            _apply_path_camera_metadata(
                camera,
                self._index + 1,
                shell_index,
                state["center"],
                actual_radius_m,
                direction,
                "SPHERICAL_SHELL_12",
                False,
                curve_name,
            )
            group.append(camera)
        debug_spec = (state["center"].copy(), state["frame"], actual_radius_scene)
        self._record_station_result(context, group, actual_radius_m, False, False, debug_spec)

    def _finish_unsafe_station(self, context, state, fallback_safe):
        settings = context.scene.gs_colmap_settings
        use_fallback = (
            getattr(settings, "shell_failure_policy", "LEGACY_SIX_FALLBACK") == "LEGACY_SIX_FALLBACK"
        )
        if use_fallback and fallback_safe:
            curve = state["source_curve"]
            curve_name = getattr(curve, "name", "") if curve else ""
            group = _generate_legacy_six_camera_group(
                settings,
                self._staging,
                state["center"],
                self._index + 1,
                curve_name,
                fallback_used=True,
            )
            self._record_station_result(context, group, 0.0, True, False, None)
        else:
            self._record_station_result(context, [], 0.0, False, True, None)

    def _advance_shell_candidate(self, context, state):
        state["candidate_index"] += 1
        state["direction_index"] = 0
        if state["candidate_index"] < len(state["candidates"]):
            candidate_scene = state["candidates"][state["candidate_index"]][1]
            self._start_position_probe(
                state, state["center"] + state["directions"][0] * candidate_scene, "shell"
            )
            return
        settings = context.scene.gs_colmap_settings
        if getattr(settings, "shell_failure_policy", "LEGACY_SIX_FALLBACK") == "LEGACY_SIX_FALLBACK":
            self._start_position_probe(state, state["center"], "fallback")
        else:
            self._finish_unsafe_station(context, state, fallback_safe=False)

    def _position_probe_passed(self, context, state):
        if state["position_mode"] == "fallback":
            self._finish_unsafe_station(context, state, fallback_safe=True)
            return
        candidate_m, candidate_scene = state["candidates"][state["candidate_index"]]
        direction = state["directions"][state["direction_index"]]
        if candidate_scene > 2e-4:
            hit, _location, _normal, _face, obj, _matrix = context.scene.ray_cast(
                self._depsgraph,
                state["center"] + direction * 1e-4,
                direction,
                distance=max(0.0, candidate_scene - 2e-4),
            )
            if hit and obj and not is_camera_mesh_visual(obj):
                self._advance_shell_candidate(context, state)
                return
        state["direction_index"] += 1
        if state["direction_index"] >= len(state["directions"]):
            self._create_safe_shell_group(context, state, candidate_m, candidate_scene)
            return
        direction = state["directions"][state["direction_index"]]
        self._start_position_probe(
            state, state["center"] + direction * candidate_scene, "shell"
        )

    def _generate_station_slice(self, context):
        if self._station_state is None:
            self._begin_station_probe(context)
        state = self._station_state
        if state is None:
            return

        started = time.monotonic()
        ray_budget = 12
        rays = 0
        while self._station_state is state and rays < ray_budget and time.monotonic() - started < 0.015:
            ray_index = state["clearance_ray_index"]
            directions = state["clearance_directions"]
            if ray_index >= len(directions):
                self._position_probe_passed(context, state)
                continue
            direction = directions[ray_index]
            hit, location, _normal, _face, _obj, _matrix = context.scene.ray_cast(
                self._depsgraph,
                state["position"] + direction * 1e-4,
                direction,
                distance=state["search"],
            )
            distance = (location - state["position"]).length if hit else state["search"]
            rays += 1
            if distance + 1e-8 < state["clearance"]:
                if state["position_mode"] == "fallback":
                    self._finish_unsafe_station(context, state, fallback_safe=False)
                else:
                    self._advance_shell_candidate(context, state)
                continue
            state["clearance_ray_index"] += 1

        if self._station_state is state:
            total_stations = len(self._sampled)
            candidate_count = len(state["candidates"])
            candidate_number = min(candidate_count, state["candidate_index"] + 1)
            direction_number = min(12, state["direction_index"] + 1)
            subprogress = state["direction_index"] / 12.0
            fraction = 0.05 + 0.70 * (self._index + subprogress) / max(1, total_stations)
            mode = "回退安全检测" if state["position_mode"] == "fallback" else (
                f"半径 {candidate_number}/{candidate_count} · 位置 {direction_number}/12"
            )
            self._set_progress(
                context,
                fraction,
                f"站点 {self._index + 1}/{total_stations} · {mode} · 相机 {len(self._cameras)}",
            )

    def _begin_commit(self, context):
        settings = context.scene.gs_colmap_settings
        settings.camera_build_cancelable = False
        self._commit_started = True
        old_cameras = [obj for obj in self._collection.objects if obj.type == "CAMERA"]
        old_debug = [obj for obj in bpy.data.objects if obj.get("gs_shell_debug_proxy")]
        self._cleanup_objects = old_cameras + old_debug
        self._commit_cleanup_total = len(self._cleanup_objects)
        _clear_path_camera_rig_report(context.scene)
        self._phase = "commit_cleanup"
        self._set_progress(context, 0.76, "正在提交新阵列")

    def _commit_move_batch(self, context):
        total = len(self._cameras)
        started = time.monotonic()
        moved = 0
        while self._move_index < total and moved < 24 and time.monotonic() - started < 0.04:
            camera = self._cameras[self._move_index]
            if self._collection not in tuple(camera.users_collection):
                self._collection.objects.link(camera)
            if self._staging in tuple(camera.users_collection):
                self._staging.objects.unlink(camera)
            object_name = str(camera.get("_gs_build_final_object_name", camera.name))
            data_name = str(camera.get("_gs_build_final_data_name", camera.data.name))
            camera.name = object_name
            camera.data.name = data_name
            for key in ("_gs_build_final_object_name", "_gs_build_final_data_name"):
                if key in camera:
                    del camera[key]
            self._move_index += 1
            moved += 1
        fraction = 0.83 + 0.05 * self._move_index / max(1, total)
        self._set_progress(context, fraction, f"提交相机 {self._move_index}/{total}")

    def _store_report(self, context):
        radii = self._actual_radii
        _store_path_camera_rig_report(context.scene, {
            "mode": "SPHERICAL_SHELL_12",
            "station_count": len(self._sampled),
            "cameras_per_station": 12,
            "requested_shell_radius": self._requested_radius,
            "average_actual_shell_radius": sum(radii) / len(radii) if radii else 0.0,
            "minimum_actual_shell_radius": min(radii) if radii else 0.0,
            "maximum_actual_shell_radius": max(radii) if radii else 0.0,
            "adaptive_shrunk_station_count": self._adaptive_shrunk,
            "legacy_fallback_station_count": self._fallback_count,
            "skipped_station_count": self._skipped_count,
            "duplicate_camera_origin_count": _duplicate_camera_origin_count(context.scene, self._cameras),
        })

    def _complete(self, context):
        elapsed = time.monotonic() - self._generation_started
        self._store_report(context)
        settings = context.scene.gs_colmap_settings
        settings.camera_build_progress = 1.0
        settings.camera_build_status = (
            f"完成：{len(self._sampled)} 个站点 · {len(self._cameras)} 台相机 · {_fmt_time(elapsed)}"
        )
        try:
            context.view_layer.update()
        except Exception:
            pass
        self._finish(context)
        self.report({"INFO"}, f"{tr(settings, 'msg_cameras_done')}: {len(self._cameras)}")

    def invoke(self, context, _event):
        global _LIVE_UPDATE_LOCK
        settings = context.scene.gs_colmap_settings
        if not self._uses_incremental_shell(settings):
            return self.execute(context)
        if settings.camera_build_active:
            self.report({"WARNING"}, "球壳相机生成任务已经在运行")
            return {"CANCELLED"}

        self._phase = "prepare"
        self._commit_started = False
        self._cancel_error = None
        settings.camera_build_active = True
        settings.camera_build_cancelable = True
        settings.camera_build_cancel_requested = False
        settings.camera_build_progress = 0.0
        settings.camera_build_status = "正在准备球壳相机任务"
        _LIVE_UPDATE_LOCK = True
        context.window_manager.progress_begin(0, 1000)
        self._timer = context.window_manager.event_timer_add(0.02, window=context.window)
        context.window_manager.modal_handler_add(self)
        self._tag_redraw(context)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        cameras = create_rig(context.scene, settings)
        self.report({"INFO"}, f"{tr(settings, 'msg_cameras_done')}: {len(cameras)}")
        return {"FINISHED"}

    def modal(self, context, event):
        settings = context.scene.gs_colmap_settings
        if (
            (event.type == "ESC" or settings.camera_build_cancel_requested)
            and settings.camera_build_cancelable
            and self._phase != "cancel_cleanup"
        ):
            self._begin_cancel(context)
            return {"RUNNING_MODAL"}
        if event.type != "TIMER":
            return {"PASS_THROUGH"}

        try:
            if self._phase == "prepare":
                self._prepare(context)
            elif self._phase == "staging_cleanup":
                self._remove_batch(self._cleanup_objects)
                if not self._cleanup_objects:
                    self._phase = "generate"
            elif self._phase == "generate":
                if self._index < len(self._sampled):
                    self._generate_station_slice(context)
                if self._index >= len(self._sampled):
                    self._begin_commit(context)
            elif self._phase == "commit_cleanup":
                self._remove_batch(self._cleanup_objects)
                done = self._commit_cleanup_total - len(self._cleanup_objects)
                fraction = 0.76 + 0.07 * done / max(1, self._commit_cleanup_total)
                self._set_progress(context, fraction, f"清理旧阵列 {done}/{self._commit_cleanup_total}")
                if not self._cleanup_objects:
                    self._phase = "commit_move"
            elif self._phase == "commit_move":
                self._commit_move_batch(context)
                if self._move_index >= len(self._cameras):
                    self._remove_empty_staging()
                    self._phase = "debug"
            elif self._phase == "debug":
                if getattr(settings, "show_shell_debug_mesh", False) and self._debug_index < len(self._debug_specs):
                    station_index, (shell_center, frame, radius_scene) = self._debug_specs[self._debug_index]
                    _create_shell_debug_proxy(context.scene, station_index, shell_center, frame, radius_scene)
                    self._debug_index += 1
                    fraction = 0.88 + 0.02 * self._debug_index / max(1, len(self._debug_specs))
                    self._set_progress(
                        context, fraction, f"创建球壳代理 {self._debug_index}/{len(self._debug_specs)}"
                    )
                else:
                    self._phase = "style"
            elif self._phase == "style":
                style_enabled = bool(getattr(settings, "camera_mesh_style", True)) and not bpy.app.background
                if style_enabled and self._style_index < len(self._cameras):
                    started = time.monotonic()
                    count = 0
                    while (
                        self._style_index < len(self._cameras)
                        and count < 12
                        and time.monotonic() - started < 0.04
                    ):
                        ensure_camera_mesh_visual(context.scene, settings, self._cameras[self._style_index])
                        self._style_index += 1
                        count += 1
                    fraction = 0.90 + 0.10 * self._style_index / max(1, len(self._cameras))
                    self._set_progress(
                        context, fraction, f"创建相机外观 {self._style_index}/{len(self._cameras)}"
                    )
                else:
                    self._complete(context)
                    return {"FINISHED"}
            elif self._phase == "cancel_cleanup":
                self._remove_batch(self._cleanup_objects)
                if not self._cleanup_objects:
                    self._remove_empty_staging()
                    if self._cancel_error:
                        settings.camera_build_status = f"生成失败：{self._cancel_error}"
                        self.report({"ERROR"}, str(self._cancel_error))
                    else:
                        settings.camera_build_status = "已取消，原相机阵列保持不变"
                        self.report({"WARNING"}, "球壳相机生成已取消")
                    self._finish(context)
                    return {"CANCELLED"}
        except Exception as exc:
            if not self._commit_started:
                self._begin_cancel(context, error=exc)
                return {"RUNNING_MODAL"}
            settings.camera_build_status = f"提交失败：{exc}"
            self._finish(context)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        return {"RUNNING_MODAL"}


class GSCOLMAP_OT_cancel_camera_build(Operator):
    bl_idname = "gs_colmap.cancel_camera_build"
    bl_label = "取消球壳相机生成"

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        if settings.camera_build_active and settings.camera_build_cancelable:
            settings.camera_build_cancel_requested = True
        return {"FINISHED"}


class GSCOLMAP_OT_training_recommendations(Operator):
    bl_idname = "gs_colmap.training_recommendations"
    bl_label = "Apply Training Recommendations"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        cameras = active_dataset_cameras(scene, settings)
        report = apply_training_recommendations(scene, settings, cameras)
        self.report(
            {"INFO"},
            f"Training consistency warnings: {report['warning_count']}",
        )
        return {"FINISHED"}


class GSCOLMAP_OT_camera_mesh_style(Operator):
    bl_idname = "gs_colmap.camera_mesh_style"
    bl_label = "相机 Mesh 样式"
    bl_description = "为已有真实相机添加或移除绿色金字塔 Mesh 外观，不影响渲染和导出"
    bl_options = {"REGISTER", "UNDO"}

    action: EnumProperty(items=(
        ("APPLY_SCENE", "转换场景相机", "为场景中全部相机创建绿色金字塔外观"),
        ("APPLY_SELECTED", "转换选中相机", "为选中的相机或相机代理创建外观"),
        ("REMOVE_SCENE", "移除场景外观", "移除场景中全部绿色金字塔相机外观"),
    ), default="APPLY_SCENE")

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        if self.action == "APPLY_SELECTED":
            cameras = {
                obj if obj.type == "CAMERA" else obj.parent
                for obj in context.selected_objects
                if obj.type == "CAMERA" or is_camera_mesh_visual(obj)
            }
            cameras.discard(None)
        else:
            cameras = {obj for obj in context.scene.objects if obj.type == "CAMERA"}
        if self.action == "REMOVE_SCENE":
            settings["camera_mesh_style"] = False
            for camera in cameras:
                remove_camera_mesh_visual(camera)
            self.report({"INFO"}, f"已移除 {len(cameras)} 台相机的 Mesh 外观")
        else:
            if self.action == "APPLY_SCENE":
                settings["camera_mesh_style"] = True
            count = sum(
                ensure_camera_mesh_visual(context.scene, settings, camera) is not None
                for camera in cameras
            )
            self.report({"INFO"}, f"已转换 {count} 台相机为绿色金字塔 Mesh 样式")
        return {"FINISHED"}



class GSCOLMAP_OT_create_scientific_cameras(Operator):
    bl_idname = "gs_colmap.create_scientific_cameras"
    bl_label = "创建科学相机阵列"
    bl_options = {"REGISTER", "UNDO"}

    _timer = None
    _state = None
    _index = 0
    _stage = "prefilter"
    _visual_states = None
    _original_scene_frame = None
    _original_scene_subframe = None

    def _progress(self, settings, stage, current, total):
        fraction = _scientific_progress_fraction(stage, current, total)
        settings.scientific_planning_progress = fraction
        settings.scientific_planning_status = f"{stage}: {current}/{total}"
        bpy.context.window_manager.progress_update(int(fraction * 1000))

    def _finish(self, context):
        if self._timer is not None:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        context.window_manager.progress_end()
        context.scene.gs_colmap_settings.scientific_planning_active = False
        restore_camera_mesh_visuals(self._visual_states)
        self._visual_states = None
        if self._original_scene_frame is not None:
            context.scene.frame_set(
                self._original_scene_frame, subframe=self._original_scene_subframe or 0.0
            )
            self._original_scene_frame = None
            self._original_scene_subframe = None

    def invoke(self, context, _event):
        settings = context.scene.gs_colmap_settings
        scientific_planner.clear_debug_display(context.scene)
        origin_mode = getattr(settings, "scientific_origin_mode", "MANUAL_CURVE")
        components = path_components(settings)
        if origin_mode == "AUTO_GRID_PATH" and not components:
            visual_states = hide_camera_mesh_visuals(context.scene)
            try:
                generated = build_floorplan_path(context.scene, settings)
            finally:
                restore_camera_mesh_visuals(visual_states)
            if generated:
                settings.path_object = generated["primary"]
                settings.path_collection = generated["collection"]
                components = path_components(settings)
        if origin_mode in {"MANUAL_CURVE", "AUTO_GRID_PATH"} and not components:
            self.report({"ERROR"}, "请先指定路径曲线或路径曲线集合")
            return {"CANCELLED"}
        self._visual_states = hide_camera_mesh_visuals(context.scene)
        settings.scientific_cancel_requested = False
        if is_pose_sequence_mode(settings):
            self._original_scene_frame = context.scene.frame_current
            self._original_scene_subframe = context.scene.frame_subframe
            context.scene.frame_set(int(settings.sequence_source_scene_frame))
        settings.scientific_planning_active = True
        settings.scientific_planning_status = "准备科学路径规划"
        component_counts = path_component_counts(settings, components)
        context.window_manager.progress_begin(0, 1000)
        try:
            self._state = scientific_planner.prepare_scientific(
                context.scene, settings, components, component_counts,
                progress=lambda stage, current, total: self._progress(settings, stage, current, total),
                cancel=lambda: bool(settings.scientific_cancel_requested),
            )
        except Exception as exc:
            self._finish(context)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        if self._state is None:
            self._finish(context)
            self.report({"ERROR"}, "未找到合法科学机位")
            return {"CANCELLED"}
        self._index = 0
        self._stage = "prefilter"
        self._timer = context.window_manager.event_timer_add(0.01, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        settings = context.scene.gs_colmap_settings
        if event.type == "ESC" or settings.scientific_cancel_requested:
            settings.scientific_planning_status = "科学规划已取消，正式相机未更改"
            self._finish(context)
            return {"CANCELLED"}
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        try:
            if self._stage == "prefilter":
                self._index = scientific_planner.cast_prepared_batch(
                    self._state, self._index, count=4,
                    progress=lambda stage, current, total: self._progress(settings, stage, current, total),
                    cancel=lambda: bool(settings.scientific_cancel_requested),
                )
                if self._index < len(self._state["all_candidates"]):
                    return {"RUNNING_MODAL"}
                scientific_planner.prepare_final_quality(
                    self._state,
                    progress=lambda stage, current, total: self._progress(settings, stage, current, total),
                    cancel=lambda: bool(settings.scientific_cancel_requested),
                )
                self._index = 0
                self._stage = "final_quality"
                return {"RUNNING_MODAL"}
            self._index = scientific_planner.cast_final_quality_batch(
                self._state, self._index, count=4,
                progress=lambda stage, current, total: self._progress(settings, stage, current, total),
                cancel=lambda: bool(settings.scientific_cancel_requested),
            )
            if self._index < len(self._state["final_candidates"]):
                return {"RUNNING_MODAL"}
            plan = scientific_planner.finalize_scientific(
                self._state,
                progress=lambda stage, current, total: self._progress(settings, stage, current, total),
                cancel=lambda: bool(settings.scientific_cancel_requested),
            )
            collection = collection_get(settings.camera_collection)
            cameras = _apply_scientific_plan(
                context.scene, settings, collection, plan, replace_old=True,
                components=path_components(settings),
            )
        except scientific_planner.PlanningCancelled:
            settings.scientific_planning_status = "科学规划已取消，正式相机未更改"
            self._finish(context)
            return {"CANCELLED"}
        except Exception as exc:
            settings.scientific_planning_status = f"科学规划失败：{exc}"
            self._finish(context)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self._finish(context)
        if is_pose_sequence_mode(settings):
            sequence = active_pose_sequence(context.scene, settings)
            count = len(sequence.frames) if sequence else 0
            self.report({"INFO"}, f"科学位姿序列已规划：{count} 个姿态，1 个相机对象")
        else:
            self.report({"INFO"}, f"科学相机阵列已创建：{len(cameras)}")
        return {"FINISHED"}


class GSCOLMAP_OT_cancel_scientific_planning(Operator):
    bl_idname = "gs_colmap.cancel_scientific_planning"
    bl_label = "取消科学规划"

    def execute(self, context):
        context.scene.gs_colmap_settings.scientific_cancel_requested = True
        return {"FINISHED"}


class GSCOLMAP_OT_create_sequence_preview(Operator):
    bl_idname = "gs_colmap.create_sequence_preview"
    bl_label = "Create Pose Preview"
    bl_description = "Create optional quaternion keyframes for viewport inspection only"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        sequence = active_pose_sequence(scene, settings, prefer_disk=False)
        capture = bpy.data.objects.get(pose_sequence.CAPTURE_CAMERA_NAME)
        if sequence is None or capture is None:
            self.report({"ERROR"}, "Create a scientific pose sequence first.")
            return {"CANCELLED"}
        pose_sequence.create_preview_keyframes(scene, capture, sequence)
        pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
        self.report({"INFO"}, f"Created preview keyframes for {len(sequence.frames)} poses")
        return {"FINISHED"}


class GSCOLMAP_OT_clear_sequence_preview(Operator):
    bl_idname = "gs_colmap.clear_sequence_preview"
    bl_label = "Clear Pose Preview"
    bl_description = "Remove optional pose preview keyframes without changing the manifest"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        sequence = active_pose_sequence(scene, settings, prefer_disk=False)
        capture = bpy.data.objects.get(pose_sequence.CAPTURE_CAMERA_NAME)
        if capture is None:
            return {"CANCELLED"}
        pose_sequence.clear_preview_keyframes(scene, capture, sequence)
        if sequence is not None:
            pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
        return {"FINISHED"}


class GSCOLMAP_OT_refresh_sequence_debug(Operator):
    bl_idname = "gs_colmap.refresh_sequence_debug"
    bl_label = "Refresh Pose Display"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        sequence = active_pose_sequence(scene, settings)
        if sequence is None:
            self.report({"ERROR"}, "No scientific pose sequence is available.")
            return {"CANCELLED"}
        markers = build_pose_sequence_debug(scene, settings, sequence)
        self.report({"INFO"}, f"Displayed {len(markers)} lightweight pose markers")
        return {"FINISHED"}


class GSCOLMAP_OT_clear_sequence_debug(Operator):
    bl_idname = "gs_colmap.clear_sequence_debug"
    bl_label = "Clear Pose Display"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        clear_pose_sequence_debug(context.scene)
        context.scene.gs_colmap_settings.sequence_debug_mode = "OFF"
        return {"FINISHED"}


class GSCOLMAP_OT_render_pose_sequence(Operator):
    bl_idname = "gs_colmap.render_pose_sequence"
    bl_label = "Render Pose Sequence"
    bl_options = {"REGISTER"}
    mode: EnumProperty(items=(("PENDING", "Pending Only", ""), ("FULL", "Full Sequence", "")), default="PENDING")

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        old_incremental = settings.incremental
        try:
            settings.incremental = self.mode == "PENDING"
            return bpy.ops.gs_colmap.render_dataset("INVOKE_DEFAULT")
        finally:
            settings.incremental = old_incremental


class GSCOLMAP_OT_auto_floorplan_path(Operator):
    bl_idname = "gs_colmap.auto_floorplan_path"
    bl_label = "Auto Floor-plan Path"
    bl_options = {"REGISTER", "UNDO"}

    def invoke(self, context, event):
        settings = context.scene.gs_colmap_settings
        if settings.floorplan_method != "CONTOUR" or bpy.app.background:
            return self.execute(context)
        try:
            self._job = contour_jobs.start(context.scene, __file__)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        settings.contour_running = True
        settings.contour_status = "正在后台生成；基于启动时的场景快照"
        self._timer = context.window_manager.event_timer_add(.5, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        job = self._job
        if event.type == "ESC":
            job["cancelled"] = True
        if event.type != "TIMER" and not job["cancelled"]:
            return {"PASS_THROUGH"}
        try:
            settings = job["scene"].gs_colmap_settings
            if job["cancelled"]:
                contour_jobs.dispose(job)
                settings.contour_running = False
                settings.contour_status = "排线已取消"
                context.window_manager.event_timer_remove(self._timer)
                return {"CANCELLED"}
            if job["process"].poll() is None:
                settings.contour_status = contour_jobs.status(job)
                for area in context.screen.areas:
                    area.tag_redraw()
                return {"PASS_THROUGH"}
            result = contour_jobs.finish(job)
            settings.contour_running = False
            settings.contour_status = f"预览完成：{result['report']['route_count']} 条路径"
            contour_jobs.dispose(job)
            context.window_manager.event_timer_remove(self._timer)
            self.report({"INFO"}, settings.contour_status)
            return {"FINISHED"}
        except Exception as exc:
            try:
                job["scene"].gs_colmap_settings.contour_running = False
                job["scene"].gs_colmap_settings.contour_status = str(exc)
            except (ReferenceError,AttributeError):
                pass
            contour_jobs.dispose(job,keep_logs=True)
            context.window_manager.event_timer_remove(self._timer)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        visual_states = hide_camera_mesh_visuals(context.scene)
        try:
            result = build_floorplan_path(context.scene, settings)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            restore_camera_mesh_visuals(visual_states)
        if result is None:
            self.report({"ERROR"}, "No walkable interior found (need a floor, walls and a ceiling). Try lowering Wall Margin / Path Spacing.")
            return {"CANCELLED"}
        settings.path_object = result["primary"]
        settings.path_collection = result["collection"]
        settings.rig_mode = "PATH"
        method = "轮廓排线预览" if settings.floorplan_method == "CONTOUR" else "井字排线"
        self.report({"INFO"}, f"{method}已生成：{len(result['objects'])} 条曲线，{result['points']} 个点。接着点击创建相机阵列。")
        return {"FINISHED"}




















class GSCOLMAP_OT_cancel_contour(Operator):
    bl_idname = "gs_colmap.cancel_contour"
    bl_label = "取消轮廓排线"

    def execute(self, context):
        contour_jobs.cancel(context.scene)
        return {"FINISHED"}


class GSCOLMAP_OT_select_contour_role(Operator):
    bl_idname = "gs_colmap.select_contour_role"
    bl_label = "选择排线路径"
    role: EnumProperty(items=[("MAIN", "主线", ""), ("DETAIL", "细部短线", "")], default="DETAIL")

    def execute(self, context):
        collection = context.scene.gs_colmap_settings.path_collection
        if not collection:
            return {"CANCELLED"}
        selected = [obj for obj in collection.all_objects if obj.get("gs_route_role") == self.role
                    and obj.name in context.view_layer.objects]
        for obj in context.selected_objects:
            obj.select_set(False)
        for obj in selected:
            obj.select_set(True)
        if selected:
            context.view_layer.objects.active = selected[0]
        return {"FINISHED"}


class GSCOLMAP_OT_view_contour_layer(Operator):
    bl_idname = "gs_colmap.view_contour_layer"
    bl_label = "分层查看排线"
    layer: StringProperty(default='ALL')

    def execute(self, context):
        collection=context.scene.gs_colmap_settings.path_collection
        if not collection:return {'CANCELLED'}
        for obj in collection.all_objects:
            if obj.get('gs_contour_version') and obj.name in context.view_layer.objects:
                obj.hide_set(self.layer!='ALL' and obj.get('gs_floorplan_layer')!=self.layer)
        return {'FINISHED'}


class GSCOLMAP_OT_generate_patch_preview(Operator):
    bl_idname = "gs_colmap.generate_patch_preview"
    bl_label = "生成补齐预览"
    bl_description = "分析指定局部区域，只生成满足欠覆盖收益、视差和重叠要求的预览相机"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        existing = dataset_export_cameras(scene, settings)
        if not existing:
            self.report({"ERROR"}, "请先生成并保留原始相机")
            return {"CANCELLED"}
        selected = list(context.selected_objects)
        path_points = []
        if settings.patch_limit_to_path:
            for component in path_components(settings):
                path_points.extend(point.copy() for point in component["points"])
        wm = context.window_manager
        visual_states = hide_camera_mesh_visuals(scene)
        settings.patch_progress = 0.0
        settings.patch_status = "正在分析目标区域"
        wm.progress_begin(0, 1000)

        def progress(stage, current, total):
            fraction = current / max(1, total)
            settings.patch_progress = fraction
            settings.patch_status = f"{stage}: {current}/{total}"
            wm.progress_update(int(fraction * 1000))

        try:
            plan = coverage_patch.plan_patch(
                scene,
                settings,
                existing,
                selected_objects=selected,
                path_points=path_points,
                progress=progress,
            )
            cameras = coverage_patch.create_preview(scene, settings, plan)
            sync_camera_mesh_visuals(scene, settings, cameras)
        except Exception as exc:
            settings.patch_status = f"补齐分析失败: {exc}"
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            wm.progress_end()
            restore_camera_mesh_visuals(visual_states)
        settings.patch_progress = 1.0
        before = plan.report["patch_region_coverage_before"]
        after = plan.report["patch_region_coverage_after"]
        if cameras:
            settings.patch_status = f"预览 {len(cameras)} 台；覆盖率 {before:.1%} -> {after:.1%}"
        else:
            settings.patch_status = f"现有相机已满足目标；覆盖率 {before:.1%}，无需新增"
        self.report({"INFO"}, settings.patch_status)
        return {"FINISHED"}


class GSCOLMAP_OT_apply_patch_preview(Operator):
    bl_idname = "gs_colmap.apply_patch_preview"
    bl_label = "应用补齐"
    bl_description = "把当前仍保留的预览相机转为正式补齐相机，并写入补齐历史"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        try:
            cameras, report = coverage_patch.apply_preview(scene, settings)
            sequence = active_pose_sequence(scene, settings, prefer_disk=False)
            if sequence is not None:
                added = pose_sequence.append_patch_samples(sequence, settings, cameras)
                pose_sequence.save_sequence(scene, settings, sequence, write_disk=True)
                for camera in list(cameras):
                    _remove_camera(camera)
                cameras = []
                applied_count = len(added)
            else:
                sync_camera_mesh_visuals(scene, settings, cameras)
                applied_count = len(cameras)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        settings.patch_status = f"已应用 {len(cameras)} 台补齐相机；可执行仅渲染补齐相机"
        if sequence is not None:
            settings.patch_status = f"Applied {applied_count} Coverage Patch poses"
        self.report({"INFO"}, settings.patch_status)
        return {"FINISHED"}


class GSCOLMAP_OT_clear_patch_preview(Operator):
    bl_idname = "gs_colmap.clear_patch_preview"
    bl_label = "清除预览"
    bl_description = "只删除补齐预览，不删除原始相机或已应用的正式补齐相机"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        coverage_patch.clear_preview(context.scene)
        context.scene.gs_colmap_settings.patch_status = "补齐预览已清除"
        return {"FINISHED"}


class GSCOLMAP_OT_render_patch_cameras(Operator):
    bl_idname = "gs_colmap.render_patch_cameras"
    bl_label = "仅渲染补齐相机"
    bl_description = "只渲染正式补齐相机，随后合并更新 COLMAP、transforms 和补齐报告"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        if not settings.output_dir:
            self.report({"ERROR"}, tr(settings, "msg_no_output"))
            return {"CANCELLED"}
        sequence = active_pose_sequence(context.scene, settings)
        if sequence is not None:
            available = [sample for sample in sequence.frames if sample.is_coverage_patch]
        else:
            available = coverage_patch.final_cameras(context.scene)
        if not available:
            self.report({"ERROR"}, "没有已应用的正式补齐相机")
            return {"CANCELLED"}
        if settings.background_render:
            return bpy.ops.gs_colmap.render_dataset_background("INVOKE_DEFAULT", patch_only=True)
        scene = context.scene
        state = _save_render_state(scene)
        try:
            count, point_count = render_dataset(scene, settings, patch_only=True)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            _restore_render_state(scene, state)
        settings.patch_status = f"补齐渲染完成: {count} 张，稀疏点 {point_count}"
        self.report({"INFO"}, settings.patch_status)
        return {"FINISHED"}


class GSCOLMAP_OT_render_quality(Operator):
    """One-click Cycles quality preset with persistent data disabled by default."""
    bl_idname = "gs_colmap.render_quality"
    bl_label = "Render Quality Preset"
    bl_options = {"REGISTER", "UNDO"}
    mode: EnumProperty(items=(("DRAFT", "Draft", ""), ("STANDARD", "Standard", ""),
                              ("ULTRA", "Photoreal", "")), default="STANDARD")

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        samples = apply_render_quality(context.scene, settings, self.mode)
        self.report({"INFO"}, f"{tr(settings, 'quality')}: {self.mode} ({samples} spp, OIDN, clamp)")
        return {"FINISHED"}


class GSCOLMAP_OT_preview_view(Operator):
    """Render ONE preview from the CURRENT viewport angle. Uses the GS engine / samples /
    resolution but YOUR OWN Colour Management (view transform, exposure, gamma) -- the preview is
    exactly what you see, and nothing in the scene is changed. Opens it in a new window; not saved."""
    bl_idname = "gs_colmap.preview_view"
    bl_label = "Preview (current view)"

    def execute(self, context):
        import tempfile
        scene = context.scene
        settings = scene.gs_colmap_settings
        rv3d = getattr(context, "region_data", None)
        space = getattr(context, "space_data", None)
        if rv3d is None or space is None or getattr(space, "type", "") != "VIEW_3D":
            rv3d = space = None
            for area in context.screen.areas:
                if area.type == "VIEW_3D":
                    space = area.spaces.active
                    rv3d = space.region_3d
                    break
        if rv3d is None or space is None:
            self.report({"WARNING"}, "Hover a 3D viewport, then click Preview.")
            return {"CANCELLED"}

        r = scene.render
        vs = scene.view_settings
        cyc = getattr(scene, "cycles", None)
        saved = dict(cam=scene.camera, fp=r.filepath, eng=r.engine,
                     rx=r.resolution_x, ry=r.resolution_y, rp=r.resolution_percentage,
                     ff=r.image_settings.file_format, cmode=r.image_settings.color_mode,
                     cdep=r.image_settings.color_depth, film=r.film_transparent,
                     vt=vs.view_transform, look=vs.look, exp=vs.exposure, gam=vs.gamma,
                     samples=getattr(cyc, "samples", None) if cyc else None)
        cam_data = bpy.data.cameras.new("GS_PreviewCam")
        cam = bpy.data.objects.new("GS_PreviewCam", cam_data)
        scene.collection.objects.link(cam)
        tmp = os.path.join(tempfile.gettempdir(), "gs_view_preview.png")
        err = None
        try:
            cam.matrix_world = rv3d.view_matrix.inverted()
            if getattr(rv3d, "is_perspective", True):
                cam_data.lens = space.lens
            else:
                cam_data.type = "ORTHO"
                try:
                    cam_data.ortho_scale = rv3d.view_distance * 2.0
                except Exception:
                    pass
            scene.camera = cam
            configure_render(scene, settings, apply_color=False)   # keep the user's colour management
            r.film_transparent = False
            r.image_settings.color_mode = "RGB"
            bpy.ops.render.render(write_still=False)   # blocking -> Render Result, no file
            rr = bpy.data.images.get("Render Result")
            r.image_settings.file_format = "PNG"; r.image_settings.color_mode = "RGB"; r.image_settings.color_depth = "8"
            if rr is not None:
                rr.save_render(tmp, scene=scene)       # bake the GS look into the preview image
        except Exception as exc:
            err = str(exc)
        finally:
            scene.camera = saved["cam"]; r.filepath = saved["fp"]; r.engine = saved["eng"]
            r.resolution_x = saved["rx"]; r.resolution_y = saved["ry"]; r.resolution_percentage = saved["rp"]
            r.image_settings.file_format = saved["ff"]; r.image_settings.color_mode = saved["cmode"]
            r.image_settings.color_depth = saved["cdep"]; r.film_transparent = saved["film"]
            vs.view_transform = saved["vt"]; vs.look = saved["look"]; vs.exposure = saved["exp"]; vs.gamma = saved["gam"]
            if cyc and saved["samples"] is not None:
                try: cyc.samples = saved["samples"]
                except Exception: pass
            try: bpy.data.objects.remove(cam, do_unlink=True)
            except Exception: pass
            try: bpy.data.cameras.remove(cam_data)
            except Exception: pass

        if err:
            self.report({"ERROR"}, "Preview failed: " + err); return {"CANCELLED"}
        if not os.path.exists(tmp):
            self.report({"ERROR"}, "Preview produced no image."); return {"CANCELLED"}
        old = bpy.data.images.get("GS Preview")
        if old is not None:
            try: bpy.data.images.remove(old)
            except Exception: pass
        img = bpy.data.images.load(tmp, check_existing=False)
        img.name = "GS Preview"
        try:
            bpy.ops.wm.window_new()
            win = context.window_manager.windows[-1]
            area = max(win.screen.areas, key=lambda a: a.width * a.height)
            area.type = "IMAGE_EDITOR"
            area.spaces.active.image = img
        except Exception:
            self.report({"INFO"}, "GS Preview rendered -> open the 'GS Preview' image in an Image Editor.")
        return {"FINISHED"}


class GSCOLMAP_OT_render_dataset(Operator):
    bl_idname = "gs_colmap.render_dataset"
    bl_label = "Render Dataset"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        if not settings.output_dir:
            self.report({"ERROR"}, tr(settings, "msg_no_output"))
            return {"CANCELLED"}
        if settings.background_render:
            return bpy.ops.gs_colmap.render_dataset_background("INVOKE_DEFAULT")
        scene = context.scene
        st = _save_render_state(scene)        # never persistently change the user's Colour Management
        try:
            count, point_count = render_dataset(scene, settings)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            _restore_render_state(scene, st)
        self.report({"INFO"}, f"{tr(settings, 'msg_done')}: {count} views, {point_count} points")
        return {"FINISHED"}


class GSCOLMAP_OT_render_selected_outputs(Operator):
    bl_idname = "gs_colmap.render_selected_outputs"
    bl_label = "Render Selected Outputs"
    bl_description = "Render the selected RGB/data products; RGB is optional"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        if not settings.output_dir:
            self.report({"ERROR"}, tr(settings, "msg_no_output"))
            return {"CANCELLED"}
        if not _resolved_output_plan(settings).has_outputs:
            self.report({"ERROR"}, "Select at least one Render Output.")
            return {"CANCELLED"}
        if settings.background_render:
            return bpy.ops.gs_colmap.render_dataset_background("INVOKE_DEFAULT")
        state = _save_render_state(context.scene)
        try:
            count, point_count = render_dataset(context.scene, settings)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            _restore_render_state(context.scene, state)
        self.report({"INFO"}, f"Selected outputs rendered: {count} views")
        return {"FINISHED"}


class GSCOLMAP_OT_reset_render(Operator):
    bl_idname = "gs_colmap.reset_render"
    bl_label = "Restart Render"
    bl_description = ("Forget resume progress so the NEXT render starts from frame 1. "
                      "Use this (not the progress bar) when you want to re-render from scratch.")
    bl_options = {"REGISTER"}

    delete_files: bpy.props.BoolProperty(
        name="Also delete rendered frames",
        description="Also delete images/depth/normal/id/material_id/objects/masks (cannot be undone)",
        default=False,
    )

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        clear_render_state(settings)
        try:
            settings.render_progress = 0.0
        except Exception:
            pass
        if self.delete_files and settings.output_dir:
            import shutil
            for d in ("images", "depth", "normal", "objects", "_gs_tmp", "depth_view", "id", "material_id", "masks"):
                try:
                    shutil.rmtree(Path(settings.output_dir) / d, ignore_errors=True)
                except Exception:
                    pass
        settings.render_status = "Render reset — next render starts from frame 1"
        self.report({"INFO"}, settings.render_status)
        return {"FINISHED"}


class GSCOLMAP_OT_render_dataset_background(Operator):
    bl_idname = "gs_colmap.render_dataset_background"
    bl_label = "Render Dataset in Background"
    bl_options = {"REGISTER"}

    patch_only: BoolProperty(name="Only Patch Cameras", default=False)
    _timer = None

    def invoke(self, context, event):
        global _BACKGROUND_RENDER
        settings = context.scene.gs_colmap_settings
        if not settings.output_dir:
            self.report({"ERROR"}, tr(settings, "msg_no_output"))
            return {"CANCELLED"}
        if _BACKGROUND_RENDER and _BACKGROUND_RENDER["process"].poll() is None:
            self.report({"WARNING"}, "A background render is already running.")
            return {"CANCELLED"}
        try:
            _BACKGROUND_RENDER = launch_background_render(context, settings, patch_only=self.patch_only)
        except Exception as exc:
            settings.render_status = f"Failed to start background render: {exc}"
            self.report({"ERROR"}, settings.render_status)
            return {"CANCELLED"}
        settings.render_progress = 0.0
        settings.render_log_path = _BACKGROUND_RENDER["log_path"]
        settings.render_status = "Background render started"
        context.window_manager.modal_handler_add(self)
        self._timer = context.window_manager.event_timer_add(0.5, window=context.window)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        global _BACKGROUND_RENDER
        settings = context.scene.gs_colmap_settings
        if event.type == "ESC":
            self.cancel(context)
            return {"CANCELLED"}
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        if not _BACKGROUND_RENDER:
            self.cancel(context)
            return {"CANCELLED"}

        progress = self._read_progress(_BACKGROUND_RENDER["progress_path"])
        if progress:
            settings.render_progress = float(progress.get("progress", 0.0))
            total = int(progress.get("total", 0) or 0)
            current = int(progress.get("current", 0) or 0)
            message = progress.get("message", "")
            state = progress.get("state", "")
            suffix = f" ({current}/{total})" if total else ""
            settings.render_status = f"{state}: {message}{suffix}"

        process = _BACKGROUND_RENDER["process"]
        return_code = process.poll()
        if return_code is None:
            return {"RUNNING_MODAL"}

        self._close_job()
        if progress and progress.get("state") == "done" and return_code == 0:
            settings.render_progress = 1.0
            settings.render_status = "Done: dataset generated"
            self.report({"INFO"}, tr(settings, "msg_done"))
            return {"FINISHED"}
        error = progress.get("error", "") if progress else ""
        settings.render_status = f"Background render failed. {error} Log: {settings.render_log_path}"
        self.report({"ERROR"}, settings.render_status)
        return {"CANCELLED"}

    def cancel(self, context):
        global _BACKGROUND_RENDER
        settings = context.scene.gs_colmap_settings
        if _BACKGROUND_RENDER and _BACKGROUND_RENDER["process"].poll() is None:
            _terminate_process_tree(_BACKGROUND_RENDER["process"])
            settings.render_status = "Background render cancelled"
        self._close_job()

    def _close_job(self):
        global _BACKGROUND_RENDER
        if self._timer:
            try:
                bpy.context.window_manager.event_timer_remove(self._timer)
            except Exception:
                pass
            self._timer = None
        if _BACKGROUND_RENDER:
            try:
                _BACKGROUND_RENDER["log_handle"].close()
            except Exception:
                pass
            _BACKGROUND_RENDER = None

    @staticmethod
    def _read_progress(progress_path):
        path = Path(progress_path)
        if not path.exists():
            return None
        try:
            with open(path, "r", encoding="utf-8") as handle:
                return json.load(handle)
        except Exception:
            return None


class GSCOLMAP_OT_cancel_background_render(Operator):
    bl_idname = "gs_colmap.cancel_background_render"
    bl_label = "Cancel Background Render"
    bl_options = {"REGISTER"}

    def execute(self, context):
        global _BACKGROUND_RENDER
        settings = context.scene.gs_colmap_settings
        if _BACKGROUND_RENDER and _BACKGROUND_RENDER["process"].poll() is None:
            _terminate_process_tree(_BACKGROUND_RENDER["process"])
            try:
                _BACKGROUND_RENDER["log_handle"].close()
            except Exception:
                pass
            _BACKGROUND_RENDER = None
            settings.render_status = "Background render cancelled"
            self.report({"INFO"}, settings.render_status)
            return {"FINISHED"}
        settings.render_status = "No background render is running"
        self.report({"INFO"}, settings.render_status)
        return {"CANCELLED"}


class GSCOLMAP_OT_export_colmap(Operator):
    bl_idname = "gs_colmap.export_colmap"
    bl_label = "Export COLMAP Only"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = context.scene.gs_colmap_settings
        if not settings.output_dir:
            self.report({"ERROR"}, tr(settings, "msg_no_output"))
            return {"CANCELLED"}
        try:
            cameras = dataset_export_cameras(context.scene, settings)
            if not cameras:
                cameras = create_rig(context.scene, settings)
            if not cameras:
                self.report({"ERROR"}, "No cameras found. Create the camera rig first (Create Cameras).")
                return {"CANCELLED"}
            image_names = [
                image_name for _camera, _stem, image_name
                in dataset_camera_items(context.scene, settings, cameras)
            ]
            points = sample_sparse_points(context.scene, cameras, settings)
            ensure_dir(Path(settings.output_dir) / "images")
            write_colmap(context.scene, settings, cameras, image_names, points)
            write_report(context.scene, settings, cameras, image_names, points)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            self.report({"ERROR"}, f"Export COLMAP failed: {exc}")
            return {"CANCELLED"}
        self.report({"INFO"}, f"{tr(settings, 'msg_export_done')}: {len(cameras)} views, {len(points)} points")
        return {"FINISHED"}


class GSCOLMAP_OT_export_transforms(Operator):
    bl_idname = "gs_colmap.export_transforms"
    bl_label = "Export transforms.json Only"
    bl_options = {"REGISTER"}

    def execute(self, context):
        scene = context.scene
        settings = scene.gs_colmap_settings
        if not settings.output_dir:
            self.report({"ERROR"}, tr(settings, "msg_no_output"))
            return {"CANCELLED"}
        try:
            cameras = dataset_export_cameras(scene, settings)
            if not cameras:
                raise RuntimeError("No cameras or pose sequence are available.")
            image_names = [
                Path(camera.sample.rgb_path).name if getattr(camera, "sample", None)
                else image_name
                for camera, _stem, image_name in dataset_camera_items(scene, settings, cameras)
            ]
            write_transforms_only(scene, settings, cameras, image_names)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Exported transforms.json with {len(cameras)} frames")
        return {"FINISHED"}


class GSCOLMAP_PT_coverage_patch(Panel):
    bl_label = "Coverage Patch / 样本补齐"
    bl_idname = "GSCOLMAP_PT_coverage_patch"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "GS"
    bl_options = {"DEFAULT_CLOSED"}
    bl_order = 1

    def draw(self, context):
        layout = self.layout
        settings = context.scene.gs_colmap_settings
        layout.prop(settings, "patch_mode", text="补齐模式")
        if settings.patch_mode == "SELECTED_OBJECTS":
            layout.label(text="目标：当前选中的 Mesh 对象", icon="RESTRICT_SELECT_OFF")
        elif settings.patch_mode == "BOUNDS":
            layout.prop(settings, "patch_bounds_object", text="包围盒对象")
        else:
            layout.label(text="自动分析主要欠覆盖对象", icon="VIEWZOOM")

        thresholds = layout.box()
        thresholds.label(text="停止条件", icon="CHECKMARK")
        row = thresholds.row(align=True)
        row.prop(settings, "patch_min_observation_count", text="最低次数")
        row.prop(settings, "patch_recommended_observation_count", text="推荐次数")
        thresholds.prop(settings, "patch_target_coverage_ratio", text="目标覆盖率")
        thresholds.prop(settings, "patch_max_camera_count", text="最大补拍数量")

        candidates = layout.box()
        candidates.label(text="局部候选约束", icon="CAMERA_DATA")
        candidates.prop(settings, "patch_candidate_radius", text="候选半径 (m)")
        candidates.prop(settings, "patch_camera_safety_distance", text="安全距离 (m)")
        candidates.prop(settings, "patch_min_overlap_ratio", text="最低重叠率")
        candidates.prop(settings, "patch_allow_polar", text="允许极向关键帧")
        candidates.prop(settings, "patch_limit_to_path", text="限制在已有路径附近")
        candidates.prop(settings, "patch_prefer_existing_connect", text="优先连接现有相机图")
        candidates.prop(settings, "patch_priority", text="补齐策略")

        layout.operator("gs_colmap.generate_patch_preview", text="生成预览", icon="HIDE_OFF")
        row = layout.row(align=True)
        row.operator("gs_colmap.apply_patch_preview", text="应用补齐", icon="CHECKMARK")
        row.operator("gs_colmap.clear_patch_preview", text="清除预览", icon="TRASH")
        layout.operator("gs_colmap.render_patch_cameras", text="仅渲染补齐相机", icon="RENDER_STILL")

        preview_count = len(coverage_patch.preview_cameras(context.scene))
        final_count = len(coverage_patch.final_cameras(context.scene))
        layout.label(text=f"预览 {preview_count} 台 / 正式补齐 {final_count} 台", icon="INFO")
        progress = layout.column()
        progress.enabled = False
        progress.prop(settings, "patch_progress", text="进度", slider=True)
        if settings.patch_status:
            layout.label(text=settings.patch_status[:120], icon="TIME")


class GSCOLMAP_PT_panel(Panel):
    bl_label = "GS Dataset"
    bl_idname = "GSCOLMAP_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "GS"

    def draw(self, context):
        draw_gs_colmap_panel(self, context)


class GSCOLMAP_PT_render_properties(Panel):
    bl_label = "GS Dataset"
    bl_idname = "GSCOLMAP_PT_render_properties"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "render"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        draw_gs_colmap_panel(self, context)


class GSCOLMAP_PT_node_panel(Panel):
    bl_label = "GS Dataset"
    bl_idname = "GSCOLMAP_PT_node_panel"
    bl_space_type = "NODE_EDITOR"
    bl_region_type = "UI"
    bl_category = "GS"

    def draw(self, context):
        draw_gs_colmap_panel(self, context)


class GSCOLMAP_MT_render_menu(bpy.types.Menu):
    bl_label = "GS Dataset"
    bl_idname = "GSCOLMAP_MT_render_menu"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.gs_colmap_settings
        layout.operator("gs_colmap.create_cameras", text=tr(settings, "create_cameras"), icon="CAMERA_DATA")
        layout.separator()
        layout.operator("gs_colmap.render_dataset", text=tr(settings, "render"), icon="RENDER_STILL")
        layout.operator("gs_colmap.export_colmap", text=tr(settings, "export_only"), icon="EXPORT")


def draw_gs_colmap_render_menu(self, context):
    self.layout.separator()
    self.layout.menu("GSCOLMAP_MT_render_menu", icon="OUTLINER_OB_CAMERA")


def draw_gs_colmap_panel(self, context):
    layout = self.layout
    layout.label(text=f"GS UI v{_addon_version_str()}", icon="CHECKMARK")
    scene = getattr(context, "scene", None)
    if scene is None:
        layout.label(text="GS Dataset: no active scene", icon="ERROR")
        return
    if not hasattr(scene, "gs_colmap_settings"):
        layout.label(text="GS Dataset loaded, settings missing", icon="ERROR")
        layout.label(text="Restart Blender or reinstall the extension.")
        return
    settings = scene.gs_colmap_settings
    incremental_shell = (
        settings.rig_mode == "PATH"
        and settings.path_capture_mode == "LEGACY_PANORAMA_CUBE"
        and settings.path_station_array_mode == "SPHERICAL_SHELL_12"
    )
    try:
        self.bl_label = tr(settings, "panel")
    except Exception:
        pass

    try:
        layout.prop(settings, "language", text=tr(settings, "language"))
        layout.prop(settings, "output_dir", text=tr(settings, "output_dir"))
        mesh_guided.draw_panel(layout, context)

        box = layout.box()
        box.label(text=tr(settings, "rig"), icon="CAMERA_DATA")
        box.prop(settings, "rig_mode", text="")
        live_row = box.row()
        live_row.enabled = not incremental_shell and not settings.camera_build_active
        live_row.prop(settings, "live_update_cameras", text=tr(settings, "live_update"))
        if settings.rig_mode == "PATH":
            box.prop(settings, "path_object", text=tr(settings, "path"))
            box.prop(settings, "path_collection", text=tr(settings, "path_collection"))
            box.prop(settings, "path_capture_mode", text="采集模式")
            if settings.path_capture_mode == "LEGACY_PANORAMA_CUBE":
                box.prop(settings, "path_station_array_mode", text="采集阵列")
                if settings.path_station_array_mode == "LEGACY_SIX":
                    box.prop(settings, "path_look_mode", text=tr(settings, "path_look"))
                    if settings.path_look_mode == "TARGET":
                        box.prop(settings, "target_object", text=tr(settings, "target"))
                else:
                    shell = box.column(align=True)
                    shell.prop(settings, "shell_radius", text="球壳半径 (m)")
                    shell.prop(settings, "shell_radius_mode", text="半径模式")
                    if settings.shell_radius_mode == "CLEARANCE_ADAPTIVE":
                        shell.prop(settings, "shell_min_radius", text="最小球壳半径 (m)")
                    shell.prop(settings, "shell_failure_policy", text="失败处理")
                    shell.prop(settings, "show_shell_debug_mesh", text="显示球壳调试代理")
            box.prop(settings, "path_count_mode", text=tr(settings, "path_count_mode"))
            if settings.path_count_mode == "DENSITY":
                box.prop(settings, "path_camera_density", text=tr(settings, "path_density"), slider=True)
                box.prop(settings, "max_path_cameras", text="Max")
            else:
                box.prop(settings, "camera_count", text=tr(settings, "count"), slider=True)
            box.prop(settings, "path_samples_per_segment", text=tr(settings, "path_samples"), slider=True)
            if settings.path_capture_mode == "LEGACY_PANORAMA_CUBE":
                try:
                    path_items = path_components(settings)
                    station_count = sum(path_component_counts(settings, path_items)) if path_items else max(1, settings.camera_count)
                except Exception:
                    station_count = max(1, settings.camera_count)
                cameras_per_station = 12 if settings.path_station_array_mode == "SPHERICAL_SHELL_12" else 6
                budget = box.column(align=True)
                budget.label(text=f"路径站点数: {station_count}")
                budget.label(text=f"每站点相机数: {cameras_per_station}")
                budget.label(
                    text=f"预计总相机数: {station_count * cameras_per_station}",
                    icon="CAMERA_DATA",
                )
                if settings.path_station_array_mode == "SPHERICAL_SHELL_12":
                    progress = box.column(align=True)
                    progress.enabled = False
                    progress.prop(settings, "camera_build_progress", text="生成进度", slider=True)
                    box.label(
                        text=settings.camera_build_status,
                        icon="TIME" if settings.camera_build_active else "INFO",
                    )
                    if settings.camera_build_active and settings.camera_build_cancelable:
                        box.operator("gs_colmap.cancel_camera_build", text="取消生成", icon="CANCEL")
            if settings.path_capture_mode == "SCIENTIFIC_THREE_LAYER":
                science = box.box()
                science.prop(settings, "scientific_realization_mode", text="Realization Backend")
                if settings.scientific_realization_mode == "SCIENTIFIC_POSE_SEQUENCE":
                    science.prop(settings, "sequence_source_scene_frame", text="Source Scene Frame")
                    science.prop(settings, "sequence_create_preview_keyframes", text="Create Preview Keyframes")
                    try:
                        sequence = active_pose_sequence(context.scene, settings)
                    except Exception:
                        sequence = None
                    if sequence is not None:
                        enabled_count = sum(sample.render_enabled for sample in sequence.frames)
                        science.label(text=f"Blender camera objects: 1", icon="CAMERA_DATA")
                        science.label(text=f"Planned poses: {len(sequence.frames)}")
                        science.label(text=f"Training frames: {enabled_count} / Segments: {len(sequence.segments)}")
                    preview_row = science.row(align=True)
                    preview_row.operator("gs_colmap.create_sequence_preview", text="Create Preview", icon="PLAY")
                    preview_row.operator("gs_colmap.clear_sequence_preview", text="Clear", icon="X")
                    debug = science.box()
                    debug.prop(settings, "sequence_debug_mode", text="Pose Display")
                    if settings.sequence_debug_mode in {"CURRENT", "NEIGHBORHOOD"}:
                        debug.prop(settings, "sequence_debug_frame", text="Logical Frame")
                    if settings.sequence_debug_mode == "NEIGHBORHOOD":
                        debug.prop(settings, "sequence_debug_neighbor_count", text="Before / After")
                    if settings.sequence_debug_mode == "SAMPLED":
                        debug.prop(settings, "sequence_debug_stride", text="Every N Poses")
                    debug_row = debug.row(align=True)
                    debug_row.operator("gs_colmap.refresh_sequence_debug", text="Refresh", icon="FILE_REFRESH")
                    debug_row.operator("gs_colmap.clear_sequence_debug", text="Clear", icon="X")
                    render_row = science.row(align=True)
                    render_row.operator("gs_colmap.render_pose_sequence", text="Render Pending", icon="RENDER_STILL").mode = "PENDING"
                    render_row.operator("gs_colmap.render_pose_sequence", text="Render Full", icon="FILE_REFRESH").mode = "FULL"
                    science.operator("gs_colmap.export_transforms", text="Export transforms.json Only", icon="EXPORT")
                science.prop(settings, "scientific_origin_mode", text="光心来源")
                if settings.scientific_origin_mode in {"FREE_SPACE", "SMALL_SPACE", "HYBRID"}:
                    science.prop(settings, "scientific_auto_small_space", text="自动识别小空间")
                    science.prop(settings, "free_space_grid_resolution", text="自由空间网格分辨率")
                    science.prop(settings, "free_space_candidate_spacing", text="自由空间候选间距")
                    science.prop(settings, "free_space_max_origin_count", text="最大候选光心数")
                    science.prop(settings, "free_space_doorway_priority", text="门洞优先")
                    science.prop(settings, "near_field_protection", text="近场采集保护")
                    if settings.near_field_protection:
                        distance_row = science.row(align=True)
                        distance_row.prop(settings, "near_field_recommended_distance_min", text="推荐距离下限")
                        distance_row.prop(settings, "near_field_recommended_distance_max", text="推荐距离上限")
                        science.prop(settings, "near_field_dominant_surface_ratio", text="单一近表面占比上限")
                        science.prop(settings, "near_field_maximum_camera_ratio", text="近景机位占比上限")
                science.prop(settings, "scientific_budget_mode", text="图片预算模式")
                if settings.scientific_budget_mode == "USER_FIXED_BUDGET":
                    science.prop(settings, "scientific_fixed_budget", text="固定图片预算")
                science.prop(settings, "scientific_global_reachable_coverage", text="Global reachable coverage")
                science.prop(settings, "scientific_coverage_driven", text="Coverage-driven origins")
                science.prop(settings, "coverage_driven_max_cameras", text="Coverage origin cap")
                science.prop(settings, "scientific_post_clipping_recast", text="Recast after clipping")
                science.prop(settings, "scientific_validate_training_consistency", text="Validate training consistency")
                science.operator("gs_colmap.training_recommendations", text="Apply training recommendations", icon="CHECKMARK")
                science.label(text="科学三层覆盖", icon="CAMERA_DATA")
                science.prop(settings, "scientific_layer_count", text="高度层数")
                science.prop(settings, "scientific_target_overlap", text="目标重叠率")
                science.prop(settings, "scientific_minimum_step", text="最小路径间距")
                science.prop(settings, "scientific_maximum_step", text="最大路径间距")
                science.prop(settings, "scientific_camera_clearance", text="相机安全距离")
                science.prop(settings, "scientific_view_budget_multiplier", text="图片预算倍率")
                science.prop(settings, "scientific_ray_quality", text="射线质量")
                science.prop(settings, "scientific_auto_coverage", text="自动覆盖优化")
                science.prop(settings, "scientific_auto_floor_ceiling", text="自动补地面和天花板")
                science.prop(settings, "scientific_show_debug", text="显示调试结果")
                science.prop(settings, "scientific_planning_progress", text="规划进度", slider=True)
                science.label(text=settings.scientific_planning_status, icon="INFO")
                advanced = science.row(align=True)
                advanced.prop(settings, "scientific_advanced_expand", text="", icon="TRIA_DOWN" if settings.scientific_advanced_expand else "TRIA_RIGHT", emboss=False)
                advanced.label(text="高级参数")
                if settings.scientific_advanced_expand:
                    science.prop(settings, "scientific_minimum_observations", text="表面最低观察次数")
                    science.prop(settings, "scientific_preferred_observations", text="表面推荐观察次数")
                    science.prop(settings, "scientific_maximum_heading_change", text="最大朝向变化（度）")
                    science.prop(settings, "scientific_minimum_overlap", text="最低邻居重叠率")
                    science.prop(settings, "scientific_maximum_incidence_angle", text="最大入射角（度）")
                    if settings.scientific_origin_mode in {"FREE_SPACE", "SMALL_SPACE", "HYBRID"} and settings.near_field_protection:
                        science.prop(settings, "near_field_unsuitable_distance", text="不适合作为光心的距离")
                        science.prop(settings, "near_field_step_distance_ratio", text="近场步长/目标距离")
                        science.prop(settings, "near_field_minimum_origin_spacing", text="近场最小去重距离")
                        science.prop(settings, "near_field_doorway_target_overlap", text="玄关目标重叠率")
                        science.prop(settings, "near_field_minimum_mid_overlap", text="近景与中远景最低重叠")
                        science.prop(settings, "near_field_required_mid_neighbors", text="所需中远景邻居数")
                        science.prop(settings, "near_field_minimum_baseline", text="近景与中远景真实基线")
                        science.prop(settings, "near_field_minimum_environment_ratio", text="最低共同环境比例")
                    science.prop(settings, "scientific_keep_candidates", text="保留候选相机")
                    science.prop(settings, "scientific_show_overlap_graph", text="显示相机重叠图")
            route_box = box.box()
            route_head = route_box.row(align=True)
            route_head.prop(settings, "floorplan_expand", text="",
                            icon="TRIA_DOWN" if settings.floorplan_expand else "TRIA_RIGHT", emboss=False)
            route_head.label(text="室内多层排线", icon="CURVE_PATH")
            if settings.floorplan_expand:
                route_box.prop(settings, "floorplan_method", text="排线方法")
                route_box.prop(settings, "floorplan_space_mode", text="空间范围")
                if settings.floorplan_method == "CONTOUR":
                    route_box.prop(settings, "floorplan_spacing", text="轮廓间距（m）")
                    route_box.prop(settings, "contour_probe_spacing")
                    route_box.prop(settings, "contour_clearance")
                    route_box.prop(settings, "contour_floor_collection")
                    if not settings.contour_floor_collection:
                        route_box.prop(settings, "contour_min_floor_area")
                    if settings.floorplan_space_mode != "ALL":
                        route_box.prop(settings, "floorplan_seed_mode", text="起点来源")
                        if settings.floorplan_seed_mode == "OBJECT":
                            route_box.prop(settings, "floorplan_seed_object", text="起点物体")
                    route_box.prop(settings, "contour_max_bridge")
                    route_box.prop(settings, "contour_max_step")
                    route_box.prop(settings, "contour_smoothing")
                    route_box.prop(settings, "contour_detail_enabled")
                    if settings.contour_detail_enabled:
                        route_box.prop(settings, "contour_detail_precision")
                        route_box.prop(settings, "contour_detail_grid")
                        route_box.prop(settings, "contour_detail_clearance")
                        route_box.prop(settings, "contour_detail_budget")
                        route_box.prop(settings, "contour_detail_distance")
                route_box.prop(settings, "floorplan_layer_mode", text="排线层数")
                if settings.floorplan_method == "GRID":
                    route_box.prop(settings, "floorplan_curve_density", text="曲线密度（点/米）")
                if settings.floorplan_layer_mode != "ONE":
                    route_box.prop(settings, "floorplan_low_height", text="低层高度（离地，0.10-0.50m）")
                if settings.floorplan_layer_mode in {"ONE", "THREE", "FOUR"}:
                    route_box.prop(settings, "floorplan_mid_height", text="中层高度（离地，m）")
                if settings.floorplan_layer_mode == "FOUR":
                    route_box.prop(settings, "floorplan_high_height", text="高层高度（离地，m）")
                if settings.floorplan_layer_mode != "ONE":
                    route_box.prop(settings, "floorplan_top_height", text="顶层高度（离地，m）")
                    if settings.floorplan_method=='CONTOUR':
                        route_box.prop(settings,'contour_adapt_top')
                        if settings.contour_adapt_top:route_box.prop(settings,'floorplan_ceiling_offset',text='顶层距顶棚（m）')
                if settings.contour_running:
                    route_box.label(text=settings.contour_status, icon="TIME")
                    route_box.operator("gs_colmap.cancel_contour", icon="CANCEL")
                else:
                    route_box.operator("gs_colmap.auto_floorplan_path", text="生成轮廓预览" if settings.floorplan_method == "CONTOUR" else "生成井字排线", icon="MOD_CURVE")
                if settings.floorplan_method == "CONTOUR":
                    route_box.label(text="新建预览集合；保留已有曲线", icon="INFO")
                    try:
                        report = json.loads(context.scene.get("gs_contour_last_report", "{}"))
                        if report:
                            route_box.label(text=f"最近生成：{report.get('main_route_count', report['route_count'])} 条主线")
                            detail = report.get("detail_coverage")
                            if detail and detail.get("target_count"):
                                route_box.label(text=f"细部短线：{detail['detail_lines']} 条")
                                route_box.label(text=f"新增充分观察：{detail['final_observed_cells']-detail['main_observed_cells']} 个表面单元")
                                metric_label='分层采样表面' if detail.get('layers') else '采样表面'
                                route_box.label(text=f"{metric_label}：{detail['main_surface_ratio']:.1%} → {detail['final_surface_ratio']:.1%}")
                                for layer in detail.get('layers', []):
                                    label={'Low':'低层','Middle':'中层','High':'顶层','Middle_2':'高中层'}.get(layer['layer'],layer['layer'])
                                    route_box.label(text=f"{label}：{layer['main_ratio']:.0%} → {layer['final_ratio']:.0%} · {layer['detail_lines']} 条补线")
                                route_box.label(text=f"仍待补足：{detail['remaining_surface_cells']} 个采样单元")
                                if detail.get('new_gap_cells'):route_box.label(text=f"补入空隙：{detail['new_gap_cells']} 个通行节点")
                                select_row = route_box.row(align=True)
                                select_row.operator("gs_colmap.select_contour_role", text="选中主线").role = "MAIN"
                                select_row.operator("gs_colmap.select_contour_role", text="选中细部短线").role = "DETAIL"
                                view_row=route_box.row(align=True)
                                for key,label in [('ALL','全部'),('Low','低层'),('Middle','中层'),('High','顶层')]:
                                    view_row.operator('gs_colmap.view_contour_layer',text=label).layer=key
                                active = context.active_object
                                if active and active.get("gs_route_role") == "DETAIL":
                                    route_box.label(text=f"所选短线新增观察：{active.get('gs_detail_gain', 0)} 个表面单元")
                                    if active.get('gs_gap_gain'):route_box.label(text=f"所选短线补入空隙：{active['gs_gap_gain']} 个节点")
                            route_box.label(text="实际相机视场与重叠仍需验证")
                    except (ValueError, KeyError):
                        pass
        else:
            box.prop(settings, "target_object", text=tr(settings, "target"))
            box.prop(settings, "camera_count", text=tr(settings, "count"), slider=True)
        if settings.rig_mode in {"CYLINDER", "HALF_CYLINDER"}:
            box.prop(settings, "rings", text=tr(settings, "rings"))
            box.prop(settings, "height", text=tr(settings, "height"))
        if settings.rig_mode == "VOLUME":
            box.prop(settings, "volume_size", text=tr(settings, "volume"))
            row = box.row(align=True)
            row.prop(settings, "volume_x", text="X")
            row.prop(settings, "volume_y", text="Y")
            row.prop(settings, "volume_z", text="Z")
            box.prop(settings, "volume_jitter", text=tr(settings, "jitter"))
            box.prop(settings, "exclude_collection", text=tr(settings, "exclude"))
        box.prop(settings, "radius", text=tr(settings, "radius"))
        box.prop(settings, "focal_length", text=tr(settings, "fov"))
        if settings.rig_mode == "PATH" and settings.path_capture_mode == "SCIENTIFIC_THREE_LAYER":
            box.label(text="固定透视内参；科学模式不会生成同光心六面相机", icon="INFO")
        elif (
            settings.rig_mode == "PATH"
            and settings.path_capture_mode == "LEGACY_PANORAMA_CUBE"
            and settings.path_station_array_mode == "SPHERICAL_SHELL_12"
        ):
            box.label(text="固定透视内参；每个路径站点生成12台径向球壳相机", icon="INFO")
        else:
            box.prop(settings, "camera_model", text=tr(settings, "camera_model"))
            if settings.camera_model == "PANORAMA_CUBE":
                box.label(text="6× perspective faces / position · use square resolution / 每点6张透视·建议方形分辨率", icon="INFO")
        if settings.rig_mode == "PATH" and settings.path_capture_mode == "SCIENTIFIC_THREE_LAYER":
            create_label = (
                "规划科学位姿序列" if settings.scientific_realization_mode == "SCIENTIFIC_POSE_SEQUENCE"
                else "创建科学相机阵列"
            )
            box.operator("gs_colmap.create_scientific_cameras", text=create_label, icon="CAMERA_DATA")
            if settings.scientific_planning_active:
                box.operator("gs_colmap.cancel_scientific_planning", text="取消科学规划", icon="CANCEL")
        else:
            create_row = box.row()
            create_row.enabled = not settings.camera_build_active
            create_text = "生成球壳相机" if incremental_shell else tr(settings, "create_cameras")
            create_row.operator("gs_colmap.create_cameras", text=create_text, icon="CAMERA_DATA")

        style = box.box()
        style.prop(settings, "camera_mesh_style", text="绿色金字塔相机样式")
        if settings.camera_mesh_style:
            style.prop(settings, "camera_mesh_size", text="金字塔尺寸 (m)")
        row = style.row(align=True)
        row.operator("gs_colmap.camera_mesh_style", text="转换场景相机", icon="MESH_CONE").action = "APPLY_SCENE"
        row.operator("gs_colmap.camera_mesh_style", text="转换选中", icon="RESTRICT_SELECT_OFF").action = "APPLY_SELECTED"
        style.operator("gs_colmap.camera_mesh_style", text="移除 Mesh 外观", icon="TRASH").action = "REMOVE_SCENE"

        box = layout.box()
        box.label(text=tr(settings, "render"), icon="RENDER_STILL")
        row = box.row(align=True)
        row.prop(settings, "resolution_x", text="X")
        row.prop(settings, "resolution_y", text="Y")
        box.prop(settings, "image_format", text=tr(settings, "format"))
        box.prop(settings, "render_engine", text=tr(settings, "engine"))
        row = box.row(align=True)        # one-click Cycles quality (samples/denoise/clamp/bounces)
        row.label(text=tr(settings, "quality"))
        row.operator("gs_colmap.render_quality", text=tr(settings, "q_draft")).mode = "DRAFT"
        row.operator("gs_colmap.render_quality", text=tr(settings, "q_std")).mode = "STANDARD"
        row.operator("gs_colmap.render_quality", text=tr(settings, "q_ultra")).mode = "ULTRA"
        if settings.render_engine == "CYCLES":
            box.prop(settings, "cycles_samples", text=tr(settings, "cycles_samples"))
            box.prop(settings, "cycles_denoise", text=tr(settings, "cycles_denoise"))
            box.prop(settings, "cycles_device", text=tr(settings, "cycles_device"))
            box.prop(settings, "cycles_backend", text="Cycles Backend / 后端")
            if settings.cycles_backend == "HIP":
                box.prop(settings, "hip_rt_mode", text="HIP-RT / 硬件光追")
                if settings.background_render:
                    box.prop(settings, "hip_chunk_size", text="HIP Frames per Process / HIP 每进程帧数")
                box.prop(settings, "hip_memory_safe_mode", text="HIP Memory-safe / HIP 内存安全")
                box.prop(settings, "hip_oom_fallback", text="HIP OOM -> CPU / HIP 显存不足转 CPU")
            box.prop(settings, "cycles_persistent_data", text="Persistent Data / 持久数据")
        row = box.row(align=True)
        row.prop(settings, "color_look", text="Color / 色彩")
        row.prop(settings, "color_exposure", text="Exp / 曝光")
        box.prop(settings, "background_render", text=tr(settings, "background_render"))
        if settings.background_render:
            box.prop(settings, "background_chunk_size", text="Frames per Process / 每进程帧数")
        # resume / restart: 'Resume' continues an interrupted render; 'Restart' clears the
        # checkpoint so the next render starts from frame 1 (use this, not the progress bar).
        rrow = box.row(align=True)
        rrow.prop(settings, "incremental", text="Resume / 断点续渲")
        rrow.operator("gs_colmap.reset_render", text="Restart / 重渲", icon="LOOP_BACK")
        if settings.background_render:
            sub = box.column(align=True)
            sub.enabled = False  # progress bar is a status read-out, NOT a control
            sub.prop(settings, "render_progress", text=tr(settings, "progress"), slider=True)
            if settings.render_status:
                box.label(text=settings.render_status, icon="TIME")
            if _BACKGROUND_RENDER and _BACKGROUND_RENDER["process"].poll() is None:
                box.operator("gs_colmap.cancel_background_render", text=tr(settings, "cancel_render"), icon="CANCEL")
        box.prop(settings, "transparent_background", text="Transparent / 透明")

        passes = box.box()
        passes.label(text="Render Outputs / 渲染输出", icon="RENDERLAYERS")
        beauty = passes.column(align=True)
        beauty.label(text="Beauty")
        beauty.prop(settings, "render_rgb", text="RGB Beauty / RGB 图像")
        geometry = passes.column(align=True)
        geometry.label(text="Geometry")
        geometry.prop(settings, "export_depth", text="Scene Depth / 场景深度")
        geometry.prop(settings, "export_object_depth", text="Object Depth / 单物体深度")
        geometry.prop(settings, "export_normal", text="Scene Normal / 场景法线")
        geometry.prop(settings, "export_object_normal", text="Object Normal / 单物体法线")
        segmentation = passes.column(align=True)
        segmentation.label(text="Segmentation")
        segmentation.prop(settings, "export_object_mask", text="Object Masks / 单物体 Mask")
        segmentation.prop(settings, "export_id", text="Object ID / 物体 ID")
        segmentation.prop(settings, "export_material_id", text="Material ID / 材质 ID")
        if settings.export_depth or settings.export_object_depth:
            passes.prop(settings, "depth_format", text="Depth Format / 深度格式")
        if settings.export_object_depth or settings.export_object_normal or settings.export_object_mask:
            passes.prop(settings, "object_split_mode", text="Object Split Storage / 切分存储")
            passes.prop(settings, "object_group_mode", text="Item / 物品粒度")
            passes.label(text="Object ID is resolved automatically / 自动解析内部 ID 依赖", icon="INFO")
        passes.operator(
            "gs_colmap.render_selected_outputs",
            text="RENDER SELECTED OUTPUTS / 渲染所选输出",
            icon="RENDER_STILL",
        )

        box = layout.box()
        box.label(text="COLMAP", icon="EXPORT")
        box.prop(settings, "point_samples_per_view", text=tr(settings, "point_samples"))
        box.prop(settings, "point_dedup_size", text=tr(settings, "dedup"))
        box.operator("gs_colmap.preview_view", text="Preview (current view) / 视口预览", icon="RESTRICT_RENDER_OFF")
        box.operator("gs_colmap.render_dataset", text=tr(settings, "render"), icon="RENDER_STILL")
        box.operator("gs_colmap.export_colmap", text=tr(settings, "export_only"), icon="EXPORT")

    except Exception as exc:
        layout.separator()
        layout.label(text="GS Dataset UI error", icon="ERROR")
        layout.label(text=str(exc)[:120])


classes = (
    GSCOLMAP_Settings,
    GSCOLMAP_OT_create_cameras,
    GSCOLMAP_OT_cancel_camera_build,
    GSCOLMAP_OT_training_recommendations,
    GSCOLMAP_OT_camera_mesh_style,
    GSCOLMAP_OT_create_scientific_cameras,
    GSCOLMAP_OT_cancel_scientific_planning,
    GSCOLMAP_OT_create_sequence_preview,
    GSCOLMAP_OT_clear_sequence_preview,
    GSCOLMAP_OT_refresh_sequence_debug,
    GSCOLMAP_OT_clear_sequence_debug,
    GSCOLMAP_OT_render_pose_sequence,
    GSCOLMAP_OT_auto_floorplan_path,
    GSCOLMAP_OT_cancel_contour,
    GSCOLMAP_OT_select_contour_role,
    GSCOLMAP_OT_view_contour_layer,
    GSCOLMAP_OT_generate_patch_preview,
    GSCOLMAP_OT_apply_patch_preview,
    GSCOLMAP_OT_clear_patch_preview,
    GSCOLMAP_OT_render_patch_cameras,
    GSCOLMAP_OT_render_quality,
    GSCOLMAP_OT_preview_view,
    GSCOLMAP_OT_render_dataset,
    GSCOLMAP_OT_render_selected_outputs,
    GSCOLMAP_OT_reset_render,
    GSCOLMAP_OT_render_dataset_background,
    GSCOLMAP_OT_cancel_background_render,
    GSCOLMAP_OT_export_colmap,
    GSCOLMAP_OT_export_transforms,
    GSCOLMAP_PT_panel,
    GSCOLMAP_PT_coverage_patch,
    GSCOLMAP_PT_render_properties,
    GSCOLMAP_PT_node_panel,
    GSCOLMAP_MT_render_menu,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.gs_colmap_settings = PointerProperty(type=GSCOLMAP_Settings)
    mesh_guided.register()
    bpy.types.TOPBAR_MT_render.append(draw_gs_colmap_render_menu)
    if contour_jobs.shutdown not in bpy.app.handlers.load_pre:
        bpy.app.handlers.load_pre.append(contour_jobs.shutdown)


def unregister():
    contour_jobs.shutdown()
    if contour_jobs.shutdown in bpy.app.handlers.load_pre:
        bpy.app.handlers.load_pre.remove(contour_jobs.shutdown)
    try:
        bpy.types.TOPBAR_MT_render.remove(draw_gs_colmap_render_menu)
    except Exception:
        pass
    try:
        mesh_guided.unregister()
    except Exception:
        pass
    if hasattr(bpy.types.Scene, "gs_colmap_settings"):
        del bpy.types.Scene.gs_colmap_settings
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
