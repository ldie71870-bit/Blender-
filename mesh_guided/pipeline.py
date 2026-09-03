import json
import shutil
from pathlib import Path

from .cameras.camera_export import export_cameras
from .cameras.camera_selector import COLLECTION_NAME, prepare_mesh_guided_cameras
from .gaussians.gaussian_export import merge_and_export
from .gaussians.surface_sampler import allocate_sample_counts, sample_mesh_part
from .materials.material_parser import MaterialSampler, describe_materials
from .mesh.evaluated_mesh import enumerate_mesh_instances, extract_mesh_instance, merge_training_mesh
from .render.gbuffer_renderer import TriangleIdRenderer, render_normal
from .render.render_queue import DatasetFinalizer
from .state import PipelineState, find_latest_state
from .training.config_writer import write_training_config
from .training.environment_v2 import check_training_environment
from .training.process_manager_v2 import poll_training, start_training
from .utils.json_io import atomic_write_json, read_json
from .utils.logging import log_exception, task_logger
from .utils.memory import estimated_sample_bytes, fits_memory_budget
from .utils.paths import create_task_tree, numbered_name, resolve_task_root
from .validation import validate_dataset


PLUGIN_VERSION = "1.4.0"


class PipelineError(RuntimeError):
    pass


class PipelineRunner:
    """One bounded unit of work per step, driven by a Blender modal timer."""

    def __init__(self, context, settings, resume=False):
        self.context = context
        self.scene = context.scene
        self.settings = settings
        discovered = None
        if resume:
            if settings.last_task_dir:
                discovered = PipelineState.load(settings.last_task_dir)
            if discovered is None and settings.output_root:
                discovered = find_latest_state(settings.output_root)
        if discovered is not None:
            self.root = Path(discovered.task_root)
        else:
            self.root = resolve_task_root(settings)
        existing = discovered or (PipelineState.load(self.root) if resume else None)
        self.state = existing or PipelineState(task_root=str(self.root))
        self.logger = task_logger(self.root)
        self.instances = None
        self.metadata = None
        self.quotas = None
        self.cameras = None
        self.camera_records = None
        self.validation_ids = None
        self.triangle_id_renderer = None
        self.finalizer = None
        self.legacy_snapshot = None
        self.finished = False
        self.settings.last_task_dir = str(self.root)

    def _set_stage(self, stage, message, index=0, total=0):
        self.state.stage = stage
        self.state.message = message
        self.state.current_index = int(index)
        self.state.total_items = int(total)
        self.state.status = "RUNNING"
        self.state.error = ""
        self.state.save()
        self.settings.pipeline_status = message
        self.settings.pipeline_progress = (float(index) / total) if total else 0.0

    def _close_triangle_id_renderer(self):
        renderer = self.triangle_id_renderer
        self.triangle_id_renderer = None
        if renderer is None:
            return
        try:
            renderer.close()
        except Exception as exc:
            self.logger.warning("Triangle ID renderer cleanup failed: %s", exc)

    def _fail(self, exc):
        self.state.status = "ERROR"
        self.state.error = str(exc)
        self.state.message = str(exc)
        self.state.save()
        self.settings.pipeline_status = str(exc)
        log_exception(self.logger, self.state.stage, self.state.current_index, exc)
        self._close_triangle_id_renderer()
        self._restore_legacy_settings()

    def step(self):
        if self.finished:
            return True
        try:
            method = getattr(self, f"_stage_{self.state.stage.lower()}", None)
            if method is None:
                raise PipelineError(f"未知流水线阶段: {self.state.stage}")
            method()
            return self.finished
        except Exception as exc:
            self._fail(exc)
            raise

    def _stage_preflight(self):
        import bpy

        if not bpy.data.filepath:
            raise PipelineError("当前 blend 文件尚未保存。请先保存场景，以便可靠解析相对贴图路径。")
        if not self.settings.output_root:
            raise PipelineError("未设置输出根目录。")
        if self.settings.work_mode != "INIT_ONLY":
            render_engine = self.scene.render.engine
            if render_engine not in {"CYCLES", "BLENDER_EEVEE"}:
                raise PipelineError(
                    f"当前渲染器 {render_engine} 无法由 Blender Python 稳定自动渲染。"
                    "插件不会静默切换到 Cycles；请改用 Cycles/EEVEE，或仅生成初始高斯。"
                )
        existing_entries = [path for path in self.root.iterdir()
                            if path.name not in {"pipeline.log", "pipeline_state.json"}] if self.root.exists() else []
        if existing_entries and not self.settings.overwrite_task:
            previous = PipelineState.load(self.root)
            if previous and previous.status not in {"DONE", "ERROR", "CANCELLED"}:
                self.state = previous
                self.settings.pipeline_status = "检测到未完成任务，请使用“继续上次任务”。"
                raise PipelineError(self.settings.pipeline_status)
            raise PipelineError(f"任务目录已存在: {self.root}。请更换任务名或启用覆盖。")
        if self.root.exists() and self.settings.overwrite_task:
            for relative in ("config", "mesh", "cameras", "images", "depth", "normal", "object_id",
                             "triangle_id", "gaussians", "training", "temp"):
                shutil.rmtree(self.root / relative, ignore_errors=True)
        create_task_tree(self.root)
        instances = enumerate_mesh_instances(self.scene, self.settings)
        if not instances:
            raise PipelineError("场景中没有符合筛选条件的可渲染 Mesh。请检查隐藏状态和 Mesh 集合设置。")
        if not fits_memory_budget(self.settings.target_gaussian_count, self.settings.memory_limit_mb):
            estimate = estimated_sample_bytes(self.settings.target_gaussian_count) / (1024 ** 2)
            raise PipelineError(f"预计内存 {estimate:.0f} MB 超过上限 {self.settings.memory_limit_mb} MB，请降低初始高斯数量或提高内存上限。")
        estimated_bytes = estimated_sample_bytes(self.settings.target_gaussian_count)
        required_disk_bytes = estimated_bytes * 2 + 512 * 1024 * 1024
        free_disk_bytes = shutil.disk_usage(self.root).free
        if free_disk_bytes < required_disk_bytes:
            raise PipelineError(
                f"输出磁盘可用空间不足：至少需要约 {required_disk_bytes / (1024 ** 3):.2f} GiB，"
                f"当前只有 {free_disk_bytes / (1024 ** 3):.2f} GiB。"
            )
        self.instances = instances
        atomic_write_json(self.root / "config" / "plugin_version.json", {
            "addon": "blender_gs_colmap_exporter",
            "version": PLUGIN_VERSION,
            "baseline": "blender_gs_colmap_exporter-1.2.7-chunked-background-render",
        })
        atomic_write_json(self.root / "config" / "scene_config.json", {
            "schema_version": "1.0",
            "blend_file": bpy.data.filepath,
            "mode": self.settings.work_mode,
            "target_initial_gaussians": int(self.settings.target_gaussian_count),
            "normal_thickness_ratio": float(self.settings.normal_thickness_ratio),
            "evaluated_dependency_graph": True,
            "packed_image_access": "bpy.data.images pixel buffer",
            "world_unit_to_meters": float(self.scene.unit_settings.scale_length or 1.0),
            "coordinate_system": "Blender world: +X right, +Y forward, +Z up",
        })
        write_training_config(self.root)
        atomic_write_json(self.root / "config" / "preflight_report.json", {
            "mesh_instance_count": len(instances),
            "estimated_sample_memory_bytes": estimated_bytes,
            "required_disk_bytes": required_disk_bytes,
            "free_disk_bytes": free_disk_bytes,
            "render_engine": self.scene.render.engine,
        })
        self.logger.info("stage=PREFLIGHT meshes=%s root=%s", len(instances), self.root)
        self._set_stage("EXTRACT_MESH", "正在提取最终计算后的 Mesh", 0, len(instances))

    def _ensure_instances(self):
        if self.instances is None:
            self.instances = enumerate_mesh_instances(self.scene, self.settings)

    def _ensure_metadata(self):
        if self.metadata is None:
            self.metadata = read_json(self.root / "mesh" / "mesh_metadata.partial.json", []) or []

    def _stage_extract_mesh(self):
        self._ensure_instances()
        self._ensure_metadata()
        index = int(self.state.current_index)
        if index >= len(self.instances):
            part_paths = [self.root / "temp" / "mesh_parts" / item["part_file"] for item in self.metadata]
            training_mesh = merge_training_mesh(
                part_paths,
                self.metadata,
                self.root / "mesh" / "training_mesh.npz",
                world_unit_to_meters=float(self.scene.unit_settings.scale_length or 1.0),
            )
            atomic_write_json(self.root / "mesh" / "mesh_metadata.json", {
                "schema_version": "1.0",
                "evaluated_dependency_graph": True,
                "instance_count": len(self.metadata),
                "triangle_count": sum(item["triangle_count"] for item in self.metadata),
                "surface_area": sum(item["surface_area"] for item in self.metadata),
                "training_topology": "training_mesh.npz",
                "training_mesh": training_mesh,
                "instances": self.metadata,
            })
            material_names = sorted({name for item in self.metadata for name in item["material_names"] if name})
            material_descriptions = describe_materials(material_names)
            atomic_write_json(self.root / "mesh" / "material_metadata.json", {
                "materials": material_descriptions,
            })
            missing_images = [item["base_color_image"] for item in material_descriptions
                              if item["base_color_image"] and not item["image_packed"] and not item["image_has_data"]]
            if missing_images:
                raise PipelineError(
                    "以下材质图像既未打包也无法读取: " + ", ".join(sorted(set(missing_images)))
                    + "。请修复路径或执行 Pack Resources。"
                )
            preflight = read_json(self.root / "config" / "preflight_report.json", {}) or {}
            preflight.update({
                "missing_uv_objects": [item["object_name"] for item in self.metadata if not item["has_uv"]],
                "procedural_materials": [item["name"] for item in material_descriptions if item["procedural_base_color"]],
                "missing_images": missing_images,
                "zero_area_triangle_count": sum(item["zero_area_triangle_count"] for item in self.metadata),
            })
            atomic_write_json(self.root / "config" / "preflight_report.json", preflight)
            self._set_stage("EXPORT_REFERENCE_MESH", "正在导出 GLB 参考 Mesh")
            return
        instance = self.instances[index]
        part = self.root / "temp" / "mesh_parts" / f"{index + 1:06d}.npz"
        metadata = extract_mesh_instance(instance, part, index + 1)
        if index < len(self.metadata):
            self.metadata[index] = metadata
        else:
            self.metadata.append(metadata)
        atomic_write_json(self.root / "mesh" / "mesh_metadata.partial.json", self.metadata)
        self.logger.info("stage=EXTRACT_MESH object=%s triangles=%s", metadata["object_name"], metadata["triangle_count"])
        self._set_stage("EXTRACT_MESH", f"已提取 {metadata['object_name']}", index + 1, len(self.instances))

    def _stage_export_reference_mesh(self):
        import bpy

        target = self.root / "mesh" / "scene.glb"
        try:
            bpy.ops.export_scene.gltf(
                filepath=str(target), export_format="GLB", use_visible=True,
                export_apply=True, export_materials="EXPORT",
            )
        except Exception as exc:
            raise PipelineError(f"GLB 参考 Mesh 导出失败: {exc}") from exc
        self._ensure_metadata()
        self.quotas = allocate_sample_counts(
            self.metadata,
            self.settings.target_gaussian_count,
            minimum_per_object=32,
            small_object_weight=self.settings.small_object_weight,
        )
        atomic_write_json(self.root / "temp" / "sample_plan.json", {
            "counts": self.quotas,
            "target": int(sum(self.quotas)),
        })
        self._set_stage("SAMPLE_MESH", "正在按三角形面积采样 Mesh", 0, len(self.metadata))

    def _ensure_sample_plan(self):
        self._ensure_metadata()
        if self.quotas is None:
            plan = read_json(self.root / "temp" / "sample_plan.json", {}) or {}
            self.quotas = plan.get("counts") or allocate_sample_counts(
                self.metadata, self.settings.target_gaussian_count,
                small_object_weight=self.settings.small_object_weight,
            )

    def _stage_sample_mesh(self):
        self._ensure_sample_plan()
        index = int(self.state.current_index)
        if index >= len(self.metadata):
            self._set_stage("EXPORT_GAUSSIANS", "正在合并并导出初始高斯")
            return
        metadata = self.metadata[index]
        mesh_part = self.root / "temp" / "mesh_parts" / metadata["part_file"]
        sample_part = self.root / "temp" / "sample_parts" / f"{index + 1:06d}.npz"
        triangle_offset = sum(item["triangle_count"] for item in self.metadata[:index])
        sampler = MaterialSampler(metadata["material_names"])
        sampling_mode = self.settings.sampling_mode
        curvature_weight = self.settings.curvature_weight if sampling_mode != "UNIFORM" else 0.0
        texture_weight = self.settings.texture_weight if sampling_mode == "MIXED" else 0.0
        sample_mesh_part(
            mesh_part, sample_part, self.quotas[index], sampler,
            seed=self.settings.random_seed + index,
            curvature_weight=curvature_weight,
            texture_weight=texture_weight,
            triangle_id_offset=triangle_offset,
        )
        self.logger.info("stage=SAMPLE_MESH object=%s samples=%s", metadata["object_name"], self.quotas[index])
        self._set_stage("SAMPLE_MESH", f"已采样 {metadata['object_name']}", index + 1, len(self.metadata))

    def _stage_export_gaussians(self):
        paths = sorted((self.root / "temp" / "sample_parts").glob("*.npz"))
        result = merge_and_export(
            paths,
            self.root / "gaussians",
            normal_thickness_ratio=self.settings.normal_thickness_ratio,
            compressed=self.settings.compress_npz,
            world_unit_to_meters=float(self.scene.unit_settings.scale_length or 1.0),
        )
        self.state.initial_gaussians_complete = True
        self.logger.info("stage=EXPORT_GAUSSIANS count=%s", result["count"])
        if self.settings.work_mode == "INIT_ONLY":
            self._set_stage("VALIDATE", f"Initial Gaussian generation complete: {result['count']:,}")
            return
        self._set_stage("PREPARE_CAMERAS", f"已生成 {result['count']:,} 个初始高斯")

    def _stage_prepare_cameras(self):
        from .. import active_dataset_cameras, create_rig
        self.scene.view_layers[0].update()

        legacy = self.scene.gs_colmap_settings
        candidates = [camera for camera in active_dataset_cameras(self.scene, legacy)
                      if not camera.get("gs_mesh_guided_camera")]
        if not candidates:
            old_mode = legacy.rig_mode
            old_live = legacy.live_update_cameras
            old_count = legacy.camera_count
            try:
                legacy.live_update_cameras = False
                if old_mode == "EXISTING":
                    legacy.rig_mode = "CYLINDER"
                legacy.camera_count = max(8, min(int(self.settings.minimum_camera_count), 200))
                create_rig(self.scene, legacy)
                candidates = active_dataset_cameras(self.scene, legacy)
            finally:
                legacy.rig_mode = old_mode
                legacy.camera_count = old_count
                legacy.live_update_cameras = old_live
        if not candidates:
            raise PipelineError("场景没有相机，且现有相机排线系统未能生成候选相机。")
        collection, self.cameras = prepare_mesh_guided_cameras(
            self.scene, candidates, maximum=self.settings.maximum_camera_count,
            deduplicate=self.settings.automatic_camera_selection,
        )
        self.camera_records, train, validation = export_cameras(
            self.scene, self.cameras, self.root,
            validation_ratio=self.settings.validation_ratio,
            width=self.settings.render_resolution_x,
            height=self.settings.render_resolution_y,
            world_unit_to_meters=float(self.scene.unit_settings.scale_length or 1.0),
        )
        self.validation_ids = {item["id"] for item in validation}
        atomic_write_json(self.root / "cameras" / "visibility_report.json", {
            "candidate_count": len(candidates),
            "selected_count": len(self.cameras),
            "selection": "deterministic position-and-orientation de-duplication",
            "target_surface_coverage": float(self.settings.target_surface_coverage),
            "measured_surface_coverage": None,
            "note": "Triangle visibility scoring is experimental and was not used; no fabricated coverage value is reported.",
        })
        if self.settings.work_mode == "INIT_ONLY":
            self._set_stage("VALIDATE", "正在验证初始高斯与相机数据")
        else:
            self._set_stage("START_RENDER", f"已选择 {len(self.cameras)} 台训练相机")

    def _snapshot_legacy_settings(self, legacy):
        keys = (
            "output_dir", "camera_collection", "auto_create_rig", "rig_mode", "resolution_x",
            "resolution_y", "image_prefix", "image_format", "render_rgb", "export_depth", "depth_format",
            "export_id", "export_object_depth", "export_normal", "export_object_normal",
            "export_object_mask", "export_material_id", "object_split_mode", "background_render",
            "incremental", "transparent_background", "render_engine",
        )
        self.legacy_snapshot = {key: getattr(legacy, key) for key in keys}

    def _restore_legacy_settings(self):
        if not self.legacy_snapshot or not hasattr(self.scene, "gs_colmap_settings"):
            return
        legacy = self.scene.gs_colmap_settings
        live = legacy.live_update_cameras
        legacy.live_update_cameras = False
        try:
            for key, value in self.legacy_snapshot.items():
                setattr(legacy, key, value)
        finally:
            legacy.live_update_cameras = live
            self.legacy_snapshot = None

    def _stage_start_render(self):
        import bpy

        legacy = self.scene.gs_colmap_settings
        self._snapshot_legacy_settings(legacy)
        legacy.live_update_cameras = False
        legacy.output_dir = str(self.root)
        legacy.camera_collection = COLLECTION_NAME
        legacy.auto_create_rig = False
        legacy.rig_mode = "EXISTING"
        legacy.resolution_x = int(self.settings.render_resolution_x)
        legacy.resolution_y = int(self.settings.render_resolution_y)
        legacy.image_prefix = "mesh_guided"
        legacy.image_format = "PNG"
        legacy.render_rgb = True
        legacy.export_depth = True
        legacy.depth_format = "EXR"
        legacy.export_id = True
        legacy.export_object_depth = False
        legacy.export_normal = False
        legacy.export_object_normal = False
        legacy.export_object_mask = False
        legacy.export_material_id = False
        legacy.object_split_mode = "VIRTUAL_SPLIT"
        legacy.background_render = True
        legacy.incremental = True
        legacy.transparent_background = bool(self.settings.transparent_background)
        legacy.render_engine = self.scene.render.engine
        result = bpy.ops.gs_colmap.render_dataset_background("INVOKE_DEFAULT")
        if "CANCELLED" in result:
            raise PipelineError(f"无法启动后台数据渲染: {legacy.render_status}")
        self._set_stage("WAIT_RENDER", "后台渲染已启动")

    def _stage_wait_render(self):
        from .. import _BACKGROUND_RENDER

        legacy = self.scene.gs_colmap_settings
        self.settings.pipeline_progress = float(legacy.render_progress)
        self.settings.pipeline_status = legacy.render_status
        self.state.message = legacy.render_status
        self.state.save()
        if _BACKGROUND_RENDER is not None:
            return
        self._ensure_cameras_from_scene()
        rendered_images = [path for path in (self.root / "images").glob("*.*") if path.is_file()]
        if self.camera_records and len(rendered_images) >= len(self.camera_records):
            self._restore_legacy_settings()
            self._set_stage("RENDER_NORMALS", "正在渲染相机空间法线", 0, len(self.cameras))
            return
        if legacy.render_status.startswith("Done:"):
            self._restore_legacy_settings()
            self._ensure_cameras_from_scene()
            self._set_stage("RENDER_NORMALS", "正在渲染相机空间法线", 0, len(self.cameras))
            return
        if "failed" in legacy.render_status.lower() or "cancel" in legacy.render_status.lower():
            raise PipelineError(legacy.render_status)

    def _ensure_cameras_from_scene(self):
        if self.cameras is None:
            import bpy
            collection = bpy.data.collections.get(COLLECTION_NAME)
            self.cameras = sorted((obj for obj in collection.objects if obj.type == "CAMERA"), key=lambda obj: obj.name) if collection else []
        if self.camera_records is None:
            payload = read_json(self.root / "cameras" / "cameras.json", {}) or {}
            self.camera_records = payload.get("cameras", [])
            validation = read_json(self.root / "cameras" / "validation.json", {}) or {}
            self.validation_ids = {int(item["id"]) for item in validation.get("cameras", [])}

    def _stage_render_normals(self):
        self._ensure_cameras_from_scene()
        index = int(self.state.current_index)
        if index >= len(self.cameras):
            self._close_triangle_id_renderer()
            self._set_stage("FINALIZE_DATASET", "正在整理训练/验证数据", 0, len(self.cameras))
            return
        camera_id = index + 1
        split = "validation" if camera_id in self.validation_ids else "train"
        normal_target = self.root / "normal" / split / numbered_name(camera_id, ".exr")
        triangle_target = self.root / "triangle_id" / split / numbered_name(camera_id, ".npy")
        render_normal(self.scene, self.cameras[index], normal_target)
        if self.triangle_id_renderer is None:
            self.triangle_id_renderer = TriangleIdRenderer(
                self.scene, self.root / "mesh" / "training_mesh.npz"
            )
        self.triangle_id_renderer.render(self.cameras[index], triangle_target)
        self._set_stage(
            "RENDER_NORMALS", f"已渲染法线与 Triangle ID {camera_id}/{len(self.cameras)}",
            camera_id, len(self.cameras),
        )

    def _stage_finalize_dataset(self):
        self._ensure_cameras_from_scene()
        if self.finalizer is None:
            self.finalizer = DatasetFinalizer(self.root, self.camera_records, self.validation_ids)
            self.finalizer.index = int(self.state.current_index)
        done = self.finalizer.step()
        self._set_stage("FINALIZE_DATASET", "正在转换对象 ID 并整理目录", self.finalizer.index, self.finalizer.total)
        if done:
            self._set_stage("VALIDATE", "正在验证数据一致性")

    def _stage_validate(self):
        expect_images = self.settings.work_mode != "INIT_ONLY"
        report = validate_dataset(self.root, expect_images=expect_images)
        if not report["valid"]:
            raise PipelineError("数据集验证失败: " + "; ".join(report["errors"][:5]))
        self.state.data_validation_complete = True
        if self.settings.work_mode == "GENERATE_AND_TRAIN":
            self._set_stage("START_TRAINING", "正在检查外部训练环境")
        else:
            self._set_stage("DONE", "Mesh 引导高斯任务完成")

    def _stage_start_training(self):
        result = check_training_environment(self.settings)
        if not result["ok"]:
            raise PipelineError(" ".join(result["errors"]))
        pid, command, log_path = start_training(self.settings, self.root, result)
        self.state.training_pid = int(pid)
        atomic_write_json(self.root / "training" / "launch.json", {
            "pid": pid, "command": command, "log": str(log_path),
        })
        self._set_stage("WAIT_TRAINING", f"训练已启动，PID {pid}")

    def _stage_wait_training(self):
        result = poll_training(self.root)
        progress = result["progress"]
        current = int(progress.get("current", progress.get("iteration", 0)) or 0)
        total = int(progress.get("total", progress.get("total_iterations", 0)) or 0)
        stage = progress.get("stage", "training")
        self.settings.pipeline_progress = float(current / total) if total else 0.0
        self.settings.pipeline_status = f"{stage}: {current}/{total}" if total else stage
        if result["running"]:
            return
        if result["return_code"] == 0:
            point_cloud = self.root / "training" / "output" / "point_cloud.ply"
            if not point_cloud.is_file() or point_cloud.stat().st_size <= 0:
                raise PipelineError(
                    "训练进程已返回成功，但未生成 training/output/point_cloud.ply。"
                )
            self.state.last_checkpoint = str(point_cloud)
            self._set_stage("DONE", "训练完成")
        elif result["return_code"] is None:
            raise PipelineError("训练进程状态丢失。数据集已保留，请查看 training/logs/trainer.log。")
        else:
            raise PipelineError(f"训练进程退出码 {result['return_code']}，请查看 training/logs/trainer.log。")

    def _stage_done(self):
        self.state.status = "DONE"
        self.state.message = "Mesh 引导高斯任务完成"
        self.state.save()
        self.settings.pipeline_progress = 1.0
        self.settings.pipeline_status = self.state.message
        self._close_triangle_id_renderer()
        if not self.settings.keep_intermediates:
            shutil.rmtree(self.root / "temp", ignore_errors=True)
        self._restore_legacy_settings()
        self.finished = True

    def cancel(self):
        if self.state.stage == "WAIT_RENDER":
            self.state.stage = "START_RENDER"
        elif self.state.stage == "WAIT_TRAINING":
            self.state.stage = "START_TRAINING"
            self.state.training_pid = 0
        self._close_triangle_id_renderer()
        self._restore_legacy_settings()
        self.state.status = "CANCELLED"
        self.state.message = "任务已取消，可从 pipeline_state.json 继续"
        self.state.save()
        self.settings.pipeline_status = self.state.message
        self.finished = True

    def pause(self):
        if self.state.stage == "WAIT_RENDER":
            self.state.stage = "START_RENDER"
        elif self.state.stage == "WAIT_TRAINING":
            self.state.stage = "START_TRAINING"
            self.state.training_pid = 0
        self._close_triangle_id_renderer()
        self._restore_legacy_settings()
        self.state.status = "PAUSED"
        self.state.message = "任务已暂停；已完成输出和 checkpoint 保留"
        self.state.save()
        self.settings.pipeline_status = self.state.message
        self.finished = True

