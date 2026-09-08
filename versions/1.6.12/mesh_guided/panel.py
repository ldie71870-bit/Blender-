from pathlib import Path


def _header(box, settings, property_name, label, icon):
    row = box.row(align=True)
    expanded = bool(getattr(settings, property_name))
    row.prop(settings, property_name, text="", icon="TRIA_DOWN" if expanded else "TRIA_RIGHT", emboss=False)
    row.label(text=label, icon=icon)
    return expanded


def draw_mesh_guided_panel(layout, context):
    scene = context.scene
    if not hasattr(scene, "gs_mesh_guided_settings"):
        return
    settings = scene.gs_mesh_guided_settings
    box = layout.box()
    row = box.row(align=True)
    row.prop(settings, "show_panel", text="", icon="TRIA_DOWN" if settings.show_panel else "TRIA_RIGHT", emboss=False)
    row.label(text="Mesh 引导高斯", icon="POINTCLOUD_DATA")
    if not settings.show_panel:
        return
    box.prop(settings, "work_mode", text="模式")
    box.prop(settings, "output_root")
    row = box.row(align=True)
    row.prop(settings, "task_name")
    row.prop(settings, "create_date_subdir", text="", icon="SORTTIME")
    row = box.row(align=True)
    row.prop(settings, "overwrite_task")
    row.prop(settings, "keep_intermediates")

    section = box.box()
    if _header(section, settings, "show_mesh_settings", "Mesh 与采样", "MESH_DATA"):
        section.prop(settings, "mesh_collection")
        row = section.row(align=True)
        row.prop(settings, "exclude_hidden")
        row.prop(settings, "exclude_render_disabled")
        section.prop(settings, "sampling_mode")
        section.prop(settings, "target_gaussian_count")
        row = section.row(align=True)
        row.prop(settings, "curvature_weight")
        row.prop(settings, "texture_weight")
        section.prop(settings, "small_object_weight")
        section.prop(settings, "normal_thickness_ratio")
        row = section.row(align=True)
        row.prop(settings, "memory_limit_mb")
        row.prop(settings, "compress_npz")

    if settings.work_mode != "INIT_ONLY":
        section = box.box()
        if _header(section, settings, "show_camera_settings", "相机与真值", "CAMERA_DATA"):
            section.prop(settings, "automatic_camera_selection")
            section.prop(settings, "target_surface_coverage")
            row = section.row(align=True)
            row.prop(settings, "minimum_camera_count")
            row.prop(settings, "maximum_camera_count")
            row = section.row(align=True)
            row.prop(settings, "render_resolution_x")
            row.prop(settings, "render_resolution_y")
            section.prop(settings, "validation_ratio")
            section.prop(settings, "transparent_background")

    if settings.work_mode == "GENERATE_AND_TRAIN":
        section = box.box()
        if _header(section, settings, "show_training_settings", "训练器", "CONSOLE"):
            section.prop(settings, "trainer_path")
            section.prop(settings, "training_python")
            section.prop(settings, "training_script")
            section.prop(settings, "cuda_device")

    progress = box.column(align=True)
    progress.enabled = False
    progress.prop(settings, "pipeline_progress", slider=True)
    box.label(text=settings.pipeline_status, icon="INFO")
    box.operator("gs_colmap.mesh_guided_run", icon="PLAY")
    row = box.row(align=True)
    resume = row.operator("gs_colmap.mesh_guided_run", text="继续上次任务", icon="RECOVER_LAST")
    resume.resume = True
    row.operator("gs_colmap.mesh_guided_pause", text="暂停", icon="PAUSE")
    row.operator("gs_colmap.mesh_guided_cancel", text="取消", icon="CANCEL")
    row = box.row(align=True)
    row.operator("gs_colmap.mesh_guided_load_preview", text="加载初始高斯", icon="POINTCLOUD_DATA")
    row.operator("gs_colmap.mesh_guided_open_output", text="打开目录", icon="FILE_FOLDER")
    row.operator("gs_colmap.mesh_guided_load_preview", text="加载训练结果", icon="IMPORT").trained = True
    if settings.last_task_dir and (Path(settings.last_task_dir) / "pipeline_state.json").exists():
        box.operator("gs_colmap.mesh_guided_delete_state", text="删除任务状态", icon="TRASH")

