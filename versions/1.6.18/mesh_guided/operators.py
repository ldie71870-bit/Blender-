import os
from pathlib import Path

import bpy
from bpy.props import BoolProperty
from bpy.types import Operator

from .pipeline import PipelineRunner
from .state import PipelineState, remove_state
from .training.process_manager_v2 import cancel_training, pause_training, poll_training, resume_training


_ACTIVE_RUNNER = None


class GSCOLMAP_OT_mesh_guided_run(Operator):
    bl_idname = "gs_colmap.mesh_guided_run"
    bl_label = "一键生成 Mesh 引导高斯"
    bl_options = {"REGISTER"}

    resume: BoolProperty(name="继续上次任务", default=False)
    _timer = None

    def invoke(self, context, event):
        global _ACTIVE_RUNNER
        if _ACTIVE_RUNNER is not None:
            self.report({"WARNING"}, "已有 Mesh 引导任务正在运行。")
            return {"CANCELLED"}
        settings = context.scene.gs_mesh_guided_settings
        try:
            _ACTIVE_RUNNER = PipelineRunner(context, settings, resume=self.resume)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        context.window_manager.modal_handler_add(self)
        self._timer = context.window_manager.event_timer_add(0.1, window=context.window)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        global _ACTIVE_RUNNER
        if event.type == "ESC":
            self.cancel(context)
            return {"CANCELLED"}
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        try:
            finished = _ACTIVE_RUNNER.step()
        except Exception as exc:
            self._close(context)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        if finished:
            root = _ACTIVE_RUNNER.root
            status = _ACTIVE_RUNNER.state.status
            message = _ACTIVE_RUNNER.state.message
            self._close(context)
            self.report({"INFO"}, f"{message}: {root}" if status != "DONE" else f"Mesh 引导任务完成: {root}")
            return {"FINISHED"}
        return {"RUNNING_MODAL"}

    def cancel(self, context):
        global _ACTIVE_RUNNER
        if _ACTIVE_RUNNER is not None:
            try:
                if getattr(__import__("blender_gs_colmap_exporter", fromlist=["_BACKGROUND_RENDER"]), "_BACKGROUND_RENDER", None):
                    bpy.ops.gs_colmap.cancel_background_render()
            except Exception:
                pass
            cancel_training(_ACTIVE_RUNNER.root)
            _ACTIVE_RUNNER.cancel()
        self._close(context)

    def _close(self, context):
        global _ACTIVE_RUNNER
        if self._timer is not None:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        _ACTIVE_RUNNER = None


class GSCOLMAP_OT_mesh_guided_cancel(Operator):
    bl_idname = "gs_colmap.mesh_guided_cancel"
    bl_label = "取消 Mesh 引导任务"

    def execute(self, context):
        global _ACTIVE_RUNNER
        if _ACTIVE_RUNNER is None:
            self.report({"INFO"}, "没有运行中的 Mesh 引导任务。")
            return {"CANCELLED"}
        try:
            bpy.ops.gs_colmap.cancel_background_render()
        except Exception:
            pass
        cancel_training(_ACTIVE_RUNNER.root)
        _ACTIVE_RUNNER.cancel()
        self.report({"INFO"}, "已请求取消；已完成输出保留。")
        return {"FINISHED"}


class GSCOLMAP_OT_mesh_guided_pause(Operator):
    bl_idname = "gs_colmap.mesh_guided_pause"
    bl_label = "暂停 Mesh 引导任务"

    def execute(self, context):
        global _ACTIVE_RUNNER
        if _ACTIVE_RUNNER is None:
            self.report({"INFO"}, "没有运行中的 Mesh 引导任务。")
            return {"CANCELLED"}
        try:
            bpy.ops.gs_colmap.cancel_background_render()
        except Exception:
            pass
        state = poll_training(_ACTIVE_RUNNER.root).get("status", {}).get("state")
        if state == "paused":
            resume_training(_ACTIVE_RUNNER.root)
            self.report({"INFO"}, "已请求继续训练。")
        else:
            pause_training(_ACTIVE_RUNNER.root)
            self.report({"INFO"}, "已请求在安全迭代后暂停并保存 Checkpoint。")
        return {"FINISHED"}


class GSCOLMAP_OT_mesh_guided_delete_state(Operator):
    bl_idname = "gs_colmap.mesh_guided_delete_state"
    bl_label = "删除任务状态"

    def execute(self, context):
        root = context.scene.gs_mesh_guided_settings.last_task_dir
        if not root:
            return {"CANCELLED"}
        remove_state(root)
        self.report({"INFO"}, "任务状态已删除，数据文件未删除。")
        return {"FINISHED"}


class GSCOLMAP_OT_mesh_guided_open_output(Operator):
    bl_idname = "gs_colmap.mesh_guided_open_output"
    bl_label = "打开输出目录"

    def execute(self, context):
        root = Path(context.scene.gs_mesh_guided_settings.last_task_dir)
        if not root.is_dir():
            self.report({"ERROR"}, "输出目录不存在。")
            return {"CANCELLED"}
        os.startfile(str(root))
        return {"FINISHED"}


class GSCOLMAP_OT_mesh_guided_load_preview(Operator):
    bl_idname = "gs_colmap.mesh_guided_load_preview"
    bl_label = "加载初始高斯预览"

    trained: BoolProperty(name="加载训练结果", default=False)
    def execute(self, context):
        root = Path(context.scene.gs_mesh_guided_settings.last_task_dir)
        path = (
            root / "training" / "output" / "point_cloud.ply"
            if self.trained else root / "gaussians" / "init_gaussians.ply"
        )
        if not path.is_file():
            self.report({"ERROR"}, f"未找到 {path.name}。")
            return {"CANCELLED"}
        try:
            if hasattr(bpy.ops.wm, "ply_import"):
                bpy.ops.wm.ply_import(filepath=str(path))
            else:
                bpy.ops.import_mesh.ply(filepath=str(path))
        except Exception as exc:
            self.report({"ERROR"}, f"加载 PLY 失败: {exc}")
            return {"CANCELLED"}
        return {"FINISHED"}


classes = (
    GSCOLMAP_OT_mesh_guided_run,
    GSCOLMAP_OT_mesh_guided_cancel,
    GSCOLMAP_OT_mesh_guided_pause,
    GSCOLMAP_OT_mesh_guided_delete_state,
    GSCOLMAP_OT_mesh_guided_open_output,
    GSCOLMAP_OT_mesh_guided_load_preview,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    global _ACTIVE_RUNNER
    _ACTIVE_RUNNER = None
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

