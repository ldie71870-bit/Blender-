"""Own-process contour preview jobs. Snapshot in, curve collection out."""
from __future__ import annotations
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
import bpy

JOBS = {}


def write_json(path, payload):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
    os.replace(temporary, path)


def worker(root, addon_init, scene_name):
    root = Path(root)
    try:
        spec = importlib.util.spec_from_file_location('gs_contour_worker_addon', addon_init,
                                                     submodule_search_locations=[str(Path(addon_init).parent)])
        addon = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = addon
        spec.loader.exec_module(addon)
        addon.register()
        scene = bpy.data.scenes[scene_name]
        if bpy.context.window:
            bpy.context.window.scene = scene
        last = [0.0]
        def progress(stage, current, total):
            now = time.monotonic()
            if now-last[0] > .5 or current in (0,total):
                write_json(root/'progress.json',dict(stage=stage,current=current,total=total))
                last[0] = now
        states = addon.hide_camera_mesh_visuals(scene)
        try:
            result = addon.contour_blender.generate(scene, scene.gs_colmap_settings, progress)
        finally:
            addon.restore_camera_mesh_visuals(states)
        bpy.data.libraries.write(str(root/'preview.blend'), {result['collection']}, fake_user=True)
        write_json(root/'result.json',dict(ok=True,collection=result['collection'].name,report=result['report']))
    except Exception as exc:
        write_json(root/'result.json',dict(ok=False,error=str(exc)))
        traceback.print_exc()
        raise


def start(scene, addon_init):
    key = scene.as_pointer()
    if key in JOBS:
        raise RuntimeError('此场景已有排线任务正在运行')
    root = Path(tempfile.mkdtemp(prefix='gs_contour_'))
    log = None
    try:
        snapshot = root/'scene.blend'
        bpy.data.libraries.write(str(snapshot), {scene}, path_remap='ABSOLUTE')
        log = (root/'worker.log').open('w',encoding='utf-8')
        command = [bpy.app.binary_path,'--background','--factory-startup','--disable-autoexec',
                   str(snapshot),'--python-exit-code','1','--python',str(Path(__file__).resolve()),
                   '--',str(root),str(Path(addon_init).resolve()),scene.name]
        process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                   creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        job = dict(key=key,root=root,process=process,log=log,scene=scene,cancelled=False)
        JOBS[key] = job
        return job
    except Exception:
        if log:
            log.close()
        shutil.rmtree(root,ignore_errors=True)
        raise


def status(job):
    path = job['root']/'progress.json'
    if path.exists():
        try:
            return json.loads(path.read_text(encoding='utf-8')).get('stage','正在生成')
        except (OSError,ValueError):
            pass
    return '正在加载场景快照'


def finish(job):
    if job['process'].poll() is None:
        raise RuntimeError('后台任务尚未退出')
    path = job['root']/'result.json'
    if not path.exists():
        raise RuntimeError(f'排线进程未完成（退出码 {job["process"].returncode}），日志：{job["root"] / "worker.log"}')
    result = json.loads(path.read_text(encoding='utf-8'))
    if not result.get('ok'):
        raise RuntimeError(result.get('error','后台排线失败'))
    scene = job['scene']
    with bpy.data.libraries.load(str(job['root']/'preview.blend'),link=False) as (source,target):
        if result['collection'] not in source.collections:
            raise RuntimeError('预览集合缺失')
        target.collections = [result['collection']]
    collection = target.collections[0]
    scene.collection.children.link(collection)
    settings = scene.gs_colmap_settings
    settings.path_collection = collection
    settings.path_object = next(obj for obj in collection.all_objects if obj.get('gs_route_role')=='MAIN')
    settings.rig_mode = 'PATH'
    scene['gs_contour_last_report'] = json.dumps(result['report'],ensure_ascii=False)
    from .contour_blender import show_latest_preview
    show_latest_preview(scene,collection)
    return result


def dispose(job, keep_logs=False):
    if job['process'].poll() is None:
        job['process'].terminate()
        try:
            job['process'].wait(timeout=2)
        except subprocess.TimeoutExpired:
            job['process'].kill()
            job['process'].wait(timeout=2)
    job['log'].close()
    JOBS.pop(job['key'],None)
    if not keep_logs:
        # Owned mkdtemp directory, never a user-supplied path.
        shutil.rmtree(job['root'],ignore_errors=True)


def cancel(scene):
    job = JOBS.get(scene.as_pointer())
    if job:
        job['cancelled'] = True


@bpy.app.handlers.persistent
def shutdown(*_args):
    for job in list(JOBS.values()):
        job['cancelled'] = True
        try:
            job['scene'].gs_colmap_settings.contour_running = False
        except (ReferenceError,AttributeError):
            pass
        dispose(job)


if __name__ == '__main__':
    worker(*sys.argv[sys.argv.index('--')+1:])
