"""Real subprocess snapshot/load/append and cancellation acceptance."""
import runpy,time
from pathlib import Path
import bpy

namespace=runpy.run_path(str(Path(__file__).with_name('blender_contour_acceptance.py')))
addon=namespace['addon']
scene=bpy.context.scene
settings=scene.gs_colmap_settings
settings.floorplan_layer_mode='ONE'
settings.contour_detail_enabled=True
settings.contour_detail_budget=2
settings.floorplan_mid_height=1.3
original_path=bpy.data.filepath
original_dirty=bpy.data.is_dirty
original_curves={o.name for o in scene.objects if o.type=='CURVE'}
job=addon.contour_jobs.start(scene,addon.__file__)
try:
    deadline=time.monotonic()+90
    while job['process'].poll() is None and time.monotonic()<deadline:
        time.sleep(.1)
    assert job['process'].poll() is not None,'Worker timed out'
    result=addon.contour_jobs.finish(job)
    assert result['report']['route_count']>0
    assert original_curves <= {o.name for o in scene.objects if o.type=='CURVE'}
    assert bpy.data.filepath==original_path
    assert all(o.get('gs_explicit_capture_height') for o in settings.path_collection.objects)
    print('BACKGROUND_ROUNDTRIP_OK',result['report']['route_count'],flush=True)
finally:
    addon.contour_jobs.dispose(job)
job=addon.contour_jobs.start(scene,addon.__file__)
before={o.name for o in scene.objects}
addon.contour_jobs.cancel(scene)
assert job['cancelled']
addon.contour_jobs.dispose(job)
assert job['process'].poll() is not None
assert before=={o.name for o in scene.objects}
assert not addon.contour_jobs.JOBS
print('BACKGROUND_CANCEL_OK',flush=True)
