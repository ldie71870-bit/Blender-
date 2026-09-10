"""Exercise production modal code in Blender, including legacy-module upgrades.

Run with Blender --background --factory-startup --python-exit-code 1 --python
this_file.py. Window-manager/event doubles drive the real modal method; real
worker processes and .blend collection imports cover completion/cancel/failure.
"""
import importlib.util
import json
import math
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import bpy


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'contour_modal_regression_addon', ROOT / '__init__.py',
    submodule_search_locations=[str(ROOT)],
)
addon = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = addon
spec.loader.exec_module(addon)
addon.register()
jobs = addon.contour_jobs
scene = bpy.context.scene
settings = scene.gs_colmap_settings


class ModalHarness:
    def __init__(self, job):
        self._job = job
        self._timer = object()
        self.reports = []
        self.removed_timers = []
        self.redraws = 0
        self.context = SimpleNamespace(
            scene=job['scene'],
            window_manager=SimpleNamespace(event_timer_remove=self.remove_timer),
            screen=SimpleNamespace(areas=[SimpleNamespace(tag_redraw=self.redraw)]),
        )

    def report(self, level, message):
        self.reports.append((level, message))

    def remove_timer(self, timer):
        assert timer is self._timer
        self.removed_timers.append(timer)

    def redraw(self):
        self.redraws += 1

    def step(self, event_type='TIMER'):
        return addon.GSCOLMAP_OT_auto_floorplan_path.modal(
            self, self.context, SimpleNamespace(type=event_type),
        )


class MustKeepRunning:
    def poll(self):
        return None

    def terminate(self):
        raise AssertionError('An optional progress display error killed the worker')


def assert_cleaned(job, harness, keep_logs=False):
    assert job['key'] not in jobs.JOBS, 'Task registration leaked'
    assert job['process'].poll() is not None, 'Worker still running'
    assert job['log'].closed, 'Worker log handle leaked'
    assert harness.removed_timers == [harness._timer], 'Timer not removed exactly once'
    assert not job['scene'].gs_colmap_settings.contour_running
    assert job['root'].exists() == keep_logs, 'Temporary result cleanup mismatch'


def wait_modal(harness, timeout=90):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = harness.step()
        if result in ({'FINISHED'}, {'CANCELLED'}):
            return result
        assert result == {'PASS_THROUGH'}, result
        time.sleep(.1)
    raise AssertionError('Contour modal worker timed out')


# Emulate upgrading a package while its old contour_jobs stays in sys.modules.
# The newly installed UI may coexist with an old modal which still calls these
# methods. Delete both interfaces, then reload only the package as Blender
# does: Python retains the submodule object, so the compatibility layer must
# repair that exact object.
for attr in ('status', 'fraction'):
    delattr(jobs, attr)
addon.unregister()
# The fixture package is intentionally loaded from an isolated file spec, so
# importlib.reload cannot rediscover it by name. Re-executing that same spec
# keeps the module dictionary and its contour_jobs submodule alive, matching
# Blender's extension reload behavior.
spec.loader.exec_module(addon)
assert addon.contour_jobs is jobs
assert callable(jobs.status) and callable(jobs.fraction)
addon.register()
scene = bpy.context.scene
settings = scene.gs_colmap_settings

try:
    with tempfile.TemporaryDirectory(prefix='gs_modal_progress_test_') as temp:
        root = Path(temp)
        job = dict(root=root, process=MustKeepRunning(), scene=scene,
                   cancelled=False, key=scene.as_pointer())
        harness = ModalHarness(job)
        settings.contour_running = True
        settings.contour_progress = 0.
        assert harness.step('MOUSEMOVE') == {'PASS_THROUGH'}
        assert harness.redraws == 0
        previous = 0.
        cases = [
            None,
            '{',
            '[]',
            'null',
            '"not a progress object"',
            '{"stage":"识别地面和夹层","current":80,"total":100}',
            '{"stage":"识别地面和夹层","current":1,"total":100}',
            '{"stage":"建立场景几何查询","current":NaN,"total":1}',
            '{"stage":"建立场景几何查询","current":Infinity,"total":1}',
            '{"stage":"建立场景几何查询","current":1,"total":Infinity}',
            '{"stage":"建立场景几何查询","current":1,"total":0}',
            '{"stage":null,"current":{},"total":[]}',
            '{"stage":"相机验证","current":200,"total":100}',
            None,
        ]
        progress_path = root / 'progress.json'
        for raw in cases:
            if raw is None:
                progress_path.unlink(missing_ok=True)
            else:
                progress_path.write_text(raw, encoding='utf8')
            legacy_status = jobs.status(job)
            legacy_fraction = jobs.fraction(job)
            assert harness.step() == {'PASS_THROUGH'}, raw
            value = settings.contour_progress
            assert math.isfinite(value) and previous <= value < 1., (raw, previous, value)
            assert legacy_status == settings.contour_status
            # Blender RNA stores FloatProperty values as single precision.
            assert math.isclose(legacy_fraction, value, abs_tol=1e-6)
            assert isinstance(settings.contour_status, str) and settings.contour_status
            assert settings.contour_running and not harness.removed_timers
            previous = value
        assert previous > 0., 'Valid progress never reached the UI'
        settings.contour_running = False
        print('CONTOUR_MODAL_LEGACY_PROGRESS_OK', len(cases), flush=True)

    # Small genuine interior for subprocess generation and .blend append.
    for obj in list(scene.objects):
        bpy.data.objects.remove(obj, do_unlink=True)

    def box(name, location, dimensions):
        bpy.ops.mesh.primitive_cube_add(size=1., location=location)
        obj = bpy.context.object
        obj.name = name
        obj.scale = dimensions
        return obj

    floor = box('ModalFixtureFloor', (2., 2., -.1), (4., 4., .2))
    box('ModalFixtureCeiling', (2., 2., 3.1), (4., 4., .2))
    for x in (0., 4.):
        box('ModalFixtureWall', (x, 2., 1.5), (.2, 4., 3.))
    for y in (0., 4.):
        box('ModalFixtureWall', (2., y, 1.5), (4., .2, 3.))
    floors = bpy.data.collections.new('ModalFixtureFloors')
    scene.collection.children.link(floors)
    floors.objects.link(floor)
    settings.floorplan_method = 'CONTOUR'
    settings.floorplan_space_mode = 'ALL'
    settings.floorplan_layer_mode = 'ONE'
    settings.floorplan_mid_height = 1.3
    settings.floorplan_spacing = .8
    settings.contour_probe_spacing = .35
    settings.contour_clearance = .12
    settings.contour_floor_collection = floors
    settings.contour_detail_enabled = False
    settings.contour_peek_enabled = False
    settings.contour_cross_grid_enabled = False
    settings.contour_camera_audit_enabled = False
    scene.cursor.location = (2., 2., 1.3)
    bpy.context.view_layer.update()
    original_path = bpy.data.filepath

    job = jobs.start(scene, addon.__file__)
    settings.contour_running = True
    harness = ModalHarness(job)
    assert wait_modal(harness) == {'FINISHED'}, harness.reports
    assert settings.contour_progress == 1.
    assert settings.path_collection and settings.path_object
    curves = [o for o in settings.path_collection.all_objects if o.type == 'CURVE']
    assert curves and settings.path_object in curves
    assert all(o.get('gs_explicit_capture_height') for o in curves)
    report = json.loads(scene['gs_contour_last_report'])
    assert report['route_count'] == len(curves) > 0
    assert bpy.data.filepath == original_path, 'Worker snapshot changed user file path'
    assert_cleaned(job, harness)
    print('CONTOUR_MODAL_REAL_IMPORT_OK', len(curves), flush=True)

    # ESC must cancel a live worker without modifying the existing route result.
    original_objects = set(scene.objects.keys())
    original_collection = settings.path_collection
    job = jobs.start(scene, addon.__file__)
    settings.contour_running = True
    harness = ModalHarness(job)
    assert harness.step('ESC') == {'CANCELLED'}
    assert_cleaned(job, harness)
    assert set(scene.objects.keys()) == original_objects
    assert settings.path_collection == original_collection
    print('CONTOUR_MODAL_ESC_CLEANUP_OK', flush=True)

    # A genuinely invalid empty scene must report worker failure, keep its log,
    # and still release process/timer/task state. It must not append a result.
    empty = bpy.data.scenes.new('ModalEmptyFailureFixture')
    empty.gs_colmap_settings.floorplan_method = 'CONTOUR'
    job = jobs.start(empty, addon.__file__)
    empty.gs_colmap_settings.contour_running = True
    harness = ModalHarness(job)
    assert wait_modal(harness) == {'CANCELLED'}, harness.reports
    assert_cleaned(job, harness, keep_logs=True)
    assert len(empty.objects) == 0 and empty.gs_colmap_settings.path_collection is None
    assert any('ERROR' in level for level, _ in harness.reports)
    assert (job['root'] / 'worker.log').is_file()
    print('CONTOUR_MODAL_FAILURE_CLEANUP_OK', flush=True)
    # It is an owned job directory; use production cleanup after checking logs.
    jobs.dispose(job)
    bpy.data.scenes.remove(empty)
finally:
    addon.unregister()

print('CONTOUR_MODAL_REGRESSION_OK', flush=True)
