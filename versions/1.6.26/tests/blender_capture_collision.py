"""Real beveled paths cannot obstruct clearance; real architecture still does."""
import importlib.util
import inspect
import sys
import time
from pathlib import Path
from types import MethodType, SimpleNamespace

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('capture_collision_test', ROOT / '__init__.py',
    submodule_search_locations=[str(ROOT)])
addon = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = addon
spec.loader.exec_module(addon)
addon.register()
collision = addon.capture_collision
scene = bpy.context.scene
settings = scene.gs_colmap_settings
settings.live_update_cameras = False
for obj in list(scene.objects): bpy.data.objects.remove(obj, do_unlink=True)


def box(name, location, scale):
    bpy.ops.mesh.primitive_cube_add(size=1., location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    return obj


def tube(name, x, y, radius=.04, collection=None):
    data = bpy.data.curves.new(name + '_Data', 'CURVE')
    data.dimensions = '3D'
    data.bevel_depth = radius
    data.bevel_resolution = 2
    data.use_fill_caps = True
    spline = data.splines.new('POLY')
    spline.points.add(1)
    spline.points[0].co = (x, y - .5, 1., 1.)
    spline.points[1].co = (x, y + .5, 1., 1.)
    obj = bpy.data.objects.new(name, data)
    (collection or scene.collection).objects.link(obj)
    return obj


wall = box('TrueWall', (5., 0., 1.5), (.2, 12., 3.))
box('Floor', (2., 0., -.1), (8., 12., .2))
box('Ceiling', (2., 0., 3.1), (8., 12., .2))
paths = []
for i in range(96):
    obj = tube(f'OverlappingPreview_{i:03d}', .2 + i * .025, 0., radius=.055)
    obj['gs_contour_version'] = '1.6.12'
    paths.append(obj)
role_only = tube('RoleOnlyPath', 3., 0.)
role_only['gs_route_role'] = 'DETAIL'
paths.append(role_only)
manual = bpy.data.collections.new('UserSelectedManualPaths')
scene.collection.children.link(manual)
for i in range(52):
    paths.append(tube(f'Handmade_{i:03d}', .4 + i * .06, 0., radius=.06, collection=manual))
settings.path_collection = manual
decoration = tube('RealDecorativePipe', 1.2, 4., radius=.12)
settings.rig_mode = 'PATH'
settings.path_count_mode = 'COUNT'
settings.camera_count = 50
settings.path_capture_mode = 'LEGACY_PANORAMA_CUBE'
settings.path_station_array_mode = 'SPHERICAL_SHELL_12'
settings.station_plan_enabled = False
settings.shell_radius_mode = 'FIXED'
settings.shell_radius = .05
settings.shell_surface_clearance = .005
settings.shell_surface_detail_enabled = False
settings.show_shell_debug_mesh = False
for index, obj in enumerate(paths):
    obj.hide_render = index % 2 == 0
    obj.show_in_front = index % 3 == 0
    obj.select_set(index % 5 == 0)
bpy.context.view_layer.objects.active = paths[0]
bpy.context.view_layer.update()


def display_state():
    return (bpy.context.view_layer.objects.active,
            [(obj.name, obj.hide_get(), obj.hide_viewport, obj.hide_render, obj.show_in_front,
              obj.select_get(), obj.data.bevel_depth, tuple(tuple(p.co) for p in obj.data.splines[0].points))
             for obj in paths])


original_display = display_state()
dep = bpy.context.evaluated_depsgraph_get()
token = collision.begin(scene, settings)
try:
    hit = addon._capture_ray_cast(scene, dep, Vector((0., 0., 1.)), Vector((1., 0., 0.)), 6.)
    assert hit[0] and hit[4] == wall and abs(hit[1].x - 4.9) < 1e-4, hit
    assert not addon._capture_ray_cast(scene, dep, Vector((0., 0., 1.)), Vector((1., 0., 0.)), 4.)[0]
    hit = addon._capture_ray_cast(scene, dep, Vector((0., 4., 1.)), Vector((1., 0., 0.)), 6.)
    assert hit[0] and hit[4] == decoration, 'A real decorative curve was silently removed from collision'
    assert not addon._shell_position_is_safe(scene, dep, Vector((4.89, 2., 1.)), .05), 'Real wall clearance accepted'
    assert addon._shell_position_is_safe(scene, dep, Vector((2., 0., 1.)), .05), 'Display paths obstructed station clearance'
finally:
    collision.end(token)
assert display_state() == original_display
# Standalone callers must use the same exclusions without opening a job.
hit = addon._capture_ray_cast(scene, dep, Vector((0., 0., 1.)), Vector((1., 0., 0.)), 6.)
assert hit[0] and hit[4] == wall and abs(hit[1].x - 4.9) < 1e-4
# Isolate role-only and manually assigned paths so each exemption is exercised
# independently of the contour-version stack in front of them.
hit = addon._capture_ray_cast(scene, dep, Vector((2.9, 0., 1.)), Vector((1., 0., 0.)), 3.)
assert hit[0] and hit[4] == wall
assert display_state() == original_display
print('CAPTURE_COLLISION_149_PATHS_REAL_WALL_AND_CURVE_OK', flush=True)

begin, end = collision.begin, collision.end
opened, closed = [], []
def tracked_begin(*args, **kwargs):
    token = begin(*args, **kwargs)
    opened.append(token)
    return token
def tracked_end(token):
    closed.append(token)
    return end(token)
collision.begin, collision.end = tracked_begin, tracked_end


class ModalHarness:
    def __init__(self):
        self._timer = object()
        self.removed = []
        self.reports = []
    def __getattr__(self, name):
        value = inspect.getattr_static(addon.GSCOLMAP_OT_create_cameras, name)
        if isinstance(value, staticmethod): return value.__func__
        return MethodType(value, self) if inspect.isfunction(value) else value
    def report(self, level, message): self.reports.append((level, message))


class Context:
    def __init__(self, harness):
        self.window_manager = SimpleNamespace(progress_update=lambda *_: None,
            progress_end=lambda: None, event_timer_remove=lambda token: harness.removed.append(token))
    def __getattr__(self, name): return getattr(bpy.context, name)


try:
    cameras = addon.create_rig(scene, settings)
    assert len(cameras) == 600, len(cameras)
    assert len({obj.get('station_index') for obj in cameras}) == 50
    assert len(opened) == len(closed) and len(opened) >= 1, (opened, closed)
    assert all(token in closed for token in opened)
    assert display_state() == original_display
    print('CAPTURE_COLLISION_50_STATIONS_600_CAMERAS_OK', flush=True)

    harness = ModalHarness()
    context = Context(harness)
    settings.camera_build_active = True
    settings.camera_build_cancelable = True
    harness._commit_started = False
    harness._prepare(context)
    assert len(harness._sampled) == 50
    deadline = time.monotonic() + 20.
    while harness._index == 0 and time.monotonic() < deadline:
        harness._generate_station_slice(context)
    assert harness._index == 1 and len(harness._cameras) == 12
    assert len(opened) == len(closed) + 1, 'Modal generation did not own a collision scope'
    old_timer = harness._timer
    assert harness.modal(context, SimpleNamespace(type='ESC')) == {'RUNNING_MODAL'}
    for _ in range(100):
        result = harness.modal(context, SimpleNamespace(type='TIMER'))
        if result == {'CANCELLED'}: break
    else: raise AssertionError('Modal cancellation did not complete')
    assert harness.removed == [old_timer]
    assert not settings.camera_build_active
    assert len(opened) == len(closed) and all(token in closed for token in opened)
    assert all(obj.name in scene.objects for obj in cameras), 'Cancellation removed prior formal cameras'
    assert not any(obj.name.startswith('GS_BUILD_') for obj in scene.objects)
    assert display_state() == original_display
    print('CAPTURE_COLLISION_MODAL_SCOPE_CANCEL_OK', flush=True)

    # Closed scopes cannot freeze geometry for later operations.
    wall.location.x += 1.
    bpy.context.view_layer.update()
    next_token = collision.begin(scene, settings)
    try:
        hit = addon._capture_ray_cast(scene, bpy.context.evaluated_depsgraph_get(),
            Vector((0., 0., 1.)), Vector((1., 0., 0.)), 7.)
        assert hit[0] and hit[4] == wall and abs(hit[1].x - 5.9) < 1e-4
    finally: collision.end(next_token)
    assert len(opened) == len(closed)
    print('CAPTURE_COLLISION_NEXT_JOB_SEES_GEOMETRY_CHANGE_OK', flush=True)
finally:
    collision.begin, collision.end = begin, end
    addon.unregister()

print('CAPTURE_COLLISION_REGRESSION_OK', flush=True)
