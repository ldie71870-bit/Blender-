"""Reproduce partial installs, stale class identities, repeated reload and rollback."""
import importlib
import importlib.util
import sys
from pathlib import Path
import bpy

ROOT = Path(__file__).resolve().parents[1]
name = 'gs_lifecycle_upgrade_test'
spec = importlib.util.spec_from_file_location(name, ROOT / '__init__.py', submodule_search_locations=[str(ROOT)])
addon = importlib.util.module_from_spec(spec)
sys.modules[name] = addon
spec.loader.exec_module(addon)

def registered(cls):
    return addon.addon_lifecycle._registered(cls)

def check_live():
    for cls in addon.classes:
        assert registered(cls) is cls, cls.__name__
    for handlers, func in ((bpy.app.handlers.load_pre, addon.coverage_heat.reset),
                           (bpy.app.handlers.depsgraph_update_post, addon.coverage_heat.changed)):
        assert sum(f.__module__ == func.__module__ and f.__name__ == func.__name__ for f in handlers) == 1
    assert hasattr(bpy.types.Scene, 'gs_colmap_settings')
    assert hasattr(bpy.types.Scene, 'gs_mesh_guided_settings')

def check_clean():
    assert all(registered(cls) is None for cls in addon.classes)
    assert not hasattr(bpy.types.Scene, 'gs_colmap_settings')
    assert not hasattr(bpy.types.Scene, 'gs_mesh_guided_settings')
    for key in dir(bpy.app.handlers):
        handlers = getattr(bpy.app.handlers, key)
        if isinstance(handlers, list):
            assert not any(getattr(f, '__module__', '').startswith(name + '.') for f in handlers), key

# Exact screenshot: only old coverage operators remain after incomplete uninstall.
for cls in addon.coverage_heat.CLASSES:
    bpy.utils.register_class(cls)
try:
    addon._register_impl()
except ValueError as exc:
    assert 'already registered' in str(exc)
else:
    raise AssertionError('Failed to reproduce original duplicate registration')
addon.register(); check_live()
print('PARTIAL_INSTALL_RECOVERED', flush=True)

settings = bpy.context.scene.gs_colmap_settings
settings.coverage_brush_radius = .731
settings.coverage_patch_budget = 7
for _ in range(3):
    addon.register(); check_live()
    assert abs(bpy.context.scene.gs_colmap_settings.coverage_brush_radius - .731) < 1e-5
    assert bpy.context.scene.gs_colmap_settings.coverage_patch_budget == 7
print('REPEATED_REGISTER_SETTINGS_PRESERVED', flush=True)

# Old unregister failing before the class loop must not strand anything.
cache = addon.coverage_heat.coverage_cache
old = cache.unregister
def fail(_heat): raise RuntimeError('injected cache cleanup failure')
cache.unregister = fail
addon.unregister(); check_clean()
addon.unregister(); check_clean()
cache.unregister = old
addon.register(); check_live()
print('FAILED_CACHE_CLEANUP_RECOVERED', flush=True)

# New Python class identities while actual old operators stay registered.
old_class = addon.coverage_heat.CLASSES[0]
importlib.reload(addon.coverage_heat)
assert addon.coverage_heat.CLASSES[0] is not old_class
spec.loader.exec_module(addon)
addon.register(); check_live()
print('STALE_CLASS_IDENTITIES_RECOVERED', flush=True)

# register rollback after class registration, before mesh-guided registration.
addon.unregister()
old_register = addon.mesh_guided.register
def fail_register(): raise RuntimeError('injected install failure')
addon.mesh_guided.register = fail_register
try:
    addon.register()
except RuntimeError as exc:
    assert 'injected install failure' in str(exc)
else:
    raise AssertionError('Fault was not injected')
check_clean()
addon.mesh_guided.register = old_register
addon.register(); check_live()
addon.unregister(); check_clean()
print('LIFECYCLE_UPGRADE_REGRESSION_OK', flush=True)
