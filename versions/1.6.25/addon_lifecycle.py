"""Idempotent extension teardown, including partially registered old classes."""
import sys
import bpy


def _attempt(label, function, *args):
    try:
        function(*args)
    except Exception as exc:
        # Cleanup must continue after a failed cache save or stale RNA handle.
        print('[GS lifecycle]', label, str(exc))


def _owned(name, package):
    return name == package or name.startswith(package + '.')


def _registered(cls):
    """Use actual RNA registration, not the identity of newly imported classes."""
    for base in (bpy.types.Operator, bpy.types.Panel, bpy.types.PropertyGroup,
                 bpy.types.UIList, bpy.types.Menu):
        if issubclass(cls, base):
            identifier = getattr(getattr(cls, 'bl_rna', None), 'identifier', '')
            # Unregistered classes inherit their base's bl_rna identifier.
            names = [cls.__name__]
            if identifier and identifier != base.__name__:
                names.append(identifier)
            if base is bpy.types.Operator:
                operator = getattr(cls, 'bl_idname', '')
                if '.' in operator:
                    left, right = operator.split('.', 1)
                    names.append(left.upper() + '_OT_' + right)
            for name in names:
                actual = base.bl_rna_get_subclass_py(name, None)
                if actual is not None and actual is not base:
                    return actual
    return None


def cleanup(namespace):
    package = namespace['__name__']
    heat = namespace.get('coverage_heat') or sys.modules.get(package + '.coverage_heat')
    if heat is not None:
        # Save while scene properties still exist. Failure cannot strand classes.
        _attempt('coverage shutdown', heat.unregister)
        handle = getattr(heat, 'HANDLE', None)
        if handle is not None:
            _attempt('coverage draw', bpy.types.SpaceView3D.draw_handler_remove, handle, 'WINDOW')
            heat.HANDLE = None
    capture = namespace.get('capture_collision') or sys.modules.get(package + '.capture_collision')
    if capture is not None:
        _attempt('camera collision shutdown', capture.shutdown)
    jobs = namespace.get('contour_jobs') or sys.modules.get(package + '.contour_jobs')
    if jobs is not None:
        _attempt('path shutdown', jobs.shutdown)

    for scene in tuple(getattr(bpy.data, 'scenes', ())):
        for name in ('_restore_path_inspection', '_restore_station_plan_view'):
            function = namespace.get(name)
            if function is not None:
                _attempt(name, function, scene)
        function = namespace.get('sync_native_camera_sizes')
        if function is not None and hasattr(scene, 'gs_colmap_settings'):
            _attempt('native camera sizes', function, scene, False)

    for key, space in (('_CAMERA_OVERLAY_HANDLE', bpy.types.SpaceView3D),
                       ('_STATION_PLAN_IMAGE_HANDLE', bpy.types.SpaceImageEditor)):
        handle = namespace.get(key)
        if handle is not None:
            _attempt(key, space.draw_handler_remove, handle, 'WINDOW')
            namespace[key] = None
    for keymap, item in namespace.get('_PATH_INSPECTION_KEYMAPS', ()):
        _attempt('keymap', keymap.keymap_items.remove, item)
    namespace.get('_PATH_INSPECTION_KEYMAPS', []).clear()

    # Match handler ownership, so functions from a previous import are removed.
    for name in dir(bpy.app.handlers):
        handlers = getattr(bpy.app.handlers, name)
        if isinstance(handlers, list):
            for function in tuple(handlers):
                if _owned(getattr(function, '__module__', ''), package):
                    handlers.remove(function)
    modules = [module for name, module in tuple(sys.modules.items())
               if module is not None and _owned(name, package)]
    for module in modules:
        for function in tuple(vars(module).values()):
            if callable(function) and _owned(getattr(function, '__module__', ''), package):
                try:
                    if bpy.app.timers.is_registered(function):
                        bpy.app.timers.unregister(function)
                except (TypeError, ValueError):
                    pass
    menu = bpy.types.TOPBAR_MT_render
    functions = getattr(menu.draw, '_draw_funcs', ())
    for function in tuple(functions):
        if _owned(getattr(function, '__module__', ''), package):
            _attempt('menu', menu.remove, function)

    guided = namespace.get('mesh_guided')
    if guided is not None:
        # Its legacy unregister aborts when only part of its classes exists.
        operators = sys.modules.get(package + '.mesh_guided.operators')
        if operators is not None:
            operators._ACTIVE_RUNNER = None
    # Removing RNA descriptors leaves scene ID properties/settings intact.
    for name in ('gs_colmap_settings', 'gs_mesh_guided_settings'):
        if hasattr(bpy.types.Scene, name):
            delattr(bpy.types.Scene, name)
    candidates = list(namespace.get('classes', ()))
    for module in modules:
        candidates.extend(value for value in tuple(vars(module).values())
                          if isinstance(value, type) and _owned(value.__module__, package)
                          and issubclass(value, (bpy.types.Operator, bpy.types.Panel,
                              bpy.types.PropertyGroup, bpy.types.UIList, bpy.types.Menu)))
    seen = set()
    for cls in reversed(candidates):
        actual = _registered(cls)
        if actual is not None and actual not in seen and _owned(actual.__module__, package):
            seen.add(actual)
            _attempt('unregister ' + actual.__name__, bpy.utils.unregister_class, actual)


def before_reload(namespace):
    # Must run before module globals overwrite the old draw/keymap handles.
    if namespace.get('classes'):
        cleanup(namespace)
