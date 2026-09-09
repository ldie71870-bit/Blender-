"""Regression checks for a surface cursor with bounded interactive work."""
import importlib.util,sys,types
from pathlib import Path
from mathutils import Vector
from mathutils.bvhtree import BVHTree

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('brush_interaction_addon',ROOT/'__init__.py',submodule_search_locations=[str(ROOT)])
addon=importlib.util.module_from_spec(spec);sys.modules[spec.name]=addon;spec.loader.exec_module(addon)
heat=addon.coverage_heat

# Idle events and sub-pixel jitter are rejected before view-ray construction.
assert heat.mouse_sample_due(None,(10,10),0.,0.)
assert not heat.mouse_sample_due((10,10),(11,10),0.,1.)
assert not heat.mouse_sample_due((10,10),(14,10),0.,.01)
assert heat.mouse_sample_due((10,10),(14,10),0.,.04)
assert heat.mouse_sample_due((10,10),(10,10),0.,0.,True)

# Cursor geometry is projected to the real surface and includes a small centre cross.
bvh=BVHTree.FromPolygons([(-2,-2,0),(2,-2,0),(2,2,0),(-2,2,0)],[(0,1,2,3)])
st=dict(scale=1.,probe=types.SimpleNamespace(geometry=(bvh,[],[])))
origin=Vector((0,0,2));direction=Vector((0,0,-1));hit_info=heat.surface_hit(st,origin,direction)
hit=heat.surface_cursor(st,origin,direction,.35,False,7,hit_info)
assert hit is not None and st['cursor']['area']==7
assert len(st['cursor']['lines'])==52
assert all(abs(Vector(p).z-.004)<.003 for p in st['cursor']['lines'])

# The modal brush has no private timer, so an idle mouse does no periodic BVH work.
source=(ROOT/'coverage_heat.py').read_text(encoding='utf8')
brush=source[source.index('class GS_OT_coverage_brush'):source.index('class GS_OT_undo_brush')]
assert 'event_timer_add' not in brush and "event.type=='TIMER'" not in brush
assert 'hit_info=surface_hit' in brush and 'paint(st,origin,direction,radius,event.shift,hit_info)' in brush
print('BRUSH_INTERACTION_OK: surface ring+cross, 30 Hz/2 px gate, one reusable hit, no idle timer')
