import bpy,sys,importlib.util
from pathlib import Path
root=Path(__file__).resolve().parents[1];spec=importlib.util.spec_from_file_location('undo_test',root/'__init__.py',submodule_search_locations=[str(root)])
a=importlib.util.module_from_spec(spec);sys.modules[spec.name]=a;spec.loader.exec_module(a);a.register();h=a.coverage_heat
st=dict(selected={1},batch=None,dirty=False);h.STATES[bpy.context.scene.as_pointer()]=st
h.begin_stroke(st);st['selected'].add(2);st['selected'].add(3);h.finish_stroke(st)
h.begin_stroke(st);st['selected'].discard(1);st['selected'].discard(2);h.finish_stroke(st)
assert len(st['brush_history'])==2
assert bpy.ops.gs_colmap.undo_coverage_brush()=={'FINISHED'} and st['selected']=={1,2,3}
assert h.undo_stroke(st) and st['selected']=={1}
h.begin_stroke(st);h.finish_stroke(st);assert not st['brush_history']
h.begin_stroke(st);st['selected'].add(4);assert h.undo_stroke(st) and st['selected']=={1}
assert not h.undo_stroke(st)
for i in range(60):h.begin_stroke(st);st['selected'].add(i+10);h.finish_stroke(st)
assert len(st['brush_history'])==50
print('BRUSH_UNDO_OK: whole strokes, erase restore, empty stroke, active stroke, 50-step cap')
a.unregister()
