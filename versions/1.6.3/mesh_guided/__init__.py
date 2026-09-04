"""Mesh-Guided Gaussian Splatting extension for the existing GS exporter."""


def register():
    from . import operators, properties
    properties.register()
    operators.register()


def unregister():
    from . import operators, properties
    operators.unregister()
    properties.unregister()


def draw_panel(layout, context):
    from .panel import draw_mesh_guided_panel
    draw_mesh_guided_panel(layout, context)

