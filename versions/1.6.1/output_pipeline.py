"""Output selection and dependency planning for the AutoGS renderer.

This module deliberately has no Blender dependency so the output graph can be
unit-tested without launching Blender.
"""

from dataclasses import dataclass


RGB = "rgb"
SCENE_DEPTH = "scene_depth"
OBJECT_DEPTH = "object_depth"
SCENE_NORMAL = "scene_normal"
OBJECT_NORMAL = "object_normal"
OBJECT_MASK = "object_mask"
OBJECT_ID = "object_id"
MATERIAL_ID = "material_id"

PASS_BEAUTY = "beauty"
PASS_DEPTH = "depth"
PASS_NORMAL = "normal"
PASS_OBJECT_ID = "object_id"
PASS_MATERIAL_ID = "material_id"

VIRTUAL_SPLIT = "VIRTUAL_SPLIT"
PHYSICAL_FILES = "PHYSICAL_FILES"


@dataclass(frozen=True)
class RenderOutputConfig:
    rgb: bool = True
    scene_depth: bool = False
    object_depth: bool = False
    scene_normal: bool = False
    object_normal: bool = False
    object_mask: bool = False
    object_id: bool = False
    material_id: bool = False
    object_split_mode: str = VIRTUAL_SPLIT

    @classmethod
    def from_settings(cls, settings):
        return cls(
            rgb=bool(getattr(settings, "render_rgb", True)),
            scene_depth=bool(getattr(settings, "export_depth", False)),
            object_depth=bool(getattr(settings, "export_object_depth", False)),
            scene_normal=bool(getattr(settings, "export_normal", False)),
            object_normal=bool(getattr(settings, "export_object_normal", False)),
            object_mask=bool(getattr(settings, "export_object_mask", False)),
            object_id=bool(getattr(settings, "export_id", False)),
            material_id=bool(getattr(settings, "export_material_id", False)),
            object_split_mode=str(getattr(settings, "object_split_mode", VIRTUAL_SPLIT)),
        )

    def requested_outputs(self):
        values = {
            RGB: self.rgb,
            SCENE_DEPTH: self.scene_depth,
            OBJECT_DEPTH: self.object_depth,
            SCENE_NORMAL: self.scene_normal,
            OBJECT_NORMAL: self.object_normal,
            OBJECT_MASK: self.object_mask,
            OBJECT_ID: self.object_id,
            MATERIAL_ID: self.material_id,
        }
        return frozenset(name for name, enabled in values.items() if enabled)


@dataclass(frozen=True)
class ResolvedOutputPlan:
    config: RenderOutputConfig
    requested_outputs: frozenset
    required_internal_passes: frozenset
    persisted_buffers: frozenset

    @property
    def physical_split(self):
        return self.config.object_split_mode == PHYSICAL_FILES

    @property
    def needs_object_groups(self):
        return PASS_OBJECT_ID in self.required_internal_passes

    @property
    def has_outputs(self):
        return bool(self.requested_outputs)

    def saves(self, output_name):
        return output_name in self.persisted_buffers


def resolve_required_passes(config):
    """Resolve render passes separately from user-visible output selections.

    Virtual object outputs persist their scene buffer and object-ID buffer because
    those files *are* the runtime representation of the requested object output.
    Physical outputs keep dependency buffers temporary unless explicitly selected.
    """
    requested = config.requested_outputs()
    required = set()
    if config.rgb:
        required.add(PASS_BEAUTY)
    if config.scene_depth or config.object_depth:
        required.add(PASS_DEPTH)
    if config.scene_normal or config.object_normal:
        required.add(PASS_NORMAL)
    if config.object_id or config.object_depth or config.object_normal or config.object_mask:
        required.add(PASS_OBJECT_ID)
    if config.material_id:
        required.add(PASS_MATERIAL_ID)

    persisted = set(requested)
    if config.object_split_mode == VIRTUAL_SPLIT:
        persisted.difference_update({OBJECT_DEPTH, OBJECT_NORMAL, OBJECT_MASK})
        if config.object_depth:
            persisted.add(SCENE_DEPTH)
        if config.object_normal:
            persisted.add(SCENE_NORMAL)
        if config.object_depth or config.object_normal or config.object_mask:
            persisted.add(OBJECT_ID)

    return ResolvedOutputPlan(
        config=config,
        requested_outputs=requested,
        required_internal_passes=frozenset(required),
        persisted_buffers=frozenset(persisted),
    )


def output_manifest(plan):
    config = plan.config
    return {
        "rgb": config.rgb,
        "scene_depth": config.scene_depth,
        "object_depth": config.object_depth,
        "scene_normal": config.scene_normal,
        "object_normal": config.object_normal,
        "object_mask": config.object_mask,
        "object_id": config.object_id,
        "material_id": config.material_id,
        "internal_object_id_used": PASS_OBJECT_ID in plan.required_internal_passes,
        "object_split_mode": "virtual" if not plan.physical_split else "physical",
        "normal_space": "world",
        "normal_range": [-1.0, 1.0],
        "normal_format": "PNG_UINT16_RAW_RGB_ENCODED",
        "depth_space": "camera_view_z",
        "depth_unit": "meter",
        "depth_invalid_value": 0,
        "mask_values": [0, 255],
        "required_internal_passes": sorted(plan.required_internal_passes),
        "persisted_buffers": sorted(plan.persisted_buffers),
    }
