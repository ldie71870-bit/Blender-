import shutil
from pathlib import Path

from ..utils.json_io import atomic_write_json, read_json
from ..utils.paths import numbered_name
from .gbuffer_renderer import id_png_to_npy


class DatasetFinalizer:
    def __init__(self, root, camera_records, validation_ids):
        self.root = Path(root)
        self.records = list(camera_records)
        self.validation_ids = set(int(value) for value in validation_ids)
        self.index = 0
        self.images = sorted(path for path in (self.root / "images").glob("*.*") if path.is_file())
        self.depth = sorted(path for path in (self.root / "depth").glob("*.*") if path.is_file())
        self.ids = sorted(path for path in (self.root / "id").glob("*.png") if path.is_file())
        self.id_map = read_json(self.root / "id" / "id_map.json", {"items": []})

    @property
    def total(self):
        return len(self.records)

    def step(self):
        if self.index >= len(self.records):
            self._write_metadata()
            return True
        camera_id = int(self.records[self.index]["id"])
        split = "validation" if camera_id in self.validation_ids else "train"
        if self.index < len(self.images):
            source = self.images[self.index]
            target = self.root / "images" / split / numbered_name(camera_id, source.suffix.lower())
            shutil.copy2(source, target)
        if self.index < len(self.depth):
            source = self.depth[self.index]
            target = self.root / "depth" / split / numbered_name(camera_id, source.suffix.lower())
            shutil.copy2(source, target)
        if self.index < len(self.ids):
            target = self.root / "object_id" / split / numbered_name(camera_id, ".npy")
            id_png_to_npy(self.ids[self.index], target, self.id_map)
        self.index += 1
        done = self.index >= len(self.records)
        if done:
            self._write_metadata()
        return done

    def _write_metadata(self):
        atomic_write_json(self.root / "config" / "ground_truth_format.json", {
            "schema_version": "1.0",
            "rgb": {"path": "images/{train,validation}/NNNNNN.png", "color_space": "sRGB", "alpha_is_mask": True},
            "depth": {"path": "depth/{train,validation}/NNNNNN.exr", "semantic": "camera_positive_z", "dtype": "float32", "invalid": "alpha <= 0.5"},
            "normal": {"path": "normal/{train,validation}/NNNNNN.exr", "space": "camera", "encoding": "signed_normal = rgb * 2 - 1", "invalid": "alpha <= 0.99"},
            "object_id": {"path": "object_id/{train,validation}/NNNNNN.npy", "dtype": "int32", "background": -1},
            "triangle_id": {
                "available": True,
                "path": "triangle_id/{train,validation}/NNNNNN.npy",
                "dtype": "int64",
                "background": -1,
                "semantic": "row index in mesh/training_mesh.npz triangles",
            },
        })

