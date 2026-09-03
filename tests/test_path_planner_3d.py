import importlib.util
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "blender_gs_colmap_exporter" / "path_planner_3d.py"
SPEC = importlib.util.spec_from_file_location("gs_path_planner_3d_test", MODULE_PATH)
planner = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = planner
SPEC.loader.exec_module(planner)


def cell(x, y, z, layer=None, clearance=1.5, surface_id="", portal_candidate=False):
    layer = int(round(z * 100)) if layer is None else layer
    return planner.WalkableCell(
        (x, y, layer), (float(x), float(y), float(z + 1.6)),
        float(z), clearance, surface_id, portal_candidate,
    )


class MultiLevelPlannerTests(unittest.TestCase):
    def config(self, **overrides):
        values = dict(
            grid_spacing=1.0,
            floor_tolerance_m=0.06,
            maximum_connector_step_m=0.36,
            minimum_floor_cells=4,
            stitch_distance_m=1.10,
            smoothing_iterations=2,
            large_room_area_m2=1000.0,
        )
        values.update(overrides)
        return planner.PlannerConfig(**values)

    def test_two_floors_and_straight_stair_are_continuous(self):
        cells = [cell(x, y, 0.0) for x in range(4) for y in range(3)]
        cells += [cell(x, y, 0.30 * (x - 3)) for x in range(4, 10) for y in range(3)]
        cells += [cell(x, y, 1.80) for x in range(10, 14) for y in range(3)]
        result = planner.plan_walkable_paths(cells, self.config())
        self.assertEqual(2, len(result.regions))
        self.assertEqual(1, len(result.connectors))
        self.assertEqual("STAIR", result.connectors[0].kind)
        self.assertTrue(any(len(fragment.region_ids) == 2 for fragment in result.final_fragments))
        stair = result.connectors[0].points
        self.assertTrue(all(right[2] >= left[2] for left, right in zip(stair, stair[1:])))
        self.assertTrue(all(abs(right[2] - left[2]) <= 0.36 + 1e-9 for left, right in zip(stair, stair[1:])))

    def test_same_xy_floors_never_stitch(self):
        cells = [cell(x, 0, 0.0, 0) for x in range(5)]
        cells += [cell(x, 0, 3.0, 30) for x in range(5)]
        result = planner.plan_walkable_paths(cells, self.config(stitch_distance_m=5.0))
        self.assertEqual(2, len(result.regions))
        self.assertEqual(0, len(result.connectors))
        self.assertEqual(2, len(result.final_fragments))
        self.assertTrue(all(len(fragment.region_ids) == 1 for fragment in result.final_fragments))

    def test_s_corridor_is_smoothed_without_right_angle_only_output(self):
        route = []
        route += [(x, 0) for x in range(5)]
        route += [(4, y) for y in range(1, 5)]
        route += [(x, 4) for x in range(3, -1, -1)]
        route += [(0, y) for y in range(5, 9)]
        cells = [cell(x, y, 0.0) for x, y in route]
        result = planner.plan_walkable_paths(cells, self.config(minimum_floor_cells=2))
        path = max(result.final_fragments, key=lambda item: len(item.points)).points
        angles = []
        for index in range(1, len(path) - 1):
            before = planner._sub(path[index], path[index - 1])
            after = planner._sub(path[index + 1], path[index])
            angles.append(math.degrees(planner._angle(before, after)))
        self.assertGreater(len(path), len(route))
        self.assertLess(max(angles), 90.0)
        self.assertTrue(any(abs(point[0] - round(point[0])) > 1e-6 and abs(point[1] - round(point[1])) > 1e-6 for point in path))

    def test_fragment_stitching_reduces_fragment_count(self):
        fragments = [
            planner.PathFragment("a", [(0, 0, 1.6), (1, 0, 1.6)], ("Floor_0",)),
            planner.PathFragment("b", [(1.1, 0, 1.6), (2, 0, 1.6)], ("Floor_0",)),
            planner.PathFragment("c", [(2.1, 0, 1.6), (3, 0, 1.6)], ("Floor_0",)),
        ]
        stitched = planner.stitch_fragments(fragments, self.config())
        self.assertLess(len(stitched), len(fragments))
        self.assertEqual(1, len(stitched))

    def test_void_is_not_bridged(self):
        cells = [cell(x, 0, 0.0) for x in range(4)]
        cells += [cell(x, 0, 0.0) for x in range(8, 12)]
        result = planner.plan_walkable_paths(cells, self.config(stitch_distance_m=10.0))
        self.assertEqual(2, len(result.regions))
        self.assertEqual(2, len(result.final_fragments))
        for fragment in result.final_fragments:
            self.assertFalse(any(left[0] < 5 < right[0] for left, right in zip(fragment.points, fragment.points[1:])))

    def test_reachable_seed_excludes_disconnected_cavity(self):
        room = [cell(x, y, 0.0, 0) for x in range(4) for y in range(2)]
        cavity = [cell(x, y, 0.0, 0) for x in range(10, 14) for y in range(2)]
        result = planner.plan_walkable_paths(
            room + cavity,
            self.config(),
            reachable_seed_key=(1, 0, 0),
        )
        self.assertEqual(8, result.stats["walkable_cell_count"])
        self.assertEqual(8, result.stats["excluded_unreachable_cell_count"])
        self.assertEqual(1, len(result.regions))
        self.assertTrue(all(point[0] < 5.0 for fragment in result.final_fragments for point in fragment.points))

    def test_reachable_seed_keeps_stair_connected_floors(self):
        cells = [cell(x, y, 0.0) for x in range(4) for y in range(3)]
        cells += [cell(x, y, 0.30 * (x - 3)) for x in range(4, 10) for y in range(3)]
        cells += [cell(x, y, 1.80) for x in range(10, 14) for y in range(3)]
        result = planner.plan_walkable_paths(
            cells,
            self.config(),
            reachable_seed_key=(1, 0, 0),
        )
        self.assertEqual(2, len(result.regions))
        self.assertEqual(1, len(result.connectors))
        self.assertEqual(0, result.stats["excluded_unreachable_cell_count"])

    def test_seed_room_reaches_second_room_through_door(self):
        left = [cell(x, y, 0.0) for x in range(4) for y in range(4)]
        doorway = [cell(4, 1, 0.0), cell(5, 1, 0.0)]
        right = [cell(x, y, 0.0) for x in range(6, 10) for y in range(4)]
        result = planner.plan_walkable_paths(
            left + doorway + right,
            self.config(),
            reachable_seed_key=(1, 1, 0),
        )
        self.assertEqual(len(left) + len(doorway) + len(right), result.stats["walkable_cell_count"])
        self.assertEqual(0, result.stats["excluded_unreachable_cell_count"])
        self.assertTrue(any(point[0] >= 6.0 for fragment in result.final_fragments for point in fragment.points))

    def test_real_door_portal_keeps_two_rooms_and_is_reported(self):
        left = [cell(x, y, 0.0) for x in range(5) for y in range(5)]
        door = [cell(5, 2, 0.0, clearance=0.35, portal_candidate=True)]
        right = [cell(x, y, 0.0) for x in range(6, 11) for y in range(5)]
        result = planner.plan_walkable_paths(
            left + door + right,
            self.config(room_minimum_lanes=2, coverage_target_ratio=0.80),
            reachable_seed_key=(1, 2, 0),
        )
        self.assertEqual(0, result.stats["excluded_unreachable_cell_count"])
        self.assertGreaterEqual(result.stats["room_region_count"], 2)
        self.assertGreaterEqual(result.stats["portal_count"], 1)
        self.assertTrue(any(fragment.kind in {"PORTAL", "STITCHED"} for fragment in result.final_fragments))

    def test_three_rooms_connected_by_two_real_doors(self):
        rooms = []
        for start in (0, 6, 12):
            rooms.extend(cell(x, y, 0.0) for x in range(start, start + 5) for y in range(5))
        doors = [
            cell(5, 2, 0.0, clearance=0.35, portal_candidate=True),
            cell(11, 2, 0.0, clearance=0.35, portal_candidate=True),
        ]
        result = planner.plan_walkable_paths(
            rooms + doors,
            self.config(room_minimum_lanes=2),
            reachable_seed_key=(1, 2, 0),
        )
        self.assertEqual(0, result.stats["excluded_unreachable_cell_count"])
        self.assertGreaterEqual(result.stats["room_region_count"], 3)
        self.assertGreaterEqual(result.stats["portal_count"], 2)
        self.assertTrue(any(point[0] >= 12.0 for fragment in result.final_fragments for point in fragment.points))

    def test_large_room_has_multiple_coverage_lanes_and_target_coverage(self):
        room = [cell(x, y, 0.0) for x in range(12) for y in range(9)]
        result = planner.plan_walkable_paths(
            room,
            self.config(room_minimum_lanes=3, coverage_radius_m=1.0, coverage_target_ratio=0.85),
            reachable_seed_key=(1, 1, 0),
            stitch_fragments_enabled=False,
        )
        room_lanes = [fragment for fragment in result.raw_fragments if fragment.kind == "ROOM_COVERAGE"]
        self.assertGreaterEqual(len(room_lanes), 3)
        self.assertGreaterEqual(result.stats["path_spatial_coverage_ratio"], 0.85)
        self.assertEqual(0, result.stats["uncovered_room_count"])

    def test_upper_floor_is_excluded_without_stairs(self):
        lower = [cell(x, y, 0.0, 0) for x in range(4) for y in range(3)]
        upper = [cell(x, y, 2.4, 24) for x in range(8, 12) for y in range(3)]
        result = planner.plan_walkable_paths(
            lower + upper,
            self.config(),
            reachable_seed_key=(1, 1, 0),
        )
        self.assertEqual(len(lower), result.stats["walkable_cell_count"])
        self.assertEqual(len(upper), result.stats["excluded_unreachable_cell_count"])
        self.assertEqual(1, len(result.regions))
        self.assertEqual(0, len(result.connectors))
        self.assertTrue(all(point[2] < 2.0 for fragment in result.final_fragments for point in fragment.points))

    def test_final_smoothing_remains_inside_reachable_mask(self):
        route = [cell(x, 0, 0.0) for x in range(5)]
        route += [cell(4, y, 0.0) for y in range(1, 5)]
        result = planner.plan_walkable_paths(
            route,
            self.config(minimum_floor_cells=2, smoothing_iterations=4),
            reachable_seed_key=(0, 0, 0),
        )
        mask_validator = planner._walkable_mask_validator(
            {item.key: item for item in route}, self.config(minimum_floor_cells=2)
        )
        self.assertTrue(all(
            mask_validator(left, right)
            for fragment in result.final_fragments
            for left, right in zip(fragment.points, fragment.points[1:])
        ))

    def test_graph_edge_validator_blocks_wall_crossing(self):
        cells = [cell(x, 0, 0.0) for x in range(8)]

        def no_wall_crossing(left, right):
            return not (left[0] <= 3.0 and right[0] >= 4.0)

        result = planner.plan_walkable_paths(
            cells,
            self.config(),
            edge_validator=no_wall_crossing,
            reachable_seed_key=(1, 0, 0),
        )
        self.assertEqual(4, result.stats["walkable_cell_count"])
        self.assertEqual(4, result.stats["excluded_unreachable_cell_count"])
        self.assertTrue(all(point[0] <= 3.0 for fragment in result.final_fragments for point in fragment.points))

    def test_narrow_false_ramp_does_not_connect_seed_room_to_cavity(self):
        room = [cell(x, y, 0.0, 0) for x in range(8) for y in range(4)]
        false_ramp = [
            cell(x, y, -0.30 * (x - 7))
            for x in range(8, 14) for y in range(2)
        ]
        cavity = [cell(x, 0, -1.80) for x in range(14, 18)]
        result = planner.plan_walkable_paths(
            room + false_ramp + cavity,
            self.config(),
            reachable_seed_key=(1, 1, 0),
        )
        self.assertEqual(1, len(result.regions))
        self.assertEqual(0, len(result.connectors))
        self.assertEqual(32, result.stats["walkable_cell_count"])
        self.assertTrue(all(point[2] > 1.0 for fragment in result.final_fragments for point in fragment.points))

    def test_flat_adjacent_surfaces_are_separate_reachable_regions(self):
        intended = [cell(x, 0, 0.0, surface_id="main_floor") for x in range(4)]
        shell = [cell(x, 0, 0.0, surface_id="outer_shell") for x in range(4, 8)]
        result = planner.plan_walkable_paths(
            intended + shell,
            self.config(),
            reachable_seed_key=(1, 0, 0),
        )
        self.assertEqual(1, len(result.regions))
        self.assertEqual(4, result.stats["walkable_cell_count"])
        self.assertTrue(all(point[0] < 4.0 for fragment in result.final_fragments for point in fragment.points))

    def test_scientific_default_is_one_capture_camera(self):
        source = (ROOT / "blender_gs_colmap_exporter" / "__init__.py").read_text(encoding="utf-8")
        self.assertIn('scientific_realization_mode: EnumProperty(name="Realization Backend", items=SCIENTIFIC_REALIZATION_MODE_ITEMS, default="SCIENTIFIC_POSE_SEQUENCE")', source)
        self.assertIn('cameras = [capture]', source)
        self.assertIn('for camera in list(cameras):\n            _remove_camera(camera)', source)

    def test_multilevel_route_does_not_depend_on_capture_mode(self):
        source = (ROOT / "blender_gs_colmap_exporter" / "__init__.py").read_text(encoding="utf-8")
        start = source.index("def build_floorplan_path(")
        body = source[start:source.index("\ndef ", start + 10)]
        self.assertIn('if bool(getattr(settings, "multilevel_planning", True)):', body)
        self.assertNotIn('path_capture_mode == "SCIENTIFIC_THREE_LAYER"', body)

    def test_arc_length_sampling_densifies_turns_and_stairs(self):
        straight = planner.sample_arc_length([(0, 0, 0), (8, 0, 0)], 1.0)
        bent = planner.sample_arc_length([(0, 0, 0), (4, 0, 0), (4, 4, 2)], 1.0)
        self.assertGreater(len(bent), len(straight))
        self.assertTrue(any(sample.critical for sample in bent[1:-1]))
        self.assertTrue(any(sample.on_connector for sample in bent))


if __name__ == "__main__":
    unittest.main()
