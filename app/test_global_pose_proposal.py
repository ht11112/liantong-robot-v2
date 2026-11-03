"""Synthetic, offline proposal tests; NumPy stays in an independent directory."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

try:
    import numpy as np
except ImportError:
    _numpy_dir = Path(__file__).resolve().parents[2] / "tmp/project-review-20261027/test-deps"
    sys.path.insert(0, str(_numpy_dir))
    import numpy as np

from global_pose_proposal import (global_pose_proposal, map_identity, pgm_cells,
                                  scan_points, verified_map, endpoint_agreement)


def saved_scene(asymmetric=True):
    info = dict(width=80, height=80, resolution=.1, origin=[-4., -4., 0.])
    grid = np.full((80, 80), -1, dtype=int)
    grid[10:71, 10:71] = 0
    grid[10, 10:71] = grid[70, 10:71] = 100
    grid[10:71, 10] = grid[10:71, 70] = 100
    if asymmetric:
        grid[49:57, 53:59] = 100
        grid[25:29, 21:34] = 100
        grid[60:63, 25:29] = 100
    return pack(info, grid)


def pack(info, grid):
    cells = [int(v) for v in grid.ravel()]
    doc = dict(info=info, data=cells)
    identity = map_identity(info, cells)
    pixels = bytes(254 if int(v) == 0 else 0 if int(v) >= 65 else 205
                   for v in grid[::-1].ravel())
    pgm = ("P5\n%d %d\n255\n" % (info["width"], info["height"])).encode() + pixels
    return doc, pgm, identity


def ray_scan(doc, pose, stamp=100., extrinsics=(.13, 0., 0.)):
    info, cells = doc["info"], doc["data"]
    width, height, resolution = info["width"], info["height"], info["resolution"]
    ox, oy, _ = info["origin"]
    x, y, yaw = pose
    ex, ey, eyaw = extrinsics
    lx = x + math.cos(yaw) * ex - math.sin(yaw) * ey
    ly = y + math.sin(yaw) * ex + math.cos(yaw) * ey
    ranges = []
    for index in range(360):
        a = -math.pi + index * 2 * math.pi / 360 + yaw + eyaw
        distance = None
        for value in np.arange(.05, 8., .025):
            gx = int(math.floor((lx + value * math.cos(a) - ox) / resolution))
            gy = int(math.floor((ly + value * math.sin(a) - oy) / resolution))
            if not 0 <= gx < width or not 0 <= gy < height:
                break
            if cells[gy * width + gx] >= 65:
                distance = float(value)
                break
        ranges.append(distance)
    secs = int(stamp)
    return dict(header=dict(stamp_secs=secs, stamp_nsecs=int(round((stamp - secs) * 1e9)),
                            source_stamp=stamp, frame_id="p2p_laser"),
                raw_header=dict(frame_id="map"), navigation_fov_deg=180,
                angle_min=-math.pi, angle_increment=2 * math.pi / 360,
                range_min=.05, range_max=8., ranges=ranges)


def snapshots(doc, pose):
    return [ray_scan(doc, pose, stamp=100. + i * .1) for i in range(3)]


class GlobalProposalTests(unittest.TestCase):
    def test_full_map_search_refines_all_modes_and_finds_pose_far_from_origin(self):
        doc, pgm, identity = saved_scene()
        truth = [1.2, -1.1, math.radians(73)]
        result = global_pose_proposal(doc, pgm, {"snapshots": snapshots(doc, truth)}, identity)
        self.assertEqual(result["refined_mode_count"], 5)
        self.assertTrue(result["search"]["every_selected_distinct_mode_refined"])
        self.assertLess(math.hypot(result["pose"][0] - truth[0], result["pose"][1] - truth[1]), .15)
        self.assertLess(abs(math.atan2(math.sin(result["pose"][2] - truth[2]),
                                      math.cos(result["pose"][2] - truth[2]))), math.radians(3))
        self.assertTrue(result["seed_candidate_accepted"], result)
        self.assertTrue(all(r["used"] >= 60 and r["near_wall_ratio"] >= .65
                            for r in result["frame_agreement"]))
        self.assertEqual([r["source_stamp"] for r in result["frame_agreement"]], [100., 100.1, 100.2])
        self.assertFalse(result["localization_verified"])
        self.assertFalse(result["device_permit"])
        self.assertFalse(result["source_freshness_revalidated"])
        self.assertLess(result["search"]["score_matrix_bytes"], 80 * 80 * 72 * 8)

    def test_symmetric_room_remains_ambiguous_after_fair_refinement(self):
        doc, pgm, identity = saved_scene(False)
        result = global_pose_proposal(doc, pgm, snapshots(doc, [0., 0., math.radians(25)]), identity)
        self.assertFalse(result["seed_candidate_accepted"])
        self.assertLess(result["mode_score_margin"], .08)
        self.assertEqual(result["refined_mode_count"], 5)

    def test_unknown_endpoint_cells_do_not_match_nearby_occupied_cells(self):
        info = dict(width=50, height=50, resolution=.1, origin=[-2.5, -2.5, 0.])
        grid = np.full((50, 50), -1, dtype=int)
        grid[24:27, 24:27] = 0
        for row in range(50):
            for col in range(50):
                radius = math.hypot((col + .5) * .1 - 2.5, (row + .5) * .1 - 2.5)
                if 1.35 < radius < 1.45:
                    grid[row, col] = 100
        doc, pgm, identity = pack(info, grid)
        scan = dict(angle_min=-math.pi, angle_increment=2 * math.pi / 360,
                    range_min=.05, range_max=8., ranges=[1.] * 360)
        scans = [dict(scan, stamp_s=100. + i) for i in range(3)]
        result = global_pose_proposal(doc, pgm, scans, identity, refine_radius_m=.05)
        self.assertFalse(result["seed_candidate_accepted"])
        self.assertTrue(all(row["near_wall_ratio"] < .65 for row in result["frame_agreement"]))

    def test_unknown_endpoint_does_not_borrow_a_neighboring_wall_match(self):
        info = dict(width=3, height=3, resolution=.1, origin=[0., 0., 0.])
        cells = [-1] * 9
        cells[3], cells[5] = 0, 100
        # Endpoint in middle unknown cell, one 0.1m cell from occupied wall.
        result = endpoint_agreement(info, cells, [(0.1, 0.)], [.05, .15, 0.])
        self.assertEqual(result["matched"], 0)
        cells[4] = 0
        known = endpoint_agreement(info, cells, [(0.1, 0.)], [.05, .15, 0.])
        self.assertEqual(known["matched"], 1)
        cells[4] = 50  # Canonical map_saver identity treats this as unknown.
        self.assertEqual(endpoint_agreement(info, cells, [(0.1, 0.)], [.05, .15, 0.])["matched"], 0)

    def test_wrong_map_id_and_changed_pgm_are_rejected_before_search(self):
        doc, pgm, identity = saved_scene()
        with self.assertRaisesRegex(ValueError, "expected saved map"):
            global_pose_proposal(doc, pgm, [], "0" * 64)
        changed = pgm[:-1] + bytes([254])
        with self.assertRaisesRegex(ValueError, "identities differ"):
            verified_map(doc, changed, identity)
        malformed = dict(doc, data=[True] + doc["data"][1:])
        with self.assertRaises(ValueError):
            verified_map(malformed, pgm, identity)

    def test_snapshot_mounting_must_match_explicit_proposal_extrinsics(self):
        doc, pgm, identity = saved_scene()
        scans = snapshots(doc, [1.2, -1.1, math.radians(73)])
        for changes in (dict(x_m=.14), dict(y_m=.01), dict(yaw_rad=.1),
                        dict(x_m=True), dict(y_m=float("nan")), dict(yaw_rad=None)):
            mounting = dict(x_m=.13, y_m=0., yaw_rad=0.)
            mounting.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                global_pose_proposal(doc, pgm, dict(snapshots=scans, mounting=mounting), identity)
        context = dict(snapshots=scans, mounting=dict(x_m=.13, y_m=0., yaw_rad=0.),
                       frames=dict(laser="p2p_laser"), boot_id="current-boot", nonce="current-nonce")
        result = global_pose_proposal(doc, pgm, context, identity)
        self.assertTrue(result["seed_candidate_accepted"])
        self.assertEqual((result["boot_id"], result["nonce"]), ("current-boot", "current-nonce"))

    def test_snapshot_laser_frame_and_all_scan_header_frames_must_match(self):
        doc, pgm, identity = saved_scene()
        scans = snapshots(doc, [0., 0., 0.])
        for frame in ("other_laser", "", None):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                global_pose_proposal(doc, pgm, dict(snapshots=scans, frames=dict(laser=frame)), identity)
        scans[1]["header"]["frame_id"] = "mixed_laser"
        with self.assertRaisesRegex(ValueError, "snapshot laser frame"):
            global_pose_proposal(doc, pgm, dict(snapshots=scans, frames=dict(laser="p2p_laser")), identity)
        with self.assertRaisesRegex(ValueError, "header frames"):
            global_pose_proposal(doc, pgm, scans, identity)

    def test_repeated_or_reversed_scan_sources_are_rejected(self):
        doc, pgm, identity = saved_scene()
        scan = ray_scan(doc, [0., 0., 0.])
        with self.assertRaisesRegex(ValueError, "different advancing"):
            global_pose_proposal(doc, pgm, [scan] * 3, identity)
        with self.assertRaisesRegex(ValueError, "different advancing"):
            global_pose_proposal(doc, pgm, list(reversed(snapshots(doc, [0., 0., 0.]))), identity)

    def test_snapshot_special_values_and_exact_source_stamp_are_preserved(self):
        scan = dict(header=dict(stamp_secs=1790000000, stamp_nsecs=7,
                                source_stamp=1790000000., frame_id="laser"),
                    raw_header=dict(frame_id="map"), navigation_fov_deg=180,
                    angle_min=-math.pi, angle_increment=2 * math.pi / 360,
                    range_min=.05, range_max=8., ranges=[1.] * 360)
        scan["ranges"][185:190] = [None, "inf", "-inf", float("nan"), float("inf")]
        points, source = scan_points(scan, (.13, 0., 0.), 180)
        self.assertGreaterEqual(len(points), 60)
        self.assertEqual(source["exact_source_key"], (1790000000, 7))
        self.assertEqual(source["original_header"], scan["header"])
        self.assertEqual(source["raw_header"], scan["raw_header"])

    def test_explicit_extrinsics_are_applied_to_all_endpoint_coordinates(self):
        scan = dict(stamp_s=1., angle_min=-math.pi / 2, angle_increment=math.pi / 180,
                    range_min=.05, range_max=8., ranges=[1.] * 181)
        old, _ = scan_points(scan, (.13, 0., 0.), 180)
        moved, _ = scan_points(scan, (.23, -.1, 0.), 180)
        np.testing.assert_allclose(moved - old, np.tile([.1, -.1], (len(old), 1)), atol=1e-12)

    def test_pgm_parser_preserves_whitespace_first_pixel_and_rejects_extra_bytes(self):
        width, height, cells = pgm_cells(b"P5\n2 1\n255\n" + bytes([10, 254]))
        self.assertEqual((width, height, cells), (2, 1, [-1, 0]))
        with self.assertRaises(ValueError):
            pgm_cells(b"P5\n2 1\n255\n" + bytes([10, 254, 0]))


if __name__ == "__main__":
    unittest.main()
