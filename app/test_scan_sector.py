import math
import unittest

from scan_sector import front_180_scan


class ScanTests(unittest.TestCase):
    def test_front_boundary_and_rear_unknown(self):
        raw = [1., 2., 3., 4., 5.]
        result = front_180_scan(raw, -math.pi, math.pi / 2, 123.5)
        self.assertTrue(math.isnan(result["ranges"][0]))
        self.assertTrue(math.isnan(result["ranges"][-1]))
        self.assertEqual(result["ranges"][1:4], [2., 3., 4.])
        self.assertEqual(raw, [1., 2., 3., 4., 5.])
        self.assertEqual(result["source_stamp"], 123.5)

    def test_mounting_yaw_and_negative_increment(self):
        result = front_180_scan([1., 2., 3.], 0., -math.pi / 2, 1., math.pi)
        self.assertTrue(math.isnan(result["ranges"][0]))
        self.assertEqual(result["ranges"][1:], [2., 3.])

    def test_input_unknown_and_clear_beams_preserved(self):
        result = front_180_scan([float("nan"), float("inf")], -.1, .1, 1.)
        self.assertTrue(math.isnan(result["ranges"][0]))
        self.assertTrue(math.isinf(result["ranges"][1]))

    def test_reject_invalid_metadata(self):
        for kwargs in ({"angle_increment": 0}, {"source_stamp": -1},
                       {"source_stamp": float("nan")}, {"angle_min": True},
                       {"angle_increment": 10 ** 1000}, {"ranges": [True]}):
            args = dict(ranges=[1.], angle_min=0., angle_increment=.1, source_stamp=1.)
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                front_180_scan(**args)


if __name__ == "__main__":
    unittest.main()
