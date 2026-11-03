import unittest

from feedback_watchdog import FeedbackWatchdog


class FeedbackTests(unittest.TestCase):
    def test_750ms_boundary(self):
        guard = FeedbackWatchdog(750)
        self.assertTrue(guard.observe(1, 0., 0.))
        self.assertTrue(guard.status(.749)["healthy"])
        self.assertEqual(guard.status(.75)["reason"], "feedback_timeout")

    def test_duplicate_never_refreshes_source(self):
        guard = FeedbackWatchdog(750)
        guard.observe(12, 0., 0.)
        self.assertFalse(guard.observe(12, .7, .7))
        self.assertEqual(guard.status(.75)["source_stamp"], 0.)
        self.assertTrue(guard.status(.75)["stop_required"])

    def test_late_new_sample_does_not_erase_timeout(self):
        guard = FeedbackWatchdog(750)
        guard.observe(1, 0., 0.)
        self.assertFalse(guard.observe(2, .8, .8))
        self.assertFalse(guard.status(.8)["start_allowed"])

    def test_progressing_sources_and_sequence_wrap(self):
        guard = FeedbackWatchdog(750)
        guard.observe(0xffffffff, 0., 0.)
        self.assertTrue(guard.observe(0, .5, .5))
        self.assertTrue(guard.status(1.)["healthy"])

    def test_missing_at_start_and_fault_are_distinct(self):
        guard = FeedbackWatchdog(750)
        self.assertFalse(guard.status(0.)["start_allowed"])
        guard.observe(1, .1, .1)
        self.assertEqual(guard.fault("driver_fault", .11)["reason"], "driver_fault")
        self.assertTrue(guard.status(.11)["stop_required"])

    def test_invalid_and_reordered_input(self):
        for bad in (True, float("nan"), float("inf"), -1, 0, 10 ** 1000):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                FeedbackWatchdog(bad)
        guard = FeedbackWatchdog(750)
        guard.observe(10, 0., 0.)
        self.assertFalse(guard.observe(9, .1, .1))
        with self.assertRaises(ValueError):
            guard.observe(11, .3, .2)


if __name__ == "__main__":
    unittest.main()
