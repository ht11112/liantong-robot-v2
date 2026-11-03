import subprocess
import unittest
from unittest.mock import Mock

from rk_static_runtime import finish_sensor, scan_evidence_in_window


class RuntimeCleanupTests(unittest.TestCase):
    def test_normal_exit_never_sends_signals(self):
        process = Mock()
        process.wait.return_value = 0
        result = finish_sensor(process, 2)
        self.assertEqual(result['sensor_exit_code'], 0)
        process.terminate.assert_not_called()
        process.kill.assert_not_called()

    def test_stuck_observer_is_reaped_after_term_then_kill(self):
        process = Mock()
        process.wait.side_effect = [subprocess.TimeoutExpired('sensor', 2),
                                    subprocess.TimeoutExpired('sensor', 5), -9]
        result = finish_sensor(process, 2)
        self.assertEqual(result['sensor_exit_code'], -9)
        self.assertTrue(result['sensor_forced_kill'])
        process.terminate.assert_called_once()
        process.kill.assert_called_once()

    def test_cleanup_error_is_recorded_so_report_can_still_be_written(self):
        process = Mock()
        process.wait.side_effect = subprocess.TimeoutExpired('sensor', 2)
        process.terminate.side_effect = OSError('termination failed')
        result = finish_sensor(process, 2)
        self.assertTrue(result['sensor_cleanup_errors'])

    def test_scans_must_have_several_sources_from_the_observation_window(self):
        result = dict(elapsed_s=30., channels={name: dict(missing=False, new_sources=10,
                      latest=dict(source_age_s=.1, future_source_stamp=False))
                      for name in ('scan', 'front180')})
        self.assertTrue(scan_evidence_in_window(result))
        result['channels']['scan']['latest']['source_age_s'] = 10000
        self.assertFalse(scan_evidence_in_window(result))

    def test_missing_or_future_scan_cannot_complete_integration(self):
        self.assertFalse(scan_evidence_in_window(dict(elapsed_s=30, channels={})))
        result = dict(elapsed_s=30, channels={name: dict(missing=False, new_sources=10,
                      latest=dict(source_age_s=-.1, future_source_stamp=True))
                      for name in ('scan', 'front180')})
        self.assertFalse(scan_evidence_in_window(result))


if __name__ == '__main__':
    unittest.main()
