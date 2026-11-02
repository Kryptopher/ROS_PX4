"""Exercise completion guards without requiring a ROS installation."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

SOURCE = Path(__file__).parents[1] / 'zed_px4_bridge' / 'mission_executor_dds.py'


class CompletionShutdownTest(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(SOURCE.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                   and n.name in ('shutdown_after_completion', 'run_completion_shutdown_script',
                                  'request_save_and_handover')]
        self.process = Mock()
        namespace = dict(time=SimpleNamespace(monotonic=lambda: 100),
                         subprocess=self.process, os=SimpleNamespace(environ={}),
                         VehicleStatus=SimpleNamespace(NAVIGATION_STATE_POSCTL=2,
                                                       NAVIGATION_STATE_OFFBOARD=14))
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)
        self.run_shutdown = namespace['shutdown_after_completion']
        self.request_handover = namespace['request_save_and_handover']
        self.node = SimpleNamespace(
            reported_mission_done=True, completion_shutdown_script='/stop.sh',
            completion_shutdown_started=False, completion_requested_at=90,
            vehicle_status=SimpleNamespace(nav_state=2), status_received_at=99,
            control_relinquished=False, position_request_at=99,
            relinquish_control=Mock(), request_completion_position=Mock(),
            manual_shutdown_requested=False, publish_event=Mock(),
            is_armed_and_offboard=Mock(return_value=True),
            get_logger=Mock())
        self.node.run_completion_shutdown_script = (
            lambda: namespace['run_completion_shutdown_script'](self.node))

    def test_confirmed_position_stops_once(self):
        self.run_shutdown(self.node)
        self.run_shutdown(self.node)
        self.process.Popen.assert_called_once()
        self.node.relinquish_control.assert_called_once()

    def test_no_shutdown_for_hover_stale_status_or_other_modes(self):
        for field, value in [('reported_mission_done', False),
                             ('status_received_at', 95),
                             ('status_received_at', 89),
                             ('vehicle_status', None),
                             ('vehicle_status', SimpleNamespace(nav_state=14)),
                             ('vehicle_status', SimpleNamespace(nav_state=4))]:
            with self.subTest(field=field, value=value):
                old = getattr(self.node, field)
                setattr(self.node, field, value)
                self.run_shutdown(self.node)
                self.process.Popen.assert_not_called()
                setattr(self.node, field, old)

    def test_operator_request_in_offboard_waits_for_position(self):
        self.node.reported_mission_done = False
        self.node.vehicle_status.nav_state = 14
        self.node.position_request_at = 99
        self.request_handover(self.node)
        self.node.request_completion_position.assert_called_once()
        self.run_shutdown(self.node)
        self.process.Popen.assert_not_called()
        self.node.relinquish_control.assert_not_called()
        self.node.vehicle_status.nav_state = 2
        self.node.status_received_at = 101
        self.run_shutdown(self.node)
        self.process.Popen.assert_called_once()

    def test_operator_request_when_not_in_offboard_stops_now(self):
        self.node.is_armed_and_offboard.return_value = False
        self.request_handover(self.node)
        self.request_handover(self.node)
        self.node.request_completion_position.assert_not_called()
        self.process.Popen.assert_called_once()

    def test_operator_request_accepts_pilot_selected_mode(self):
        self.node.manual_shutdown_requested = True
        self.node.vehicle_status.nav_state = 4
        self.run_shutdown(self.node)
        self.process.Popen.assert_called_once()

    def test_rejected_position_retries_without_relinquishing(self):
        self.node.vehicle_status.nav_state = 14
        self.node.position_request_at = 98
        self.run_shutdown(self.node)
        self.node.request_completion_position.assert_called_once()
        self.node.relinquish_control.assert_not_called()
        self.process.Popen.assert_not_called()


if __name__ == '__main__':
    unittest.main()
