"""ROS/実機なしの模擬フローテスト。実機での統合試験の代わりではありません。"""
import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

from mission_logic import CommandHeading, classify_regions


def stub_module(name, **attrs):
    module = types.ModuleType(name)
    module.__dict__.update(attrs)
    sys.modules[name] = module


class Goal:
    def __init__(self):
        self.time_allowance = NS(sec=0, nanosec=0)


# このテストプロセス内でのみROS/OpenCVを代替する。
for name in ('cv2', 'rclpy', 'action_msgs', 'geometry_msgs', 'nav2_msgs',
             'rcl_interfaces', 'std_srvs', 'std_msgs'):
    stub_module(name)
stub_module('rclpy.node', Node=object)
stub_module('rclpy.action', ActionClient=object)
stub_module('rclpy.parameter', Parameter=object)
stub_module('rclpy.signals', SignalHandlerOptions=NS(NO=0))
stub_module('action_msgs.msg', GoalStatus=NS(STATUS_SUCCEEDED=4))
stub_module('geometry_msgs.msg', PoseStamped=object)
stub_module('nav2_msgs.action', **{name: NS(Goal=Goal) for name in ('FollowWaypoints', 'NavigateToPose', 'Spin')})
stub_module('rcl_interfaces.msg', Parameter=object, ParameterValue=object,
            ParameterType=NS(PARAMETER_BOOL=1))
stub_module('rcl_interfaces.srv', GetParameters=object, SetParameters=object)
stub_module('std_srvs.srv', Trigger=NS(Request=object))
stub_module('std_msgs.msg', String=object)
spec = importlib.util.spec_from_file_location('mission_test_target', Path(__file__).with_name('mission.py'))
mission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mission)


class FakeMission(mission.Mission):
    def __init__(self):
        self.events = []
        self.cfg = {'routes': {letter: letter + '.yaml' for letter in 'abcd'}}
        self.heading = CommandHeading()
        self.frame = 'map'
        self.images = []
        self.run_dir = Path('test_images')
        self.follow, self.navigate, self.spin_client = 'follow', 'navigate', 'spin'
        self.state = 'STARTING'
        self.confirmed = False
        self.hooks = NS(
            choose_box_image=lambda records, ctx: records[2] if len(records) == 6 else records[0],
            target_command_heading=lambda rec, ctx: rec['command_deg'],
            box_goal=lambda rec, ctx: {'x': 5., 'y': 6., 'yaw': 0.},
            box_visible=lambda path, ctx: True,
        )
        self.next_client = NS(call_async=lambda req: NS(success=True, message='next'))

    def get_logger(self): return Mock()
    def preflight(self): return Path('routes'), {'x': 1., 'y': 0.}
    def set_state(self, state):
        self.state = state
        self.events.append(('state', state))
    def pose(self, point): return point
    def load_route(self, path):
        self.events.append(('route', path.name))
        return [path.name]
    def start_action(self, client, goal, feedback=None):
        self.events.append(('start', client))
    def finish_action(self): return NS(missed_waypoints=[])
    def rotate(self, degrees):
        self.events.append(('spin', degrees))
        self.heading.commit(degrees)
    def capture(self, label):
        record = {'path': label, 'command_deg': self.heading.degrees}
        self.images.append(record)
        self.events.append(('capture', label, self.heading.degrees))
        return record
    def ocr(self, record): return 'c'
    def layers(self, enabled): self.events.append(('layers', enabled))
    def approach_halfway(self, point): self.events.append(('halfway', point))
    def wait_arrival(self, point):
        self.layers(False)
        self.events.append(('manual_wait', point))
    def wait_future(self, future, timeout=None): return future


class Tests(unittest.TestCase):
    def test_ocr_single_region_filter_and_order(self):
        payload = {'regions': [{'text': 'abc'}, {'text': ' A '}, {'text': ' C '}, {'text': 'a'}]}
        self.assertEqual(classify_regions(payload), 'c')
        for letter in 'abcd':
            self.assertEqual(classify_regions({'regions': [{'text': letter}]}), letter)
        for texts in ([], ['ab'], ['A'], ['B'], ['D'], ['hello']):
            with self.assertRaises(ValueError):
                classify_regions({'regions': [{'text': t} for t in texts]})

    def test_full_flow_and_six_command_angles(self):
        node = FakeMission()
        node.run()
        self.assertIn(('route', 'c.yaml'), node.events)
        captures = [e for e in node.events if e[0] == 'capture' and e[1].startswith('search_')]
        self.assertEqual([e[2] for e in captures], [0, 15, 30, 45, 60, 75])
        spins = [e[1] for e in node.events if e[0] == 'spin']
        self.assertEqual(spins, [90, -180, 15, 15, 15, 15, 15, -45, -30])
        self.assertEqual(node.heading.degrees, 0)
        self.assertEqual([e[1] for e in node.events if e[0] == 'layers'], [False, False, True])
        self.assertEqual(node.events[-1], ('state', 'DONE'))

    def test_no_box_stops_before_approach(self):
        node = FakeMission()
        node.hooks.choose_box_image = lambda records, ctx: None
        with self.assertRaises(RuntimeError): node.run()
        self.assertFalse(any(e[0] == 'layers' for e in node.events))
        self.assertNotIn(('state', 'NEXT_FILE'), node.events)

    def test_missed_waypoint_stops_before_scan(self):
        node = FakeMission()
        count = [0]
        def result():
            count[0] += 1
            return NS(missed_waypoints=[0] if count[0] == 2 else [])
        node.finish_action = result
        with self.assertRaises(RuntimeError): node.run()
        self.assertFalse(any(e[0] == 'capture' and e[1].startswith('search_') for e in node.events))

    def test_midway_right_selection_and_inverse(self):
        node = FakeMission()
        node.heading.commit(30)
        node.hooks.box_visible = lambda path, ctx: False
        node.hooks.choose_box_image = lambda records, ctx: records[1]
        node.midway_check()
        self.assertEqual([r['command_deg'] for r in node.images], [30, 40, 20])
        self.assertEqual(node.heading.degrees, 20)
        node.face_command(0)
        self.assertEqual(node.heading.degrees, 0)
        self.assertEqual(node.events[-1], ('spin', -20))

    def test_midway_left_selection(self):
        node = FakeMission()
        node.heading.commit(30)
        node.hooks.box_visible = lambda path, ctx: False
        node.hooks.choose_box_image = lambda records, ctx: records[0]
        node.midway_check()
        self.assertEqual(node.heading.degrees, 40)
        self.assertEqual(node.events[-1], ('spin', 20))

    def test_midway_no_box(self):
        node = FakeMission()
        node.hooks.box_visible = lambda path, ctx: False
        node.hooks.choose_box_image = lambda records, ctx: None
        with self.assertRaises(RuntimeError): node.midway_check()

    def test_halfway_feedback(self):
        node = FakeMission()
        node.initial_remaining = node.remaining = None
        node.halfway = False
        for value, expected in ((0, False), (10, False), (6, False), (5, True)):
            node.nav_feedback(NS(feedback=NS(distance_remaining=value)))
            self.assertEqual(node.halfway, expected)
        self.assertEqual(node.initial_remaining, 10)

    def test_manual_confirmation_state_guard(self):
        node = FakeMission()
        response = NS(success=None, message='')
        node.confirm_callback(None, response)
        self.assertFalse(response.success)
        self.assertFalse(node.confirmed)
        node.state = 'WAIT_CONFIRM'
        node.confirm_callback(None, response)
        self.assertTrue(response.success)
        self.assertTrue(node.confirmed)

    def test_wait_arrival_requires_person_even_after_abort(self):
        node = FakeMission()
        node.result_future = NS(done=lambda: True, result=lambda: NS(status=6))
        pumps = []
        def pump():
            pumps.append(node.state)
            node.confirm_callback(None, NS(success=False, message=''))
        node.pump = pump
        node.cancel_action = Mock()
        mission.Mission.wait_arrival(node, {'x': 1, 'y': 2})
        self.assertEqual(pumps, ['WAIT_CONFIRM'])
        self.assertEqual(node.state, 'CONFIRMED')
        node.cancel_action.assert_called_once()

    def test_failed_action_never_commits_spin_angle(self):
        node = FakeMission()
        node.cfg['spin_time_allowance'] = 45
        node.settle = lambda: None
        node.finish_action = Mock(side_effect=RuntimeError('Spin aborted'))
        with self.assertRaises(RuntimeError): mission.Mission.rotate(node, 15)
        self.assertEqual(node.heading.degrees, 0)

    def test_failure_restores_both_original_values(self):
        node = FakeMission()
        node.ocr_process = None
        node.original_layers = {'/local': True, '/global': False}
        node.cancel_action = Mock()
        calls = []
        node.layer_call = lambda name, value: calls.append((name, value))
        self.assertEqual(node.cleanup_failure(), [])
        self.assertEqual(calls, [('/local', True), ('/global', False)])
        node.cancel_action.assert_called_once()
        self.assertNotIn(('state', 'NEXT_FILE'), node.events)


if __name__ == '__main__':
    unittest.main(verbosity=2)
