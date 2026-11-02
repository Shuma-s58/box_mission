#!/usr/bin/env python3
"""Bの走行終了後に起動するROS 2 Humble用外部制御ノード。"""
import argparse
import importlib.util
import json
import math
import subprocess
import time
from datetime import datetime
from pathlib import Path

import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose, Spin
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import Trigger
from std_msgs.msg import String

from mission_logic import CommandHeading, classify_regions
from visual_approach import VisualApproach
from approach_logic import planned_travel


class Mission(VisualApproach, Node):
    def __init__(self, config_path):
        config_path = Path(config_path).expanduser().resolve()
        self.config_dir = config_path.parent
        self.cfg = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        super().__init__('box_mission', parameter_overrides=[
            Parameter('use_sim_time', value=bool(self.cfg.get('use_sim_time', False)))])
        self.frame = self.cfg.get('frame_id', 'map')
        self.heading = CommandHeading()
        self.state = 'STARTING'
        self.confirmed = False
        self.handle = None
        self.result_future = None
        self.pending_goal = None
        self.original_layers = {}
        self.layer_clients = {}
        self.images = []
        self.ocr_process = None
        self.last_wp = None
        self.follow = ActionClient(self, FollowWaypoints, self.cfg.get('follow_action', '/follow_waypoints'))
        self.navigate = ActionClient(self, NavigateToPose, self.cfg.get('navigate_action', '/navigate_to_pose'))
        self.spin_client = ActionClient(self, Spin, self.cfg.get('spin_action', '/spin'))
        self.next_client = self.create_client(Trigger, self.cfg.get('next_file_service', '/waypoint_sender_node/next_file'))
        self.confirm_service = self.create_service(Trigger, '~/confirm_arrival', self.confirm_callback)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.run_dir = self.path(self.cfg['image_dir']) / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.run_dir.mkdir(parents=True, exist_ok=False)
        hook_path = self.path(self.cfg.get('hooks_file', 'box_hooks.py'))
        spec = importlib.util.spec_from_file_location('mission_box_hooks', hook_path)
        self.hooks = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.hooks)
        self.setup_visual()

    def path(self, value):
        path = Path(str(value)).expanduser()
        return path.resolve() if path.is_absolute() else (self.config_dir / path).resolve()

    def set_state(self, state):
        self.state = state
        self.state_pub.publish(String(data=state))
        self.get_logger().info(f'STATE: {state}')

    def pump(self):
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped.')
        rclpy.spin_once(self, timeout_sec=0.05)

    def wait_future(self, future, timeout=None):
        start = time.monotonic()
        while not future.done():
            self.pump()
            if timeout is not None and time.monotonic() - start > timeout:
                raise TimeoutError('ROS service/goal response timeout.')
        return future.result()

    def settle(self, duration=None):
        deadline = time.monotonic() + float(self.cfg.get('settle_seconds', 0.5) if duration is None else duration)
        while time.monotonic() < deadline:
            self.pump()

    def context(self):
        return {'frame_id': self.frame, 'command_deg': self.heading.degrees,
                'image_dir': str(self.run_dir), 'images': list(self.images),
                'detect': self.detect_boxes, 'approach': self.ap}

    def confirm_callback(self, request, response):
        if self.state != 'WAIT_CONFIRM':
            response.success = False
            response.message = f'到達確認を受け付ける状態ではありません: {self.state}'
        else:
            self.confirmed = True
            response.success = True
            response.message = '到達確認を受理。停止を維持し、逆回転・レイヤー復帰・next_fileを行います。'
        return response

    def pose(self, point):
        values = [float(point.get(k, 0.0)) for k in ('x', 'y', 'z', 'yaw')]
        if 'x' not in point or 'y' not in point or not all(math.isfinite(v) for v in values):
            raise ValueError('ゴールには有限値のx/yを指定してください。')
        x, y, z, yaw = values
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def load_route(self, path):
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        points = data.get('waypoints') if isinstance(data, dict) else None
        if not isinstance(points, list) or not points:
            raise ValueError(f'空または形式不正のwaypoint YAML: {path}')
        return [self.pose(point) for point in points]

    def start_action(self, client, goal, feedback=None):
        if self.handle is not None:
            raise RuntimeError('このノードのActionがまだ実行中です。')
        self.pending_goal = client.send_goal_async(goal, feedback_callback=feedback)
        handle = self.wait_future(self.pending_goal, self.cfg.get('service_timeout', 10.0))
        self.pending_goal = None
        if handle is None or not handle.accepted:
            raise RuntimeError('Nav2がゴールを拒否しました。')
        self.handle = handle
        self.result_future = handle.get_result_async()
        self.get_logger().info(f'Accepted goal_id={bytes(handle.goal_id.uuid).hex()}')

    def finish_action(self):
        result = self.wait_future(self.result_future)
        self.handle = None
        self.result_future = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Nav2 Actionが正常終了しませんでした: status={result.status}')
        return result.result

    def cancel_action(self):
        if self.handle is None and self.pending_goal is not None:
            handle = self.wait_future(self.pending_goal, self.cfg.get('service_timeout', 10.0))
            self.pending_goal = None
            if handle is not None and handle.accepted:
                self.handle = handle
                self.result_future = handle.get_result_async()
        if self.handle is None:
            return
        if not self.result_future.done():
            reply = self.wait_future(self.handle.cancel_goal_async(), self.cfg.get('service_timeout', 10.0))
            if not reply.goals_canceling and not self.result_future.done():
                raise RuntimeError('Actionのキャンセルが拒否されました。')
        self.wait_future(self.result_future, self.cfg.get('cancel_timeout', 15.0))
        self.handle = None
        self.result_future = None
        self.settle()

    def _rotate_action(self, degrees):
        degrees = float(degrees)
        if abs(degrees) < 1e-8:
            return
        goal = Spin.Goal()
        goal.target_yaw = math.radians(degrees)
        allowance = float(self.cfg.get('spin_time_allowance', 45.0))
        goal.time_allowance.sec = int(allowance)
        goal.time_allowance.nanosec = int((allowance % 1) * 1e9)
        self.get_logger().info(f'Spin指令: {degrees:+.1f} deg')
        self.start_action(self.spin_client, goal)
        self.finish_action()
        self.heading.commit(degrees)
        self.settle()

    def rotate(self, degrees):
        if abs(float(degrees)) < 1e-8:
            return
        self.stop_direct()
        if self.direct_owned:
            self.set_direct(False)
        # 箱の手前の前進停止領域は、角速度のみのSpinでは外す。
        # 内側の車体保護領域とCollision Monitor自体は有効のまま。
        self.front_stop(False)
        try:
            self._rotate_action(degrees)
        finally:
            self.front_stop(True)

    def front_stop(self, enabled):
        node = self.ap['monitor_node'].rstrip('/')
        name = self.ap['front_stop_parameter']
        client = self.create_client(SetParameters, node+'/set_parameters')
        try:
            timeout = float(self.cfg['service_timeout'])
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError('Collision Monitorのパラメータサービスがありません。')
            req = SetParameters.Request()
            req.parameters = [ParameterMsg(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=bool(enabled)))]
            res = self.wait_future(client.call_async(req), timeout)
            if len(res.results) != 1 or not res.results[0].successful:
                raise RuntimeError('前方停止領域の切替失敗')
        finally:
            self.destroy_client(client)

    def face_command(self, target):
        self.rotate(self.heading.delta_to(target))

    def capture_usb(self):
        camera_cfg = self.cfg['camera']
        device = camera_cfg.get('device', '/dev/video0')
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise RuntimeError(f'カメラを開けません: {device}')
            if camera_cfg.get('width'):
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(camera_cfg['width']))
            if camera_cfg.get('height'):
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(camera_cfg['height']))
            for _ in range(int(camera_cfg.get('warmup_frames', 10))):
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('カメラのフレーム取得に失敗しました。')
                self.pump()
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError('写真撮影に失敗しました。')
        finally:
            cap.release()
        return frame

    def capture(self, label):
        self.settle()
        if self.cfg['camera']['source'] == 'ros':
            frame, stamp = self.capture_ros()
        else:
            frame, stamp = self.capture_usb(), {'clock': 'host_capture'}
        path = self.run_dir / f'{len(self.images):02d}_{label}.png'
        if not cv2.imwrite(str(path), frame):
            raise RuntimeError(f'画像保存に失敗: {path}')
        record = {'path': str(path), 'command_deg': self.heading.degrees,
                  'stamp': stamp, 'intrinsics': self.intrinsics(frame.shape[1], frame.shape[0])}
        self.images.append(record)
        (self.run_dir/'images.json').write_text(json.dumps(self.images, indent=2), encoding='utf-8')
        self.get_logger().info(f'Captured: {path}, command={self.heading.degrees:+.1f}')
        return record

    def ocr(self, record):
        cfg = self.cfg['ocr']
        result_path = self.run_dir / 'ocr_result.json'
        cmd = [str(self.path(cfg['python'])) if '/' in str(cfg['python']) else str(cfg['python']),
               str(self.path(cfg['script'])), record['path'],
               '--json', str(result_path), '-o', str(self.run_dir / 'ocr_annotated.png'),
               '--device', str(cfg.get('device', 'auto'))]
        if cfg.get('model'):
            cmd += ['--model', str(cfg['model'])]
        # OCR用venvを別プロセスにしてROSのPython依存と分離する。
        with (self.run_dir / 'ocr.log').open('w', encoding='utf-8') as log:
            self.ocr_process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
            started = time.monotonic()
            while self.ocr_process.poll() is None:
                self.pump()
                if time.monotonic() - started > float(cfg.get('timeout', 300.0)):
                    raise TimeoutError('OCRタイムアウト。ocr.logを確認してください。')
            code = self.ocr_process.returncode
            self.ocr_process = None
        if code != 0:
            raise RuntimeError(f'OCR失敗: exit={code}。ocr.logを確認してください。')
        letter = classify_regions(json.loads(result_path.read_text(encoding='utf-8')))
        self.get_logger().info(f'OCR分岐: {letter}')
        return letter

    def layer_call(self, node, enabled):
        name = 'obstacle_layer.enabled'
        timeout = float(self.cfg.get('service_timeout', 10.0))
        if node not in self.layer_clients:
            get = self.create_client(GetParameters, node + '/get_parameters')
            set_client = self.create_client(SetParameters, node + '/set_parameters')
            self.layer_clients[node] = (get, set_client)
        get, set_client = self.layer_clients[node]
        if not get.wait_for_service(timeout_sec=timeout) or not set_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'costmapパラメータサービスが見つかりません: {node}')
        if node not in self.original_layers:
            req = GetParameters.Request()
            req.names = [name]
            result = self.wait_future(get.call_async(req), timeout)
            if len(result.values) != 1 or result.values[0].type != ParameterType.PARAMETER_BOOL:
                raise RuntimeError(f'{node}: {name}がboolで存在していません。')
            self.original_layers[node] = result.values[0].bool_value
        request = SetParameters.Request()
        request.parameters = [ParameterMsg(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=enabled))]
        response = self.wait_future(set_client.call_async(request), timeout)
        if len(response.results) != 1 or not response.results[0].successful:
            raise RuntimeError(f'パラメータ変更失敗: {node}: {response.results}')
        self.get_logger().info(f'{node}: {name}={enabled}')

    def layers(self, enabled):
        for node in self.cfg.get('costmap_nodes', ['/local_costmap/local_costmap', '/global_costmap/global_costmap']):
            self.layer_call(node.rstrip('/'), enabled)

    def wp_feedback(self, message):
        index = message.feedback.current_waypoint
        if index != self.last_wp:
            self.last_wp = index
            self.get_logger().info(f'Current waypoint: {index} (0-based)')

    def preflight(self):
        route_dir = self.path(self.cfg['route_dir'])
        goal_file = route_dir / self.cfg.get('initial_goal_file', 'goal.yaml')
        data = yaml.safe_load(goal_file.read_text(encoding='utf-8'))
        if not isinstance(data, dict) or not isinstance(data.get('goal'), dict):
            raise ValueError('初期goal YAMLは goal: {x: ..., y: ..., yaw: ...} 形式にしてください。')
        self.pose(data['goal'])
        route_paths = [(route_dir / self.cfg['routes'][letter]).resolve() for letter in 'abcd']
        if len(set(route_paths)) != 4:
            raise ValueError('a/b/c/dには4つの別々の経路ファイルを指定してください。')
        for letter in 'abcd':
            path = route_dir / self.cfg['routes'][letter]
            if path.resolve() == goal_file.resolve():
                raise ValueError('初期goalと分岐経路は別ファイルにしてください。')
            self.load_route(path)
        script = self.path(self.cfg['ocr']['script'])
        if not script.is_file():
            raise ValueError(f'florence2_ocr.pyが見つかりません: {script}')
        for client in (self.follow, self.navigate, self.spin_client):
            if not client.wait_for_server(timeout_sec=float(self.cfg.get('service_timeout', 10.0))):
                raise RuntimeError('Nav2 Actionサーバーが利用できません。')
        if not self.next_client.wait_for_service(timeout_sec=float(self.cfg.get('service_timeout', 10.0))):
            raise RuntimeError('waypoint_sender_node/next_fileサービスが利用できません。')
        if not self.path(self.cfg['detector']['script']).is_file():
            raise ValueError('detector.scriptが見つかりません。')
        self.check_monitor()
        self.wait_condition(lambda: self.gate_status is not None and self.gate_status.get('scan_ok')
                            and time.monotonic()-self.gate_received < float(self.ap['monitor_timeout']),
                            self.cfg['service_timeout'], 'box_command_gate/low_scanが準備できません。')
        if self.gate_status['owner'] != 'nav':
            raise RuntimeError('ゲートがmission所有のままです。前回の停止理由を確認して解除してください。')
        self.wait_condition(lambda: self.odom is not None and time.monotonic()-self.odom_received < float(self.ap['odom_timeout']),
                            self.cfg['service_timeout'], 'オドメトリがありません。')
        publishers = self.get_publishers_info_by_topic(self.ap['monitor_output_topic'])
        expected = self.ap['monitor_node'].rstrip('/').split('/')[-1]
        if len(publishers) != 1 or publishers[0].node_name != expected:
            raise RuntimeError('最終速度トピックのpublisherはCollision Monitorだけにしてください。')
        self.front_stop(True)
        return route_dir, data['goal']

    def run(self):
        route_dir, initial_point = self.preflight()
        self.set_state('INITIAL_GOAL')
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(initial_point)
        self.start_action(self.navigate, goal)
        self.finish_action()
        self.set_state('OCR_CAPTURE')
        self.rotate(90.0)
        letter = self.ocr(self.capture('ocr'))
        self.rotate(-180.0)
        self.set_state('SELECTED_ROUTE')
        goal = FollowWaypoints.Goal()
        goal.poses = self.load_route(route_dir / self.cfg['routes'][letter])
        self.start_action(self.follow, goal, self.wp_feedback)
        result = self.finish_action()
        if result.missed_waypoints:
            raise RuntimeError(f'未到達waypoint: {result.missed_waypoints}')
        # ここが復帰方向の0度。以降はSpinの指令角度のみを累積する。
        self.heading.reset()
        self.set_state('BOX_SEARCH')
        records = []
        for index in range(6):
            records.append(self.capture(f'search_{index}'))
            if index < 5:
                self.rotate(15.0)
        selected = self.hooks.choose_box_image(records, self.context())
        if selected is None:
            raise RuntimeError('6枚の写真にboxが写っていません。')
        if selected not in records:
            raise ValueError('box選択は渡された6枚から選択してください。')
        self.set_state('ALIGN_BOX')
        aligned = self.align_box(selected)
        distance = self.hooks.box_distance(aligned, self.context())
        length = planned_travel(distance, self.ap)
        self.get_logger().info(f'画像距離={distance:.3f} m / 予定接近={length:.3f} m / 半分={length/2:.3f} m')
        self.layers(False)
        result = self.approach_halfway(length)
        if result == 'halfway':
            self.set_state('MIDWAY_CAPTURE')
            aligned = self.midway_check()
            distance = self.hooks.box_distance(aligned, self.context())
            remaining = planned_travel(distance, self.ap) + float(self.ap['final_extra_m'])
            remaining = min(remaining, float(self.ap['max_total_travel_m'])-self.total_travel)
            self.drive_straight(remaining, final=True)
        else:
            self.get_logger().warn('半分地点より前に停止。途中撮影を省略して人の確認を待ちます。')
        self.wait_arrival()
        self.set_state('RETURN_HEADING')
        # 並進やNav2自身の旋回は角度計算に含めず、指令の累積を逆回転する。
        self.face_command(0.0)
        if self.direct_owned:
            self.set_direct(False)
        self.layers(True)
        self.set_state('NEXT_FILE')
        reply = self.wait_future(self.next_client.call_async(Trigger.Request()), self.cfg.get('service_timeout', 10.0))
        if not reply.success:
            raise RuntimeError(f'next_file失敗: {reply.message}')
        self.set_state('DONE')

    def cleanup_failure(self):
        self.set_state('FAILED')
        if self.ocr_process is not None and self.ocr_process.poll() is None:
            self.ocr_process.terminate()
            try:
                self.ocr_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.ocr_process.kill()
                self.ocr_process.wait()
        errors = []
        try:
            self.stop_direct()
        except Exception as error:
            errors.append(f'直進停止失敗: {error}')
        try:
            self.cancel_action()
        except Exception as error:
            errors.append(f'キャンセル失敗: {error}')
        try:
            self.front_stop(True)
        except Exception as error:
            errors.append(f'前方停止領域の復帰失敗: {error}')
        # 異常終了時はゲート所有権を自動解放せず、古いNav2指令への復帰を防ぐ。
        # 変更途中に片方のcostmapだけ失敗した場合も、変更前の値へ戻す。
        for node, original in list(self.original_layers.items()):
            try:
                self.layer_call(node, original)
            except Exception as error:
                errors.append(f'レイヤー復帰失敗: {error}')
        for error in errors:
            self.get_logger().error(error)
        return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).with_name('config.yaml')))
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = Mission(args.config)
        node.run()
    except (Exception, KeyboardInterrupt) as error:
        code = 130 if isinstance(error, KeyboardInterrupt) else 1
        if node is not None:
            node.get_logger().error(f'処理中断: {error}')
            node.cleanup_failure()
        else:
            print(f'初期化失敗: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())