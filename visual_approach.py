"""ROS画像、Florence subprocess、オドメトリ直進制御。Missionの補助クラス。"""
import json
import math
import subprocess
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from lifecycle_msgs.srv import GetState
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool

from approach_logic import positive, planned_travel, progress, yaw_from_quaternion


class VisualApproach:
    def setup_visual(self):
        self.ap = self.cfg['approach']
        for name in ('speed_mps', 'max_total_travel_m', 'box_height_m', 'max_distance_m',
                     'odom_timeout', 'monitor_timeout', 'max_drive_seconds', 'stall_seconds',
                     'max_cross_track_m', 'max_heading_error_deg', 'stop_hold_seconds',
                     'min_bbox_height_px', 'angular_tolerance_deg', 'max_turn_step_deg'):
            positive(self.ap[name], name)
        if float(self.ap['speed_mps']) > float(self.ap['max_speed_mps']):
            raise ValueError('接近速度が設定上限を超えています。')
        camera = self.cfg['camera']
        # 本実装の単眼距離式は水平・前向きカメラ専用。pitch付き画像への誤適用を防ぐ。
        if any(abs(float(camera.get(k, 0))) > 1e-6 for k in ('pitch_rad', 'yaw_rad', 'roll_rad', 'lateral_m')):
            raise ValueError('この版は水平・前向き・車体中心線上のカメラ用です。')
        if camera.get('mirrored', False):
            raise ValueError('鏡像画像は使用できません。入力側で反転を戻してください。')
        self.bridge = CvBridge()
        self.image_msg = self.info_msg = self.odom = self.output = self.gate_status = None
        self.odom_received = self.output_received = self.gate_received = 0.
        self.odom_speed = None
        self.last_odom_stamp = None
        self.odom_frame = None
        self.direct_owned = False
        self.vision_cache = {}
        self.total_travel = 0.
        self.vel_pub = self.create_publisher(Twist, self.ap['cmd_vel_topic'], 1)
        self.gate_client = self.create_client(SetBool, self.ap['gate_service'])
        self.monitor_client = self.create_client(GetState, self.ap['monitor_node'].rstrip('/')+'/get_state')
        self.create_subscription(Odometry, self.ap['odom_topic'], self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, self.ap['monitor_output_topic'], self.output_cb, 1)
        self.create_subscription(String, self.ap['gate_status_topic'], self.gate_cb, 1)
        self.create_subscription(CameraInfo, camera['info_topic'], self.info_cb, qos_profile_sensor_data)
        if camera['source'] == 'ros':
            self.create_subscription(Image, camera['image_topic'], self.image_cb, qos_profile_sensor_data)
        elif camera['source'] != 'usb':
            raise ValueError('camera.source は ros または usb です。')

    def image_cb(self, msg):
        self.image_msg = msg

    def info_cb(self, msg):
        self.info_msg = msg

    def odom_cb(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        # 同一timestampの再配信では鮮度を更新しない。
        if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
            return
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
            return
        frame = (msg.header.frame_id, msg.child_frame_id)
        if self.odom_frame is not None and frame != self.odom_frame:
            raise RuntimeError('接近中にodomフレームが変更されました。')
        self.odom_frame = frame
        self.last_odom_stamp = stamp
        self.odom = (p.x, p.y, yaw)
        twist = msg.twist.twist
        self.odom_speed = (math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z))
        self.odom_received = time.monotonic()

    def output_cb(self, msg):
        self.output = msg
        self.output_received = time.monotonic()

    def gate_cb(self, msg):
        self.gate_status = json.loads(msg.data)
        self.gate_received = time.monotonic()

    def wait_condition(self, condition, timeout, message):
        start = time.monotonic()
        while not condition():
            self.pump()
            if time.monotonic()-start > float(timeout):
                raise TimeoutError(message)

    def check_monitor(self):
        timeout = float(self.cfg['service_timeout'])
        if not self.monitor_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('Collision Monitorのget_stateサービスがありません。')
        state = self.wait_future(self.monitor_client.call_async(GetState.Request()), timeout)
        if state.current_state.id != 3:
            raise RuntimeError('Collision MonitorがACTIVEではありません。')

    def set_direct(self, enabled):
        if enabled:
            self.check_monitor()
        if not self.gate_client.wait_for_service(timeout_sec=float(self.cfg['service_timeout'])):
            raise RuntimeError('box_command_gateが起動していません。')
        request = SetBool.Request()
        request.data = bool(enabled)
        response = self.wait_future(self.gate_client.call_async(request), float(self.cfg['service_timeout']))
        if not response.success:
            raise RuntimeError(f'指令所有権の変更失敗: {response.message}')
        self.direct_owned = bool(enabled)
        if enabled:
            acquired = time.monotonic()
            self.wait_condition(lambda: self.gate_received >= acquired and self.gate_status is not None
                                and self.gate_status.get('owner') == 'mission' and self.gate_status.get('scan_ok'),
                                self.cfg['service_timeout'], 'mission所有権の反映を確認できません。')

    def stop_direct(self):
        # 新しいNav2指令と競合しないよう、このメソッドは所有中だけ呼ぶ。
        if not getattr(self, 'direct_owned', False):
            return
        start = time.monotonic()
        stable_since = None
        while True:
            self.vel_pub.publish(Twist())
            self.pump()
            now = time.monotonic()
            speed = self.odom_speed
            stationary = (speed is not None and now-self.odom_received < float(self.ap['odom_timeout'])
                          and speed[0] < 0.02 and speed[1] < 0.03)
            stable_since = (stable_since if stable_since is not None else now) if stationary else None
            if (stable_since is not None and now-stable_since >= 0.2
                    and now-start >= float(self.ap['stop_settle_seconds'])):
                break
            if now-start > float(self.ap['stop_timeout_seconds']):
                raise TimeoutError('ゼロ指令後の停止をオドメトリで確認できません。')
        self.vel_pub.publish(Twist())

    def capture_ros(self):
        barrier = self.get_clock().now().nanoseconds
        self.image_msg = None
        def fresh():
            m = self.image_msg
            return m is not None and m.header.stamp.sec*10**9+m.header.stamp.nanosec >= barrier
        self.wait_condition(fresh, self.cfg['camera']['timeout'], '停止・回転後の新しい画像を受信できません。')
        msg = self.image_msg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        return frame, {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec,
                       'frame_id': msg.header.frame_id}

    def intrinsics(self, width, height):
        c = self.cfg['camera']
        if c['source'] == 'usb':
            data = dict(c['intrinsics'])
        else:
            self.wait_condition(lambda: self.info_msg is not None, c['timeout'], 'CameraInfoがありません。')
            m = self.info_msg
            if self.image_msg.header.frame_id != m.header.frame_id:
                raise ValueError('ImageとCameraInfoのframe_idが一致しません。')
            if any(abs(v) > 1e-8 for v in m.d):
                raise ValueError('歪みあり画像です。画像を補正し、補正後のCameraInfoを指定してください。')
            data = {'width': m.width, 'height': m.height, 'fx': m.k[0], 'fy': m.k[4], 'cx': m.k[2], 'cy': m.k[5]}
        if [data['width'], data['height']] != [width, height]:
            raise ValueError('画像とCameraInfo/内部パラメータの解像度が違います。')
        positive(data['fx'], 'fx'); positive(data['fy'], 'fy')
        if not (0 <= float(data['cx']) < width and 0 <= float(data['cy']) < height):
            raise ValueError('主点が画像範囲外です。')
        return data

    def detect_boxes(self, image_path):
        if image_path in self.vision_cache:
            return self.vision_cache[image_path]
        cfg = self.cfg['detector']
        image = Path(image_path)
        result = image.with_suffix('.boxes.json')
        cmd = [str(self.path(cfg['python'])) if '/' in cfg['python'] else cfg['python'],
               str(self.path(cfg['script'])), str(image), '--json', str(result),
               '-o', str(image.with_suffix('.boxes.png')), '--device', cfg['device'], '--text', cfg['text']]
        if cfg.get('model'):
            cmd += ['--model', str(cfg['model'])]
        with image.with_suffix('.boxes.log').open('w') as log:
            self.ocr_process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
            start = time.monotonic()
            while self.ocr_process.poll() is None:
                self.pump()
                if time.monotonic()-start > float(cfg['timeout']):
                    raise TimeoutError('箱検出タイムアウト。boxes.logを確認してください。')
            code = self.ocr_process.returncode
            self.ocr_process = None
        if code:
            raise RuntimeError(f'箱検出異常終了: {code}')
        payload = json.loads(result.read_text(encoding='utf-8'))
        # Florenceの語句指定だけに頼らず、任意の青色フィルタも利用。
        if cfg.get('blue_filter', True):
            original = cv2.imread(str(image))
            if original is None or list(payload['image_size']) != [original.shape[1], original.shape[0]]:
                raise ValueError('箱検出結果の画像サイズ不一致。')
            hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, tuple(cfg['blue_hsv_lower']), tuple(cfg['blue_hsv_upper']))
            kept = []
            for region in payload['regions']:
                box = list(map(float, region['bbox']))
                if len(box) != 4 or not all(math.isfinite(v) for v in box):
                    raise ValueError('不正なbbox')
                x1,y1,x2,y2 = box
                if not (0 <= x1 < x2 <= original.shape[1] and 0 <= y1 < y2 <= original.shape[0]):
                    raise ValueError('bbox範囲外')
                roi = mask[int(y1):math.ceil(y2), int(x1):math.ceil(x2)]
                ratio = cv2.countNonZero(roi)/roi.size
                region['blue_fraction'] = ratio
                if ratio >= float(cfg['min_blue_fraction']):
                    kept.append(region)
            payload['regions'] = kept
        payload['detected'] = bool(payload['regions'])
        payload['count'] = len(payload['regions'])
        image.with_suffix('.filtered.json').write_text(json.dumps(payload, indent=2), encoding='utf-8')
        self.vision_cache[image_path] = payload
        return payload

    def align_box(self, selected):
        self.face_command(float(selected['command_deg']))
        for attempt in range(int(self.ap['max_align_iterations'])+1):
            record = self.capture(f'align_{len(self.images)}')
            obs = self.hooks.box_observation(record, self.context())
            angle = float(obs['bearing_deg'])
            if abs(angle) <= float(self.ap['angular_tolerance_deg']):
                return record
            if attempt == int(self.ap['max_align_iterations']):
                break
            step = float(self.ap['max_turn_step_deg'])
            self.rotate(max(-step, min(step, angle)))
        raise RuntimeError('箱を中央へ合わせられませんでした。')

    def drive_straight(self, distance, final=False):
        distance = positive(distance, 'drive distance')
        if self.total_travel+distance > float(self.ap['max_total_travel_m'])+1e-8:
            raise ValueError('総接近距離の上限を超えます。')
        self.set_direct(True)
        self.wait_condition(lambda: self.odom is not None and time.monotonic()-self.odom_received < float(self.ap['odom_timeout']),
                            self.cfg['service_timeout'], '新しいオドメトリがありません。')
        origin = self.odom
        started = time.monotonic()
        best, last_progress, stopped_since = 0., started, None
        traveled = 0.
        self.set_state('APPROACH_FINAL' if final else 'APPROACH_HALF')
        try:
            while True:
                self.pump()
                now = time.monotonic()
                if now-self.odom_received > float(self.ap['odom_timeout']):
                    raise TimeoutError('オドメトリが途絶しました。')
                if now-self.gate_received > float(self.ap['monitor_timeout']) or self.gate_status is None or not self.gate_status.get('scan_ok') or self.gate_status.get('owner') != 'mission':
                    raise RuntimeError('指令ゲート/low_scanが異常です。')
                if now-started > float(self.ap['max_drive_seconds']):
                    raise TimeoutError('直進時間の上限です。')
                along, cross, heading = progress(origin, self.odom)
                if abs(cross) > float(self.ap['max_cross_track_m']) or abs(math.degrees(heading)) > float(self.ap['max_heading_error_deg']) or along < -0.1:
                    raise RuntimeError('直進経路から逸脱しました。')
                if abs(along-traveled) > float(self.ap['max_odom_step_m']):
                    raise RuntimeError('オドメトリが不連続です。')
                traveled = max(0., along)
                if traveled > best+0.01:
                    best, last_progress = traveled, now
                if now-last_progress > float(self.ap['stall_seconds']):
                    raise RuntimeError('前進できません。')
                # 初期のゼロ指令の残りを停止と誤認しないため、短い猶予後に評価する。
                if now-started > float(self.ap['output_grace_seconds']):
                    if self.output is None or now-self.output_received > float(self.ap['monitor_timeout']):
                        raise TimeoutError('Collision Monitorの速度出力が途絶しました。')
                    blocked = abs(self.output.linear.x) < 0.005 and abs(self.output.angular.z) < 0.005
                    stopped_since = (stopped_since if stopped_since is not None else now) if blocked else None
                    if stopped_since is not None and now-stopped_since >= float(self.ap['stop_hold_seconds']):
                        self.get_logger().info('正の前進指令に対するゼロ出力を検知。接近を停止して確認待ちにします。')
                        return 'blocked'
                if traveled >= distance:
                    if final:
                        raise RuntimeError('再推定距離＋余裕分まで進みましたがMonitorの停止を確認できません。')
                    return 'halfway'
                cmd = Twist()
                cmd.linear.x = float(self.ap['speed_mps'])
                self.vel_pub.publish(cmd)
        finally:
            self.total_travel += traveled
            self.stop_direct()
            # 減速中に進んだ距離も、次区間の上限に反映する。
            if self.odom is not None:
                settled, _, _ = progress(origin, self.odom)
                self.total_travel += max(0., settled-traveled)
            self.get_logger().info(f'直進区間: {traveled:.3f} m / 総接近距離: {self.total_travel:.3f} m')

    def approach_halfway(self, distance):
        self.total_travel = 0.
        self.set_state('ALIGN_BOX')
        return self.drive_straight(distance*0.5)

    def midway_check(self):
        center = self.capture('midway_center')
        if self.hooks.box_visible(center['path'], self.context()):
            return self.align_box(center)
        base = self.heading.degrees
        self.rotate(10.)
        left = self.capture('midway_left10')
        self.rotate(-20.)
        right = self.capture('midway_right10')
        selected = self.hooks.choose_box_image([left, right], self.context())
        if selected is None:
            raise RuntimeError('途中の左右10度でも箱が見つかりません。')
        self.get_logger().info(f'途中探索の基準指令角度: {base:.1f}')
        return self.align_box(selected)

    def wait_arrival(self):
        self.confirmed = False
        self.set_state('WAIT_CONFIRM')
        self.get_logger().info('停止しました。箱へ到達したことを人が確認してconfirm_arrivalを呼んでください。')
        while not self.confirmed:
            self.vel_pub.publish(Twist())
            self.pump()
        self.stop_direct()
        self.set_state('CONFIRMED')