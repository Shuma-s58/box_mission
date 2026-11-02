"""ROS非依存の画像幾何・直進距離計算。光軸がbase_link前方、水平なカメラを想定。"""
import math


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def select_box(payload, min_height_px=8):
    w, h = payload['image_size']
    positive(w, 'width'); positive(h, 'height')
    valid = []
    for region in payload['regions']:
        box = list(map(float, region['bbox']))
        if len(box) != 4 or not all(math.isfinite(v) for v in box):
            raise ValueError('Invalid bbox')
        x1, y1, x2, y2 = box
        if not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h):
            raise ValueError('bbox outside image')
        if y2-y1 >= min_height_px:
            valid.append(dict(region, bbox=box, area_px2=(x2-x1)*(y2-y1)))
    return max(valid, key=lambda b: b['area_px2']) if valid else None


def bearing_deg(box, intrinsics):
    """ROS左旋回が正。カメラ画像の右方向は負。"""
    u = (box[0]+box[2])/2
    return math.degrees(math.atan2(float(intrinsics['cx'])-u,
                                   positive(intrinsics['fx'], 'fx')))


def estimate_distance(box, intrinsics, cfg):
    w, h = intrinsics['width'], intrinsics['height']
    margin = float(cfg.get('edge_margin_px', 2))
    x1, y1, x2, y2 = box
    if x1 <= margin or y1 <= margin or x2 >= w-margin or y2 >= h-margin:
        raise ValueError('箱が画像端で欠けています。距離推定できません。')
    distance = positive(intrinsics['fy'], 'fy') * positive(cfg['box_height_m'], 'box_height_m') / positive(y2-y1, 'bbox height')
    if not float(cfg['min_distance_m']) <= distance <= float(cfg['max_distance_m']):
        raise ValueError(f'推定距離が範囲外: {distance:.3f} m')
    return distance


def planned_travel(distance, cfg):
    # カメラから箱までの光軸距離 -> base_link前端から箱までの距離。
    length = float(distance) + float(cfg['camera_forward_m']) - float(cfg['robot_front_m']) - float(cfg['stand_off_m'])
    if not math.isfinite(length) or length <= 0:
        raise ValueError('接近予定距離が0以下です。既に近いか、距離推定を確認してください。')
    if length > positive(cfg['max_total_travel_m'], 'max_total_travel_m'):
        raise ValueError('接近予定距離が走行上限を超えています。')
    return length


def yaw_from_quaternion(q):
    values = [float(q.x), float(q.y), float(q.z), float(q.w)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid quaternion')
    n = math.sqrt(sum(v*v for v in values))
    if n < 1e-8:
        raise ValueError('Zero quaternion')
    x, y, z, w = [v/n for v in values]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def progress(origin, current):
    x, y, yaw = origin
    dx, dy = current[0]-x, current[1]-y
    return (dx*math.cos(yaw)+dy*math.sin(yaw),
            -dx*math.sin(yaw)+dy*math.cos(yaw),
            math.atan2(math.sin(current[2]-yaw), math.cos(current[2]-yaw)))