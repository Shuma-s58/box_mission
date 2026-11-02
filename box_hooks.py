"""Florence-2 bboxの選択と距離推定。推論そのものはMission側で実行。"""
import json
from pathlib import Path
from approach_logic import select_box, bearing_deg, estimate_distance


def detection(record, context):
    path = record['path'] if isinstance(record, dict) else record
    payload = context['detect'](path)
    # フィルタ済み候補から最大面積を選ぶ。物体の追跡IDではない。
    selected = select_box(payload, context['approach']['min_bbox_height_px'])
    return payload, selected


def box_visible(image_path, context):
    return detection(image_path, context)[1] is not None


def choose_box_image(records, context):
    # READMEの「最初の検出写真」を維持。
    for record in records:
        if box_visible(record['path'], context):
            return record
    return None


def target_command_heading(record, context):
    return float(record['command_deg'])


def box_observation(record, context):
    payload, selected = detection(record, context)
    if selected is None:
        raise ValueError('箱が検出されませんでした。')
    k = record['intrinsics']
    if list(payload['image_size']) != [k['width'], k['height']]:
        raise ValueError('推論画像とカメラ内部パラメータの解像度が一致しません。')
    return {'bbox': selected['bbox'], 'bearing_deg': bearing_deg(selected['bbox'], k)}


def box_distance(record, context):
    observation = box_observation(record, context)
    observation['distance_m'] = estimate_distance(observation['bbox'], record['intrinsics'], context['approach'])
    Path(record['path']).with_suffix('.distance.json').write_text(json.dumps(observation, indent=2), encoding='utf-8')
    return observation['distance_m']