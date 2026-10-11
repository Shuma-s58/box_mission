"""ROSに依存しない、文字分類・回転指令角度の管理。"""
import math


def classify_regions(payload):
    """OCR出力順。a,b,dは小文字のみ、cはCも許可。複数文字の領域は除外。"""
    for region in payload.get('regions', []):
        text = str(region.get('text', '')).strip()
        if len(text) == 1 and text in ('a', 'b', 'c', 'C', 'd'):
            return 'c' if text == 'C' else text
    raise ValueError('OCR: 有効な1文字(a,b,c,C,d)が検出されませんでした。')


class CommandHeading:
    """左が正。実測姿勢を使わず、成功したSpinの指令を累積する。"""
    def __init__(self):
        self.degrees = 0.0

    def reset(self):
        self.degrees = 0.0

    def delta_to(self, target):
        target = float(target)
        if not math.isfinite(target):
            raise ValueError('撮影方向は有限値で指定してください。')
        return target - self.degrees

    def commit(self, delta):
        self.degrees += float(delta)
