"""後で実装するbox処理。mission.pyとは別ファイル。
画像のrecordは {path: str, command_deg: float}。
contextにはframe_id、現在の指令角度、撮影データを渡す。
"""


def box_visible(image_path, context):
    """後で実装1: 写真に青いboxが写っている場合True、なければFalse。"""
    raise NotImplementedError('box_hooks.box_visible に青いboxの判定処理を実装してください。')


def choose_box_image(records, context):
    """後で実装1: 6枚から選択。現在の土台は最初にboxが写る写真を採用。"""
    for record in records:
        if box_visible(record['path'], context):
            return record
    return None


def target_command_heading(record, context):
    """後で実装2: 向きを決める。既定では選択写真の回転指令角度。
    戻り値はYAML走行完了時を0度とする累積指令角度[deg]。
    実測姿勢へ変更しないでください。
    """
    return float(record['command_deg'])


def box_goal(record, context):
    """後で実装3: boxへのmap座標ゴール。
    戻り値: {'x': float, 'y': float, 'yaw': float}。x/y[m]、yaw[rad]。
    """
    raise NotImplementedError('box_hooks.box_goal にbox位置のゴール算出処理を実装してください。')
