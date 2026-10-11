# box_mission — B完了後の撮影・OCR・分岐走行・box接近

ROS 2 Humble用の外部Pythonノードです。waypoint_tools本体への変更はありません。Bを走り切ってから起動してください。Bの終了待ち自体はこのコードに含みません。

## 実装範囲

初期ゴール、Spin、USBカメラ撮影、Florence2 OCR、4経路の分岐、走行結果確認、6枚探索、途中撮影・左右10°確認、local/globalレイヤーの変更、人の到達確認サービス、逆回転、next_file送信を実装しています。

**指定どおり「あとで実装」の青いboxの認識と位置推定は未実装です。** `box_hooks.py`に接続口を用意しています。未実装のまま実行すると、6枚の探索写真を保存した後、box判定で明確にエラー終了します。boxの座標を仮定して走らせる処理は入れていません。

## ファイル

| ファイル | 内容 |
| --- | --- |
| mission.py | ROS制御、撮影、OCR起動、全体の流れ |
| mission_logic.py | 文字分類、回転指令角度の計算 |
| box_hooks.py | 後で実装1〜3の接続口 |
| config.yaml | フォルダ、ファイル対応、カメラ、OCR設定 |
| routes/goal.yaml | 分岐前のOCR撮影地点 |
| routes/a.yaml〜d.yaml | 分岐する4つの経路。waypoint_toolsと同形式 |
| test_mission.py | ROS/実機なしで実行するロジック・制御フローの模擬テスト |

同じフォルダにgoal.yamlと4経路を置けます。4経路はconfig.yamlのroutesで名前を明示して区別するため、フォルダ内のファイル数は制限しません。

## 準備

ZIPをホームディレクトリへ展開する例:

```bash
cd ~
unzip box_mission.zip
sudo apt update
sudo apt install python3-opencv python3-yaml ros-humble-nav2-msgs
```

`config.yaml`を編集します。

- `route_dir`: 5つのYAMLが入っているフォルダ。
- `initial_goal_file`: OCR撮影地点のYAML。通常goal.yaml。
- `routes`: 認識文字a,b,c,dに対応する4経路のファイル名。
- `image_dir`: 撮影画像の保存フォルダ。
- `camera.device`: USBカメラのデバイス。通常/dev/video0。
- `ocr.python`: 動作確認済みFlorence2環境のPython。例~/florence2_work/.venv/bin/python。
- `ocr.script`: 使用するflorence2_ocr.pyのパス。
- `use_sim_time`: 実機はfalse、Gazeboなどはtrue。trueなら/clockが必要。

初期ゴールと経路の同梱座標は形式を示すサンプルです。実際の地図の座標へ変更してください。座標x/y/zはm、YAMLのyawはrad、フレームはconfigのframe_idです。

## 起動

Nav2・自己位置推定・Collision Monitor・通常のwaypoint_sender_nodeを普段どおりに起動します。Spin behaviorとFollowWaypointsが有効になっている必要があります。

Bを走り切った後:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/box_mission
python3 mission.py --config config.yaml
```

`~/ros2_ws`は実際のワークスペース名へ変更します。制御ノードはROSが入っているsystem Pythonで起動してください。OCRのみ設定されたvenvのPythonで別プロセス実行します。モデル/依存関係は使用するflorence2_work環境側でインストールしておいてください。

通常のsenderはBのファイル位置のまま待機させます。この処理中はnext_file/send_allやRVizから別ゴールを送らないでください。同じNav2のゴールが置き換えられます。

## 人による到達確認

ロボットがboxの近くに到達し、Collision Monitorにより停止したことを人が確認したら、別ターミナルで呼びます。

```bash
source /opt/ros/humble/setup.bash
ros2 service call /box_mission/confirm_arrival std_srvs/srv/Trigger '{}'
```

途中撮影後にboxへの走行を再開してWAIT_CONFIRMになってから受け付けます。早い段階の呼び出しは拒否します。サービスは到達を人が認定する入力であり、Collision Monitorの停止そのものを自動判定するものではありません。

受理後、自分のNavigateToPoseをキャンセルして終端を待ち、累積Spin指令を逆回転し、両方のobstacle_layer.enabledをtrueに戻して、以下を送ります。

```bash
ros2 service call /waypoint_sender_node/next_file std_srvs/srv/Trigger '{}'
```

サービス応答のsuccessは確認入力の受理を表します。逆回転・レイヤー復帰・次経路送信の完了はログのSTATE: DONEで確認してください。

## 処理の順序

1. goal.yamlのゴールへNavigateToPoseで移動。
2. /spinへ左90°（+π/2）の指令を送る。
3. USBカメラの撮影画像を保存。
4. 指定のflorence2_ocr.pyを`--json`付きで実行し、保存JSONを読む。
5. /spinへ右180°（-π）の指令を送る。
6. OCR結果に対応したa/b/c/d.yamlをFollowWaypointsへ送る。
7. STATUS_SUCCEEDEDかつmissed_waypointsが空であることを確認。
8. ここを指令角度の0°としてリセット。0°,15°,30°,45°,60°,75°で合計6枚を撮影。Spinは5回。
9. box_hooksからboxが写った写真を選び、その写真の指令角度へ回転。
10. box_hooksでboxのゴールを作る。両方のobstacle_layer.enabledをfalseにして接近。
11. 最初の正のdistance_remainingの半分以下になったら、自分のNavigateToPoseをキャンセルし、停止を待って撮影。
12. 写真にboxがなければ、現在の指令方向から左10°で撮影。続いて右20°回転し、基準から右10°で撮影。boxが写った写真の指令方向に合わせる。
13. 同じboxゴールへのNavigateToPoseを再送。レイヤーをfalseに再設定し、人の到達確認を待つ。
14. 確認後、累積指令角度を逆回転して0°へ戻す。
15. 両レイヤーをtrueにし、通常のsenderのnext_fileを呼ぶ。

## OCRの扱い

現在の[florence2_ocr.py](https://github.com/Shuma-s58/florence2_work/blob/main/florence2_ocr.py)の`--json`形式（regions内のtext）に合わせています。

- 空白除去後に1文字のみの領域を分類対象にします。
- 有効文字はa,b,c,C,d。Cはcとして扱います。A/B/Dは指定に含まれていないため除外します。
- 有効な候補が複数あればOCR出力順の最初を採用します。文字の画像上の左上順へ並べ替えません。
- 複数文字の領域から1文字を切り出す処理は行いません。
- 有効候補なし、OCR異常終了、タイムアウトは失敗扱い。

## 後で実装する別ファイル

`box_hooks.py`の関数を置き換えます。

### あとで実装1

`box_visible(image_path, context) -> bool`: 写真に青いboxがあるか判定。未実装ではNotImplementedErrorになります。

`choose_box_image(records, context) -> record | None`: 渡された写真から1枚を選択。既定の土台ではbox_visibleを順に呼び、最初のTrueを採用します。6枚全部になければNone。途中の左右2枚の選択にも利用します。

### あとで実装2

`target_command_heading(record, context) -> float`: YAML走行終了時を0°とした累積指令角度[deg]。現状の土台は写真に保存したcommand_degをそのまま返します。必要なら別の選択ロジックに差し替えます。

### あとで実装3

`box_goal(record, context) -> dict`: `{'x': ..., 'y': ..., 'yaw': ...}`を返します。フレームはconfigのframe_id。未実装ではNotImplementedErrorになります。

contextにはframe_id、現在の指令角度、画像一覧、画像保存フォルダを渡します。recordには画像パスpathと指令角度command_degが入ります。画像から距離や座標を得る方法は、このファイル側へ後で実装します。

## 回転角度と「半分」の定義

向きの管理にTF/IMUの実測姿勢は使いません。成功したSpinへ送った角度だけを累積し、撮影と復帰に使います。Spin内部の到達判定はNav2が行います。

**Nav2の並進中の自動旋回と、Spinの実際の回転誤差は累積値に入りません。** したがって逆回転は「指令角度を相殺する」動作です。地図上で走行終了直後の実測向きへ厳密に戻ることは保証しません。これは実測向きではなく指令角度を使うという指定に合わせています。

「半分」は接近開始後の最初の正のNav2残り経路長に対する50%です。再計画で経路長が変わる場合、実際の走行距離の半分とは一致しません。半分を確認する前にActionが終了した場合は、失敗扱いにして次へ進みません。

接近後半では、Collision Monitorが止めた後にProgressChecker等でNavigateToPoseが異常終了しても、人の確認を待ち続けます。状態コードをログに出し、人の確認なしにはnext_fileを送りません。人は本当にboxまで到達したか確認してからサービスを呼んでください。

## 保存と状態

指定画像フォルダの下に実行時刻のサブフォルダを作ります。PNG、images.json（画像と累積指令角度）、ocr_result.json、ocr_annotated.png、ocr.logを保存します。

```bash
ros2 topic echo /box_mission/state
```

状態topicは遷移時に配信します。途中から監視した場合は次の遷移から表示されます。

## 失敗・中断

OCR失敗、box未検出、経路の未到達、Action/サービス失敗は終了コード1です。Ctrl+Cは130、正常終了は0です。失敗時は自分のActionをキャンセルし、変更したレイヤーを変更前の値へ戻します。逆回転やnext_fileは行いません。キャンセル・復帰に失敗した場合はログに出します。Collision Monitorはこのコードから無効化しません。

obstacle_layer.enabledの変更では、他レイヤーのコストや既存のコストの消去までは行いません。Nav2のclear_costmapは呼びません。

## 検証

```bash
cd ~/box_mission
python3 test_mission.py
```

Python構文と、OCR分類・指令角度・6枚撮影・経路選択・途中の左右確認・人の確認・逆回転・レイヤー復帰・next_fileの模擬フローを確認しています。実際のROS通信、USBカメラ、Florence2推論、Nav2/Gazebo/実機での統合動作は未検証です。
