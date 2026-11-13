# ボタンで操作して XYZ をコピーする

`keyboard.py` の代わりに使う GUI 操作ノードです。**X±・Y±・Z±・Pitch± をクリックするだけ**で動かせます。Enter キーは不要です。

表示するのは、この画面から指令した **目標座標**です。モータの現在角度を読むサービスは必要ありません。既存の `keyboard.py` / `robocon.py` と同じ寸法・逆運動学・モータID・`/set_position` を使います。

## 起動方法

以下の4ファイルを同じフォルダに置いてください。colcon build は不要です。

- `arm_pose_jogger.py`（今回の操作画面）
- `arm_pose_viewer.py`（共通の表示画面）
- `arm_kinematics.py`（座標計算）
- この手順書（任意）

普段の Ubuntu 20.04 / Foxy の Docker 環境を起動します。GUI 表示は `keyboard.py` を使っていた環境の Terminator などから行ってください。Tkinter が未導入なら、一度だけ次を実行します。

```bash
sudo apt-get update
sudo apt-get install python3-tk
```

ターミナル1（普段使っているコントローラ。既に起動済みなら追加起動は不要）：

```bash
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 run dynamixel_sdk_examples read_write_trapezoidal_node
```

ターミナル2（ファイルを置いたフォルダで）：

```bash
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash
python3 arm_pose_jogger.py
```

操作中は `keyboard.py` と `robocon.py` を終了しておいてください。他のプログラムが同時に指令すると、画面の目標と実機の姿勢が一致しなくなります。

## 使い方

1. **Move to start pose** をクリックします。手先の目標 `[0.1, 0.0, 0.1, 90.0]` に移動します。起動しただけでは動きません。開始前に数値を実測できないため、この既知の目標姿勢を操作の基準にします。グリッパの開閉状態はこの操作では変わりません。
2. アームが落ち着いたら、**X + / X -、Y + / Y -、Z + / Z -、Pitch + / Pitch -** をクリックして位置を調整します。
3. `XYZ step (mm)` で1クリックの移動量を **1 / 5 / 10 / 20 mm** から選べます。初期値は5 mmです。`Pitch step (deg)` は **1 / 3 / 5度**、初期値は3度です。
4. `Velocity` と `Acceleration` で速度・加速度の指令値を選べます。初期値は元のプログラムと同じ **25 / 5** です。これらはSDKの設定単位で、m/sなどではありません。
5. 数値と上面図・側面図を確認し、実機が停止してから **Copy [x, y, z, pitch]** をクリックします。
6. プログラムに貼り付けます。**Copy move(...) code** なら、呼び出し全体をコピーできます。

コピー例：

```python
[0.1050, 0.0000, 0.1000, 90.00]
```

```python
open_manipulator_x.move([0.1050, 0.0000, 0.1000, 90.00], 25, 5)
```

XYZ の表示・コピー単位は **m**、pitch は **度**です。pitch の正方向は元のコードと同じ下向きです。画面の図は目標姿勢の図です。

`Open gripper` / `Close gripper` は ID 15 に開閉指令を送ります。角度は `keyboard.py` と同じ **-50度 / 0度**です（`robocon.py` の閉角度10度とは異なります）。

## 動作上の扱い

- 1クリックごとに1回だけ送信します。押しっぱなしでの連続移動や、操作の予約は行いません。
- 各指令後は元のコードと同じ **2秒間**、操作とコピーを無効にします。これは待ち時間であり、モータの移動完了を検出したものではありません。まだ動いている場合は停止を待ってください。必要なら `python3 arm_pose_jogger.py --wait 4` で待ち時間を延ばせます。
- 起動時には何も送信しません。ROSの購読先が見つからない場合も送信せず、画面に理由を表示します。購読先があることは、モータが正常に動いた保証にはなりません。
- 到達できない目標、NaN、0〜4095のモータ位置範囲を外れる目標は送信しません。元の `atan(y/x)` で表現できる範囲に合わせ、X > 0、Z >= 0 に限定しています。実機の関節可動限界や周辺物との接触は計算していません。
- 表示値は**最後に送信した目標**です。実際の停止位置や動作成功をフィードバックで確認する機能はありません。4モータへの指令は順次送信されます。
- 起動姿勢を変えたい場合：`python3 arm_pose_jogger.py --start 0.15 0.0 0.15 90`。実機がその姿勢に移動するのは `Move to start pose` を押したときです。
- ウィンドウを閉じると追加の指令送信を終了します。既に送った移動指令はキャンセルしません。

## 実機なしで試す

```bash
python3 arm_pose_jogger.py --demo
```

ROS・モータには接続せず、同じボタンで図と数値を操作できます。

## 検証

```bash
python3 -m unittest discover -s tests -v
```

逆運動学・順運動学の整合、各軸の増減、到達不可姿勢の拒否、待機中の連打拒否、コピー文字列、コントローラ未接続時の送信抑止をテストしています。Ubuntu/Foxy と実機での動作は、開発環境では未確認です。
