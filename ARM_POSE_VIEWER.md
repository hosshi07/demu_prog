# 手先の XYZ モニタ（Ubuntu 20.04 / ROS2 Foxy）

**画面のボタンでアームを操作して座標をコピーする場合は、[ARM_POSE_JOGGER.md](ARM_POSE_JOGGER.md) の `arm_pose_jogger.py` を使ってください。** 以下は表示専用モニタの手順です。

表示専用モニタを使う場合は `arm_pose_viewer.py`・`arm_kinematics.py`・`arm_pose_jogger.py` を同じ場所に置いて実行します。共通の計算・描画処理は現在 `arm_pose_jogger.py` にあります。パッケージ作成・colcon build は不要です。既存プログラムは変更していません。

画面は **XYZ の数値（m）、上から見た図、横から見た図**を表示します。グリッドは 10 cm。「Copy」ボタンで `[x, y, z, pitch]` をコピーでき、その形式は既存の `move()` に対応します。コピーだけではアームは動きません。GUI はフォントの導入が不要な英語表記です。

## 1. 起動

ROS と現在使っているワークスペースを読み込みます。Docker を使う場合は、普段 `keyboard.py` を実行する、GUI 表示可能な同じ Docker 環境で実行してください。

```bash
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash
```

Tkinter がなければ一度だけインストールします。

```bash
sudo apt-get update
sudo apt-get install python3-tk
```

別ターミナルで、普段使用しているコントローラを起動します（既に起動していれば追加起動は不要）。

```bash
ros2 run dynamixel_sdk_examples read_write_trapezoidal_node
```

**提示された `read_write_trapezoidal_node.cpp` には `/get_position` があることを確認済みです。実環境で同じコントローラを起動しているか、以下で確認できます。**

```bash
ros2 service list -t
ros2 interface show dynamixel_sdk_custom_interfaces/srv/GetPosition
```

`/get_position` が `dynamixel_sdk_custom_interfaces/srv/GetPosition` 型で、リクエストに `id`、レスポンスに `position` がある場合：

```bash
python3 arm_pose_viewer.py
```

起動したまま既存の `keyboard.py` / `robocon.py` を使うと、モータ ID 11〜14 のエンコーダ値から手先位置を計算します。15 はグリッパ開閉用なので XYZ 計算に使用しません。モニタはモータ指令を送らず、USB ポートも直接開きません。

サービス名が違う場合：

```bash
python3 arm_pose_viewer.py --service /別のサービス名
```

取得できない間は待機表示、最後の取得から2秒以上経過すると灰色の旧データ表示になり、コピーも無効になります。正常なサンプル取得時のみトピックを発行します。4関節を順番に読む方式で完全な同時計測ではないため、座標を記録するときはアームが停止してからコピーしてください。取得には約0.2秒以上の周期を設けています。

`GetPosition` が位置値だけを返す実装では、コントローラ内部の通信エラーと有効な値を完全には区別できません。モニタはタイムアウトと範囲外値を検出しますが、コントローラがエラー時に 0 や過去値を正常応答として返すケースは判別できません。異常な座標の場合はコントローラ側のログも確認してください。

## 2. 現在角度取得サービスがない場合

追加のドライバ修正をせずに、**最後に送った目標座標の推定**を表示できます。

```bash
python3 arm_pose_viewer.py --source command
```

モニタを先に起動してから既存の操作プログラムを起動してください。`/set_position` を購読し、ID 11〜14 がすべて届いてから表示します。開始前の指令は取得できません。画面に **COMMAND / last target, NOT measured** と常時表示します。移動中や指令未達の場合、実際の手先位置とは異なります。指令がない間は最後の目標を保持します。指令の更新中には、各関節の新旧指令が一時的に混在する可能性があるため、停止後に確認してください。

## 3. 他のノードから取得

現在角度取得モードでは、`geometry_msgs/msg/PointStamped` 型で以下に発行します。

```bash
ros2 topic echo /arm/end_effector_position
```

`point.x`, `point.y`, `point.z` がメートル単位の手先座標です。指令モードでは区別のため `/arm/commanded_end_effector_position` に発行します。

GUI が使えない場合もノードとして動作します。

```bash
python3 arm_pose_viewer.py --no-gui
# または
python3 arm_pose_viewer.py --source command --no-gui
```

## 4. 座標と計算方法

- 原点・軸方向は既存 `move([x, y, z, pitch])` の計算に合わせています。Z は上向き、XY はアーム台座を中心とする水平面です。X/Y の実機上の方向は関節1のゼロ位置に依存します。
- ArUco のワールド座標やカメラ座標への変換はしません。
- `frame_id` の既定値 `arm_base` は、この計算上の座標系のラベルです。TF は発行しません。既存 TF と一致を確認できた場合のみ `--frame base_link` などで変更してください。
- 寸法は `L1=0.077`, `L2=0.13`, `L3=0.124`, `L4=0.126` m、補正角は `atan2(0.024, 0.128)`。関節角は既存コードと同じ `(position - 2048) * 0.088` 度です。
- 既存の逆運動学を逆算した順運動学です。pitch は `q2 + q3 + q4`、下向きが正。厳密なURDFとの一致や、たわみ・工具の追加長は考慮していません。実機で工具先端との対応を確認してください。
- 既存コードの単回転位置制御を前提に、取得値は 0〜4095 の範囲を受け付けます。

## 5. 実機なしで画面を確認

```bash
python3 arm_pose_viewer.py --demo
```

Tkinter のみで動き、ROS やモータには接続しません。

計算と通信処理のテスト（ROS 不要）：

```bash
python3 -m unittest discover -s tests -v
```

Ubuntu/Foxy・実機での接続確認は、このリポジトリを編集した環境では実施できていません。
