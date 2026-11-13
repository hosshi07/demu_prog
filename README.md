# Docker 内で USB を再接続する

実習PCの **Ubuntu 上で動くローカルの通常の Docker Engine（rootful、ユーザー名前空間の再マッピングなし）** を対象にしています。`start-usb-hotplug.sh` は既存の Foxy イメージから `hosshi-ros2-usb` という別のコンテナを作ります。Docker Desktop・WSL や rootless Docker では、USBの見え方やデバイスのアクセス制御が異なります。

配布フォルダの旧起動スクリプトは空だったため、実習PCの現在の設定は現地で確認してください。個別の `--device=/dev/videoN` や `--device=/dev/ttyUSB0` でコンテナ作成時のデバイスを渡す運用なら、次の方式に切り替えると、抜き差し後も同じコンテナから新しいデバイスを参照できます。

## 初回の切り替え

実習PCの Ubuntu ホストで、`hosshi_space` に移動して作業します。以下の Docker コマンドは、実習PCで普段使っている権限で実行してください。

1. イメージが存在することと、USBを挿したときの機器名を確認します。

   ```bash
   docker ps -a
   docker image inspect demulab2025/ros2:latest --format '{{.Id}}'
   ls -l /dev/serial/by-id/ /dev/serial/by-path/
   ls -l /dev/v4l/by-id/ /dev/v4l/by-path/
   ```

   同じ見た目の機器が複数あるときは、対象機器を挿す前後の一覧を比べます。`by-id` がなければ、同じ物理ポートを使う前提で `by-path` を選びます。カメラに複数のリンクが出る場合は、実際に画像を配信できるリンクを確認します。別のコンテナでカメラやアームを使用中なら、使っている ROS ノードを終了して機器を解放します。

2. アームを使用する場合は、手順1で確認した**アームのUSBシリアル変換器のリンク**を渡します。下の `実際のリンク名` を置き換えてください。カメラだけなら引数なしで起動できます。イメージのタグが異なる場合は、実習PCにあるタグを `HOSSHI_IMAGE` で指定します。

   ```bash
   bash start-usb-hotplug.sh '/dev/serial/by-id/実際のリンク名'
   # カメラだけの場合
   # bash start-usb-hotplug.sh
   ```

3. コンテナが稼働していることを確認し、中に入ります。

   ```bash
   docker ps --filter name=hosshi-ros2-usb
   docker exec -it hosshi-ros2-usb bash
   ```

   コンテナ内では以下を確認します。アームの `stat` の結果が `bc:` で始まれば、リンク先は Linux の USB シリアル機器（メジャー番号188）です。カメラの場合は `51:`（メジャー番号81）です。これは機器ファイルの確認で、ROSノードによる通信は次の節で確認します。

   ```bash
   source /opt/ros/foxy/setup.bash
   echo "$ROS_DISTRO"
   ls -l /host-dev/v4l/by-id/ /host-dev/v4l/by-path/
   # アームを設定した場合
   readlink -f /dev/ttyUSB0
   stat -Lc '%t:%T %n' /dev/ttyUSB0
   ```

## USBが外れたとき

1. 対象機器を開いていた ROS ノードを終了します。アームの場合は、先に指令を出している実験ノードを停止し、実習の手順に従って安全を確保します。初回の抜き差し確認はカメラで行えます。
2. USBを挿し直します。`by-path` を使う場合は同じ物理ポートに挿します。ホストの `/dev` の変化がコンテナの `/host-dev` に反映されます。アームなら `stat -Lc '%t:%T %n' /dev/ttyUSB0`、カメラなら `ls -l /host-dev/v4l/by-id/` または `ls -l /host-dev/v4l/by-path/` で確認します。
3. 必要な機器のノードだけを起動し直します。既に開いていたUSB接続はノードが開き直す必要があります。**Dockerの停止・起動や、抜き差しごとの `chmod` は通常の復旧手順から外せます。**

カメラを起動する例です。指定したカメラリンクがない場合は、機器を確認してから `/host-dev/videoN` も使えます。数字は抜き差しで変わることがあります。

```bash
source /opt/ros/foxy/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/host-dev/v4l/by-id/実際のリンク名
```

別の端末から同じコンテナに入り、Foxy の `setup.bash` を読み込んで `ros2 topic hz /image_raw` を実行します。抜き差し前と、再接続後にカメラノードだけを起動し直した後で、画像が届くことを確認します。ホストで `docker ps --filter name=hosshi-ros2-usb` を見れば、同じコンテナで稼働を続けていることを確認できます。

アーム側は、コンテナ内の `/dev/ttyUSB0` が選んだ機器へのリンクになるため、既存の `read_write_trapezoidal_node` をそのまま使えます。このノードは起動時に**モータのトルクを有効化**します。リンク先が意図した機器であることと、実機の安全を確認した段階で手動で起動します。

## 作業フォルダと運用

- ホストの `hosshi_space` はコンテナの `/work/hosshi_space` に対応します。新しいパッケージやノード、ビルド結果はこの中の `colcon_ws` にまとめます。既存コンテナのワークスペースは自動で引き継がれません。DYNAMIXELを使う場合は、実習で使う版のソースをこの作業領域の `colcon_ws/src` に用意します。Foxyを読み込んだ端末で `cd /work/hosshi_space/colcon_ws` に移動し、`colcon build --symlink-install --packages-up-to dynamixel_sdk_examples` を実行します。起動する端末ではビルド後に `source /work/hosshi_space/colcon_ws/install/setup.bash` も実行します。
- USBを開く端末はコンテナ内の root で動きます。ホストに残すソースやビルド結果を自分のユーザーで作るには、ホストから `docker exec -it --user "$(id -u):$(id -g)" -e HOME=/tmp hosshi-ros2-usb bash` で開いた端末を使えます。
- 新コンテナは端末からの確認用で、GUI転送設定は含みません。ROS 2 ノードは同じコンテナ内で動かす構成です（`ROS_LOCALHOST_ONLY=1`）。
- `/dev` はサブマウントを除いて別パスに読み取り専用で共有します。追加するデバイスアクセスは V4L2（81）とUSBシリアル（188）の読み書きです。同じ種類の別の機器もアクセス範囲に入るため、使う機器は上記のリンクで選びます。
- ホストで機器が見えない場合は、まずケーブル・ポート・ホストのUSB認識を確認します。ホストでは見えるのにコンテナで機器を開けない場合は、`docker inspect hosshi-ros2-usb --format '{{json .HostConfig.DeviceCgroupRules}}'` と `docker info` を確認します。
- 実習後に止めた場合は `docker start hosshi-ros2-usb` で再利用できます。機器の設定やイメージを変更するときだけ、`docker stop hosshi-ros2-usb` → `docker rm hosshi-ros2-usb` → 新しい引数で起動し直します。削除するのはこのコンテナで、ホストの `hosshi_space` のファイルは残ります。

参考資料は [Dockerの動的デバイス設定](https://docs.docker.com/reference/cli/docker/container/run/#using-dynamically-created-devices---device-cgroup-rule)、[バインドマウント](https://docs.docker.com/engine/storage/bind-mounts/#recursive-mounts)、[LinuxのUSBシリアル](https://cdn.kernel.org/doc/html/latest/usb/usb-serial.html)、[V4L2のデバイス番号](https://www.kernel.org/doc/html/v6.15/userspace-api/media/v4l/open.html) です。
