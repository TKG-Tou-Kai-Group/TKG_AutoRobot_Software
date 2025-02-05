# CoRE_TKG_AutoRobot

チームTKGがCoRE2025で開発した自動ロボットのソースコード類一式

## 想定している実行環境
- Core 2024 向け自動ロボット
- PC
  - x86_64 CPU
  - NVIDIA社製GPUと、その最新のドライバ
  - Ubuntu22.04
  - ROS2 Humble

PC <---Ethernet---> RaspberryPi <--CAN/GPIO--> 自動ロボット
   <---USB---> [RealSense D455](https://www.intelrealsense.com/depth-camera-d455/)
   <---Ethernet---> [UST-30LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=195)

## 環境構築
- x86_64系のCPUをNVIDIAのGPUを搭載したPCを用意する
- Ubuntu 22.04をインストールする
- NVIDIAのドライバをインストールする。`nvidia-smi`が動作すればだいたいOK。
- [setup.sh](./setup.sh)の手順を参考にDockerやRealSenseのドライバをセットアップする
- RealSenseを接続する
- PC-自動ロボットLANハブ間のLANケーブルを接続する

## 使い方
1. リポジトリをクローンする
   ```bash
   git clone https://github.com/TKG-Tou-Kai-Group/TKG_AutoRobot_Software.git
   ```

3. サブモジュールを取得する
   ```bash
   cd TKG_AutoRobot_Software
   git submodule update --init --recursive
   ```

4. Docker イメージをシェルスクリプトを使ってビルドする
   ```bash
   cd docker
   ./build_docker_image.sh
   ```

5. Docker コンテナを立ち上げる
   ```bash
   ./launch_docker.sh
   ```

6. ROS 2のソースコードをビルドする
   ```bash
   colcon build && source install/setup.sh
   ```

7. パッケージを起動する

   赤チーム用
   ```bash
   ros2 launch tkg_autorobot_launcher
   ```
   青チーム用
   ```bash
   ros2 launch tkg_autorobot_launcher
   ```

## パッケージ構成
ワークスペースに格納されているパッケージの概要を以下に示す。詳細は各パッケージのREADME.mdに記載している。

- [CoRE_AutoRobot_2024_sample](https://github.com/scramble-robot/CoRE_AutoRobot_2024_sample)
  実行委員会が使用したプログラム、動作確認および一部コントローラを流用

- [tkg_autorobot_controller](https://github.com/TKG-Tou-Kai-Group/tkg_autorobot_controller)
  チームTKGで開発した自動ロボット砲塔姿勢制御プログラム、静止摩擦を考慮したLQR制御が特徴

- tkg_autorobot_urg_object_detector
  測域センサを用いて周囲の物体の位置を取得するプログラム

- tkg_autorobot_commander
  各センサや上記パッケージからの情報を元に自動ロボットの行動を決定するプログラム、動作を速くするために画像処理なども含む

- tkg_autorobot_launcher
  自動ロボットを動作させるために必要なlaunchファイルを格納したパッケージ

- [urg_node2](https://github.com/Hokuyo-aut/urg_node2)
  測域センサ用ROS 2パッケージ、aptでインストールできないのでサブモジュールとして導入
