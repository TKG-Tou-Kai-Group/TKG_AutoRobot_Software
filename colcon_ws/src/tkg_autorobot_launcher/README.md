# tkg_autorobot_launcher

CoRE 2025 自動ロボットの立ち上げに必要なlaunchファイルを格納するパッケージ

## launchファイルのリスト

- rs_launch.py

  RealSense用launchファイル：標準ファイルから最高解像度および最高FPSに設定するように変更

- urg_node2.launch.py

  測域センサ用launchファイル：configフォルダに格納した設定ファイルを用いて設定。configファイルは砲塔の可動範囲に合わせて前方180度の範囲に制限

- robot_launcher.launch.py

  チームに依存しないセンサ類や測域センサでの物体検知プログラムを一括で立ち上げるlaunchファイル

- red_team_robot_launcher.launch.py
- blue_team_robot_launcher.launch.py

  それぞれ赤チームおよび青チームに所属している際に使用するlaunchファイル：上記robot_launcher.launch.pyの立ち上げを含む
