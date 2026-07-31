# Real Vehicle ROS 2 Integration

## Phase 6 real-sensor controller dry run

The Phase 6 launch connects the real UDP LiDAR and encoder streams to the
saved-map localizer, MPC/MPPI, and Safety Node without launching an actuator
node or UDP actuator bridge.

```text
/scan + /odom -> saved-map localizer
                 -> /dry_run/pose
                 -> MPC or MPPI
                 -> /dry_run/control_raw
                 -> fault gate
                 -> Safety Node
                 -> /dry_run/control_safe
```

`/dry_run/control_safe` is intentionally different from
`/vehicle/control_safe`, so a separately running actuator bridge cannot
subscribe to Phase 6 commands. A graph guard latches an emergency stop if a
hardware-capable or mock-vehicle node is discovered.

Start the PC launch only after the standalone Pi LiDAR and encoder agents are
running:

```bash
PI_SENSOR_HOST=raspberrypi.local \
CONTROLLER_TYPE=mppi \
FAULT_MODE=none \
scripts/run_real_controller_dry_run_pc.sh
```

Record and evaluate a 60-second baseline in a second PC terminal:

```bash
scripts/run_controller_dry_run_trial.sh \
  --controller-type mppi \
  --fault-mode none \
  --trial-name phase6_mppi_baseline_trial1
```

Available explicitly triggered fault modes are `scan_timeout`,
`pose_timeout`, `control_timeout`, and `emergency_stop`. The launch and trial
must use the same mode. A 300-second baseline wrapper and MPC/MPPI comparison
tool are also installed:

```bash
scripts/run_controller_endurance_trial.sh \
  --controller-type mppi \
  --trial-name phase6_mppi_endurance_trial1

scripts/compare_controller_dry_runs.sh \
  --include-failed
```

The complete terminal order, safety checks, fault matrix, and result locations
are documented in
`experiments/docs/Research/2026-07-23/phase6_real_sensor_controller_dry_run_implementation_and_procedure.md`.

## Phase 4 real map creation

The PC can receive the real LiDAR and encoder streams, run `slam_toolbox`, and
show the occupancy map in RViz without starting any controller or actuator
bridge.

```bash
PI_SENSOR_HOST=192.168.11.2 \
  scripts/run_real_mapping_pc.sh
```

Check `/scan`, `/odom`, `/map`, diagnostics, and `map -> base_link`:

```bash
scripts/check_real_mapping.sh
```

Save both the occupancy grid and the reusable `slam_toolbox` pose graph:

```bash
scripts/save_real_map.sh small_test_area_01
```

Output is stored under
`experiments/real_vehicle/maps/small_test_area_01/`. Mapping is a hand-push
sensor test; this launch file contains no PWM output path.

## Standalone Pi LiDAR transport

The Raspberry Pi does not need ROS 2 for the real RPLIDAR path. The official
SLAMTEC SDK agent normalizes each revolution to 360 beams and sends three UDP
packets to port 5010. The PC bridge validates CRC, reassembles the scan, and
publishes `/scan`.

PC-side synthetic check:

```bash
colcon build --packages-select real_vehicle_integration --symlink-install
source scripts/ros_env.sh
scripts/test_lidar_udp_local.sh
```

PC receiver with RViz:

```bash
PI_LIDAR_HOST=raspberrypi.local START_RVIZ=true \
  scripts/run_real_lidar_pc.sh
```

The current Pi address may be used instead of `raspberrypi.local`. Leave
`PI_LIDAR_HOST` empty only during local synthetic tests; setting it rejects UDP
datagrams from other hosts.

Deploy and build the C++ agent on the Pi:

```bash
PI_TARGET=adrccar@raspberrypi.local scripts/deploy_pi_lidar_agent.sh
```

Start the Pi agent manually before enabling its systemd service:

```bash
cd ~/real_vehicle_pi_lidar_agent
PC_LIDAR_HOST=PC_IP_OR_HOSTNAME ./run_lidar_agent.sh --max-scans 100
```

The localizer can be added with `START_LOCALIZATION=true`, but it will wait for
`/odom`. Do not interpret a missing pose as a LiDAR transport fault when no
real odometry publisher is running.

Python、Raspberry Pi、ROS 2だけでRCカーのLiDAR、自己位置推定、MPC/MPPI、
安全監視、アクチュエータ境界を接続するFoxy用`ament_python`パッケージである。
SimulinkとMotiveは使用しない。

## 安全上の初期状態

- `pwm_backend: mock`
- `hardware_output_enabled: false`
- `allow_uncalibrated_speed_to_duty: false`
- Safety Nodeは`DISARMED`
- Actuator Nodeの実PWM backendは未実装

この状態ではGPIO、pigpio、PWMへ出力しない。

## 依存関係

~~~bash
source /opt/ros/foxy/setup.bash
sudo apt install ros-foxy-ackermann-msgs
sudo apt install python3-numpy python3-scipy python3-yaml
~~~

`ackermann_msgs`以外の使用メッセージは現在のPC環境で確認済みである。

## ビルド

リポジトリルートで実行する。

~~~bash
source /opt/ros/foxy/setup.bash
cd experiments/real_vehicle/integration_ws
colcon build --symlink-install
source install/setup.bash
export F1TENTH_GYM_ROOT=/home/ubuntuyuzz/Desktop/f1tenth_gym
~~~

## ハードウェアなし起動

~~~bash
ros2 launch real_vehicle_integration simulation.launch.py
~~~

標準では停止状態のため、別端末からセンサ状態を確認してarm/startする。

~~~bash
ros2 service call /safety/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /safety/start std_srvs/srv/Trigger "{}"
~~~

停止:

~~~bash
ros2 service call /safety/stop std_srvs/srv/Trigger "{}"
~~~

自動起動はシミュレーションだけで使用する。

~~~bash
ros2 launch real_vehicle_integration simulation.launch.py \
  auto_arm:=true auto_start:=true controller_type:=mppi
~~~

別端末から5秒間のROS graph検査を実行できる。

~~~bash
source /opt/ros/foxy/setup.bash
source experiments/real_vehicle/integration_ws/install/setup.bash
ros2 run real_vehicle_integration smoke_check --ros-args -p duration_s:=5.0
~~~

検査対象は`/scan`、`/odom`、推定姿勢、制御要求、安全指令、アクチュエータ診断
である。`hardware_output_seen`は`false`でなければならない。

## 緊急停止とwatchdog

緊急停止:

~~~bash
ros2 topic pub -1 /vehicle/emergency_stop std_msgs/msg/Bool "{data: true}"
~~~

解除するときは停止入力を`false`へ戻し、Safety NodeとActuator Nodeをそれぞれ
明示的にリセットする。リセット後もActuator出力はdisarm状態を維持する。

~~~bash
ros2 topic pub -1 /vehicle/emergency_stop std_msgs/msg/Bool "{data: false}"
ros2 service call /safety/reset_fault std_srvs/srv/Trigger "{}"
ros2 service call /actuator/reset_emergency_stop std_srvs/srv/Trigger "{}"
~~~

Safety Nodeからの指令が`0.10 s`途切れると、Actuator Nodeは独立して
`command_timeout`を検出し、ESC `10.30%`、ステア`10.895%`へ戻す。

## ログ

標準のCSV出力先:

~~~text
experiments/real_vehicle/results/ros2_integration/integration.csv
~~~

保存内容には推定姿勢、安全指令、アクチュエータ状態、duty候補、
`hardware_output_enabled`を含む。

## Raspberry Piへ配置する範囲

現在のPiは32-bit Raspbian 11、Python 3.9で、ROS 2は入っていない。SDカードを
再構築しない現在の構成では、ROS 2 NodeをPiへ配置しない。

```text
PC: ROS 2 Localization / Controller / Safety / Logger / UDP Bridge
Pi: standalone Python UDP Agent / pigpio / 100 ms watchdog
```

Pi Agentの実装と手順は`experiments/real_vehicle/pi_agent/README.md`を参照する。
旧`raspberry_pi_side.launch.py`と`install_raspberry_pi.sh`は、Piへ対応ROS 2環境を
用意できた場合の代替構成として残す。

PC側の新しいlaunch:

```bash
ros2 launch real_vehicle_integration hybrid_pc.launch.py \
  pi_udp_host:=raspberrypi.local
```

`allow_uncalibrated_speed_to_duty`は標準で`false`であり、未校正の正速度を実Dutyへ
変換しない。Pi Agentも標準ではドライラン・disarm・motion禁止で起動する。

## 現在の検証結果

- `colcon build --symlink-install`: 成功
- ROS統合パッケージテスト: 47件成功
- 実機シミュレーションテスト: 49件成功
- LiDAR C++ Agentの公式SDK build: 成功
- C++ AgentからPython再構成までのUDP互換試験: 成功
- 疑似UDPから`/scan`と方向判定までのROS 2統合試験: 成功
- 6ノード同時起動: 成功
- MPPI 5秒スモーク検査: 全6系統を受信
- 最大安全速度指令: 約`0.295 m/s`
- MPC 3秒スモーク検査: 全6系統を受信
- MPC 4 m直線停止位置: 約`3.972 m`
- 緊急停止後の安全速度: `0.0 m/s`
- Safety Node停止時のActuator watchdog: `command_timeout`を確認
- 実PWM出力: 無効

MPCはcontrol blockingと解析勾配追加前に50 Hz設定で約`20〜37 ms`になる周期が
あった。追加後の定常直線ベンチマークでは平均約`2.4 ms`となった。周期超過時には
従来どおり警告し、20 ms deadlineを超えた場合は周期内の最良解へfallbackする。

## MPC計算時間短縮

MPCは12ステップの予測範囲を維持し、入力列を3ブロックへまとめて最適化する。
最適化変数は`24`から`6`へ減る。比較は次で再実行できる。

~~~bash
PYTHONPATH=. gym_env/bin/python -m \
  experiments.real_vehicle.evaluation.mpc_benchmark --iterations 50
~~~

同一PCでの最終50回計測では、数値微分の平均`17.90 ms`に対して解析勾配と
control blockingは`2.41 ms`だった。SciPyを使わないprojected-gradientも
`13.04 ms`で20 ms以内だった。4 m直径の円形1周は完走し、平均`12.79 ms`、
95 percentile `19.87 ms`、最大`25.30 ms`、半径RMSE `0.0096 m`だった。

## PCとRaspberry Piの2端末構成

両端末で`ROS_DOMAIN_ID`を同じ値にし、同じLANへ接続する。まずPi側を起動する。

~~~bash
source experiments/real_vehicle/integration_ws/scripts/ros_env.sh
experiments/real_vehicle/integration_ws/scripts/run_raspberry_pi.sh \
  auto_arm:=false auto_start:=false
~~~

PC側を別端末で起動する。

~~~bash
source experiments/real_vehicle/integration_ws/scripts/ros_env.sh
experiments/real_vehicle/integration_ws/scripts/run_pc.sh controller_type:=mppi
~~~

ハードウェアなしの同一PC上で、2つのlaunchプロセスと通信障害を検証する。

~~~bash
experiments/real_vehicle/integration_ws/scripts/test_two_terminal_network.sh all
~~~

`fault`プロファイルではscanへ`0.35 s`遅延と`25%`欠損、制御指令へ`0.04 s`
遅延と`35%`欠損をseed固定で注入する。中継統計は
`/network_fault_injector/status`へ出力する。

連続欠損、通信断、遅延揺らぎ、複製・古いmessage再送もまとめて確認できる。

~~~bash
experiments/real_vehicle/integration_ws/scripts/test_two_terminal_network.sh extended
~~~

Safety Nodeはsequenceに加えて制御timestampの単調増加を検査する。再送・順序逆転を
検出した場合はFAULTへ遷移し、明示的にreset/armするまで再開しない。

Safety process停止と再起動を自動確認する。

~~~bash
experiments/real_vehicle/integration_ws/scripts/test_safety_process_restart.sh
~~~

Safety停止後のActuator `command_timeout`と、Safety再起動後が`DISARMED`で自動走行を
再開しないことを検査する。

## ROS 2診断

次のdiagnostic topicを出力する。

```text
/vehicle/controller_status
/localization/status
/vehicle/safety_status
/vehicle/actuator_status
/network_fault_injector/status
/lidar/transport_status
/lidar/direction_status
```

Controller計算時間・deadline、LiDAR一致度・有効beam数、scan/pose/control age、
Safety状態とfault理由に加え、LiDAR UDPの更新周波数、生点数、欠損、sequence gap、
Pi-PC時計差を`integration.csv`にも保存する。

## Network namespace試験

2台のLinuxホストに近い経路を同一PC内に作る。sudo、`iproute2`、`tc`が必要である。

~~~bash
cd experiments/real_vehicle/integration_ws
DELAY_MS=25 LOSS_PERCENT=1 scripts/test_network_namespaces.sh
~~~

`rv_pc`と`rv_pi` namespace、veth、multicast routeを作り、各方向へnetemを適用して
DDS smoke testを実行する。終了時はnamespaceを自動削除する。

## Raspberry Piセットアップ

この節はPi側にもROS 2 Foxyを用意できる場合だけ使用する。現在確認した32-bit
Raspbian 11環境には適用しない。SDカードを維持する場合は、上記Standalone UDP Agentを
使用する。

~~~bash
experiments/real_vehicle/integration_ws/scripts/install_raspberry_pi.sh
~~~

依存パッケージを導入してworkspaceをビルドする。GPIOピンとPWM backendが未確定の
ため、Actuator Nodeは常に`hardware_output_enabled:=false`で起動する。

## systemd

サービスを検証してインストールする。既定ではenableしない。

~~~bash
experiments/real_vehicle/integration_ws/scripts/install_systemd_service.sh
~~~

内容を確認後、自動起動を有効化する場合だけ次を使う。

~~~bash
experiments/real_vehicle/integration_ws/scripts/install_systemd_service.sh --enable
experiments/real_vehicle/integration_ws/scripts/verify_systemd_stop.sh
~~~

停止処理は最初に緊急停止topicを送信し、Actuator Nodeへ`SIGINT`を送り、backendを
中立化して閉じる。`verify_systemd_stop.sh`は停止後のinactive状態とjournal内の
`node_shutdown`を確認する。実機未接続のため、今回はサービス定義の静的検査までを
PC上で実行する。
