# Raspberry Pi Standalone UDP Actuator Agent

現在のRaspberry Pi OSを再構築せず、PC上のROS 2制御系からUDPで安全指令を受ける
Python 3.9対応Agentである。Pi側にROS 2は不要で、標準ライブラリだけでドライランできる。
実PWMを使う場合だけ既存の`pigpio` Python moduleと`pigpiod`を使用する。

## 安全な初期状態

- デフォルトはドライランであり、GPIOへ出力しない。
- `--enable-hardware`だけでは駆動指令を許可しない。
- 実PWMには、車輪浮上または床上走行範囲と、物理電源遮断の確認フラグが必要である。
- 駆動にはさらに`--arm-output --allow-motion`の両方が必要である。
- ESC駆動Dutyは`10.10〜10.16%`だけを許可する。
- 停止時はESC `10.30%`、ステアリング`10.895%`へ戻す。
- 指令が`0.20 s`途切れるとPi単独で中立へ戻す。
- timeout済みsessionから遅れて届いたpacketは再受理しない。
- 古いtimestamp、重複sequence、別session、別送信元IPを拒否する。

UDPは暗号化・認証を行わない。車両試験専用の信頼できるLAN内だけで使用し、UDP
port `5005`をインターネットへ公開しない。

## PCから配置

リポジトリルートで実行する。Wi-FiのIPが変わってもmDNSが利用できれば
`raspberrypi.local`で接続できる。

```bash
PI_TARGET=adrccar@raspberrypi.local \
experiments/real_vehicle/integration_ws/scripts/deploy_pi_udp_agent.sh
```

mDNSが使えない場合だけ、現在のIPを指定する。

```bash
PI_TARGET=adrccar@192.168.11.11 \
experiments/real_vehicle/integration_ws/scripts/deploy_pi_udp_agent.sh
```

## Piでドライラン

```bash
ssh adrccar@raspberrypi.local
cd ~/real_vehicle_pi_agent
./run_agent.sh
```

表示に`mode=dry-run`、`armed=False`、`allow_motion=False`、
`watchdog_timeout_s=0.200`が含まれることを確認する。
この状態ではどのUDP指令を受けてもGPIOへ出力しない。

`0.20 s`は実Wi-Fi上の60秒試験を3回実施し、20 Hzの3600指令を全受理した
実測値である。`0.10 s`は3回中1回で走行中watchdogが作動したため使用しない。

PCの別端末から通信だけを確認する。

```bash
cd ~/Desktop/f1tenth_gym
PYTHONPATH=. gym_env/bin/python -m \
  experiments.real_vehicle.pi_agent.send_test_command raspberrypi.local
```

## PCのROS 2 Bridge

PCでworkspaceを再ビルドしてから起動する。

```bash
source /opt/ros/foxy/setup.bash
cd ~/Desktop/f1tenth_gym/experiments/real_vehicle/integration_ws
colcon build --symlink-install
source install/setup.bash
export F1TENTH_GYM_ROOT=~/Desktop/f1tenth_gym
ros2 launch real_vehicle_integration hybrid_pc.launch.py \
  pi_udp_host:=raspberrypi.local
```

標準の`allow_uncalibrated_speed_to_duty:=false`では、制御器が正速度を要求してもPiへは
中立指令しか送らない。これは地上速度からESC Dutyへの校正が未完了だからである。

## 実PWMの段階確認

最初は車輪浮上、車体固定、物理電源遮断を準備し、駆動を許可せず中立PWMだけを出す。

```bash
./run_agent.sh --enable-hardware \
  --confirm-wheels-lifted --confirm-power-cutoff
```

この段階では`armed=False`、`allow_motion=False`を維持する。GPIO 12/13の実配線、共通
GND、中立時の車輪停止、Ctrl-Cまたはsystemd停止時の中立復帰を確認するまで、
`--arm-output --allow-motion`を追加しない。

床上の短時間試験では、車輪浮上確認の代わりに走行範囲と物理電源遮断を確認する。

```bash
./run_agent.sh --enable-hardware \
  --arm-output --allow-motion \
  --confirm-track-clear --confirm-power-cutoff
```

`--confirm-track-clear`は、人や障害物がなく、車両へすぐ到達して電源を切れる床上試験で
のみ使用する。通常の単体確認では引き続き`--confirm-wheels-lifted`を使用する。

## systemd

同梱serviceは意図的にドライラン固定で、インストールしても自動enableしない。

```bash
cd ~/real_vehicle_pi_agent
./install_systemd_service.sh
sudo systemctl start real-vehicle-pi-agent.service
systemctl status real-vehicle-pi-agent.service
sudo systemctl stop real-vehicle-pi-agent.service
```

実PWM用serviceへの変更は、手動の中立・watchdog・停止試験が完了した後に別Stepで行う。
