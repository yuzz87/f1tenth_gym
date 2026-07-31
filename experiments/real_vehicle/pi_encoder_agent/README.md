# Phase 1 Encoder Agent

Raspberry PiのGPIO22/27からA/B相エンコーダを読み取り、CRC32付きUDPでPCへ送る。
Pi側はROS 2を必要としない。初期確認ではESC、ステアリング、LiDARを動かさない。

## UDP

既存のアクチュエータ`5005`、LiDAR`5010`とは分け、エンコーダは`5011/UDP`を使う。
パケットには次を含む。

```text
version, kind, session_id, sequence
source_monotonic_ns, cumulative_count, delta_count, direction
sample_period_s, pulse_age_s, invalid_transition_count
gpio_a, gpio_b, encoder_healthy, crc32
```

## PCだけでsynthetic確認

PCのターミナル1:

```bash
cd ~/Desktop/f1tenth_gym
PYTHONPATH=. gym_env/bin/python \
  experiments/real_vehicle/pi_encoder_agent/receiver.py \
  --host 0.0.0.0 --port 5011 --max-packets 20
```

PCのターミナル2:

```bash
cd ~/Desktop/f1tenth_gym
PC_ENCODER_HOST=127.0.0.1 \
PYTHONPATH=. gym_env/bin/python \
  experiments/real_vehicle/pi_encoder_agent/agent.py \
  --synthetic --max-packets 20
```

`summary accepted=20`、`sequence_gaps=0`が確認できれば、UDPとCRCの基本試験は合格。

## Piへ配置

リポジトリルートのPCで実行する。

```bash
PI_TARGET=adrccar@raspberrypi.local \
experiments/real_vehicle/integration_ws/scripts/deploy_pi_encoder_agent.sh
```

## Piでsynthetic確認

PCの受信側を先に起動してから、Piで実行する。

```bash
ssh adrccar@raspberrypi.local
cd ~/real_vehicle_pi_encoder_agent
PC_ENCODER_HOST=192.168.11.10 \
./run_encoder_agent.sh --synthetic --max-packets 100
```

## 実GPIO確認

最初は車輪を浮かせ、ESCを中立にする。GPIO信号が3.3 V対応であることと、Pi側GNDが
エンコーダGNDと共通であることを確認する。pigpiodを起動してからAgentを実行する。

```bash
ssh adrccar@raspberrypi.local
sudo pigpiod
cd ~/real_vehicle_pi_encoder_agent
PC_ENCODER_HOST=192.168.11.10 \
./run_encoder_agent.sh --rate-hz 50
```

確認順:

1. 車輪を前方向へ手で回す。
2. `count`が一方向へ変化することを確認する。
3. 車輪を後方向へ回し、変化方向が反転することを確認する。
4. 車輪を止め、`count`が止まり`pulse_age_s`だけ増えることを確認する。
5. `invalid=0`、`sequence_gaps=0`を確認する。

実機試験で、`--direction-sign -1`を指定した場合に前進がプラス、後退がマイナスになることを
確認した。現在の既定値も`-1`にしている。Phase 1では`36 count/rev`の距離換算を確定しない。
4逓倍後のカウント定義をPhase 3で実測する。
