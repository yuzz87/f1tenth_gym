# sim_key_jog.py 実装メモ

`sim_key_jog.py` は、実車用の `timer_key_jog.py` と同じようなキーボード操作で F1TENTH Gym シミュレーションを動かすためのスクリプトです。

実車用スクリプトとの大きな違いは、`pigpio` を使って GPIO へ PWM を出力しないことです。
代わりに、キーボード入力から Gym 用の action を作り、`env.step()` へ渡します。

```python
action = np.array([[steer_rad, speed_mps]])
obs, step_reward, done, info = env.step(action)
```

## 目的

実車では ESC とステアリングサーボへ duty[%] を送ります。

一方、F1TENTH Gym は duty[%] ではなく、以下の物理量を受け取ります。

| 値          | 単位 | 内容         |
| ----------- | ---- | ------------ |
| `steer_rad` | rad  | 前輪ステア角 |
| `speed_mps` | m/s  | 目標速度     |

そのため、`sim_key_jog.py` では内部的に duty[%] 風の値を持ち、それを Gym 用の物理量へ変換しています。

## ファイル構成

関係する主なファイル:

| ファイル                        | 役割                                             |
| ------------------------------- | ------------------------------------------------ |
| `modecar/sim_key_jog.py`        | Gym シミュレーション用のキーボード操作スクリプト |
| `modecar/timer_key_jog.py`      | 実車 PWM 操作用の元スクリプト                    |
| `modecar/utilities.py`          | `getkey()` の実体                                |
| `gym/f110_gym/envs/f110_env.py` | F1TENTH Gym 環境本体                             |

`timer_key_jog.py` と `sim_key_jog.py` はどちらも `import utilities` でキー入力処理を読み込みます。

## 実行方法

通常実行:

```bash
cd ~/Desktop/f1tenth_gym/modecar
python sim_key_jog.py
```

通常実行で使う設定は、`sim_key_jog.py` 上部の設定ブロックで変更します。

```python
ACTIVE_MAP_NAME = "example"
# ACTIVE_MAP_NAME = "vegas"
# ACTIVE_MAP_NAME = "berlin"
# ACTIVE_MAP_NAME = "skirk"
# ACTIVE_MAP_NAME = "stata_basement"
# ACTIVE_MAP_NAME = "levine"

INITIAL_X = 0.7
INITIAL_Y = 0.0
INITIAL_THETA = 1.37079632679
RUN_SECONDS = 6000
LOOP_HZ = 0.0
RENDER_FPS = 30.0
VIEW_MODE = "follow"  # "full" shows the whole map, "follow" follows the car.
CAMERA_RADIUS = 800.0
MAP_PADDING = 150.0
ENABLE_RENDER = True
DARK_BACKGROUND = False
TERMINAL_POLL_HZ = 30.0
STATUS_PRINT_HZ = 2.0
HELP_PRINT_SECONDS = 30.0
```

使いたいマップの行だけコメントを外し、それ以外はコメント化しておきます。

描画なしで確認したい場合は、コード内で以下にします。

```python
ENABLE_RENDER = False
```

CPU負荷を下げたい場合は、コード内で以下の値を下げます。

```python
LOOP_HZ = 20.0
RENDER_FPS = 10.0
```

マップ全体を表示する場合:

```python
VIEW_MODE = "full"
```

車両を追従表示する場合:

```python
VIEW_MODE = "follow"
```

## 同梱マップ

このリポジトリには以下のマップがあります。

| マップ名 | 画像 |
| --- | --- |
| `example` | `examples/example_map.png` |
| `vegas` | `vegas.png` |
| `berlin` | `berlin.png` |
| `skirk` | `skirk.png` |
| `stata_basement` | `stata_basement.png` |
| `levine` | `levine.pgm` |

## CPU負荷と描画

F1TENTH Gym の描画は pyglet/OpenGL で行われます。
毎ループ `env.render()` を呼ぶと CPU 使用率が高くなりやすいため、`sim_key_jog.py` では以下の制限を入れています。
また、元のレンダラはマップの障害物ピクセルを 1 点ずつ OpenGL batch に登録していたため、`vegas` のような大きいマップでは初回描画が非常に重くなります。
現在は描画用マップ点を最大 80000 点に間引き、一括で batch 登録するように変更しています。

| 設定値 | デフォルト | 内容 |
| --- | ---: | --- |
| `ACTIVE_MAP_NAME` | `example` | 同梱マップ名 |
| `SPEED_DELTA_JOG` | `0.08` | 速度キー1回あたりの duty 調整量 |
| `STEER_DELTA_JOG` | `0.02` | ステアキー1回あたりの duty 調整量 |
| `MAX_FORWARD_SPEED_MPS` | `0.5` | 最大前進速度 |
| `MAX_REVERSE_SPEED_MPS` | `0.25` | 最大後退速度 |
| `RUN_SECONDS` | `6000` | 最大実行時間 |
| `LOOP_HZ` | `0.0` | メインループの最大周波数。`0.0` は待ち時間なし |
| `RENDER_FPS` | `30.0` | 描画の最大 FPS |
| `VIEW_MODE` | `follow` | `full` は全マップ表示、`follow` は車両追従 |
| `CAMERA_RADIUS` | `800.0` | 車両追従カメラの表示範囲 |
| `MAP_PADDING` | `150.0` | マップ全体表示時の余白 |
| `ENABLE_RENDER` | `True` | 描画の有効/無効 |
| `DARK_BACKGROUND` | `False` | 元の濃紺背景を使う |
| `TERMINAL_POLL_HZ` | `30.0` | ターミナルキー入力の確認頻度 |
| `STATUS_PRINT_HZ` | `2.0` | 状態表示の頻度 |
| `HELP_PRINT_SECONDS` | `30.0` | ヘルプ再表示の間隔 |

CPU が高い場合は、まず以下のように下げます。

```python
LOOP_HZ = 20.0
RENDER_FPS = 10.0
```

さらに軽くしたい場合:

```python
LOOP_HZ = 10.0
RENDER_FPS = 5.0
```

描画ウィンドウは開くが何も見えない場合は、車両やマップがカメラ範囲外にある可能性があります。
現在の実装では車両を追従するカメラ callback を追加しています。
車が小さすぎる場合は、値を小さくして拡大表示します。

```python
CAMERA_RADIUS = 80.0
```

画面が近すぎる場合や、周囲の壁やコースまで広く見たい場合は、値を大きくします。

```python
CAMERA_RADIUS = 1000.0
```

マップ全体を見たい場合は、追従カメラではなく full view を使います。

```python
VIEW_MODE = "full"
```

余白を広げたい場合:

```python
MAP_PADDING = 300.0
```

画面が暗く見えすぎる問題を避けるため、`gym/f110_gym/envs/rendering.py` の背景を明るいグレーに変更しています。
地図点は濃いグレー、車体は赤、文字は黒で表示します。
元の F1TENTH Gym の濃紺背景を使いたい場合は以下を指定します。

```python
DARK_BACKGROUND = True
```

## キー操作

キー入力は、ターミナルにフォーカスがある場合と、描画ウィンドウにフォーカスがある場合の両方で受け付けます。
描画ウィンドウをクリックした後は、そのウィンドウ上で `w/a/s/d/n` を押してください。

| キー   | 動作                                           |
| ------ | ---------------------------------------------- |
| `w`    | 前進方向へ速度を上げる                         |
| `s`    | 後退方向へ速度を上げる、または前進速度を下げる |
| `a`    | 左へステアリングを切る                         |
| `d`    | 右へステアリングを切る                         |
| `n`    | 速度とステアリングを中立へ戻す                 |
| Enter  | 終了                                           |
| Esc    | 終了                                           |
| Ctrl-C | 終了                                           |

## 実装の流れ

### 1. Gym 環境を作成

```python
settings = get_settings()

env = gym.make(
    "f110_gym:f110-v0",
    map=settings.map_path,
    map_ext=settings.map_ext,
    num_agents=1,
    timestep=settings.timestep,
    integrator=Integrator.RK4,
)
```

`f110_gym` を import することで、`f110_gym:f110-v0` が Gym に登録されます。

### 2. 初期姿勢を設定

```python
poses = np.array([[settings.x, settings.y, settings.theta]])
obs, step_reward, done, info = env.reset(poses)
```

`num_agents=1` なので、姿勢は 1 台分だけ渡します。

形式:

```python
np.array([[x, y, theta]])
```

### 3. キー入力を読む

```python
key = read_key()
```

`read_key()` は、対話端末で実行されている場合だけ `utilities.getkey()` を呼びます。
非対話実行の場合は `0` を返します。

これは、非対話実行時に `termios` エラーで落ちないようにするためです。

### 4. duty 風の指令値を更新

```python
if key == ord("w"):
    spd_ref -= SPEED_DELTA_JOG * 2
elif key == ord("s"):
    spd_ref += SPEED_DELTA_JOG * 2
elif key == ord("a"):
    str_ref -= STEER_DELTA_JOG * 4
elif key == ord("d"):
    str_ref += STEER_DELTA_JOG * 4
elif key == ord("n"):
    str_ref = PWM_NEUTRAL_STR
    spd_ref = PWM_NEUTRAL_SPD
```

この部分は `timer_key_jog.py` の操作感に合わせています。

### 5. duty 風の値を Gym action へ変換

```python
steer_rad = duty_to_steer_rad(str_ref)
speed_mps = duty_to_speed_mps(spd_ref)
action = np.array([[steer_rad, speed_mps]])
```

Gym は duty[%] を理解しないため、ここで物理量へ変換します。

### 6. シミュレーションを 1 ステップ進める

```python
obs, step_reward, done, info = env.step(action)
```

`done` が True になるとループを終了します。
衝突や周回完了などで True になります。

### 7. 描画する

```python
env.render(mode="human")
```

`ENABLE_RENDER = False` の場合は描画しません。
また、描画に失敗した場合は自動で描画を無効化して続行します。

## duty からステア角への変換

現在の仮設定:

```python
PWM_NEUTRAL_STR = 10.88
STR_LEFT_DUTY = 8.88
STR_RIGHT_DUTY = 12.88
MAX_STEER_RAD = 0.4189
```

中立 duty を `0 rad` として、左端・右端まで線形に変換しています。

注意:

- `a` で `str_ref` が小さくなる
- `d` で `str_ref` が大きくなる
- 現在の実装では、左を正の `steer_rad`、右を負の `steer_rad` としている

変換式の考え方:

```text
左方向:
steer_rad = (neutral_str - str_duty) / (neutral_str - left_duty) * max_steer

右方向:
steer_rad = -(str_duty - neutral_str) / (right_duty - neutral_str) * max_steer
```

## duty から速度への変換

現在の仮設定:

```python
PWM_NEUTRAL_SPD = 10.55
SPD_FORWARD_DUTY = 9.50
SPD_REVERSE_DUTY = 11.50
MAX_FORWARD_SPEED_MPS = 0.5
MAX_REVERSE_SPEED_MPS = 0.25
```

`timer_key_jog.py` では `w` を押すと `spd_ref` が小さくなります。
そのため、`PWM_NEUTRAL_SPD` より小さい duty を前進として扱っています。

変換式の考え方:

```text
前進:
speed_mps = (neutral_spd - spd_duty) / (neutral_spd - forward_duty) * max_forward_speed

後退:
speed_mps = -(spd_duty - neutral_spd) / (reverse_duty - neutral_spd) * max_reverse_speed
```

## 現在の制限

このスクリプトの duty 変換は、実車校正値に基づく厳密な変換ではありません。

現在はシミュレーション操作用の仮変換です。

今後、実車とシミュレーションの操作感を近づけるには、以下の測定が必要です。

| 測定項目         | 必要な値                  |
| ---------------- | ------------------------- |
| ステアリング中立 | duty[%] と `0 rad` の対応 |
| 左最大角         | duty[%] と実ステア角[rad] |
| 右最大角         | duty[%] と実ステア角[rad] |
| ESC 中立         | duty[%] と `0 m/s` の対応 |
| 低速前進         | duty[%] と実速度[m/s]     |
| 中速前進         | duty[%] と実速度[m/s]     |
| 低速後退         | duty[%] と実速度[m/s]     |

## 確認コマンド

構文チェック:

```bash
cd ~/Desktop/f1tenth_gym
./gym_env/bin/python -m py_compile modecar/sim_key_jog.py
```

短時間のシミュレーション確認をしたい場合は、`sim_key_jog.py` 上部で以下にしてから実行します。

```python
RUN_SECONDS = 5.0
ENABLE_RENDER = False
```

```bash
cd ~/Desktop/f1tenth_gym/modecar
python sim_key_jog.py
```

## 今後の改善案

- duty 変換値をコード直書きではなく設定ファイルへ移す
- 実車校正値から `duty_to_steer_rad()` と `duty_to_speed_mps()` を更新する
- ログ CSV を出力して、`time, str_ref, spd_ref, steer_rad, speed_mps, x, y, theta` を保存する
- `MAX_FORWARD_SPEED_MPS` や `MAX_STEER_RAD` を実車校正値に合わせる
