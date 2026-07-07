# シミュレーション仕様書

## 目的

この文書は、OpenMiniCarWorks の実車校正データを F1TENTH Gym シミュレーションへ取り入れるための仕様をまとめる。

対象は次である。

- `examples/waypoint_follow.py` による Pure Pursuit 走行シミュレーション
- `modecar/sim_key_jog.py` によるキーボードジョグ走行シミュレーション
- ステアリング duty 比[%]と前輪ステア角[rad]の変換
- ESC duty 比[%]と速度[m/s]の変換

この文書で扱う単位は SI 単位を基本とする。角度は、実測表では度[deg]、制御・シミュレーション内部ではラジアン[rad]を使う。

## 現状の前提

このリポジトリの F1TENTH Gym は、PWM duty 比[%]ではなく次の action を受け取る。

```python
action = np.array([[steer_rad, speed_mps]])
obs, step_reward, done, info = env.step(action)
```

| 要素 | 単位 | 内容 |
| --- | --- | --- |
| `steer_rad` | rad | 目標前輪ステア角 |
| `speed_mps` | m/s | 目標車速 |

実車側の PWM 出力は duty 比[%]で管理しているため、実車と同じ操作量でシミュレーションするには次の変換層が必要である。

```text
steering duty[%] -> steer_rad[rad] -> F1TENTH Gym
ESC duty[%]      -> speed_mps[m/s]  -> F1TENTH Gym
```

この変換層は、Gym の物理モデル本体ではなく、`env.step()` に渡す直前のアダプタとして実装する。

## 関連ファイル

| ファイル | 役割 |
| --- | --- |
| `examples/waypoint_follow.py` | waypoint CSV を Pure Pursuit で追従するサンプル |
| `examples/config_example_map.yaml` | `waypoint_follow.py` のマップ、初期姿勢、waypoint 設定 |
| `modecar/sim_key_jog.py` | duty 風の値をキーボードで変化させて Gym を動かすスクリプト |
| `modecar/timer_key_jog.py` | 実車へ pigpio で PWM duty を出すジョグスクリプト |
| `modecar/utilities.py` | 非ブロッキングキー入力 |
| `gym/f110_gym/envs/f110_env.py` | Gym 環境本体 |
| `gym/f110_gym/envs/base_classes.py` | RaceCar と Simulator |
| `gym/f110_gym/envs/dynamic_models.py` | 車両運動モデルと PID 変換 |
| `docs/MD/modecar/duty_angle.md` | ステアリング duty 比と実測ステア角 |
| `docs/MD/modecar/duty_speed.md` | ESC duty 比と実測速度 |

## 実行仕様

### `waypoint_follow.py`

`examples/waypoint_follow.py` は `config_example_map.yaml` と waypoint CSV を相対パスで読む。そのため、必ず `examples` ディレクトリから実行する。

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym
source gym_env/bin/activate
cd examples
python3 waypoint_follow.py
```

仮想環境を activate しない場合は次で実行する。

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym/examples
../gym_env/bin/python waypoint_follow.py
```

このスクリプトは GUI 描画を使う。画面がない環境、SSH、OpenGL が使えない環境では `DISPLAY` や pyglet/OpenGL 関連で失敗する可能性がある。

### `sim_key_jog.py`

`modecar/sim_key_jog.py` は実車用 `timer_key_jog.py` に近いキーボード操作で Gym を動かす。

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym
source gym_env/bin/activate
cd modecar
python3 sim_key_jog.py
```

または次で実行する。

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym/modecar
../gym_env/bin/python sim_key_jog.py
```

キー操作は次である。

| キー | 動作 |
| --- | --- |
| `w` | 前進方向へ速度 duty 風指令を変化させる |
| `s` | 後退方向、または前進減速方向へ速度 duty 風指令を変化させる |
| `a` | 左方向へステア duty 風指令を変化させる |
| `d` | 右方向へステア duty 風指令を変化させる |
| `n` | 速度とステアリングを中立へ戻す |
| Enter | 終了 |
| Esc | 終了 |
| Ctrl-C | 終了 |

## ステアリング変換仕様

PWM 周波数は `70 Hz` とする。

実測データの有効範囲はおおむね次である。

```text
-18 deg <= steer_angle_deg <= 18 deg
-0.314159265 rad <= steer_angle_rad <= 0.314159265 rad
```

ステアリング duty 比[%]からステア角[rad]への線形近似は次である。

```text
steer_angle_rad = -0.186785136654 * steering_duty_percent + 2.018473280947
```

ステア角[rad]からステアリング duty 比[%]への逆変換は次である。

```text
steering_duty_percent = (target_steer_rad - 2.018473280947) / -0.186785136654
```

シミュレーションへ入れる場合は、ステアリング duty 比を上式で `steer_rad` へ変換し、Gym action の第1要素へ渡す。

```python
steer_rad = steering_duty_to_rad(steering_duty_percent)
action = np.array([[steer_rad, speed_mps]])
```

実装時は、変換後のステア角を実測範囲にクリップする。

```text
-0.314159265 rad <= steer_rad <= 0.314159265 rad
```

### ステアリング中立の扱い

実測中立 duty 比は次である。

```text
measured_neutral_duty_percent = 10.895 %
```

一方、線形式で `0 rad` を逆変換すると次である。

```text
fit_zero_angle_duty_percent = 10.806391330 %
```

差は `0.088608670 %`、70 Hz では約 `12.66 us` である。

そのため、シミュレーションと実車操作では次を区別する。

| 用途 | 値 |
| --- | ---: |
| 実車の手動中立、フェイルセーフ中立 | `10.895 %` |
| 線形式で `0 rad` を指令する duty 比 | `10.806391330 %` |
| Gym に渡す中立ステア角 | `0.0 rad` |

`sim_key_jog.py` の `n` キーでは、最終的に Gym へ `0.0 rad` を渡す。実車 PWM の中立 duty 比と、Gym の中立角は混同しない。

## ESC 速度変換仕様

PWM 周波数は `70 Hz` とする。

現在の ESC 実測データは前進方向のみであり、実測範囲は次である。

```text
9.7 % <= esc_duty_percent <= 10.1 %
0.448775 m/s <= velocity_mps <= 1.429766667 m/s
```

ESC duty 比[%]から速度[m/s]への線形近似は次である。

```text
velocity_mps = -2.4618 * esc_duty_percent + 25.347
```

速度[m/s]から ESC duty 比[%]への逆変換は次である。

```text
esc_duty_percent = (25.347 - target_velocity_mps) / 2.4618
```

シミュレーションへ入れる場合は、ESC duty 比を上式で `speed_mps` へ変換し、Gym action の第2要素へ渡す。

```python
speed_mps = esc_duty_to_speed_mps(esc_duty_percent)
action = np.array([[steer_rad, speed_mps]])
```

### 停止と低速域の扱い

ESC 中立 duty 比は次である。

```text
ESC neutral duty = 10.55 %
```

停止指令 `0.0 m/s` は線形式で外挿しない。ESC 中立 duty 比を停止として扱い、Gym へは `0.0 m/s` を渡す。

`0.10 m/s` や `0.30 m/s` のような低速は現在の実測範囲外である。低速域をシミュレーションへ入れる場合は、次のどちらかを明示的に選ぶ。

| 方針 | 内容 |
| --- | --- |
| 実測範囲のみ | `9.7 %`から`10.1 %`だけ線形式を使い、中立付近は `0.0 m/s` とする |
| 仮外挿 | 中立 `10.55 %` から実測最小 `10.1 %` までを仮に補間する |

初期実装では「実測範囲のみ」を標準とする。仮外挿は、コード上で設定名を付けて明示的に有効化する。

## 推奨アダプタ設計

校正値は `modecar/sim_key_jog.py` に直接散らさず、共通モジュールへ切り出す。

推奨ファイル:

```text
modecar/calibration.py
```

推奨関数:

```python
def steering_duty_to_rad(steering_duty_percent):
    ...

def steering_rad_to_duty(target_steer_rad):
    ...

def esc_duty_to_speed_mps(esc_duty_percent):
    ...

def speed_mps_to_esc_duty(target_velocity_mps):
    ...
```

このモジュールは GPIO、pigpio、Gym に依存しない純粋な変換関数だけを持つ。これにより、実車スクリプトとシミュレーションスクリプトの校正値を揃えやすくする。

## 車両パラメータ仕様

OpenMiniCarWorks の既知の前提値は次である。

```text
wheelbase = 0.25 m
wheel_diameter = 0.066 m
encoder_teeth = 36
```

F1TENTH Gym のデフォルト車両パラメータは F1TENTH 寄りである。OpenMiniCarWorks の実車に寄せる場合、少なくとも次を変更候補とする。

| パラメータ | 推奨値または扱い |
| --- | --- |
| `lf + lr` | `0.25 m` |
| `s_min` | `-0.314159265 rad` |
| `s_max` | `0.314159265 rad` |
| `v_max` | 実測最大付近の `1.43 m/s` から開始 |
| `v_min` | 後退実測がないため、最初は `0.0 m/s` または保守的な低値 |
| `length` | 実車寸法が未記録なら仮定値として明記 |
| `width` | 実車寸法が未記録なら仮定値として明記 |
| `m` | 実車質量が未記録なら仮定値として明記 |
| `a_max` | 実測がないため既定値または保守的な値 |
| `sv_max` | サーボ応答実測がないため既定値または保守的な値 |

寸法、質量、ステアリング応答、加速度上限はリポジトリ内に確定値がない場合、実装・文書内で「仮定」と明記する。

## 実装方針

### 第1段階: `sim_key_jog.py` へ反映

最初に `modecar/sim_key_jog.py` の duty 風変換を実測式に置き換える。

理由:

- すでに duty 風の `str_ref` と `spd_ref` を持っている
- Gym action へ渡す直前に変換している
- Gym 本体の物理モデルを変更しないため影響範囲が小さい
- 実車 GPIO 出力を伴わない

### 第2段階: 共通校正モジュール化

`modecar/calibration.py` を追加し、実車・シミュレーション双方から同じ変換関数を使う。

この段階で、変換関数に対する軽いチェックを追加する。

期待値の例:

```text
18.0 deg  -> 9.124462718 %
9.4 deg   -> 9.928050831 %
0.0 deg   -> 10.806391330 %
-9.0 deg  -> 11.647355630 %
-18.0 deg -> 12.488319940 %
10.0 % ESC duty -> 0.729000000 m/s
```

### 第3段階: `waypoint_follow.py` との接続

Pure Pursuit が出す `steer_rad` と `speed_mps` をそのまま Gym に入れる経路とは別に、実車 PWM を通した場合の飽和・中立差・実測範囲を模擬する経路を追加する。

候補:

```text
planner output[rad,m/s]
  -> rad/mps to duty
  -> duty clamp
  -> duty to rad/mps
  -> env.step()
```

この経路により、理想的な制御指令ではなく、実車 PWM 校正を通した後の指令で走行するシミュレーションができる。

## 検証仕様

Python の構文チェック:

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym
./gym_env/bin/python -m py_compile examples/waypoint_follow.py
./gym_env/bin/python -m py_compile modecar/*.py
```

`waypoint_follow.py` の実行:

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym/examples
../gym_env/bin/python waypoint_follow.py
```

`sim_key_jog.py` の実行:

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym/modecar
../gym_env/bin/python sim_key_jog.py
```

変換関数を追加した場合は、少なくとも次を確認する。

- ステア角 `0.0 rad` の逆変換が `10.806391330 %` 付近になる
- 実測中立 duty `10.895 %` と Gym 中立角 `0.0 rad` を混同していない
- ステア角が `-0.314159265 rad` から `0.314159265 rad` に収まる
- ESC duty `10.0 %` が約 `0.729 m/s` になる
- ESC 中立 `10.55 %` は `0.0 m/s` として扱う
- 低速域を外挿している場合は、その設定名とリスクが文書化されている

## 安全上の注意

この仕様は PC 上の F1TENTH Gym シミュレーションを対象とする。実車へ PWM を出力する仕様ではない。

実車に関係する作業では次を守る。

- 明示確認なしに実車出力を有効化しない
- `pigpio` や GPIO を PC 側検証で使わない
- ESC 中立、ステアリング中立、PWM 安全範囲を勝手に広げない
- 実車確認では駆動輪を浮かせ、低スロットル、物理電源遮断手段を用意する
- Raspberry Pi GPIO は 3.3 V ロジックであり、5 V 出力を直接入力しない
- Raspberry Pi、ESC、サーボ電源、エンコーダの GND は共通化する

## 未解決事項

- ESC の中立付近、低速前進の実測データが不足している
- 後退方向の ESC duty 比と速度の実測データがない
- 実車の車体寸法、質量、重心、ステアリング応答速度が未確定である
- タイヤすべり、サーボ遅れ、リンケージのバックラッシュは現在の線形校正に含まれない
- `waypoint_follow.py` へ実車 PWM 校正経路を入れるか、`sim_key_jog.py` 専用に留めるかは実装時に選ぶ

