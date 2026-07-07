# modecar/timer_key_jog.py を使用するために必要なもの

`timer_key_jog.py` は、Raspberry Pi から `pigpio` を使って ESC とステアリングサーボへ PWM を直接出力するジョグ操作用スクリプトです。

一方、F1TENTH Gym の `env.step()` は PWM duty[%] ではなく、次の物理量を受け取ります。

```python
np.array([[steer_rad, speed_mps]])
```

そのため、実車で動かす場合と Gym シミュレータで使う場合では、足りないものが少し違います。

## 1. 実車で動かすために必要なもの

### Raspberry Pi と pigpio

必要なもの:

- Raspberry Pi
- `pigpio`
- `pigpiod` daemon の起動
- GPIO 12 を ESC へ接続
- GPIO 13 をステアリングサーボへ接続

実行前に以下を行います。

```bash
sudo apt-get install pigpio
sudo pigpiod
```

### utilities.py

`timer_key_jog.py` は以下を import しています。

```python
import utilities
```

この中の `utilities.getkey()` がキー入力取得に使われています。
現状、このリポジトリ内には `utilities.py` が見当たらないため、実行には不足しています。

必要な機能:

- キー入力を非ブロッキングで読む
- 入力がなければ `0` を返す
- Enter が押されたら `10` を返す
- `w`, `s`, `a`, `d`, `n` の ASCII コードを返す

### 実車ごとの PWM 校正

現在の主な PWM 設定:

```python
PWM_Hz = 70
PWM_NeutralStr = 10.88
PWM_NeutralSpd = 10.55
PWM_SAFE_MIN = 7.50
PWM_SAFE_MAX = 13.00
```

これらは車体・ESC・サーボ・電源状態によって変わるため、実車で校正が必要です。

確認したい値:

- ステアリングがまっすぐになる duty[%]
- 左最大角の duty[%]
- 右最大角の duty[%]
- ESC が停止する duty[%]
- 前進し始める duty[%]
- 後退し始める duty[%]
- 安全に使える duty[%] の最小値・最大値

## 2. Gym シミュレータで使うために必要なもの

F1TENTH Gym の action は duty[%] ではありません。

`env.step()` に渡す値は以下です。

```python
action = np.array([[steer_rad, speed_mps]])
obs, reward, done, info = env.step(action)
```

そのため、`timer_key_jog.py` の `str_ref` と `spd_ref` をそのまま Gym に渡すことはできません。

必要な変換:

1. ステアリング duty[%] から実ステア角[rad] への変換
2. ESC duty[%] から実速度[m/s] への変換
3. duty[%] 指令を Gym 用の物理量へ変換するアダプタ

変換アダプタのイメージ:

```python
def duty_to_steer_rad(str_duty):
    # TODO: 実車校正結果から変換式を作る
    return steer_rad


def duty_to_speed_mps(spd_duty):
    # TODO: 実車校正結果から変換式を作る
    return speed_mps


action = np.array([[
    duty_to_steer_rad(str_ref),
    duty_to_speed_mps(spd_ref),
]])
```

## 3. 優先して測定する校正データ

### ステアリング duty[%] と実ステア角[rad]

最低限、以下の 3 点を測定します。

| 状態 | duty[%] | 実ステア角[rad] |
| --- | ---: | ---: |
| 左最大 | TODO | TODO |
| 中立 | 10.88 | 0.0 |
| 右最大 | TODO | TODO |

まずは線形近似でよいです。

```text
steer_rad = gain_str * (str_duty - neutral_str)
```

### ESC duty[%] と実速度[m/s]

最低限、以下を測定します。

| 状態 | duty[%] | 実速度[m/s] |
| --- | ---: | ---: |
| 停止 | 10.55 | 0.0 |
| 低速前進 | TODO | TODO |
| 中速前進 | TODO | TODO |
| 低速後退 | TODO | TODO |

ESC は不感帯や前後非対称があるため、ステアリングより単純な線形にならない可能性があります。

最初は前進だけを対象にして、以下のような簡単な近似から始めるのが安全です。

```text
speed_mps = gain_spd * (spd_duty - neutral_spd)
```

必要に応じて、前進用・後退用で別々の gain を持たせます。

## 4. まとめ

不足している可能性が高いもの:

- `utilities.py`
- `pigpio` と `pigpiod`
- Raspberry Pi の GPIO 配線
- ステアリング duty[%] と実ステア角[rad] の対応表
- ESC duty[%] と実速度[m/s] の対応表
- duty[%] を Gym の action へ変換するアダプタ

実車操作だけなら、まず `utilities.py` と pigpio 環境が必要です。

Gym とつなぐなら、PWM 出力部分を `env.step()` に置き換え、`str_ref` と `spd_ref` を `steer_rad` と `speed_mps` に変換する必要があります。
