# Controller Comparison Plan

## 目的

`Pure Pursuit` のベースライン計測ができたので、次の段階では詳細なパラメータ調整に入る前に、複数の制御手法を同じ実験系で試せる状態を作る。

この計画の対象は主に次の 3 手法とする。

- `Pure Pursuit`
- `MPC`
- `MPPI`

目的は「最適な設定を探すこと」ではなく、「同じ条件で差し替えて動かし、比較できる基盤を作ること」である。

## 基本方針

- 実装はまず `experiments/` 配下だけで完結させる
- 既存の `homur_oval` 実験系を共通ベンチマークとして使う
- 走行ログ、グラフ、1周判定の仕組みはそのまま流用する
- 先に controller 切替構造を作り、その後で各手法を足す
- 高度な最適化や詳細チューニングは後回しにする

## なぜこの順番か

今の時点で `Pure Pursuit` は 1 周安定して走れており、ログ取得と可視化もできている。したがって次に必要なのは、手法ごとの性能差を見るための比較軸と実験導線である。

ここでいきなり `MPPI` や `MPC` を個別実装し始めると、コードの入口やログ形式が手法ごとにばらけやすい。先に共通構造を作っておけば、あとから手法を増やしても比較しやすい。

## ゴール

最初のゴールは次の状態である。

1. `Pure Pursuit`, `MPC`, `MPPI` を config だけで切り替えられる
2. どの手法でも同じログ形式で CSV を保存できる
3. どの手法でも同じグラフ生成スクリプトで結果確認できる
4. `homur_oval` で最低限 1 周試せる

## 実装対象

### 実行スクリプト

- `experiments/run_homur_f110.py`

### 新しい controller 置き場

- `experiments/controllers/`

候補ファイル:

- `experiments/controllers/pure_pursuit_controller.py`
- `experiments/controllers/mpc_controller.py`
- `experiments/controllers/mppi_controller.py`
- `experiments/controllers/controller_factory.py`

### 設定ファイル

- `experiments/configs/homur_oval_pure_pursuit.yaml`
- `experiments/configs/homur_oval_mpc.yaml`
- `experiments/configs/homur_oval_mppi.yaml`

## フェーズ 1: controller 切替構造を作る

### 目的

既存の `run_homur_f110.py` が `Pure Pursuit` に固定されている状態をやめて、controller を選択可能にする。

### 対応内容

- config に `controller_type` を追加する
- `controller_factory` で `controller_type` に応じて controller を生成する
- `run_homur_f110.py` 側は `controller.plan(...)` だけを呼ぶ形にする

### 目標インターフェース

controller は少なくとも以下を持つ。

- `plan(obs) -> (speed, steer)`
- `render_waypoints(env_renderer)` または描画不要なら no-op

可能なら将来用に以下も考慮する。

- `reset()`
- `debug_info()`

### 完了条件

- `Pure Pursuit` を新構造経由で走らせても現状と同じ結果が出る

## フェーズ 2: Pure Pursuit を controller 化する

### 目的

既存ベースラインを新構造で動かして、以後の比較基準にする。

### 対応内容

- 現在の `PurePursuitPlanner` 呼び出しをラップする controller を作る
- 既存の waypoint 読み込みや描画ロジックは再利用する
- 出力 `(speed, steer)` はそのまま維持する

### 完了条件

- 既存の `homur_oval` ベースライン結果と一致する
- 1 周ログの値が変わらない

## フェーズ 3: MPC の最小実装

### 目的

まずは「動く最小構成の MPC」を作る。

### 初期方針

- モデルは簡易な kinematic bicycle を使う
- まずは単一車両、障害物なし、楕円コース追従だけに絞る
- 入力は `steer` と `speed`、または `steer_rate` と `accel` の簡易形で始める

### 最初のコスト候補

- 中心線からの横偏差
- heading 誤差
- 操舵入力の大きさ
- 操舵変化量

### 実装上の注意

- まずは最適性より安定して解けることを優先する
- 高度な制約や長い horizon は後回しにする
- solver 依存は慎重に選ぶ

### 完了条件

- `homur_oval` 上で少なくとも一周を試せる
- 同じ CSV 形式でログが取れる

## フェーズ 4: MPPI の最小実装

### 目的

次に「動く最小構成の MPPI」を作る。

### 初期方針

- 予測モデルは MPC と同様に簡易モデルから始める
- 最初は操舵中心の最適化でもよい
- 速度は固定、または単純な target speed 追従で十分

### 最初に持つべき主要パラメータ

- horizon
- num_samples
- lambda
- control_noise
- cost weights

### コスト候補

- 横偏差
- heading 誤差
- 制御入力の変化量
- 過大な操舵へのペナルティ

### 完了条件

- `homur_oval` 上で少なくとも一周を試せる
- 同じ CSV 形式でログが取れる

## フェーズ 5: 同一条件比較

### 比較条件

- 同じ map
- 同じ初期姿勢
- 同じ `lap-target`
- 同じログ形式
- 同じグラフ出力

### 最初に見る比較項目

- 1 周完了できるか
- ラップタイム
- `max_abs_cross_track`
- `mean_abs_cross_track`
- `max_abs_heading`
- `mean_abs_heading`
- 操舵の振れ方

### 注意

最初の段階では「最速」を競わない。まずは「走れるか」「比較可能か」を確認する。

## 実装順

1. `experiments/controllers/` を作る
2. `controller_factory.py` を作る
3. `Pure Pursuit` を新構造へ移す
4. `run_homur_f110.py` を controller 切替式にする
5. `homur_oval_pure_pursuit.yaml` を追加して動作確認する
6. `MPC` の最小 controller を追加する
7. `homur_oval_mpc.yaml` を追加する
8. `MPPI` の最小 controller を追加する
9. `homur_oval_mppi.yaml` を追加する
10. 1 周比較を行う

## 実装確認方法

### 1. Pure Pursuit の互換確認

```bash
./gym_env/bin/python experiments/run_homur_f110.py \
  --config experiments/configs/homur_oval.yaml \
  --no-render \
  --lap-target 1 \
  --max-steps 5000
```

確認ポイント:

- `lap_target_reached: True`
- `lap_count: 1`
- 既存ベースラインとほぼ同じラップタイムと偏差

### 2. controller 切替確認

将来的には config を変えるだけで以下が動く状態にする。

```bash
./gym_env/bin/python experiments/run_homur_f110.py \
  --config experiments/configs/homur_oval_pure_pursuit.yaml \
  --no-render \
  --lap-target 1 \
  --max-steps 5000
```

```bash
./gym_env/bin/python experiments/run_homur_f110.py \
  --config experiments/configs/homur_oval_mpc.yaml \
  --no-render \
  --lap-target 1 \
  --max-steps 5000
```

```bash
./gym_env/bin/python experiments/run_homur_f110.py \
  --config experiments/configs/homur_oval_mppi.yaml \
  --no-render \
  --lap-target 1 \
  --max-steps 5000
```

### 3. 共通レポート確認

```bash
latest=$(ls -t experiments/results/*.csv | head -n 1)
./gym_env/bin/python experiments/plot_run_results.py "$latest"
```

Zed で開く対象:

- `experiments/results/*_view.md`

## 現時点の推奨判断

現時点では、最初に `MPC` と `MPPI` の中身を作るより、controller 切替構造を先に作る方が合理的である。

理由:

- 比較条件を固定しやすい
- ログ形式を共通化できる
- 実験の入口が一本化される
- `Pure Pursuit` を壊さずに拡張できる

## 次の具体タスク

次に着手する対象は以下とする。

1. `experiments/controllers/` の新設
2. `Pure Pursuit` の controller 化
3. `run_homur_f110.py` の controller factory 対応
4. `homur_oval_pure_pursuit.yaml` の追加
