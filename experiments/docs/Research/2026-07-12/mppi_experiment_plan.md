# MPPI Experiment Plan

## 目的

現在の `Pure Pursuit` が楕円コース上で安定走行できているため、次の段階では MPPI 制御を導入する前に、比較用のベースライン計測環境を整備する。

この計画の目的は次の 3 点である。

1. 現在の制御結果を再現可能な形で記録する。
2. 後から `Pure Pursuit` と `MPPI` を同じ指標で比較できるようにする。
3. MPPI 実装と調整を感覚ではなくログとグラフで進められる状態を作る。

## 基本方針

- まずは `experiments/` 配下だけで完結する。
- 先にログ保存と可視化を作る。
- MPPI 導入はその後に行う。
- 比較対象は `homur_oval` の既存設定を使う。

## フェーズ 1: ベースライン計測

### 対象

- 実行スクリプト: `experiments/run_homur_f110.py`
- 設定ファイル: `experiments/configs/homur_oval.yaml`
- 制御器: `Pure Pursuit`

### 取得するデータ

最低限、各ステップで以下を保存する。

- step index
- sim time
- pose `x`, `y`, `theta`
- speed command
- steer command
- actual longitudinal speed が取得できるならその値
- waypoint に対する横偏差
- waypoint に対する heading 誤差
- lap progress または中心線に沿った進行距離
- done flag

### 保存形式

- `experiments/results/` を作る
- 1 実行 1 CSV を基本とする
- ファイル名には日時、controller 名、config 名を含める

例:

`experiments/results/2026-07-12_homur_oval_pure_pursuit.csv`

### この段階の完了条件

- 同じ条件で複数回実行してログを保存できる
- 少なくとも 1 周分のログが欠損なく取れる
- 手元で再読込しやすい列構成になっている

## フェーズ 2: 可視化

### 最初に出すグラフ

優先度が高いのは次の 3 つ。

1. 軌跡 `x-y` と waypoint の重ね描き
2. 横偏差 vs 時間
3. steer と speed command vs 時間

余力があれば追加する。

- heading 誤差 vs 時間
- 周回ごとのラップタイム
- コース進行距離に対する偏差

### 実装方針

- 実行スクリプトに plotting は直接入れない
- `experiments/` 配下に解析用スクリプトを別で置く
- 入力は CSV、出力は PNG

候補:

- `experiments/analyze_run.py`
- `experiments/plot_run_results.py`

### この段階の完了条件

- CSV から自動でグラフを出力できる
- 1 回の走行結果をすぐ確認できる
- `Pure Pursuit` ベースラインの挙動を目視と数値の両方で確認できる

## フェーズ 3: ベースライン評価

### 確認したい観点

- 安定して 1 周以上走れるか
- 横偏差がどの程度の範囲に収まっているか
- コーナーで大きな操舵振動が出ていないか
- 実行ごとのばらつきが小さいか

### この段階で残すべき記録

- 使用した config
- 使用した controller
- 実行コマンド
- 主要な結果の要約

この時点で、以後の MPPI 比較の基準として残せる状態にする。

## フェーズ 4: MPPI 導入

### 初期方針

- `experiments/` 配下に MPPI 用 planner を追加する
- 既存の `planner.plan(...) -> (speed, steer)` の呼び出し形に合わせる
- まずは簡易モデルで動作確認する
- 最初は操舵中心、必要に応じて速度最適化を追加する

### 追加する設定

`experiments/configs/` に MPPI 用 config を追加する。

候補:

- `homur_oval_mppi.yaml`

保持したい代表パラメータ:

- horizon
- num_samples
- lambda
- noise sigma
- cost weights
- target speed

### この段階の完了条件

- MPPI で最低限コースを追従できる
- 同じ CSV 形式でログが取れる
- `Pure Pursuit` と同じグラフで比較できる

## フェーズ 5: 比較実験

### 比較対象

- `Pure Pursuit`
- `MPPI`

### 比較指標

- ラップタイム
- 横偏差の平均値、最大値
- heading 誤差の平均値、最大値
- 操舵入力の滑らかさ
- 周回完走率

### 実験の進め方

- 同一 map
- 同一初期姿勢
- 同一 target speed または近い条件
- 同一評価スクリプト

条件を固定しない比較は意味が薄いので、config を明示的に保存すること。

## 実装順の提案

1. `run_homur_f110.py` に CSV ログ保存を追加
2. `experiments/results/` への出力形式を決める
3. CSV からグラフを出す解析スクリプトを追加
4. `Pure Pursuit` のベースラインを数本取る
5. MPPI planner を `experiments/` に追加
6. MPPI 用 config を追加
7. 同一指標で比較する

## 注意点

- 先に MPPI を入れると、何が改善で何が退化か判断しにくくなる
- ログ列は後方互換性を意識して増やす
- plotting と controller 本体は分離する
- 本体ライブラリ `gym/f110_gym` は、必要が出るまで変更しない

## 次の具体タスク

最初の着手対象は以下とする。

1. `Pure Pursuit` 実行時の CSV ログ保存
2. 軌跡と偏差を可視化するグラフスクリプト作成
3. `homur_oval` のベースライン取得

## 実装手順

### 1. 実行スクリプトにロガーを追加する

対象:

- `experiments/run_homur_f110.py`

対応内容:

- `RunLogger` を追加する
- 各ステップの観測値と制御入力をメモリ上に保持する
- 実行終了後に CSV として保存する
- `--results-dir` と `--no-log` を追加する

保存対象の主要列:

- `step`
- `sim_time`
- `pose_x`, `pose_y`, `pose_theta`
- `speed_cmd`, `steer_cmd`
- `linear_vel_x`, `linear_vel_y`, `ang_vel_z`
- `cross_track_error`
- `heading_error`
- `progress_distance`
- `ref_x`, `ref_y`, `ref_heading`
- `collision`
- `done`

### 2. waypoint 基準の評価量を計算する

目的:

- MPPI 導入後も同じ指標で比較するため

対応内容:

- `nearest_point_on_trajectory` を使って現在位置に最も近い中心線位置を求める
- 最近傍線分に対する符号付き横偏差を `cross_track_error` として保存する
- 参照接線方向との差を `heading_error` として保存する
- 線分累積長から `progress_distance` を計算する

注意:

- 角度差は `[-pi, pi)` に正規化する
- waypoint は閉ループとして扱う

### 3. 解析スクリプトを別ファイルで作る

対象:

- `experiments/plot_run_results.py`

対応内容:

- CSV を読み込む
- 軌跡と中心線の重ね描きを作る
- 横偏差と heading 誤差の時系列を作る
- 速度指令、操舵指令、実速度の時系列を作る
- 依存追加を避けるため、出力は `SVG` とする

この段階での出力:

- `*_sum.svg`
- `*_view.md`

### 4. 実行確認を行う

確認手順:

1. `compileall` で `experiments/` 配下の構文確認を行う
2. `homur_oval` で短いステップ数の実行を行う
3. CSV が `experiments/results/` に保存されることを確認する
4. その CSV から `SVG` が生成できることを確認する
5. `Markdown` レポートを Zed Preview で開いてグラフとして見えることを確認する

実行例:

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym

./gym_env/bin/python -m compileall -q experiments

./gym_env/bin/python experiments/run_homur_f110.py \
  --config experiments/configs/homur_oval.yaml \
  --no-render \
  --max-steps 100
```

```bash
./gym_env/bin/python experiments/plot_run_results.py \
  experiments/results/<log_file>.csv
```

Zed で確認する対象:

- `experiments/results/<log_file>_view.md`

### 5. ベースライン取得に進む

この実装後にやること:

- `homur_oval` で複数回ログを取る
- 横偏差、heading 誤差、操舵の振れを確認する
- ベースラインとして残す run を整理する
- その後に MPPI 実装へ進む

## 1 周安定性確認の実装手順

### 1. 周回到達で止める引数を追加する

対象:

- `experiments/run_homur_f110.py`

対応内容:

- `--lap-target` を追加する
- `0` のときは無効
- `1` 以上なら、指定周回数に到達した時点で実行を止める

目的:

- `done` を待たずに、ちょうど 1 周完了時点の挙動を評価できるようにする

### 2. 環境の周回情報を使って終了判定する

使う観測:

- `obs["lap_counts"][0]`
- `obs["lap_times"][0]`

対応内容:

- 各ステップで `lap_count` を確認する
- `lap_count >= lap_target` なら `lap_target_reached = True` として break する
- 衝突時や通常の `done` 判定はそのまま残す

注意:

- 環境側の `done` は 1 周専用ではないため、1 周評価は `lap_counts` 基準で切る

### 3. CSV に周回情報を追加する

追加列:

- `lap_count`
- `lap_time`

目的:

- 1 周達成時点の状態をログから後追いできるようにする
- 複数 run のラップタイム比較をしやすくする

### 4. レポートに周回結果を出す

対象:

- `experiments/plot_run_results.py`

追加する要約:

- `completed_lap_1`
- `final_lap_count`
- `final_lap_time_s`

目的:

- Zed 上で `*_view.md` を開いたときに、一周できたかどうかを先頭ですぐ判断できるようにする

### 5. 実装確認を行う

実行例:

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym

./gym_env/bin/python experiments/run_homur_f110.py \
  --config experiments/configs/homur_oval.yaml \
  --no-render \
  --lap-target 1 \
  --max-steps 5000
```

このとき確認する標準出力:

- `lap_count: 1`
- `lap_target_reached: True`
- `log_csv: ...`

続けてレポートを生成する:

```bash
latest=$(ls -t experiments/results/*.csv | head -n 1)
./gym_env/bin/python experiments/plot_run_results.py "$latest"
```

Zed で確認する対象:

- `experiments/results/*_view.md`

### 6. 実装確認用の数値チェック

最新 CSV から簡単に数値確認する例:

```bash
python3 - <<'PY'
import csv
import glob

path = sorted(glob.glob('experiments/results/*.csv'))[-1]
with open(path, newline='', encoding='utf-8') as f:
    rows = list(csv.DictReader(f))

last = rows[-1]
cte = [abs(float(r['cross_track_error'])) for r in rows]
he = [abs(float(r['heading_error'])) for r in rows]

print('csv:', path)
print('rows:', len(rows))
print('final_lap_count:', last['lap_count'])
print('final_lap_time:', last['lap_time'])
print('max_abs_cross_track:', max(cte))
print('mean_abs_cross_track:', sum(cte) / len(cte))
print('max_abs_heading:', max(he))
print('mean_abs_heading:', sum(he) / len(he))
PY
```

判定の見方:

- `final_lap_count = 1` なら 1 周完了
- `max_abs_cross_track` と `mean_abs_cross_track` が小さいほど中心線追従が安定
- `max_abs_heading` が大きすぎないかで姿勢の暴れを確認する

## 現在の実装状態

完了済み:

- `run_homur_f110.py` への CSV ログ保存追加
- `plot_run_results.py` の追加
- `homur_oval` 短時間実行での CSV 出力確認
- CSV から `SVG` 出力確認
- `--lap-target` による 1 周到達終了の追加
- `lap_count` / `lap_time` の CSV 保存追加
- `*_view.md` への 1 周達成要約追加

確認済み出力先:

- `experiments/results/`

代表的な出力物:

- `*.csv`
- `*_sum.svg`
- `*_view.md`
