# 現在行っている F1TENTH シミュレーション入門

## この実験で確認したいこと

この実験では、F1TENTH Gym の中で小型レーシングカーを楕円コースに沿って走らせ、次のことを比較する。

- `Pure Pursuit (PP)`、`MPC`、`MPPI` の走り方の違い
- 車の姿勢をシミュレータの真値で与えた場合と、LiDARから推定した場合の違い
- 1周を完走できるか、どれくらいコース中心を走れるか

まずはシミュレーションで制御器と自己位置推定の動きを確認し、その後に速度やノイズなどの条件を厳しくしていく。

## 全体の流れ

シミュレーションは、次の処理を 0.01 秒ごとに繰り返す。

```text
地図とwaypointを読み込む
        ↓
F1TENTH Gymが車を少し動かす
        ↓
車の状態を取得
（位置、向き、速度、LiDAR scan）
        ↓
LiDARを使う場合は自己位置を推定
        ↓
制御器が速度とステアリング角を決める
        ↓
次のステップへ進む
        ↓
CSVへ記録し、グラフで確認する
```

ここで重要なのは、`LiDAR` は直接ステアリングを決めるものではないという点である。
LiDARはまず「自分が地図のどこにいるか」を推定するために使い、その推定位置をPP・MPC・MPPIへ渡す。

## 使っているコース

現在のコースは `homur_oval` である。

| 項目 | 内容 |
| --- | --- |
| コース形状 | 楕円形 |
| 地図の大きさ | 幅 7 m、高さ 5 m |
| 地図画像 | 700 x 500 pixel |
| 地図解像度 | 0.01 m/pixel |
| 中心線の長さ | 約 15.8 m |
| waypoint | 中心線上に約 198 点 |
| 車の台数 | 1 台 |
| シミュレーション周期 | 0.01 s |
| 積分方法 | RK4 |
| 初期位置 | `[x, y, theta] = [3.0, 0.0, pi/2]` |
| 走行方向 | 反時計回り |

地図とwaypointは次のファイルにある。

- `experiments/maps/homur_oval.png`
- `experiments/maps/homur_oval.yaml`
- `experiments/maps/homur_oval_waypoints.csv`

waypointには、位置だけでなく、中心線の向きと基準速度も入っている。
現在のwaypoint基準速度は `1.0 m/s` である。

## 3種類の制御器

### Pure Pursuit

PPは、車の少し前にあるwaypointを目標にして、その点へ向かうようにステアリングを切る方法である。

人が自転車に乗るときに、真下ではなく少し前を見て進むイメージに近い。

- 少し前の目標点を探す
- 目標点が左右どちらにあるかを見る
- 目標点へ向かうステアリング角を出す
- waypointの基準速度を使って進む

主な設定は次の2つである。

- `tlad`: 目標点までの距離。大きいと先を見てゆっくり曲がる
- `vgain`: waypointの速度に掛ける倍率

実装場所:

- `experiments/controllers/pure_pursuit_controller.py`
- 実際のPP計算は `examples/waypoint_follow.py`

### MPC

MPCは、いまのステアリングをそのまま出すのではなく、数ステップ先までの動きを予測してから決める方法である。

候補となるステアリングの列を作り、簡易的な車両モデルで未来の位置を計算する。そして、waypointから離れにくく、向きが合い、急な操作にならない候補を選ぶ。

現在の実装では、PPの出力を初期候補として利用している。そのため、PPを土台にして未来の動きを調整する構成になっている。

主な設定は次のとおりである。

- `horizon`: 何ステップ先まで予測するか
- `target_speed`: 予測に使う速度
- `position_weight`: 中心線からのずれの重み
- `heading_weight`: 車の向きのずれの重み
- `steer_delta_weight`: ステアリング変化の大きさの重み

実装場所: `experiments/controllers/mpc_controller.py`

### MPPI

MPPIも未来を予測するが、最適化計算で1つの操作を探す代わりに、多数のステアリングパターンを試して良いものを選ぶ。

1回の制御で、次のような候補を作る。

- 少し左へ切る
- 大きく左へ切る
- 最初だけ切って戻す
- ほぼ直進する

各候補を簡易モデルで先読みし、中心線からの距離、向き、操作の滑らかさなどを点数化する。点数の良い候補を強く反映して、今回のステアリング角を決める。

現在のMPPIは、PPを基準操作として、その周辺を探索する簡略版である。また、主にステアリング列を探索しており、速度とステアリングを同時に最適化する本格的な構成ではない。

主な設定は次のとおりである。

- `horizon`: 先読みするステップ数
- `num_samples`: 試すステアリングパターンの数
- `temperature`: 良い候補にどれだけ集中するか
- `noise_sigma`: 候補をどれくらいばらつかせるか

実装場所: `experiments/controllers/mppi_controller.py`

## 自己位置推定の2方式

### 1. map-based localization

LiDARのscanと、地図から計算した予想scanを比較する。

大まかな流れは次のとおりである。

1. 前回の推定位置から、今回の位置を予測する
2. その周囲に複数の候補姿勢を作る
3. 各候補姿勢からLiDAR scanをシミュレーションする
4. 実際のscanに最も近い候補を選ぶ

実装場所: `experiments/localization/map_localizer.py`

### 2. feature-based localization

LiDAR全体をそのまま比較せず、代表的な方向の距離を取り出して特徴量にする。

現在は次の7方向を使っている。

```text
-1.8, -0.9, -0.35, 0.0, 0.35, 0.9, 1.8 [rad]
```

この距離の組み合わせ、左右の差、距離の変化などを特徴ベクトルにし、候補姿勢から得られる特徴ベクトルと比較する。

実装場所:

- `experiments/localization/feature_extractor.py`
- `experiments/localization/feature_localizer.py`

## 実験の3つのモード

| モード | 実行スクリプト | 姿勢の入力 | 目的 |
| --- | --- | --- | --- |
| 基準走行 | `run_controller_experiment.py` | シミュレータの姿勢 | 制御器そのものを比較 |
| map localization | `run_localized_controller_experiment.py` | LiDARから推定した姿勢 | 地図scan matchingを評価 |
| feature localization | `run_localized_controller_experiment.py` | LiDAR特徴から推定した姿勢 | 特徴量ベース推定を評価 |

基準走行では、シミュレータが持つ真の姿勢を制御器に渡す。
これは現実の車にそのまま存在する情報ではなく、制御器の性能を見るための基準条件である。

localized走行では、シミュレータの真の姿勢は評価用に保存するだけで、制御器には推定姿勢を渡す。
そのため、実際の走行に近い評価になる。

## 実行方法

リポジトリのルートで実行する。

```bash
cd /home/ubuntuyuzz/Desktop/f1tenth_gym
```

### 基準走行を1周する

```bash
./gym_env/bin/python experiments/run_controller_experiment.py \
  --config experiments/configs/homur_oval_pure_pursuit.yaml \
  --no-render \
  --lap-target 1 \
  --max-steps 5000
```

MPCやMPPIを試す場合は、configだけを次のように変更する。

```text
experiments/configs/homur_oval_mpc.yaml
experiments/configs/homur_oval_mppi.yaml
```

### feature localizationでPPを1周する

```bash
./gym_env/bin/python experiments/run_localized_controller_experiment.py \
  --config experiments/configs/homur_oval_feature_pure_pursuit.yaml \
  --no-render \
  --lap-target 1 \
  --max-steps 5000
```

feature localizationでMPCとMPPIを試す場合は、次のconfigを使う。

```text
experiments/configs/homur_oval_feature_mpc.yaml
experiments/configs/homur_oval_feature_mppi.yaml
```

### 画面を表示して走らせる

`--no-render` を外すと、シミュレーション画面を表示できる。

```bash
./gym_env/bin/python experiments/run_localized_controller_experiment.py \
  --config experiments/configs/homur_oval_feature_mppi.yaml \
  --lap-target 1 \
  --max-steps 5000
```

## 実行結果の読み方

実行が終わると、端末に概要が表示され、CSVログが `experiments/results/` に保存される。

```text
steps: 1550前後
lap_count: 1
lap_target_reached: True
log_csv: experiments/results/....csv
```

`done: False` でも、`lap_target_reached: True` なら指定した1周に到達している。
この場合は、環境の衝突終了ではなく、ランナーが周回数を見て停止したという意味である。

localized実験では、さらに次の値が表示される。

- `est_pose`: 最後に推定された `[x, y, theta]`
- `localization_score`: 自己位置推定のスコア。小さいほど候補と観測の差が小さい

## CSVに記録される主な値

### 走行の成否

- `lap_count`: 完了した周回数
- `lap_time`: 周回時間
- `collision`: 衝突したか
- `done`: 環境が終了したか

### 制御の安定性

- `cross_track_error`: 中心線からの横方向のずれ [m]
- `heading_error`: 中心線の向きとの差 [rad]
- `speed_cmd`: 制御器が指示した速度 [m/s]
- `steer_cmd`: 制御器が指示したステアリング角 [rad]

### 自己位置推定の正確さ

localized実験では、評価のためにシミュレータの真値との差も保存する。

- `est_error_x`, `est_error_y`: 推定位置と真値の差 [m]
- `est_error_theta`: 推定した向きと真値の差 [rad]
- `localization_scan_error`: LiDARまたは特徴量の観測誤差

基本的には、次のように読む。

- `lap_count = 1` かつ `collision = 0`: 1周を衝突せず完走
- `mean_abs_cross_track` が小さい: 中心線に近く走行
- `mean_xy_error` が小さい: 自己位置推定が真値に近い
- `lap_time` が小さい: 速く走行

速さだけで優劣を決めず、完走、追従誤差、自己位置誤差を一緒に見る。

## グラフを見る方法

CSVのパスは、実行時に表示された `log_csv` の値を使う。

```bash
./gym_env/bin/python experiments/plot_run_results.py \
  experiments/results/<実行結果のCSVファイル>.csv
```

このコマンドで、CSVと同じ場所に次の2ファイルが作られる。

- `*_sum.svg`: グラフ画像
- `*_view.md`: グラフを埋め込んだMarkdown

Zedでは `*_view.md` を開いてMarkdown Previewを表示すると、SVGを画像として確認できる。
グラフには主に次の3つが表示される。

1. 車の軌跡と中心線
2. 横ずれと向きの誤差
3. 速度、ステアリング、実速度

## 3手法を比較する方法

feature localizationの最新ログを比較する場合:

```bash
./gym_env/bin/python experiments/compare_feature_runs.py
```

出力先:

```text
experiments/results/reports/feature_controller_comparison.md
```

map localizationのPP・MPC・MPPIを比較する場合:

```bash
./gym_env/bin/python experiments/compare_localized_runs.py
```

出力先:

```text
experiments/results/reports/localized_controller_comparison.md
```

比較スクリプトは、指定がなければ `experiments/results/` にある条件に合う最新CSVを探す。
別のCSVを比較したい場合は、`--pure-pursuit-csv`、`--mpc-csv`、`--mppi-csv` で明示する。

## 現在の実験条件と注意点

現在の基準configは、3つの制御器ともおおむね `1.0 m/s` で走る設定である。
これまでの1周実験では、中心線長が約15.8 mなので、1周のシミュレーション時間は約15.5秒になる。

ただし、現在の実験は次の意味で理想化されている。

- 車は1台だけ
- コースと地図は既知
- 動く障害物はない
- `scan_noise_std` は現在 `0.0`
- `initialization_noise` は現在 `[0.0, 0.0, 0.0]`
- MPCとMPPIの先読みは簡略化した運動モデルを使う
- シミュレータ内部の真値は、自己位置推定の誤差を測るために保存される

したがって、1周できたことは「現実の車でもそのまま成功する」という意味ではない。
まずは制御器と自己位置推定の基本動作を確認できた、と考える。

## 次に行う評価

現在の自然な比較順は次のとおりである。

1. 同じ `feature_localizer` 条件でPP・MPC・MPPIを1周ずつ走らせる
2. 各CSVからグラフを作る
3. `compare_feature_runs.py` で比較表を作る
4. `lap_count`、衝突、追従誤差、自己位置誤差を確認する
5. 条件を固定したまま、必要なら速度を段階的に上げて差を見る

速度を変えるときは、制御器と自己位置推定の設定を一度に変えず、速度だけを変えると結果を解釈しやすい。

## 関連ドキュメント

- [PPとMPPIの初学者向け説明](./pp_mppi_beginner_guide.md)
- [PPの数式ベース説明](./pure_pursuit_math.md)
- [MPPIの数式ベース説明](./mppi_math_and_gap.md)
- [feature localizationの実装計画](../2026-07-12/lidar_feature_localization_plan.md)
