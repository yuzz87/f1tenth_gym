# LiDAR Map Localization Plan

## 目的

LiDAR を使った自己位置推定を、既知地図に対する `map-based localization` として実装する。

ここでの目的は、controller が直接 `env` の真値姿勢 `poses_x`, `poses_y`, `poses_theta` を使うのではなく、LiDAR scan と既知 map から推定した姿勢を使って走行できる基盤を作ることである。

今回は `feature-based localization` とは分けて進め、まずは `map-based localization` を単独で成立させる。

## この計画で扱う範囲

対象:

- 既知 map に対する LiDAR 自己位置推定
- `homur_oval` を使った単一車両の検証
- 推定姿勢を controller 入力へ差し替える実験系

今回は扱わないもの:

- LiDAR feature extractor ベースの自己位置推定
- 障害物回避
- SLAM
- 地図生成
- 多車両

## 基本方針

- 実装はまず `experiments/` 配下だけで完結させる
- 既存の controller 比較基盤は壊さない
- 最初は `homur_oval` に限定してよい
- いきなり高性能な localization を狙わない
- まずは「推定姿勢で 1 周できる」ことを目標にする

## 目標

最初の目標は以下である。

1. `env` の LiDAR scan を受け取れる
2. 既知 map と照合して `x, y, theta` を推定できる
3. controller が真値ではなく推定姿勢で動ける
4. `homur_oval` で 1 周できる
5. 推定誤差をログとして保存できる

## アプローチ候補

### 候補 1: brute-force local search

現在姿勢の近傍で候補姿勢を複数作り、各候補で LiDAR scan と map の一致度を比較して最も良い姿勢を選ぶ。

利点:

- 実装が比較的単純
- `homur_oval` のような単純マップで始めやすい
- `experiments/` 内で閉じやすい

欠点:

- 計算量が増えやすい
- 初期推定が悪いと外れやすい

### 候補 2: scan matching

前時刻の推定姿勢を初期値として、LiDAR scan と map の整合度を局所最適化する。

利点:

- `brute-force` より滑らかな推定になりやすい
- 局所探索として自然

欠点:

- 実装難度が少し上がる
- 局所解に落ちる可能性がある

### 候補 3: particle filter + map likelihood

動作モデルで粒子を予測し、LiDAR と map の一致度で重み付けする。

利点:

- 拡張性が高い
- ノイズや不確実性を扱いやすい

欠点:

- 最初の実装としては重い
- 計算コストも増える

## 今回の推奨方針

まずは `候補 1` の `brute-force local search` から始める。

理由:

- 配線確認をしながら進めやすい
- LiDAR 観測と既知 map の照合という本質をまず試せる
- 既存 controller 比較基盤に載せやすい
- 失敗時の切り分けがしやすい

## 実装構成

新設候補:

- `experiments/localization/`
- `experiments/localization/base_localizer.py`
- `experiments/localization/map_localizer.py`
- `experiments/localization/localizer_factory.py`

runner 側の追加候補:

- `experiments/run_localized_controller_experiment.py`

config 追加候補:

- `experiments/configs/homur_oval_localized_pure_pursuit.yaml`
- `experiments/configs/homur_oval_localized_mpc.yaml`
- `experiments/configs/homur_oval_localized_mppi.yaml`

## フェーズ 1: LiDAR 観測の取り回し確認

### 目的

controller 実験系とは別に、LiDAR scan を localization モジュールへ流せるようにする。

### 対応内容

- `obs["scans"]` を取得する
- まず scan shape と scan 値の範囲を確認する
- map-based localization に必要な入力を整理する

### 完了条件

- runner から localizer に scan を渡せる
- scan の基本的な観測確認ができる

## フェーズ 2: localizer のインターフェースを作る

### 目的

推定器を controller と分離して扱える形にする。

### 目標インターフェース

- `initialize(initial_pose)`
- `update(obs) -> estimated_pose`
- `debug_info()`

推定姿勢の形式:

- `(x, y, theta)`

### 完了条件

- localizer を差し替え可能な構造になる

## フェーズ 3: brute-force map localizer を実装する

### 目的

既知 map と LiDAR scan の一致度で姿勢を推定する最小実装を作る。

### 初期方針

- 前時刻推定姿勢の近傍だけを探索する
- `x`, `y`, `theta` の小さな格子探索を行う
- 候補姿勢ごとに scan と map の一致スコアを評価する

### スコア候補

- scan endpoint が壁にどれだけ一致するか
- occupied cell との一致数
- free space と衝突しないか

### 最初の簡易実装

- `x` 探索幅: 小さめ
- `y` 探索幅: 小さめ
- `theta` 探索幅: 小さめ
- 周辺局所探索だけ行う

### 完了条件

- LiDAR から更新した推定姿勢が出る
- 推定姿勢が発散せず連続的に更新される

## フェーズ 4: 推定姿勢で controller を動かす

### 目的

controller 入力を真値から推定値へ切り替える。

### 対応内容

- controller に渡す pose を `estimated_pose` に差し替える
- 速度はまず `obs["linear_vels_x"][0]` をそのまま使ってよい
- 比較のため、真値と推定値の両方をログに残す

### 完了条件

- `Pure Pursuit` が推定姿勢ベースで動作する
- `homur_oval` で少なくとも短区間走れる

## フェーズ 5: 推定誤差ログを追加する

### 目的

localization の良し悪しを定量的に見られるようにする。

### 追加したいログ

- `est_pose_x`, `est_pose_y`, `est_pose_theta`
- `gt_pose_x`, `gt_pose_y`, `gt_pose_theta`
- `est_error_x`
- `est_error_y`
- `est_error_theta`
- localization score

### 見たい指標

- 平均位置誤差
- 最大位置誤差
- 平均姿勢誤差
- 最大姿勢誤差

### 完了条件

- localization の品質を数値で評価できる

## フェーズ 6: 1 周評価

### 目的

推定姿勢ベースで 1 周完了できるか確認する。

### 比較対象

- 真値姿勢 controller
- 推定姿勢 controller

### 見る項目

- `lap_count`
- `lap_time`
- `max_abs_cross_track`
- `mean_abs_cross_track`
- localization error

### 完了条件

- 推定姿勢を使って 1 周完了できる
- 真値使用時との劣化量を把握できる

## 実装順

1. `experiments/localization/` を新設
2. localizer の共通インターフェースを作る
3. LiDAR scan を runner から localizer へ流す
4. brute-force `map_localizer` を実装する
5. 推定姿勢で `Pure Pursuit` を動かす runner を作る
6. 推定誤差ログを追加する
7. `homur_oval` で短区間確認する
8. `lap-target 1` で 1 周確認する
9. その後 `MPC` / `MPPI` に横展開する

## 実装確認方法

### 1. LiDAR 観測確認

- scan が毎ステップ localizer に渡る
- localizer が推定姿勢を返す

### 2. 短区間確認

- `--max-steps 50` などで短く回す
- 推定姿勢が大きく飛ばないか確認する

### 3. 1 周確認

- `--lap-target 1` で一周完了するか確認する
- 推定誤差と tracking 誤差を同時に見る

## 注意点

- `homur_oval` は形状が単純なので、姿勢の曖昧性が出る可能性がある
- 完全に LiDAR のみで安定化しない場合は、前時刻推定姿勢を初期値として強く使う
- 最初は速度を上げず、現状の 1.0 m/s 付近で始める
- localization と controller を一度に複雑化しない

## 次の具体タスク

最初に着手する対象は以下とする。

1. `experiments/localization/` の新設
2. scan を受けて推定姿勢を返す localizer interface の作成
3. brute-force `map_localizer` の最小実装
4. 推定姿勢で `Pure Pursuit` を動かす runner の追加
