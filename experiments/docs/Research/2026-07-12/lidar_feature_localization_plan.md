# LiDAR Feature Localization Plan

## 目的

LiDAR から地図特徴を取り出して自己位置推定する `feature-based localization` を、`map-based localization` とは別系統として実装する。

今回の目的は、既知地図との画素一致ではなく、LiDAR から得られる幾何特徴を使って `x, y, theta` を推定し、controller に渡せる実験系を作ることである。

## 今回の位置づけ

すでに `map-based localization` は動いている。したがって今回はそれを置き換えるのではなく、比較可能な第2方式として `feature-based localization` を作る。

比較対象:

- `map-based localization`
- `feature-based localization`

## 基本方針

- 実装はまず `experiments/` 配下だけで完結させる
- 既存の localized runner を壊さない
- 最初は `homur_oval` に限定してよい
- まずは「特徴抽出が安定するか」を重視する
- 最初の controller は `Pure Pursuit` に限定する

## `homur_oval` で使える特徴

このコースは単純なので、最初に使う特徴も単純でよい。

候補:

- 左壁までの距離
- 右壁までの距離
- 前方距離
- 左右距離差
- 前方近傍の壁法線方向
- 楕円壁との相対角度

最初は「特定ビーム方向の距離特徴」から始めるのがよい。

## 最初の実装案

### 案A: beam signature matching

LiDAR 全体を使わず、代表的なビーム群だけを取り出して特徴ベクトルにする。

例:

- 正面
- 左前
- 右前
- 左
- 右

これらの距離の組を特徴として、既知 map 上の候補姿勢から計算される特徴と比較する。

利点:

- 実装が単純
- 計算が軽い
- `homur_oval` ではまず成立しやすい

欠点:

- 特徴が少ないと曖昧性が残る

### 案B: line / wall feature matching

scan から壁線分を抽出し、壁の接線方向や左右対称性から姿勢を出す。

利点:

- feature-based localization らしい構成になる

欠点:

- 最初からやるには少し重い

## 今回の推奨方針

まずは **案A: beam signature matching** から始める。

理由:

- `homur_oval` では左右壁距離と前方距離の組に意味がある
- 実装が軽い
- map-based localization と比較しやすい

## 実装構成

新設候補:

- `experiments/localization/feature_localizer.py`
- `experiments/localization/feature_extractor.py`

既存更新候補:

- `experiments/localization/localizer_factory.py`
- localized 用 config

config 追加候補:

- `experiments/configs/homur_oval_feature_pure_pursuit.yaml`

## フェーズ 1: 特徴量設計

### 目的

LiDAR から取り出す特徴ベクトルを固定する。

候補:

- `front_range`
- `front_left_range`
- `front_right_range`
- `left_range`
- `right_range`
- `left_right_diff`

### 完了条件

- 1 ステップごとの feature vector が取れる
- 値の変化が妥当に見える

## フェーズ 2: 既知 map 上の参照特徴生成

### 目的

候補姿勢ごとに期待される feature vector を計算できるようにする。

方法:

- `ScanSimulator2D` を使って候補姿勢から LiDAR を生成
- その LiDAR から同じ feature vector を抽出

### 完了条件

- 候補姿勢 -> 参照特徴 の計算ができる

## フェーズ 3: feature matching localizer

### 目的

観測特徴と参照特徴の差で姿勢を選ぶ localizer を作る。

方法:

- 前時刻推定姿勢の近傍で候補姿勢を作る
- 各候補姿勢について参照特徴を計算する
- 観測特徴との差の二乗和でスコアを計算する
- 最小スコアの姿勢を採用する

### 完了条件

- `estimated_pose` が返る
- 短区間で発散しない

## フェーズ 4: localized runner へ接続

### 目的

既存 localized runner で `feature_localizer` を使えるようにする。

### 対応内容

- `localizer_factory.py` に `feature_localizer` を追加
- config で `localizer.type` を切替可能にする

### 完了条件

- `Pure Pursuit` を `feature_localizer` 経由で動かせる

## フェーズ 5: 短区間評価

### 目的

まずは `--max-steps 50` 程度で安定性を確認する。

見る項目:

- `est_error_x`
- `est_error_y`
- `est_error_theta`
- localization score

### 完了条件

- 推定姿勢が極端に飛ばない

## フェーズ 6: 1 周評価

### 目的

`feature-based localization` で 1 周完了できるか確認する。

比較対象:

- map-based localization
- feature-based localization

見る項目:

- `lap_count`
- `lap_time`
- `mean_xy_error`
- `max_xy_error`
- `mean_cross_track`
- `max_cross_track`

### 完了条件

- 1 周完了
- map-based との強み弱みが見える

## 実装順

1. `feature_extractor` を作る
2. `feature_localizer` を作る
3. `localizer_factory.py` に追加する
4. `homur_oval_feature_pure_pursuit.yaml` を作る
5. 短区間 run で確認する
6. 1 周 run で確認する
7. map-based と比較する

## 注意点

- `homur_oval` は対称性が強いので、特徴量が少なすぎると姿勢が曖昧になる
- まずは前時刻推定姿勢の近傍探索と組み合わせる
- feature 数を増やしすぎると map-based とあまり変わらない実装になる

## 次の具体タスク

1. `experiments/localization/feature_extractor.py` を追加
2. `experiments/localization/feature_localizer.py` を追加
3. `localizer_factory.py` で `feature_localizer` を選べるようにする
4. `Pure Pursuit` 用の feature-localized config を追加する
