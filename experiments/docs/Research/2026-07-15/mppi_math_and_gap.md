# MPPI Math and Gap

## 目的

`experiments/controllers/mppi_controller.py` の現在実装を数式で整理し、標準的な `MPPI` と比べて何が簡略化されているかを明確にする。

この資料の目的は次の 2 点である。

1. いまの `MPPIController` が何を最適化しているかを数式で追えるようにする。
2. 次に改良すべき箇所を優先順位つきで切り出す。

## 現在実装の数式

対象ファイル:

- `experiments/controllers/mppi_controller.py`

### 状態と入力

現在の状態は

\[
\mathbf{x}_t =
\begin{bmatrix}
x_t \\
y_t \\
\theta_t
\end{bmatrix}
\]

入力は舵角のみである。

\[
u_t = \delta_t
\]

速度は最適化対象ではなく、各制御周期で固定値

\[
v = v_{\mathrm{target}}
\]

を使って rollout している。

### 離散時間モデル

`_rollout()` の更新式は簡易 kinematic bicycle model である。

\[
x_{t+1} = x_t + v \cos\theta_t \Delta t
\]

\[
y_{t+1} = y_t + v \sin\theta_t \Delta t
\]

\[
\theta_{t+1} = \theta_t + \frac{v}{L}\tan(\delta_t)\Delta t
\]

ここで

- \(\Delta t\): シミュレーション刻み
- \(L = l_f + l_r\): wheelbase

である。

### 参照軌道

現在位置の進捗を \(s_0\) とすると、`horizon = H` に対して各ステップの参照進捗は

\[
s_k = s_0 + k v \Delta t
\qquad (k = 1, 2, \dots, H)
\]

である。トラック長で wrap した後、waypoint から線形補間して

\[
\mathbf{r}_k =
\begin{bmatrix}
x_k^{\mathrm{ref}} \\
y_k^{\mathrm{ref}} \\
\theta_k^{\mathrm{ref}}
\end{bmatrix}
\]

を作る。

### nominal 系列

現在実装は `Pure Pursuit` を nominal input generator として使っている。

\[
\delta^{\mathrm{nom}}
\]

を `Pure Pursuit` から得て、horizon 全体に複製した

\[
\mathbf{u}^{\mathrm{nom}} =
[\delta^{\mathrm{nom}}, \delta^{\mathrm{nom}}, \dots, \delta^{\mathrm{nom}}]
\]

を作る。

さらに前回の最適系列 \(\mathbf{u}^{\mathrm{prev}}\) と混ぜて探索中心を決める。

\[
\mathbf{u}^{\mathrm{base}}
=
\mathrm{clip}\left(
0.7 \mathbf{u}^{\mathrm{prev}} + 0.3 \mathbf{u}^{\mathrm{nom}}
\right)
\]

### サンプリング

候補系列はガウス雑音で作る。

\[
\boldsymbol{\epsilon}^{(i)} \sim \mathcal{N}(0, \sigma^2 I)
\]

\[
\mathbf{u}^{(i)} =
\mathrm{clip}\left(
\mathbf{u}^{\mathrm{base}} + \boldsymbol{\epsilon}^{(i)}
\right)
\]

ここで

- \(i = 1, \dots, N\)
- \(N = \mathrm{num\_samples}\)
- \(\sigma = \mathrm{noise\_sigma}\)

である。

### コスト関数

各候補系列を rollout して予測状態列

\[
\mathbf{x}_1^{(i)}, \mathbf{x}_2^{(i)}, \dots, \mathbf{x}_H^{(i)}
\]

を得る。

各時刻のステージコストは次の形である。

\[
\ell_k =
w_p \left(
(x_k - x_k^{\mathrm{ref}})^2 + (y_k - y_k^{\mathrm{ref}})^2
\right)
+ w_\theta (\theta_k - \theta_k^{\mathrm{ref}})^2
+ w_u \delta_k^2
+ w_\Delta (\delta_k - \delta_{k-1})^2
+ w_n (\delta_k - \delta_k^{\mathrm{nom}})^2
\]

ただし終端ステップ \(k = H\) では

\[
w_p \rightarrow w_p^{\mathrm{term}}, \qquad
w_\theta \rightarrow w_\theta^{\mathrm{term}}
\]

に置き換えている。

したがって候補系列 \(i\) の全コストは

\[
J^{(i)} = \sum_{k=1}^{H} \ell_k
\]

である。

### 重み計算

数値安定化のため最小コスト

\[
J_{\min} = \min_i J^{(i)}
\]

を引いた上で、各候補の重みを

\[
w_i = \exp\left(
- \frac{J^{(i)} - J_{\min}}{\lambda}
\right)
\]

とする。

ここで \(\lambda = \mathrm{temperature}\) である。

正規化重みは

\[
\tilde{w}_i = \frac{w_i}{\sum_j w_j}
\]

である。

### 出力系列

最終系列は候補系列の重み付き平均である。

\[
\mathbf{u}^* = \sum_{i=1}^{N} \tilde{w}_i \mathbf{u}^{(i)}
\]

実際に車両へ出す入力は先頭要素だけ使う。

\[
\delta_t = u_1^*
\]

残りの系列は次時刻の warm start に流用する。

## 標準的な MPPI との差

現在の実装は `MPPI` の考え方を使っているが、標準的な formulation からはかなり簡略化されている。

### 1. 入力が舵角のみ

現在は

\[
u_t = \delta_t
\]

だけを最適化している。標準的には

\[
u_t =
\begin{bmatrix}
a_t \\
\delta_t
\end{bmatrix}
\]

のように加減速も同時に扱うことが多い。

### 2. 速度が固定

現在の rollout は固定速度 \(v_{\mathrm{target}}\) を使うため、速度変化に対する応答が入っていない。

これは特に以下で不利になる。

- 高速域
- localization 誤差が乗った条件
- コーナ入口と出口で速度調整したい条件

### 3. モデルが軽い

現在の予測モデルは kinematic bicycle であり、横滑りやタイヤ特性を含まない。

したがって

- 実機寄りの限界挙動
- 高速時の heading 応答
- 操舵に対する実際の遅れ

を十分には表現できない。

### 4. nominal 制御に強く依存

現在は `Pure Pursuit` による nominal steer を基準にしている。

\[
\mathbf{u}^{\mathrm{base}} =
0.7 \mathbf{u}^{\mathrm{prev}} + 0.3 \mathbf{u}^{\mathrm{nom}}
\]

かつ

\[
w_n (\delta_k - \delta_k^{\mathrm{nom}})^2
\]

も入っているので、独立した sampling-based optimal control というより

`Pure Pursuit` の近傍を探索する補正器

として働いている。

### 5. ノイズの扱いが単純

現在は各時刻独立なガウスノイズをそのまま舵角系列に足している。

標準的には次も検討対象になる。

- 時間方向に相関のあるノイズ
- 制御チャネルごとに異なるノイズスケール
- control perturbation の importance weighting

### 6. MPPI 固有の理論項を明示していない

標準的な MPPI は path integral 系の導出に基づいており、control cost と noise 分布の関係が理論的に結びついている。

現在実装は

\[
\mathbf{u}^* = \sum_i \tilde{w}_i \mathbf{u}^{(i)}
\]

という形は持っているが、理論式にある control update の構造を厳密には再現していない。

## いまの実装の位置づけ

現在の `MPPIController` は次のように理解するのが正確である。

- `Pure Pursuit` を nominal generator とする
- steering-only の sampling-based tracker
- 軽量でチューニングしやすい最小実装

これは比較実験や localization 連携の入口としては妥当だが、`MPPI` の性能を本格的に見たいなら次の改良が必要になる。

## 次に進む改良順

### 優先度 1: 入力を 2 次元化する

制御入力を

\[
u_t =
\begin{bmatrix}
v_t \\
\delta_t
\end{bmatrix}
\]

または

\[
u_t =
\begin{bmatrix}
a_t \\
\delta_t
\end{bmatrix}
\]

に拡張する。

これで

- コーナ前で減速
- 立ち上がりで加速
- localization 誤差が大きいときの速度マージン確保

が可能になる。

### 優先度 2: rollout model を 1 段上げる

まずは簡易 dynamic bicycle までは行かなくても、

- 速度状態を持つ
- steering rate や accel 制約を入れる

だけで挙動はかなり変わる。

### 優先度 3: コストを track-centered にする

いまは参照点との差を見ているが、評価の主軸を

- cross track error
- heading error
- progress reward

に寄せると tuning しやすい。

例:

\[
\ell_k =
w_e e_{y,k}^2
+ w_\psi e_{\psi,k}^2
- w_s \Delta s_k
+ w_u \|u_k\|^2
+ w_\Delta \|u_k - u_{k-1}\|^2
\]

### 優先度 4: localization-aware cost を入れる

現在の研究テーマでは `LiDAR localization` を通した条件比較が重要である。

そのため

- localization score が悪いときは速度を抑える
- scan matching の不確かさが高いときは aggressive な操舵を避ける

といった coupling が有効である。

### 優先度 5: noise design を見直す

候補系列が各時刻独立ノイズだと操舵がギザつきやすい。次を試す価値がある。

- 時間方向に平滑化したノイズ
- 低次元パラメータ列をサンプルして補間
- steer と speed で別々のノイズ幅

## 直近の実装方針

次の実装は次の順が自然である。

1. `localized MPPI` の tuning を一度止める
2. 現在の steering-only MPPI を `docs` とコードコメントで固定する
3. `speed + steer` を持つ `MPPIControllerV2` を `experiments/` 配下に新規追加する
4. `Pure Pursuit / MPC / MPPI / MPPIv2` を同じ runner で比較する
5. その後に localization 条件で比較する

## 実装時の注意

- 現行の `mppi_controller.py` は壊さない
- 改良版は別ファイルで追加する
- config も別名で増やす
- 比較対象が増えるので report script は拡張前提で作る

## まとめ

現在の `MPPIController` は、理論的な `MPPI` をそのまま実装したものではなく、

- fixed-speed
- steering-only
- Pure Pursuit biased
- lightweight sampling controller

である。

したがって次に性能差を出したいなら、最初にやるべきことは `num_samples` や `temperature` の微調整ではなく、

`入力次元を増やして、速度も最適化対象にすること`

である。
