# Pure Pursuit Math

## 目的

現在の `Pure Pursuit` 実装が何をしているかを数式ベースで整理する。

対象コード:

- `experiments/controllers/pure_pursuit_controller.py`
- `examples/waypoint_follow.py`

## 現在実装の流れ

現在の `Pure Pursuit` は、waypoint 列の中から現在位置より少し先の `lookahead point` を選び、その点へ向かう円弧から舵角を決める幾何制御である。

流れは次の通りである。

1. 現在位置に最も近い軌道位置を求める
2. そこから `lookahead distance` だけ先の目標点を決める
3. 目標点を車両座標系で見た横ずれを求める
4. 横ずれから曲率を計算する
5. 曲率から舵角を計算する
6. waypoint 側の速度に `vgain` を掛けて速度指令を作る

## 状態

車両状態は

\[
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
\]

である。

- \(x, y\): 車両位置
- \(\theta\): 車両 heading

## lookahead 点

現在位置を

\[
\mathbf{p} =
\begin{bmatrix}
x \\
y
\end{bmatrix}
\]

とする。

まず軌道上で現在位置に最も近い位置を求める。実装では waypoint 点そのものではなく、waypoint を結ぶ線分への射影点を使って最近傍を計算している。

その後、現在位置を中心とする半径 \(L_d\) の円

\[
\|\mathbf{r} - \mathbf{p}\| = L_d
\]

と軌道の交点のうち、進行方向で最初に現れる点を `lookahead point` とする。

ここで

\[
L_d = \text{lookahead distance}
\]

であり、config では `tlad` で与えている。

目標点を

\[
\mathbf{p}_{\mathrm{target}} =
\begin{bmatrix}
x_t \\
y_t
\end{bmatrix}
\]

とする。

## 車両座標系での横ずれ

目標点との相対ベクトルは

\[
\Delta \mathbf{p} =
\begin{bmatrix}
x_t - x \\
y_t - y
\end{bmatrix}
\]

である。

Pure Pursuit ではこの目標点を車両座標系で見たときの横方向成分 \(y_L\) を使う。

\[
y_L =
\begin{bmatrix}
\sin(-\theta) & \cos(-\theta)
\end{bmatrix}
\Delta \mathbf{p}
\]

この値が正なら一方へ、負なら反対側へ曲がる。

## 曲率

Pure Pursuit は、現在位置から lookahead 点を通る円弧を考える。

その円弧の曲率は

\[
\kappa = \frac{2 y_L}{L_d^2}
\]

である。

同じことを半径 \(R\) で書くと

\[
R = \frac{1}{\kappa} = \frac{L_d^2}{2 y_L}
\]

となる。

現在のコードは半径を経由して計算している。

## 舵角

wheelbase を \(L\) とすると、bicycle model から舵角 \(\delta\) は

\[
\delta = \arctan(L \kappa)
\]

である。

曲率を代入すると

\[
\delta = \arctan\left(\frac{2Ly_L}{L_d^2}\right)
\]

となる。

実装では

\[
\delta = \arctan\left(\frac{L}{R}\right)
\]

の形で書かれているが、同じ式である。

## 速度指令

速度は Pure Pursuit の幾何から直接決めているわけではない。

waypoint に含まれる速度を \(v_{\mathrm{wp}}\) とすると、現在の実装では

\[
v = k_v \, v_{\mathrm{wp}}
\]

を使う。

ここで

\[
k_v = \text{vgain}
\]

である。

したがって現在の `Pure Pursuit` は、

- 横方向は幾何制御
- 速度は waypoint ベースの feedforward

という構成になっている。

## 制御則のまとめ

現在の実装は、最終的に次の制御則として読める。

1. 軌道上の lookahead 点 \(\mathbf{p}_{\mathrm{target}}\) を決める
2. 車両座標系横ずれ \(y_L\) を求める
3. 曲率

\[
\kappa = \frac{2 y_L}{L_d^2}
\]

を計算する
4. 舵角

\[
\delta = \arctan(L\kappa)
\]

を出す
5. 速度

\[
v = k_v v_{\mathrm{wp}}
\]

を出す

## パラメータの意味

### `tlad`

\[
tlad = L_d
\]

- 大きいほど遠くを見る
- 操舵は穏やかになる
- ただしコーナで曲がりが鈍くなる

### `vgain`

\[
vgain = k_v
\]

- waypoint 速度の倍率
- 大きいほど速く走る
- 同じ `tlad` でも速度が上がると追従が厳しくなる

## 実装上の特徴

良い点:

- 計算が軽い
- 安定しやすい
- 実装が単純で調整しやすい

弱い点:

- 最適化ベースではない
- 将来の制約を直接扱わない
- 高速条件では `tlad` に敏感
- localization 誤差があると target 点選択がそのまま崩れる

## コード対応

- 最近傍探索:
  - `nearest_point_on_trajectory(...)`
- lookahead 点探索:
  - `first_point_on_trajectory_intersecting_circle(...)`
- 舵角計算:
  - `get_actuation(...)`
- planner 本体:
  - `PurePursuitPlanner.plan(...)`
- 実験 runner からの wrapper:
  - `PurePursuitController.plan(...)`

## まとめ

現在の `Pure Pursuit` は

`lookahead 点に向かう円弧を毎ステップ幾何的に作る制御`

である。

したがって、挙動を大きく変える主要パラメータはまず

- `tlad`
- `vgain`

の 2 つである。
