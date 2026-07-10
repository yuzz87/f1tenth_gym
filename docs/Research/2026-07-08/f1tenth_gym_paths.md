# f1tenth_gym 設定ファイルの場所

日付: 2026-07-08

## 目的

F1TENTH Gymで、機体設定、マップ設定、制御手法を変更するときに確認する主なファイルの場所を記録する。

## リポジトリの場所

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym
```

## 機体設定

デフォルトの車両パラメータが定義されている場所:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\gym\f110_gym\envs\f110_env.py
```

特に確認する場所:

```text
gym\f110_gym\envs\f110_env.py
127〜130行目付近
```

実行時に車両モデルやパラメータが使われる場所:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\gym\f110_gym\envs\base_classes.py
```

車両の運動モデル:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\gym\f110_gym\envs\dynamic_models.py
```

主なパラメータ:

```text
mu, C_Sf, C_Sr, lf, lr, h, m, I,
s_min, s_max, sv_min, sv_max,
v_switch, a_max, v_min, v_max,
width, length
```

メモ:

- `lf` は重心から前輪軸までの距離。
- `lr` は重心から後輪軸までの距離。
- `lf + lr` がホイールベースになる。
- 機体寸法を変える場合は、制御器側で使っているホイールベースも確認する。

## マップ設定

サンプルのマップ設定:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples\config_example_map.yaml
```

サンプルマップ本体:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples\example_map.yaml
C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples\example_map.png
```

標準で入っているマップ:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\gym\f110_gym\envs\maps
```

例:

```text
gym\f110_gym\envs\maps\berlin.yaml
gym\f110_gym\envs\maps\berlin.png
gym\f110_gym\envs\maps\levine.yaml
gym\f110_gym\envs\maps\levine.pgm
gym\f110_gym\envs\maps\skirk.yaml
gym\f110_gym\envs\maps\skirk.png
gym\f110_gym\envs\maps\vegas.yaml
gym\f110_gym\envs\maps\vegas.png
```

LiDARスキャンとマップ読み込み:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\gym\f110_gym\envs\laser_models.py
```

マップは基本的に次のペアで読み込まれる:

```text
map_name.yaml
map_name.png または map_name.pgm
```

YAMLファイルには主に次の情報が入る:

```text
image
resolution
origin
occupied_thresh
free_thresh
```

## 制御手法

Pure Pursuitのサンプル:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples\waypoint_follow.py
```

waypointファイル:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples\example_waypoints.csv
```

重要な処理の流れ:

```text
planner.plan(...)
env.step(np.array([[steer, speed]]))
```

actionの順番:

```text
[steer, speed]
```

おすすめの進め方:

```text
コピー元:
C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples\waypoint_follow.py

コピー先の例:
C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples\my_controller.py
```

まずは元のサンプルを直接変更するのではなく、コピーした実験用ファイルを変更する。

## 変更方針

これらの設定や制御手法は変更可能。

実験では、まず次の場所に実験用ファイルを作るのが安全:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples
```

シミュレータ本体の挙動を変更する必要がある場合を除き、最初から `gym\f110_gym\envs` 配下のコアファイルを大きく変更しない方がよい。

## 実行確認

仮想環境に入る:

```powershell
cd C:\Users\homur\CFolder\Resarch\f1tenth_gym
.\gym_env\Scripts\Activate.ps1
```

サンプルを実行する:

```powershell
cd examples
python waypoint_follow.py
```

仮想環境に入らずに直接実行する場合:

```powershell
cd C:\Users\homur\CFolder\Resarch\f1tenth_gym\examples
..\gym_env\Scripts\python.exe waypoint_follow.py
```

## 記録ルール

今後の研究記録は次の場所に置く:

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\docs\Research
```

日付ごとに記録する場合は、先に日付フォルダを作成し、その中にMarkdownファイルを作成する。
