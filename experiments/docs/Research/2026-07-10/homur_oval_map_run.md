# 楕円形マップの実行方法

作成日: 2026-07-10

## 目的

F1TENTH Gymで、幅7 m、高さ5 mの楕円形周回コースを実行する。

コースの外側楕円は7 m x 5 mで、内側の障害物との間を約1 m幅の走行路としている。車両は右端の `(x, y) = (3.0, 0.0)` から反時計回りに走り始める。

## 関連ファイル

```text
experiments/maps/homur_oval.png
experiments/maps/homur_oval.yaml
experiments/maps/homur_oval_waypoints.csv
experiments/configs/homur_oval.yaml
experiments/tools/create_oval_map.py
experiments/run_homur_f110.py
```

## 通常実行

リポジトリ直下で、仮想環境のPythonを使って実行する。

```powershell
cd C:\Users\homur\CFolder\Resarch\f1tenth_gym
.\gym_env\Scripts\python.exe experiments\run_homur_f110.py --config experiments\configs\homur_oval.yaml
```

実行すると、マップ、車両、waypointを描画するGUIウィンドウが開く。

## GUIなしでの動作確認

GUIを開かず、指定ステップ数だけ実行する。地図や制御器の変更後の確認に使う。

```powershell
cd C:\Users\homur\CFolder\Resarch\f1tenth_gym
.\gym_env\Scripts\python.exe experiments\run_homur_f110.py --config experiments\configs\homur_oval.yaml --no-render --max-steps 100
```

## マップの再生成

現在と同じ7 m x 5 m、走路幅1 mのマップを再生成する。

```powershell
cd C:\Users\homur\CFolder\Resarch\f1tenth_gym
.\gym_env\Scripts\python.exe experiments\tools\create_oval_map.py --width 7 --height 5 --track-width 1 --name homur_oval
```

このコマンドは次の3ファイルを上書きする。

```text
experiments/maps/homur_oval.png
experiments/maps/homur_oval.yaml
experiments/maps/homur_oval_waypoints.csv
```

別サイズを保存したい場合は、`--name` を変える。たとえば幅8 m、高さ6 mのコースを別名で作る場合は次のように実行する。

```powershell
.\gym_env\Scripts\python.exe experiments\tools\create_oval_map.py --width 8 --height 6 --track-width 1 --name oval_8x6
```

その場合は、対応する実行設定YAMLで `map_path` と `wpt_path` も変更する。

## 確認済み内容

次のGUIなし実行を行い、マップの読み込みと100ステップの走行を確認した。

```powershell
.\gym_env\Scripts\python.exe experiments\run_homur_f110.py --config experiments\configs\homur_oval.yaml --no-render --max-steps 100
```

## TODO

- 実機コースの寸法に合わせて幅、高さ、走路幅を調整する。
- 実機で取得した地図座標系とwaypoint座標系を一致させる。
- 速度設定とPure Pursuitの追従距離を、実機の操舵性能に合わせて調整する。
