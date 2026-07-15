# Python仮想環境の有効化と無効化

作成日: 2026-07-10

## 目的

F1TENTH Gym用に作成した `gym_env` 仮想環境を有効化し、必要なPythonパッケージを使ってスクリプトを実行する。

`f110_gym` はこの仮想環境内でインストールされている。そのため、システム側のPythonで実行すると `ModuleNotFoundError: No module named 'f110_gym'` が発生することがある。

## 仮想環境を有効化する

PowerShellでリポジトリ直下へ移動してから、有効化スクリプトを実行する。

```powershell
cd C:\Users\homur\CFolder\Resarch\f1tenth_gym
.\gym_env\Scripts\Activate.ps1
```

成功すると、PowerShellの行頭に `(gym_env)` と表示される。

```text
(gym_env) PS C:\Users\homur\CFolder\Resarch\f1tenth_gym>
```

有効化後は、次のように通常の `python` コマンドで実行できる。

```powershell
python .\experiments\run_homur_f110.py --config .\experiments\configs\homur_oval.yaml
```

## 仮想環境を無効化する

仮想環境を有効化しているPowerShellで、次を実行する。

```powershell
deactivate
```

行頭の `(gym_env)` が消えれば無効化できている。

```text
PS C:\Users\homur\CFolder\Resarch\f1tenth_gym>
```

PowerShellのウィンドウを閉じる場合も、仮想環境は自動的に無効になる。

## 有効なPythonを確認する

次のコマンドで、現在使われるPython実行ファイルを確認できる。

```powershell
python -c "import sys; print(sys.executable)"
```

仮想環境が有効なら、次のパスが表示される。

```text
C:\Users\homur\CFolder\Resarch\f1tenth_gym\gym_env\Scripts\python.exe
```

## 仮想環境を有効化せずに実行する方法

仮想環境を有効化せず、仮想環境内のPythonを直接指定して実行することもできる。

```powershell
cd C:\Users\homur\CFolder\Resarch\f1tenth_gym
.\gym_env\Scripts\python.exe .\experiments\run_homur_f110.py --config .\experiments\configs\homur_oval.yaml
```

既存のサンプルを実行する場合は次のようにする。

```powershell
.\gym_env\Scripts\python.exe .\examples\waypoint_follow.py
```

## 確認方法

仮想環境を有効化した後、次のコマンドがエラーなく実行できれば、`f110_gym` をPythonから読み込めている。

```powershell
python -c "import f110_gym; print('f110_gym import OK')"
```

## 注意点

- 仮想環境を有効化していない状態で `python` を実行すると、システム側のPythonが使われる場合がある。
- `examples` フォルダ内に移動していても、仮想環境が有効でなければ `f110_gym` は自動で見つからない。
- PowerShellで有効化が拒否される場合は、実行ポリシーの設定を確認する必要がある。
