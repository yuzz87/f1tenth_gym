# RPLIDAR A1 デジタルツイン実装計画書

- 作成日: 2026-08-02
- 改訂日: 2026-08-02
- 文書版: 1.1（既存環境の非破壊を最優先化）
- 文書状態: 実装着手前
- 論理プロジェクト名: `rc_autonomy_sim`
- 対象: RPLIDAR A1M8-R6を搭載したRCカーの2Dセンサ・車両デジタルツイン

## 1. 目的

本計画書は、現在の`f1tenth_gym`にある実車向けシミュレーション、ROS 2接続、
実測LiDAR記録を基礎として、独立したデジタルツインを新規実装する手順を定める。

### 1.1 最上位の成功条件

本計画では、機能、性能、納期よりも、現在の環境を壊さないことを優先する。実装開始後の
`f1tenth_gym`、共有`.git`、既存`gym_env`、Conda `base`、system Python、ROS 2 Foxy install、
既存ROS workspace、raw data、地図を変更しない。機能実装とこの条件が両立しない処理は実行せず、
停止して設計を見直す。

実装は次の独立境界に限定する。パスは2026-08-02時点で未作成であることを確認済みだが、
Phase 0のpreflightで再検査する。

```text
SOURCE_ROOT=/home/ubuntuyuzz/Desktop/f1tenth_gym
BOOTSTRAP_ROOT=/home/ubuntuyuzz/Desktop/rc_autonomy_sim_bootstrap
PROJECT_ROOT=/home/ubuntuyuzz/Desktop/rc_autonomy_sim
STATE_ROOT=/home/ubuntuyuzz/Desktop/rc_autonomy_sim_state
```

- `SOURCE_ROOT`は計画確定後に読取専用の参照元として扱う。
- `BOOTSTRAP_ROOT`は初回snapshotとtransaction journal専用とし、最初に作るownership境界とする。
- `PROJECT_ROOT`は独立した新規Git repositoryとし、`SOURCE_ROOT/.git`を共有しない。
- `STATE_ROOT`はvenv、sandbox/rootfs cache、build、test、run、ROS log、展開済みartifact専用とする。
- 実装processが書き込めるのは`BOOTSTRAP_ROOT`、`PROJECT_ROOT`、`STATE_ROOT`だけとする。
- source、raw、mapはcopyまたはread-only mountで入力し、移動、手修正、in-place変換をしない。
- 実機I/O、外部宛UDP、既存ROS graphへの参加を禁止し、simulation内の出力だけを許可する。

Phase 0の環境保全gateは全機能taskの先行条件とする。gate失敗時は自動修復や自動revertを行わず、
processを停止して差分とlogを保存する。作業中に利用者の変更を検出した場合も、それを上書きせず、
baselineの再承認まで関連作業を停止する。再承認は既存baselineを上書きせずversionを追加し、
承認差分と理由を独立repositoryのADRへ残す。

完成時には、同じ車両モデル、地図、LiDAR契約、Controller、評価指標を用いて、
次の3種類の入力を交換可能にする。

1. 地図から生成した理想LiDAR
2. RPLIDAR A1の実測特性を加えた仮想LiDAR
3. 保存済みの実LiDAR・odometry記録

この実装は、保存地図を用いた2D ray castingと実測データ再生を対象とする。
材質、反射強度、日光、鏡面、透明面を再現する光学的3Dセンサモデルは対象外とする。

実装可否の結論は「可能」である。既存repositoryには車両model、地図ray casting、
Controller、localizer、ROS 2接続、CSV replayの基礎がある。主作業は、それらを独立packageへ
移植しながら、互いに矛盾しているLiDAR設定を共通契約へ統一し、再現可能な実測artifactと
自動受入testを追加することである。

## 2. 参照元と優先順位

参照元は用途ごとに正本を分ける。単純な一列の優先順位にはしない。

1. LiDARの公開インターフェースと目標値は
   `experiments/docs/lidar_experiment_digital_twin_essential.html`を規範とする。
   着手時にSHA-256
   `6da4634caef497fa86a68c69cd8a24cf5587f31c22f29abd871873e05ca78b99`
   を照合する。
2. 行数、周期、有効beam数などの実測統計は、保存済み`scan.csv`、`odom.csv`、
   診断CSVへ定義済みの検証・重複除去を適用した再集計結果を正本とする。
3. 座標変換、後方mask、ROS messageの現行挙動は、実機通信コードと実際に発行された
   `LaserScan`契約を実装根拠とする。ただしHTMLの規範値と矛盾する場合は判断ゲートへ送る。
4. 完全レポートは取得条件と解釈の根拠、既存シミュレーション仕様はlegacy回帰の根拠にする。

既存の派生仕様書にある72 beams、5 Hz、欠測値`range_max`は、過去のシミュレーションを
再現するための`legacy_offline_72x5`プロファイルとして残す。新しい実測準拠プロファイルへ
暗黙に流用しない。core内の無効値はlegacyでも`inf`とし、legacy回帰専用adapterだけが
legacy localizer/runner境界で`valid_mask=false`の値を`range_max`へ戻す。

## 3. 完成目標

### 3.1 Level 1: 幾何・インターフェース互換

- 360 beams、1度刻み、非重複の全周scanを生成する。
- 角度範囲を`-pi`から`pi - 1 deg`にする。
- 有効距離範囲を0.15 mから12.0 mにする。
- 無効値を`inf`で表現する。CSVでは`null`へ変換する。
- `base_link -> laser`を`x=0.25 m, y=0, z=0.11 m, yaw=0`にする。
- 後方中心±30度の自己遮蔽sectorを無効化する。
- core内部、CSV、ROS 2 `/scan`で同じscan契約を使用する。

### 3.2 Level 2: 実測特性を反映した生成モデル

- 公称周期は0.132 sを正本とし、周波数はその逆数約7.5758 Hzから導出する。
  表示上のみ約7.6 Hzへ丸め、周期と周波数を別々に設定しない。
- 保存scanから抽出した周期分布をseed付きで再現する。
- 後方固定maskと、それ以外の環境依存欠測を分離する。
- 完全scan CSVでは818 unique scans中2件ある約0.258 sの二周期scanを、
  通常周期とは別の故障注入として再現できるようにする。診断記録だけにある3件目を
  `3/818`として混ぜない。
- 実測根拠のない絶対測距精度や材質依存noiseは製品特性として設定しない。

### 3.3 Level 3: 実ログ再生

- 5試行の`scan.csv`を読み込み、phase境界の重複scanを除外する。
- `odom.csv`をscan時刻へ整列し、同一入力をlocalizerへ再生する。
- 同一の記録、設定、seedから同一結果を得られるようにする。
- CSVを正本とし、rosbagを補助入力として扱う。
- 必要に応じてROS 2 `/scan`、`/odom`、`/clock`へ再発行できるようにする。

## 4. 対象範囲

### 4.1 初期版に含める

- 独立した兄弟directoryとGit repositoryによる開発境界
- 専用Python virtual environmentと固定sandbox/rootfsまたはremote runner
- source、environment、raw dataの非破壊guardとrollback検証
- 独立したPython package
- 4状態Bicycle Model
- 速度・操舵Actuator Model
- ROS形式PGM/YAML地図読込
- occupancy gridに対する2D ray casting
- RPLIDAR A1共通scan契約
- 後方自己遮蔽mask
- 実測周期・欠測モデル
- Pure Pursuit、MPC、MPPI
- Grid Map Localizer
- CSV実ログ再生
- UDP LiDAR protocolのloopback adapter
- headless実験runner
- unit、integration、statistical、acceptance test
- CSV、JSON、図、Markdown report
- ROS 2 adapter。coreとはoptional install extraとして分離するが、初期releaseでは必須実装とする

### 4.2 初期版に含めない

- `SOURCE_ROOT`、共有`.git`、既存`gym_env`、Conda `base`、system/user site-packagesの変更
- 既存ROS workspaceの`build/`、`install/`、`log/`の更新
- `sudo apt`、global/user `pip`、Conda packageの追加・削除
- raw data、元地図、既存golden、既存実験結果への書込み
- SLAMと新規地図作成
- 3D物理シミュレーション
- LiDAR intensity・qualityモデル
- 材質別・日照別の反射モデル
- 実測根拠のない距離bias・標準偏差の同定
- GPIO、PWM、`pigpio`、実アクチュエータ出力
- `/dev/ttyUSB0`への接続と公式SDK Agentの代替
- GUI。初期版はheadless実行とreportを正本にする
- 実車閉ループ走行の性能保証
- 旋回・振動中のLiDAR精度保証

## 5. 現状と目標との差分

| 項目 | 現在の仮想系 | 実測準拠の目標 | 対応 |
| --- | --- | --- | --- |
| beam数 | 72または360 | 360 | 共通profileへ統一 |
| 角度終端 | `+pi`を含む実装あり | `pi - 1 deg` | `2*pi/360`で生成 |
| 更新周波数 | 5、5.5、10 Hzが混在 | 約7.6 Hz | schedulerを共通化 |
| 欠測値 | 12.0 mへ置換 | `inf` | 内部契約を修正 |
| 後方mask | 実機bridgeのみ | 仮想・実機共通 | 共通filterへ移動 |
| 取付位置 | ray原点だけ0.25 mの箇所あり | TFを含め完全一致 | adapterを修正 |
| 欠測モデル | 一様Bernoulli | 固定maskと実測欠測を分離 | empirical artifactを生成 |
| scan取得 | 全beam同一pose | 初期版は同一pose | rolling scanは将来拡張 |
| replay | localizer中心 | 共通ScanFrameとROSへ接続 | replay sourceを追加 |
| データ配布 | raw resultsはgit-ignore | clean環境で復元可能 | manifestとchecksumを追加 |
| 地図path | 絶対path | 相対path | asset移植時に修正 |

## 6. 非破壊実装ポリシーと設計原則

### 6.1 書込み境界

| 対象 | 実装中の扱い | 禁止操作 | 検証 |
| --- | --- | --- | --- |
| `SOURCE_ROOT` | read-only参照 | file追加・削除・編集、cache生成 | Git状態とsource manifestの前後比較 |
| `SOURCE_ROOT/.git` | 参照のみ | branch、worktree、index、stash、LFS設定の変更 | 全entry canonical manifestとGit semantic比較 |
| 既存`gym_env` | baseline実行条件の記録のみ | activate後のinstall、pip/venv操作 | `pyvenv.cfg`、package、site-packages manifest比較 |
| Conda `base`・system | 使用しない | `conda install/remove`、`pip --user`、`sudo apt` | package manifest比較 |
| 既存ROS workspace | 参照のみ | source、colcon build、生成物更新 | `build/install/log` manifest比較 |
| `BOOTSTRAP_ROOT` | snapshotとtransaction journal | baseline上書き、runtime出力 | ownership UUIDとappend-only journal検査 |
| `PROJECT_ROOT` | sourceと独立Git repository | sourceへのhardlink・symlink | `realpath`、device/inode、Git remote検査 |
| `STATE_ROOT` | 唯一のruntime書込み先 | source/rawへのpath escape | canonical path検証とI/O監査 |
| raw data・元地図 | read-only入力 | in-place変換、cache、lock、temp生成 | file一覧、size、SHA-256前後比較 |

3つの新rootは既存path、symlink、`SOURCE_ROOT`配下の場合に作成を拒否する。
sourceから必要なコードとassetを移すときはallowlistとchecksumを持つcopyだけを許可し、hardlink、
move、元fileの相対path書換えを禁止する。新しい地図YAMLだけを`PROJECT_ROOT`側で相対path化する。

source manifestは`.git`を除く全entryを対象とし、pathをNUL区切りbyte列で昇順化して、file type、
permission mode、symlink target、size、regular fileのSHA-256をcanonical JSONへ保存する。約16,000件の
ignored entryも選別せず含め、mtime/atimeだけを比較対象外とする。`.git`もobjects、reflog、refs、
worktree metadata、config、alternates、lock fileを含む全entryを同じ規則でcanonical manifest化する。
さらに`GIT_OPTIONAL_LOCKS=0`で読取ったHEAD、index tree、branch、statusをsemantic fieldsとして保存する。

ISO-001はtransaction UUIDを生成し、`BOOTSTRAP_ROOT`をatomicに`mkdir`した直後に
`O_CREAT|O_EXCL`でownership markerとappend-only journalを作る。同じUUIDとjournalがある部分作成は
再開できる。marker作成前に中断したpath、UUID不一致、内容不一致は自動削除せず、差分確認と明示承認を
要求する。承認済みcleanupも3つの新rootだけをrealpath検証して行い、`SOURCE_ROOT`へ適用しない。

全runtime出力は`${STATE_ROOT}/runs/<run_id>/`へ置く。run directoryは新規作成に限定し、既存の
非空directory、repository、raw、assets、golden、またはそれらを指すsymlinkを出力先に指定した
場合は起動時に拒否する。`HOME`、`XDG_CACHE_HOME`、`TMPDIR`、`PYTHONPYCACHEPREFIX`、
`ROS_HOME`、`ROS_LOG_DIR`、bag、plot cacheもrunまたはbuild専用directoryへ向ける。

### 6.2 Git、環境、rollback

- `PROJECT_ROOT`は未作成pathを検査してから新規作成し、そこで`git init`して専用branchで実装する。
  `SOURCE_ROOT`の
  branch/worktreeを作らず、shared object databaseやGit alternatesも使わない。
- 独立repositoryは`main`へbootstrap commitを作り、`bootstrap/environment-v1`tagを固定してから
  `feature/a1-digital-twin`で実装する。Phase checkpoint tagはfeature branchだけへ付ける。
- rollback rehearsal用`rehearsal/release-v1`はbootstrap `main`の固定SHAから作り、受入後に固定する
  `RC_SHA`を`--no-ff`でmergeする。本番`main`はPhase 8の明示承認まで更新しない。
- `REL-001`は受入対象commitを`RC_SHA`とannotated tag `rc/v1.0.0-rc.1`で固定する。
  rehearsalとPhase 8は可変なbranch tipではなく、同じtag objectと`RC_SHA`だけをmergeする。
- 2026-08-02時点でhostにDocker、Podman、Buildah、Python 3.11はなく、`bwrap`と`unshare`は
  network namespaceを含むsmoke testに成功している。hostへcontainer runtimeを導入しない。
- local isolationは`bwrap`とchecksum固定rootfsを第一候補とし、利用不能時は専用remote runnerを使う。
  rootless runtimeを使う場合もbinary、storage、configを`STATE_ROOT`へ限定し、DG-08で承認する。
- coreは`${STATE_ROOT}/venvs/`の専用venvまたは固定rootfs、ROS 2 FoxyはUbuntu 20.04・
  Python 3.8の固定rootfs/remote jobで実行する。hostの`python3`や`pip`を暗黙に使わない。
- dependency取得とtestを分離し、test時は外部networkを無効にする。
- 各Phaseの開始前後に環境保全guardを実行し、不一致なら後続taskを開始しない。
- 各Phaseは独立repository内のcommitとtagでcheckpoint化する。
- baselineは`source_baseline_vNNN.json`としてappend-onlyに保存し、active versionをmetadataで参照する。
  利用者変更を検出した場合は停止し、明示承認、canonical diff、理由を
  `PROJECT_ROOT/docs/decisions/`のADRへ記録してから新versionを追加する。過去versionを上書きしない。
- rollbackはprocessをPID単位で停止し、logを保存後、独立repositoryを前checkpointから
  再構成する。`SOURCE_ROOT`のreset、clean、checkout、stash、rebase、自動復元を行わない。
- 通常cleanupは`STATE_ROOT`配下でownership markerとrun UUIDを照合できたdirectoryだけに行う。
  ISO-001途中失敗のcleanupだけは、同一transaction UUIDを持つ3つの新rootへ明示承認後に限定する。

### 6.3 ROS、UDP、実機I/O隔離

- ROS jobはnon-root、host physical device passthroughなし、host networkなしで実行する。
  sandbox内の`/dev`は`null`、`zero`、`urandom`など最小のvirtual device allowlistだけを許可する。
- defaultの`ROS_DOMAIN_ID=0`を禁止し、job固有のnon-default IDと
  `ROS_LOCALHOST_ONLY=1`をlaunch前に検査する。
- `/clock`以外は相対topic名とし、`/digital_twin/<run_id>`namespaceへ閉じ込める。
  `/tf`と`/tf_static`もtest graph内でremapする。実機互換の絶対topic名は隔離sandbox内の
  contract testだけで検証する。
- UDP protocol v1はIPv4限定とする。receiverは`127.0.0.1`だけへbindし、senderも名前解決後の
  addressが`127.0.0.1`の場合だけ許可する。IPv6、`0.0.0.0`、外部IP、broadcast、multicastを
  設定validationで拒否する。通常testはOS割当の
  ephemeral portを使い、5010は隔離したprotocol互換testだけで使う。
- 車両plantの`ActuatorModel`と実I/O actuatorを分け、ROS graphにはin-memoryの
  `SimulatedCommandSink`だけを置く。`pigpio`、`RPi.GPIO`、serial、sysfs GPIO、
  `/dev/tty*`、UDP actuator bridgeをpackage、entry point、launchへ含めない。
- 終了時に起動PID、ROS node、DDS process、publisher、UDP listenerの残存がないことを確認する。
- 起動後もnode、publisher、subscriber、service、actionを監査し、`/clock`、`/rosout`、
  `/parameter_events`などの明示allowlist以外がrun namespace外に存在した場合は失敗させる。

### 6.4 単一のscan契約

Gym互換層、headless simulator、ROS 2 publisherが個別に角度・欠測処理を持たないようにする。
coreが返す`ScanFrame`だけを正本とし、各adapterは形式変換のみを担当する。
SDK固有の角度反転はraw hardware adapterだけで行い、coreとROS adapterへ持ち込まない。
2D ray castingには`x/y/yaw`を適用し、`z=0.11 m`は3D TF metadataとして保持する。

### 6.5 処理段階の分離

```text
ScanScheduler
    |
    v
Sample VehicleState + OccupancyMap
    |
    v
IdealRayCaster
    |
    v
RangeGate
    |
    v
RearOcclusionMask
    |
    v
MeasuredValidityModel / FaultInjector
    |
    v
AvailabilityQueue
    |
    v
ScanFrame
    +--> Localizer
    +--> Controller pipeline
    +--> CSV recorder
    +--> ROS 2 LaserScan adapter
```

schedulerは`capture_time_s = t0 + k * period`で先に発火し、その時刻のvehicle poseをsampleする。
ray casting、欠測、利用可能時刻、通信形式を分けることで、理想、実測準拠、replayを同じAPIで扱う。
同一simulation timestampでは、次のevent priorityをtestで固定する。
`physics advance -> scan available -> localizer -> controller -> new scan capture`

### 6.6 profileの版管理

少なくとも次のprofileを用意する。

| profile | 用途 |
| --- | --- |
| `legacy_offline_72x5` | 既存27 runの72-beam/5-Hz/欠測互換adapter確認 |
| `a1_ideal_360` | Level 1の決定論的scan |
| `a1_measured_2026_07` | Level 2の実測分布 |
| `a1_fault_two_period` | 約0.258 s scanの安全試験 |
| `recorded_phase5` | Level 3の実ログ再生 |

既存profileを上書きせず、新profileを追加して比較可能にする。

### 6.7 再現性

- 乱数生成器を依存注入する。
- profile、seed、地図checksum、データchecksumを出力metadataへ保存する。
- wall clockをsimulation timeへ混入させない。
- 計算時間以外のgolden比較は同じ入力で決定論的にする。

### 6.8 実測値と仮定の分離

設定には値の由来を保持する。

```yaml
source: measured_phase5_2026_07
environment: small_test_area_03
scope: environment_specific_reference
```

実測73.1%を製品仕様として一般化しない。noiseやfaultの合成値は`synthetic`と明記する。

## 7. coreデータ契約

### 7.1 A1 ScanGeometry

```text
beam_count: 360
angle_min_rad: -pi
angle_increment_rad: pi / 180
angle_max_rad: angle_min_rad + 359 * angle_increment_rad
range_min_m: 0.15
range_max_m: 12.0
frame_id: laser
```

`angle_max_rad`は独立設定にせず、beam数とincrementから導出する。

### 7.2 ScanFrame

```text
clock_domain_id: str
capture_time_s: float
available_time_s: float
scan_period_s: float
time_increment_s: float
geometry: ScanGeometry
ranges_m: float64[N]
valid_mask: bool[N]
invalid_reason: uint8[N]
source: ideal | measured | replay
session_id: uint32
sequence_id: int
metadata: mapping
```

契約条件:

- `N == geometry.beam_count`かつ`ranges_m`、`valid_mask`、`invalid_reason`のshapeが`(N,)`
- A1 profileでは`N == 360`、legacy profileでは`N == 72`
- 無効beamは`ranges_m[index] == inf`かつ`valid_mask[index] == false`
- 有効beamは`range_min_m <= range <= range_max_m`
- `invalid_reason`は`0=valid`、`1=no_hit`、`2=outside_map`、`3=range_gate`、
  `4=rear_mask`、`5=measured_missing`、`6=injected_fault`、`7=recorded_missing`の
  版固定enumとする
- `valid_mask[index] == (invalid_reason[index] == 0)`を満たす
- `ranges_m`にNaNを含めない
- `capture_time_s`と`available_time_s`は`clock_domain_id`が示す同じclock上にある
- `capture_time_s <= available_time_s`
- `sequence_id`はsource内で単調増加する

時刻の意味を次で固定する。

- `capture_time_s`: sourceがscanへ割り当てた計測時刻。ROS `LaserScan.header.stamp`へ設定する。
  generated sourceでは先頭beamの取得開始時刻とする。
- `available_time_s`: scan全体をconsumerへ渡せる時刻。publisherはこの時刻まで発行しない。
- A1 generated source: `available_time_s = capture_time_s + scan_period_s + transport_delay_s`とする。
- legacy generated source: 旧runner互換のため`available_time_s = capture_time_s`とする。
- Phase 5 CSV replay: 取得時設定`lidar_use_source_timestamp=false`のため、保存`stamp_s`は
  bridge publish時刻であり、真の先頭beam時刻は復元できない。golden互換のためrebase後の
  `capture_time_s`と`available_time_s`を同じ保存stampにし、metadataへ
  `timestamp_semantics=bridge_publish_time`、元stamp、元clock名を保持する。
- Safety outage age: subscriberがscanを契約検査後に受理したlocal時刻から計算する。
- source age: `now - header.stamp`を別に検査し、future toleranceは0.05 sとする。

これにより、scan acquisitionとtransport delayはlocalizerのlatencyへ反映しつつ、outage watchdogを
異なるepochやROS通信遅延から分離する。`time_increment_s`は`scan_period_s / N`から導出する。

### 7.3 A1実測準拠profile

```yaml
profile: a1_measured_2026_07
sensor:
  product: Slamtec RPLIDAR A1M8-R6
  scan_mode: Sensitivity
geometry:
  beam_count: 360
  angle_min_rad: -3.141592653589793
  angle_increment_rad: 0.017453292519943295
  range_min_m: 0.15
  range_max_m: 12.0
  invalid_value: inf
extrinsics:
  x_m: 0.25
  y_m: 0.0
  z_m: 0.11
  yaw_rad: 0.0
occlusion:
  mask_contract: rear_60_indices_v1
  masked_index_ranges:
    - [0, 30]
    - [331, 359]
timing:
  nominal_period_s: 0.132
measured_model:
  distribution_artifact: data/derived/a1_phase5_empirical_v1.npz
  reference_stats: data/derived/a1_phase5_reference_stats_v1.json
  source_manifest: data/manifests/phase5_replay_core_v1.json
```

`nominal_rate_hz`は保存せず、`1 / nominal_period_s`から導出する。後方±30度という意味仕様と
上記60 binの離散契約はDG-06で固定し、浮動小数の角度比較へ依存しない。

### 7.4 UDP transport契約

UDP adapterはcoreのscan生成とは分離し、既存Pi-to-PC経路をlocalhostで検証するために使う。

```text
port: 5010
protocol_version: 1
header_bytes: 64
beams_per_packet: 120
packets_per_scan: 3
packet_bytes: 544
payload: network-byte-order float32
integrity: CRC32 + scan_id + chunk_index
```

adapterは重複chunkを数え、矛盾する重複、CRC不一致、不完全scan、timeoutを破棄する。
これはGPIOや実アクチュエータへ接続しないdata transportだけの機能とする。port 5010は
wire compatibility用の規範値であり、通常のunit/integration testはOS割当のephemeral portを使う。
bind先と送信先はIPv4 `127.0.0.1`に限定し、IPv6、解決後のaddressが`127.0.0.1`でない設定、
`0.0.0.0`、broadcast、multicastを起動前に拒否する。UDP testは外部routeを持たない隔離networkで
実行し、実機portが使用中でも通常testへ影響しないようにする。

### 7.5 Legacy互換契約

`legacy_offline_72x5`は新A1の仕様ではなく、既存27 runsを新package上で再現するための
隔離されたcompatibility facadeとする。

```text
beam_count: 72
angle_min_rad: -pi
angle_increment_rad: 2 * pi / 71
angle_max_rad: +pi
range_min_m: 0.15
range_max_m: 12.0
scan_period_s: 0.2
available_time_s: capture_time_s
rear_mask: disabled
unknown_policy: legacy_free
range_policy: legacy_clip_to_limits
extrinsic_x_m: 0.25
```

`LegacyPipelineFacade`はray casterへ`unknown_policy=legacy_free`とrear mask無効を注入する。
その`LegacyScanView`を`ScanFrame`の直後、legacy localizerとrunner loggingの前に置き、no-hit、
地図外、dropoutの`valid_mask=false`を12.0 mへ変換し、`invalid_reason`から旧`valid_ratio`の
算出規則を再現する。A1 localizer、A1 Controller pipeline、ROS adapterはこのfacadeを通らない。
これにより`inf`を正本とするcore契約を崩さず旧trajectoryを比較できる。

## 8. 新規プロジェクト構成

```text
<BOOTSTRAP_ROOT>/
├── ownership.json
├── transaction.jsonl
└── baselines/source_baseline_v001.json

<PROJECT_ROOT>/
├── .gitignore
├── README.md
├── LICENSE
├── pyproject.toml
├── constraints/
│   ├── py38.lock
│   └── py311.lock
├── containers/
│   ├── core/Dockerfile
│   └── ros2-foxy/Dockerfile
├── sandbox/
│   ├── rootfs.lock.json
│   ├── run_core_bwrap.sh
│   └── run_ros_bwrap.sh
├── scripts/
│   ├── capture_source_baseline.py
│   ├── verify_source_unchanged.py
│   └── verify_io_boundary.py
├── configs/
│   ├── vehicle.yaml
│   ├── sensors/
│   │   ├── legacy_offline_72x5.yaml
│   │   ├── a1_ideal_360.yaml
│   │   └── a1_measured_2026_07.yaml
│   ├── benchmarks/lidar_reference.yaml
│   └── experiments/
│       ├── a1_smoke.yaml
│       ├── offline_baseline.yaml
│       └── replay_phase5.yaml
├── src/rc_autonomy_sim/
│   ├── types.py
│   ├── configuration.py
│   ├── models/
│   ├── controllers/
│   ├── sensors/
│   │   ├── geometry.py
│   │   ├── raycaster.py
│   │   ├── filters.py
│   │   ├── measured_model.py
│   │   ├── scheduler.py
│   │   └── replay.py
│   ├── localization/
│   ├── simulation/
│   ├── evaluation/
│   ├── reporting/
│   ├── compat/legacy.py
│   ├── ros2_adapter/
│   └── transport/udp_protocol.py
├── assets/maps/small_test_area_03/
├── data/
│   ├── manifests/
│   │   ├── phase5_replay_core_v1.json
│   │   └── small_test_area_03_v1.json
│   ├── fixtures/
│   └── derived/
├── golden/
│   ├── environment/
│   ├── legacy/
│   ├── a1/
│   └── benchmarks/
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── statistical/
│   └── acceptance/
└── docs/

<STATE_ROOT>/
├── ownership.json
├── venvs/
├── rootfs/
├── build/
├── cache/
├── artifacts/
│   ├── staging/
│   └── read_only/
└── runs/<run_id>/
```

core packageは実行時に現在の`f1tenth_gym`をimportしない。ROS 2はoptional install extra、
GUIは初期release対象外にする。`PROJECT_ROOT`にはsource、固定fixture、manifest、goldenだけを置き、
venv、build、full raw、cache、temporary file、実行結果は`STATE_ROOT`へ分離する。

## 9. 実装フェーズ

### 9.1 Phase 0: 環境隔離、要件固定、ベースライン保全

#### 目的

現在の環境を変更せずに実装できる境界を先に作り、移植による不具合と、新しいLiDAR仕様による
意図的な数値差を分離する。本Phaseはhard gateであり、完了条件をすべて満たすまでPhase 1以降の
source実装を開始しない。

2026-08-02の予備確認では、`SOURCE_ROOT`はbranch `feature/spd`、HEAD
`1496cc5c37522d3cb2bb754a6bd40567016c0ba4`であり、参照HTMLと本計画書が未追跡である。
現在のshellはConda `base`のPython 3.13.9、既存`gym_env`はPython 3.8.10で`pip check`成功、
`ROS_DISTRO=foxy`である。「clean化」せず、本書確定後に再取得したこのdirty状態をbaselineとして
そのまま保全する。

#### 作業

- `BOOTSTRAP_ROOT`、`PROJECT_ROOT`、`STATE_ROOT`が未作成、非symlink、`SOURCE_ROOT`外であることを
  read-only preflightで検査する。transaction UUIDを発行し、`BOOTSTRAP_ROOT`を作ってownership markerと
  journalを最初に保存する。
- `GIT_OPTIONAL_LOCKS=0`を設定し、`SOURCE_ROOT`全entryのcanonical manifestと`.git`専用manifest、
  branch、HEAD、status、tracked diffを`BOOTSTRAP_ROOT/baselines/source_baseline_v001.json`へ保存する。
- 既存`gym_env`の`pyvenv.cfg`、`pip freeze`、`pip check`、site-packages manifest、Conda `base`、
  user/system Python、apt/dpkg、ROS install、既存ROS workspaceの`build/install/log` manifestを
  `BOOTSTRAP_ROOT`へ保存する。
- journalを更新しながら`PROJECT_ROOT`と`STATE_ROOT`を作り、同じtransaction UUIDのownership markerを
  置く。`PROJECT_ROOT`は独立Git repositoryとして初期化し、最小READMEと境界設定だけのbootstrap
  commit/tagを作って`feature/a1-digital-twin`へ移る。Git alternates、shared object database、
  sourceへのhardlinkを使用しない。初期baselineを`golden/environment/`へchecksum付きでcopyする。
- `SOURCE_ROOT`へのwriteを検知するguardと、path traversal、symlink、hardlink、環境prefix、
  network、device、ROS domainを起動前に検証するfail-closed preflightを、環境構築・data copyより
  先に作る。
- DG-08でlocal `bwrap` + checksum固定rootfsまたは専用remote runnerを選ぶ。local backendはnon-root、
  host physical device passthroughなし、network namespace分離、source/raw read-only、write先`STATE_ROOT`
  だけでsmoke testする。
  Docker、Podman、Buildah、Python 3.11をhostへ追加しない。
- 検証済みbackend上で`STATE_ROOT`内へbootstrap legacy環境、rootfs、cache、build、run directoryを作る。
  これはPhase 0のgolden取得専用とし、Phase 1の製品用venv、lock、sandbox artifactとは分ける。
- 参照文書と本計画書のSHA-256、用途別の正本を新packageのREADMEへ記録し、以後の判断変更先を
  `PROJECT_ROOT/docs/decisions/`へ固定する。
- git-ignore対象のraw dataはmoveせず、allowlistに従って`STATE_ROOT/artifacts/staging`へ
  byte-identical copyする。checksum検証後にread-only化し、source hashが不変であることを確認する。
- legacy baselineはsourceのread-only snapshotを選定済みbootstrap sandboxで実行する。
  `PYTHONDONTWRITEBYTECODE=1`、専用`PYTHONPYCACHEPREFIX`、pytest cache、HOME、TMPDIR、出力pathを
  `STATE_ROOT`へ向け、`SOURCE_ROOT`内にcacheや結果を作らない。
- 既存27 runの集計値を`${PROJECT_ROOT}/golden/legacy/`へ保存する。
- Phase 5 replayの4合格・1評価不能を`${PROJECT_ROOT}/golden/legacy/`へ保存する。
- 対象5試行、地図、生成物のbyte sizeとSHA-256を取得する。
- 取得時設定`lidar_use_source_timestamp=false`と保存stampの意味をdataset metadataへ固定する。
- CSVのschema検証後、元file順で同一stampの先頭行を残し、その後timestamp順へ並べる
  重複除去規則を固定する。discardした行とmetadata差も監査logへ残す。
- reference hostのCPU、OS、Python/NumPy、thread数、CPU governorを保存し、
  32のfree-space位置 x 8 yaw方向からなる256 poseのbenchmark corpusを固定する。
  位置はopen area、wall近傍、unknown境界を層化して含め、corpus checksumを保存する。
  CPU governorとhost設定は記録だけを行い、現在値を変更しない。
- `legacy_offline_72x5`profileを固定する。
- 実測準拠profileの設定schemaを確定する。
- disposableなtest runを作成、停止、再構成するrollback rehearsalを行う。削除対象はownership markerと
  UUIDが一致する`STATE_ROOT`内に限定し、rehearsal後にsource/environment baselineを再照合する。

#### 成果物

- `BOOTSTRAP_ROOT/ownership.json`と`transaction.jsonl`
- `BOOTSTRAP_ROOT/baselines/source_baseline_v001.json`
- `golden/environment/source_baseline_v001.json`
- `golden/environment/host_environment.json`
- `golden/environment/io_boundary.json`
- `golden/golden.lock.json`
- `golden/legacy/`
- `data/manifests/phase5_replay_core_v1.json`
- `data/manifests/small_test_area_03_v1.json`
- `data/manifests/source_bundle_v1.json`
- `configs/sensors/legacy_offline_72x5.yaml`
- `configs/benchmarks/lidar_reference.yaml`
- `golden/benchmarks/reference_host.json`と固定pose corpus
- `scripts/capture_source_baseline.py`
- `scripts/verify_source_unchanged.py`
- `scripts/verify_io_boundary.py`
- 要件差分表

#### 完了条件

- 3つの新rootのrealpathが`SOURCE_ROOT`配下でなく、sourceとGit object、
  symlink、hardlinkを共有していない。
- transaction journalが`committed`で終わり、同一UUIDのownership markerが3つの新rootで一致する。
- source baselineの全entry canonical manifest、Git manifest、branch、HEAD、index、tracked diff、statusが
  Phase 0開始前後で一致する。利用者による変更を含む不一致時は自動復元せずgateを失敗させる。
- 既存`gym_env`、Conda `base`、user/system packages、ROS install、既存ROS workspaceのmanifestが
  Phase 0開始前後で一致し、global install commandの実行が0件である。
- bootstrap legacy環境のprefix、cache、build、temporary、run outputがすべて`STATE_ROOT`配下にある。
- DG-08で選んだsandbox backendがhost package追加なしで動作し、allowlist外device accessと外部routeがない。
- raw dataと元地図のfile一覧、size、SHA-256が前後一致し、source側の追加fileが0件である。
- legacy pipelineが27/27 runを完了し、全runで`completed=1`、`reached_goal=1`、
  `map_collision=0`、`localizer_failure_rate=0`、`hardware_output_enabled=false`になる。
- legacyのtiming以外のsummaryとtrajectoryは、同一数値環境で`1e-9`、対応環境で`1e-6`
  の数値許容差内に入る。controller処理時間はgolden比較から除外する。
- replay結果が4合格・1件は既知理由付きN/Aになる。
- 再集計が827 rows、818 unique scans、重複9件、sensor payload conflict 0件、
  unique有限点215,201件を再現する。重複行の`elapsed_s`と`phase`の違いはmetadata差として
  記録する。sensor payload比較対象はbeam数、角度、range、`scan_time_s`、`ranges_json`とする。
- 実行metadataからsource revisionとchecksumを確認できる。
- rollback rehearsal後もsource/environment/rawの全不変条件が成立し、起動process、ROS node、
  DDS process、UDP listenerが残存しない。
- 環境保全gateの機械可読resultが`pass`である。

Phase 1からPhase 7も、それぞれの開始前後にactiveな承認済みbaselineで環境保全gateを実行する。
開始前にbaselineと
異なる場合、または終了後に許可境界外の変更がある場合は、そのPhaseを不合格として後続処理を
停止する。検出した差分は保存するが、自動で消去・復元しない。利用者が変更を維持する場合は
条件付き`ISO-008`で新baselineとADRを追加し、全履歴を保持してから再開する。

### 9.2 Phase 1: package骨格と共通契約

#### 目的

Phase 0の環境保全gateを通過した独立repository内で、センサ、車両、Controller、adapterが共有する
型と設定基盤を作る。

#### 作業

- `pyproject.toml`と`src` layoutを作る。
- Python 3.8/3.11の製品用lock/constraints、専用venv、core用固定rootfs/remote CI image、
  ROS Foxy用固定rootfs/remote jobを作る。Phase 0のbootstrap legacy環境は製品artifactへ流用しない。
- `ScanGeometry`、`ScanFrame`、`SensorProfile`を実装する。
- YAML読込、相対path解決、schema validationを実装する。
- seed管理と実行metadata生成を実装する。
- hardware出力を含まないことを検査するtestを追加する。
- input/output canonical pathを検証し、source、raw、assets、goldenへの書込みを拒否する。

#### 完了条件

- `${STATE_ROOT}/venvs/`のclean virtual environmentへinstallできる。
- Python 3.11はhostへ導入せず、固定rootfsまたはremote jobでtestできる。
- 現在のリポジトリを`PYTHONPATH`へ追加せずimportできる。
- core Python 3.11環境では`/opt/ros/foxy`をsourceせず、`rclpy`なしでcore testが成功する。
- 不正なbeam数、角度、range、TF、artifact pathを起動時に拒否する。
- unit testが成功する。
- Phase終了時の環境保全gateが成功する。

### 9.3 Phase 2: Level 1 LiDAR

#### 目的

決定論的な地図ray castingから、実機と幾何互換のscanを生成する。

#### 作業

- `small_test_area_03`をchecksum検証付きでcopyし、`PROJECT_ROOT`側だけを相対path化する。
- PGMの0、205、254をoccupied、unknown、freeとして明示的に扱う。
- vehicle poseからlaser poseへの2D extrinsic変換を実装する。
- occupancy grid ray casterを移植する。
- 360非重複角度を実装する。
- range外と地図外を`inf`へ変換する。
- DG-06で確定した後方60 binのindex集合を共通filterへ実装する。
- `configs/benchmarks/lidar_reference.yaml`と固定pose corpusを使うbenchmarkを追加する。

#### 完了条件

- `ranges_m`が常に360要素である。
- 最初の角度が`-pi`、最後が`pi - 1 deg`である。
- `+pi`方向の重複beamが存在しない。
- `angle_max = angle_min + 359 * angle_increment`が許容差`1e-6 rad`で一致する。
- no-hit、地図外、range外が12.0 mやNaNではなく正の`inf`になる。
- 後方mask契約の60 binが全scanで`inf`になり、rear filter自体は他のbinをmaskしない。
- laser originにx=0.25 mが反映される。
- 前・左・右の既知障害物でROS反時計回り正角と距離を検証する。
- 解析用地図でray距離誤差が`ray_step_m`以下かつ1 map cell以下になる。
- 地図が321 x 189 px、0.05 m/px、origin `[-4.76, -7.46, 0]`として読め、
  unknown比率55.86%の基準値と一致する。
- 同じposeとseedから同じscanが得られる。
- Phase 0で固定したreference host、map/profile、256 pose corpusでscan生成p99が
  0.132 s未満になる。

### 9.4 Phase 3: 車両・localizer・Controller統合

#### 目的

Level 1 LiDARを閉ループシミュレーションで利用できるようにする。

#### 作業

- Bicycle Modelと車両plant内だけで動く`ActuatorModel`を独立packageへ移植する。
- Pure Pursuit、MPC、MPPIを共通Controller APIへ移植する。
- OccupancyMapとGrid Map Localizerを移植する。
- vehicle 100 Hz、Controller 50/100 Hz、LiDAR周期0.132 sを独立schedulerで動かす。
- LiDAR eventはphysics stepへ丸めず、`t0 + k * period`で計算し、必要ならvehicle stateを
  event時刻へ補間する。
- ground truth、推定姿勢、capture/available時刻、source/availability age、制御入力を保存する。
- `ideal`条件のheadless runnerを作る。

#### 完了条件

- 3 Controllerが同じrunnerで選択できる。
- Controllerはscanの生成元を参照しない。
- LiDAR更新がないstepでは直前scanと運動予測を使用する。
- smoke runが衝突や例外なく完了する。
- 3 Controllerの出力schemaが一致する。
- nominal schedulerは`[0, 60 s)`で455 scan eventsを生成し、10,000 eventsで
  理想時刻からの最大位相誤差が`1e-9 s`以下になる。
- `configs/experiments/a1_smoke.yaml`で3 Controller x seeds `123, 456, 789`の9 runsを
  4秒実行し、9/9完了、例外0、map collision 0、非有限state 0、hardware output 0になる。

### 9.5 Phase 4: Level 2 実測分布モデル

#### 目的

固定遮蔽、環境依存欠測、周期揺らぎを分けて再現する。

#### データ前処理

- 5本の`scan.csv`をschema検査し、trial別のrows/uniqueを
  `196/194, 190/188, 55/54, 193/191, 193/191`として照合する。
- trial単位で元file順のkeep-firstを適用し、その後`stamp_s`順へ並べて818 unique scansを得る。
- 重複行のsensor payloadにconflictがなく、差が`elapsed_s`と`phase`だけであることを確認する。
- `null`を無効値へ変換する。
- 後方mask対象beamを統計対象から分離する。
- 非後方beamの角度別有効率を算出する。
- scan単位の非後方validity maskと`scan_time_s`を対応付けて抽出する。
- 全unique 818件の集計平均`0.1315143766 s`を正本とする。HTMLの`0.131523 s`は
  重複を含む827 rowsの参考値として併記する。
- `scan_time_s > 0.2 s`が完全scan CSV内で2件、値が`0.2585029900 s`と
  `0.2580739856 s`であることを確認する。これらを除く通常816件の平均は
  `0.1312036557 s`とする。
- 処理結果と元データchecksumをversion付きderived artifactへ保存する。

#### 生成方式

初期版では次の順で実装する。

1. 後方60 binを決定論的にmaskする。
2. 同じtrial・同じphase内の連続8 scansを1 blockとし、
   `(scan_time_s, 非後方validity mask)`を組でbootstrapする。
3. 二周期scanを除いた位置でsegmentを分割し、trial境界、phase境界、segment末尾を跨がない。
   末尾wrapは行わず、eligibleな全block開始位置から一様に選ぶ。
4. 二周期scanは通常modelへ混ぜず、明示的fault profileで注入する。

必要になった場合のみ、`ideal range`とbeam角度を条件にした欠測確率へ拡張する。

#### 注意

- `dropout_probability=0.269`の一様適用は禁止する。
- 実測73.1%には後方固定maskが含まれる。
- 非後方の有限率約87.7%もPhase 5環境限定の参考値とする。
- 周期は約0.123 s帯と0.135 s帯を含むため、単一Gaussianで近似しない。
- 測距真値がないため、保存距離の分散を距離精度として使わない。
- profileは`small_test_area_03`と保存5試行に紐づく環境限定モデルとする。
- train/validationはtrial単位で分け、scan行を無作為分割しない。
- 統計CIのseed集合は`20260701, 20260702, 20260703`、1 seed当たり10,000 scansに固定する。
- leave-one-trial-outのmetric、許容帯算出code、結果JSONをsampler実装前にcommitし、
  artifact hashをPhase 4の承認対象にする。全3 seedsが許容帯内に入ることを要求する。

#### 完了条件

- raw data regressionでunique scan数が818になる。
- 公開値の再現確認として、重複を含む827 rowsで平均有効beam 263.08、
  有限距離中央値1.758 mを許容差内で再現する。
- derived artifactへの入力は重複除去後818 scansとし、通常paired samplerの校正には
  二周期2件を除く816 samples、fault catalogには除外した2件を使う。
- raw data regressionで827 rows、重複9件、raw有限点217,569件、重複除去後
  818 scans・有限点215,201件を再現する。
- 通常profileの10,000 generated scansの平均周期が`0.1312036557 s`に対して
  ±0.003 s以内になる。
- fault profileが上記2件の周期catalogだけからsampleし、通常profileへ混入しない。
- raycast結果との積を取る前のgenerated validity maskで、全360 beamのvalid ratioが
  通常816件の`0.7307632081`に対して±0.02以内になる。最終scanのvalid ratioには
  固定値を要求しない。
- 後方mask適用率が100%になる。
- valid beam数の平均・分位点、角度別有限率、隣接mask Jaccard、欠測run lengthを
  trial leave-one-outで事前生成した`reference_stats`の許容範囲と比較する。
- seed固定で周期列と欠測mask列が再現する。
- 別seedでは周期・欠測系列の少なくとも一方が変化する。

### 9.6 Phase 5: Level 3 実ログ再生

#### 目的

仮想scanと同じ入力契約で実測scanを再生し、回帰試験へ利用する。

#### 作業

- `ranges_json`を`ScanFrame`へdecodeする。
- CSVの`null`を`inf`へ戻し、契約60 binは`rear_mask`、それ以外は
  `recorded_missing`として`invalid_reason`を設定する。
- trial内で元file順の同一stamp先頭行を残し、discard記録を作ってから`stamp_s`順へ並べる。
- odometryを`odom_stamp <= scan.available_time + 1e-9 s`を満たす最新sampleへ整列する。
- source clock、simulation clock、available timeを区別する。
- replay速度をstep、real-time、最大速度から選択できるようにする。
- localizer replayとController dry runへ同じsourceを接続する。
- 入力manifest checksumと設定checksumを結果へ保存する。
- ROS再発行時は古いepoch timestampを0始まりのsimulation clockへrebaseし、元stampを
  metadataに保持する。
- rosbag読込はCSV経路の完成後に追加する。

#### 完了条件

- 5試行を読める。
- 4試行が既存基準で合格する。
- `improved_50cm_trial2`が終端データ不足としてN/Aになる。
- 同じ入力から、処理時間列を除く同じreplay結果を得られる。
- 同一数値環境では姿勢・状態量が`1e-9`以内、異なる対応環境では`1e-6 m/rad`
  以内で一致する。
- 欠落topicを含むrosbagで明確な診断またはCSV fallbackが働く。

### 9.7 Phase 6: ROS 2 adapterと安全監視

#### 目的

coreを変更せず、実機graphと通信できない隔離環境で、実機と同じROS 2 message契約を検証する。

#### 作業

- `ScanFrame -> sensor_msgs/msg/LaserScan` adapterを実装する。
- `scan`、`odom`、`localization/pose`、TFを相対topic名で発行するnodeを実装し、通常実行では
  `/digital_twin/<run_id>`namespaceへ配置する。実機互換の絶対名は隔離contract testでremapする。
- `scan`はBest Effort、Volatile、Keep Last 5、`odom`はBest Effort、Volatile、
  Keep Last 10、poseはReliable、Volatile、Keep Last 10に固定する。
- `session_id`と`sequence_id`はnamespace内の`scan_status`へReliable、Transient Local、
  Keep Last 1で発行し、Safetyはsession変更をこのtopicから受け取る。
- launch preflightでnon-default `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=1`、run固有namespace、
  namespace外の既存node/publisher不在を検査し、条件不一致ではnodeを起動しない。
- `base_link -> laser`へx=0.25 m、z=0.11 mを設定する。
- clock modeを`simulation`または`wall`で設定化する。acceptance replayは
  `use_sim_time=true`で`/clock`を発行し、wall modeはlive生成のsmoke testだけに使う。
- UDP protocol v1のencoder/decoderとlocalhost loopback nodeを実装する。通常testはephemeral portを使い、
  非loopbackのbind先・送信先を設定validationで拒否する。
- stale scan、二周期scan、停止、outageのfault injectionを追加する。
- 現行状態機械へ`DEGRADED`を追加し、availability outage ageに基づくdegradedと
  `FAULT(scan_timeout)`を分ける。
- Safetyとactuatorのwatchdogをともに50 Hzで実行する。
- Controller/Safety graphの出力は`SimulatedCommandSink`とlogだけへ接続し、実I/O backendを
  package、entry point、launchから除外する。

#### 安全状態契約

| 条件 | 状態 | 動作 |
| --- | --- | --- |
| `RUNNING`かつavailability outage age <= 0.20 s | `RUNNING` | 通常制御 |
| 0.20 s < availability outage age <= 0.30 s | `DEGRADED` | speed 0かつsteering neutral |
| `DEGRADED`で正常受理が2件未満 | `DEGRADED` | neutralを維持 |
| `DEGRADED`で正常受理が2件連続 | `RUNNING` | 通常制御へ復帰 |
| availability outage age > 0.30 s | `FAULT(scan_timeout)` | neutralを維持 |
| source/header age > 0.30 s、future > 0.05 s | `FAULT` | scanを拒否してneutral |
| session変更・不正geometry・NaN | `FAULT` | scanを拒否してneutral |

比較演算は表のとおり0.20 sと0.30 sを正常側に含め、超過時に遷移する。`RUNNING`から
`DEGRADED`へ入った後は、契約を満たすscanを2件連続受理すると`RUNNING`へ戻る。
`FAULT`からはscan復帰だけで戻さず、`reset -> DISARMED -> arm -> READY -> start -> RUNNING`
を要求する。operator stopによる既存`STOPPED`は別状態として残す。

Safety watchdogは50 Hzであるため、最後に受理したscanから`> 0.30 s`を検出してneutralを
出す最大時刻を0.32 sとする。actuator watchdogはauthorized command age `> 0.10 s`で
neutralへ移し、50 Hz tickを含む最大遅延を最後のcommandから0.12 sとする。

ここで保証する「安全」はsoftwareがneutral出力へ遷移するまでに限定する。実車の制動時間と
停止距離は未測定のため、このデジタルツインだけで物理停止距離を保証しない。

#### 完了条件

- `/scan`のheader stampが`capture_time_s`、publish時刻が`available_time_s`になり、frame、
  geometry、range、`scan_time`、`time_increment=scan_time/N`が
  core契約と一致し、publisher/subscriberのQoS互換性がある。
- replay acceptanceでは全nodeが同じ`/clock`を使用し、wall clock参照を検出すると失敗する。
- 60秒bagで各topicのpublisher送信数を記録し、`/scan`、50 Hzの`/odom`、50 Hzのposeを
  それぞれ99%以上bagへ記録する。
- scan stamp時点で`base_link -> laser`を取得でき、TFとray casting originが一致する。
- TF authorityの重複とextrapolation errorが0件になる。
- UDP loopbackで120 beams x 3 packetsから1 scanを復元し、60秒正常試験の
  invalid、incomplete、gapが0件になる。
- IPv6、`0.0.0.0`、LAN IP、broadcast、multicastを指定すると起動前に失敗し、socket監査で
  loopback外へのdatagramが0件になる。実機port 5010が使用中でもephemeral testが成功する。
- CRC不一致、矛盾する重複chunk、欠落chunk、timeoutを受入側が拒否する。
- scanなし、stale、future、重複stamp、不正geometry、NaN、破損scanではarmできず、
  不正scanはwatchdogを更新しない。
- 1 scan欠落相当の約0.264 sでは`DEGRADED`へ移るが`FAULT`にならず、2 scan以上の欠落では
  最後の受理scanから0.32 s以内に`FAULT`へ移る。
- `FAULT`遷移と同じwatchdog callbackでneutralを発行し、以後の非neutral出力が0件になる。
- scan復帰だけでは再始動せず、reset、arm、startを要求する。
- Safety node停止時もactuator側watchdogが最後のcommandから0.12 s以内にneutralへ移す。
- wheel、entry point、launchを走査し、`pigpio`、`RPi.GPIO`、serial、sysfs GPIO、`/dev/tty*`、
  UDP actuator bridge、実アクチュエータ出力経路が存在しない。
- ROS jobがnon-root、host physical device passthroughなし、host networkなしで成功し、neutral commandは
  `SimulatedCommandSink`とrun log以外へ到達しない。
- domain 0のdecoy publisherを受信せず、namespace外publisherがある場合はpreflightが拒否する。
- 起動後のnode、publisher、subscriber、service、action監査で、明示allowlist以外の
  namespace外entityが0件になる。
- test終了後に起動node、DDS process、publisher、UDP listenerが0件になり、広域`pkill`を使わない。
- Phase終了時のsource/environment/raw保全gateが成功する。

### 9.8 Phase 7: acceptance、CI、release

#### 目的

元環境を変更せず、clean-roomで再現可能かつrollback可能なrelease candidateを完成させる。
ここでいうpre-release DoDは16章の全項目からrollback drill固有の項目だけを除いた集合とし、
除外項目はREL-002、その後の最終判定はREL-003で満たす。

#### 作業

- unit、integration、statistical、acceptance testをCIへ登録する。
- `core-cleanroom`、`fixture-readonly`、`ros-foxy-isolated`、
  `full-artifact-nightly`を別jobにする。
- dependency取得jobとtest jobを分け、test jobはfresh HOMEを使う。`PROJECT_ROOT`のclean checkoutと
  検証済みartifactをread-only mount、出力先を専用mountとし、外部network無効で実行する。
- core Python 3.8/3.11とROS 2 Foxy jobを分離し、rootfs/image checksum、Python、NumPy、
  ROS distro、RMW実装をmetadataへ保存する。
- 小さなcurated fixtureを通常CIで使用する。
- 完全5試行replayをmanualまたはnightly jobにする。
- benchmark結果へCPU、OS、Python、NumPy、電源設定を記録し、cold runとwarm runを分ける。
- 最低10,000 scansでp50、p95、p99、maxを保存し、10分real-time soak testを行う。
- scan単体と`scan -> localizer` pipelineを別benchmarkとして測る。
- architecture、data dictionary、実行手順、制限事項を文書化する。
- version付きgolden結果を生成する。
- 新packageの`legacy_offline_72x5`で27 runsを再実行し、Phase 0 goldenと比較する。
- feature branch tipを`CANDIDATE_SHA`として先に記録し、そのSHAのread-only clean checkoutで
  rollback項目を除くpre-release acceptanceと新packageのlegacy 27 runsを実行する。
- 成功した`CANDIDATE_SHA`を`RC_SHA`とannotated tag `rc/v1.0.0-rc.1`で固定し、tag object、source tree、
  artifact manifestをrelease証跡へ保存する。以後の検証中にfeature branchが進んでも参照しない。
- bootstrap `main`の固定SHAから作ったdisposableなrehearsal branchへ固定`RC_SHA`をmergeし、
  同じacceptanceとlegacy 27 runsを再実行してからmerge commitを`git revert`するrollback drillを行う。
  `git reset`は使わず、revert後のtree hashをbootstrap SHAと比較し、隔離した元source baseline runnerを
  再実行する。
- release判定資料へsource/environment/rawの前後manifest、I/O監査、残存process/socket確認を含める。
- 独立repositoryの`main`へのmergeはPhase 8へ分離する。`SOURCE_ROOT`へはmergeしない。

#### 完了条件

- 元`f1tenth_gym`、既存`gym_env`、`~/.ros`、local research dataをmountしないclean cloneから
  install、unit test、smoke testを実行できる。
- 新package上のlegacy 27 runsがPhase 0 goldenの非timing値と許容差内で一致する。
- 完全artifactを取得した環境でLevel 3 acceptanceが成功する。
- sourceに旧PCの絶対pathがない。
- repositoryをread-only mountしてtestでき、bag、ROS log、report、temporary fileを含む全出力が
  許可されたrun root内だけに存在する。
- 同じconfig、seed、artifact checksumで結果を再現できる。
- 専用reference hostでsingle process、single thread、事前に管理者が固定したCPU governorを記録し、
  100 warm-up scans後に
  固定256 poseを循環して10,000 scansを測り、scan単体p99が0.132 s未満になる。
  推奨目標はp99 0.066 s未満とする。
- 同じcorpusの`scan -> localizer`性能を別に記録し、10分実行のqueue backlog、deadline miss、
  想定外Safety faultが0件になる。
- 実測とシミュレーションの結果がreport上で区別される。
- 既知の未検証事項がREADMEへ明記される。
- `RC_SHA`のread-only clean checkoutとmerge済みrehearsal branchの新package legacy結果が
  Phase 0 goldenと一致する。
- rollback drillでrevert後のtree hashがbootstrap `main`と一致し、隔離source baseline runnerが
  Phase 0 goldenを再現する。
- source、既存Python/ROS環境、raw、mapの最終manifestがactiveな承認済みbaselineと一致し、
  baseline version間の全承認diffとADRを追跡できる。
- test終了後に起動process、ROS node、DDS process、UDP listenerが残存しない。

### 9.9 Phase 8: 統合承認と公開

#### 目的

完成済みrelease candidateを、明示承認後に独立repositoryの`main`へ統合する。本Phaseは公開gateで
あり、Phase 7までの初期版完成判定には含めない。

#### 作業

- Phase 7のtest、golden、data、I/O監査、rollback証跡をDG-02で確認する。
- bootstrap `main`の期待SHA、RC tag object、`RC_SHA`を再照合し、固定`RC_SHA`を`--no-ff`でmergeする。
- merge後のsmoke、legacy golden、environment保全gateを再実行する。
- release tagとartifact checksumを発行する。

#### 完了条件

- 明示承認の記録があり、独立repositoryの`main`だけが更新される。
- `REL-001`で固定した`RC_SHA`だけがmergeされ、merge commit、親SHA、RC tag object、release tag、
  artifact checksumを追跡できる。
- merge後testとenvironment保全gateが成功し、`SOURCE_ROOT`の変更が0件である。
- 問題発生時はmerge commitを`git revert`できる手順と検証済み証跡がある。

## 10. テスト戦略

### 10.1 Environment preservation test

- Phase開始前後でsource全entryのcanonical manifest、Git manifest、branch、HEAD、index、tracked diff、
  statusをactiveな承認済みbaselineと比較する。
- 既存`gym_env`、Conda `base`、system/user packages、ROS install、既存ROS build成果物を比較する。
- raw、map、参照文書のfile一覧、size、SHA-256を比較し、追加・削除・変更が0件であることを確認する。
- 3つの新rootがsource外の独立pathで、symlink、hardlink、Git objectを共有しないことを
  確認する。
- `PROJECT_ROOT`のclean checkoutと検証済みartifactをread-only mountしてtestし、`SOURCE_ROOT`は
  mountしない。指定run root以外へのwriteをI/O監査で検出する。
- raw、source、assets、golden、それらを指すsymlinkをoutputに指定するとapplicationが拒否する。
- environment保全testの不一致時はfail closedにし、sourceを自動復元しない。
- ISO-001を各作成stepで中断し、同一transaction UUIDでは再開でき、UUID不一致では停止し、
  所有対象外をcleanupしないことを確認する。
- version付きbaseline再承認後も過去version、canonical diff、承認ADRが残り、最終gateがactive versionを
  一意に選べることを確認する。
- rollback rehearsal後もactive baselineが一致し、所有対象外をcleanupしていないことを確認する。

### 10.2 Unit test

| 対象 | 検査内容 |
| --- | --- |
| generic geometry | `N=beam_count`、shape、導出angle max |
| A1 geometry | 360、1度、`-pi`から`pi-1 deg`、非重複 |
| legacy geometry | 72、`2*pi/71`、`-pi`と`+pi`の両端込み |
| range gate | 0.15未満、12超過、NaN、inf |
| invalid reason | reason enum、valid maskとの整合、legacy変換 |
| rear mask | 契約化した60 indexだけが常時`inf` |
| extrinsics | x/y/yaw変換、zのmetadata保持 |
| scheduler | 60秒455 events、10,000-event位相誤差、empirical、二周期fault |
| clock | capture/available同一domain、ROS stamp、rebase、future tolerance |
| seed | 同一seedで同一系列 |
| CSV decoder | `null -> inf`、beam数不一致拒否 |
| manifest | checksum不一致拒否 |
| UDP codec | endian、header、CRC32、chunk順、重複、timeout |

### 10.3 Integration test

- OccupancyMapからLevel 1 scanを生成する。
- vehicle schedulerとLiDAR schedulerを独立に進める。
- scanなしstepでlocalizerを予測だけ進める。
- 3 Controllerを同じrunnerへ接続する。
- recorded sourceをlocalizerとControllerへ接続する。
- ROS adapterでrange、角度、時刻、TFを比較する。
- ROS topic別QoSと、60秒bagのpublisher数に対する99%以上の記録率を確認する。
- UDP localhost loopbackでcore `ScanFrame`と復元scanを一致比較する。
- ROS domain、namespace、device、socket、output pathのpreflight失敗系を確認する。

### 10.4 Statistical test

- 818 unique scansの再集計値を固定する。
- 通常816 scansと二周期fault 2 scansを分けて周期統計を確認する。
- generated valid beam数の平均・分位点を確認する。
- 後方maskと追加欠測を別々に集計する。
- 角度別有限率のMAE、隣接scan maskのJaccard、欠測run lengthを確認する。
- trial leave-one-outで固定した基準帯に対して複数seedを確認する。

統計testは単一sampleの完全一致ではなく、固定sample数と許容範囲で判定する。

### 10.5 Safety test

- 公称周期0.132 sの正常scanでは`RUNNING`を維持する。
- 約0.258 sのavailability gapで`DEGRADED`へ移る。
- 0.30 s超のgapを50 Hz watchdogが0.32 s以内に検出し`FAULT`へ移る。
- geometry不一致、NaN、sequence逆行、古い・future timestampを拒否する。
- `FAULT`からの復帰にreset、arm、startを要求する。
- `FAULT`以降の非neutral actuator出力が0件であることを確認する。
- Safety processを停止し、actuator watchdogが0.12 s以内にneutralへ移ることを確認する。
- neutralが`SimulatedCommandSink`とrun log以外へ出力されないことを確認する。

### 10.6 Acceptance test

```text
environment gate -> isolated install -> unit -> smoke -> Level 1 contract
                 -> Level 2 statistics -> Level 3 replay -> ROS contract
                 -> report -> rollback drill -> final environment gate
```

処理時間はマシン依存なのでgolden数値へ含めず、環境metadataとbudget超過だけを検査する。
途中または最終のenvironment gateが失敗したrelease candidateは、機能testが成功していても不合格とする。

## 11. データ管理計画

### 11.1 正本データ

CSVをLevel 3の正本とする。Phase 5 replay coreは5試行 x 4 filesの20 files、
5,136,583 bytesである。portableな診断集合は5試行 x 7 filesの35 files、
5,946,284 bytesとする。machine固有pathだけを持つ5本の`rosbag_path.txt`、601 bytesは
配布artifactから除外する。

`SOURCE_ROOT`内のoriginalは読み取りだけに使う。artifactは`STATE_ROOT/artifacts/staging`へ展開し、
manifest検証後に`STATE_ROOT/artifacts/read_only/<artifact_id>`へcopyしてread-only mountまたは
file mode `0444`、directory mode `0555`で処理する。generatorとreplayはinput tree内へcache、lock、
temporary file、`__pycache__`を作らず、入力自身、その子directory、またはsymlink経由で入力treeへ
戻るpathをoutputに指定できない。

replay core:

- `scan.csv`
- `odom.csv`
- `pose.csv`
- `summary.json`

portable診断追加分:

- `encoder_status.csv`
- `lidar_status.csv`
- `localization_status.csv`

対象試行:

- `guarded_50cm_verification1`
- `improved_50cm_trial1`
- `improved_50cm_trial2`
- `improved_50cm_trial3`
- `improved_50cm_trial4`

trial別の`scan.csv` rows/unique scansは順に`196/194`、`190/188`、`55/54`、
`193/191`、`193/191`とする。raw fileは改行を含めbyte-identicalで保存し、CRLFを
正規化しない。curated fixtureで正規化する場合はrawとcuratedのhashを別々に記録する。

### 11.2 manifest

`data/manifests/phase5_replay_core_v1.json`には次を保存する。

```text
dataset_version
source_repository_revision
trial_name
relative_path
byte_size
sha256
row_count
unique_scan_count
schema_version
license_or_usage_note
source_document_sha256
deduplication_policy
line_ending
timestamp_semantics
archive_uri
archive_byte_size
archive_sha256
extract_root
```

manifest生成後に再読込し、全fileの存在、byte size、SHA-256、schema、row数を検証する。
raw dataは手修正せず、変換は常に`${STATE_ROOT}/runs/<run_id>/`の新規directoryへ出力する。
処理前後のraw manifestを比較し、atimeを除くfile集合、size、content hashが一致することを検査する。

### 11.3 Git管理

- branch、commit、tag、LFS、worktreeなどのGit操作は独立した`PROJECT_ROOT/.git`だけを対象にする。
  `SOURCE_ROOT/.git`には設定変更を含むwrite操作を行わない。
- 小さなcurated fixtureはGitへ含める。
- 完全CSVは容量と公開可否を確認してGit LFSまたは外部artifactへ置く。
- rosbagとslam_toolbox stateは外部artifactにする。
- download後は必ずSHA-256を検証する。
- clean checkoutでfixtureを復元でき、完全artifact取得後は20 core filesを再構成できることを
  acceptance testで確認する。
- raw dataがない場合、Level 1とversion管理済みderived artifactを使うLevel 2の小規模testは
  動作し、Level 3は明確にskip理由を出す。
- 通常testからgoldenを更新できないようread-onlyで開く。golden更新は理由、before/after、
  source checksumを要求する独立commandとし、機能変更と同時に暗黙更新しない。

### 11.4 派生成果物

`data/derived/`にはrawを直接コピーせず、`STATE_ROOT`で生成・検証後に明示的に採用した
再生成可能な統計とsampling blockだけを置く。

- artifact schema versionとgenerator version
- source manifestのSHA-256
- 対象trial、rows、unique scans、有限点数
- 後方固定maskと非後方validityの分離情報
- trial境界を保持した`scan_time_s`とvalidity maskの対応block
- 二周期scanを除く通常modelと、明示的fault catalog
- block長、trial/phase/segment境界、eligible block一覧、seed集合
- leave-one-trial-outで固定した統計許容範囲

各metricの許容上限は、1 trialをheld-out、残り4 trialをreferenceとした5 foldの距離
`d_loo`の最大値を1.10倍した値とし、全値0の場合だけ`1e-12`をfloorにする。period分布には
Wasserstein distance、角度別有限率にはMAE、valid count、隣接mask Jaccard、欠測run lengthの
各1次元経験分布にはWasserstein distanceを使う。この規則と算出結果をmodel実装前に固定する。

NPZ container自体のbyte一致には依存しない。array key順、shape、dtype、little-endian、C-orderを
固定し、各arrayの正規化byte列のSHA-256とcanonical JSON metadataのSHA-256を正本にする。
同じraw、manifest、generator versionから同じsemantic hashを生成するtestを追加する。

### 11.5 地図

- `data/manifests/small_test_area_03_v1.json`へYAML、PGM、metadataのchecksumを保存する。
- checksum検証付きcopyを`PROJECT_ROOT`へ置き、copy側YAMLの`image`だけを相対pathへ変更する。
- PGM値205の扱いをunknownとしてtestで固定する。
- unknownをray castingでoccupied扱いにするかはprofileへ明示する。
- 元地図と移植地図のchecksumを別々に記録し、元地図の前後checksum一致を検査する。
- 321 x 189 px、0.05 m/px、origin `[-4.76, -7.46, 0]`を検査する。
- 画素分類の基準比率occupied 2.65%、free 41.49%、unknown 55.86%を許容差付きで照合する。

## 12. 依存関係と実行環境

### 12.1 Host保全境界

| 環境 | 用途 | 変更可否 |
| --- | --- | --- |
| 現在のConda `base` / Python 3.13.9 | baseline記録のみ | 変更禁止、実装実行に使わない |
| `SOURCE_ROOT/gym_env` / Python 3.8.10 | legacy依存の記録元 | activate、install、uninstall禁止 |
| system/user Python | interpreter存在確認 | package変更禁止 |
| host ROS 2 Foxy | interface参照 | workspace source/build、package変更、node起動禁止 |
| `${STATE_ROOT}/venvs/core-py38` | core test | 新規作成・削除可 |
| `${STATE_ROOT}/venvs/core-py311` | core test | 新規作成・削除可 |
| local `bwrap` + pinned rootfs | core/legacy sandbox | non-root、外部network・host deviceなし |
| pinned remote/OCI core job | Python 3.11 clean-room | hostへruntimeを導入しない |
| pinned ROS Foxy rootfs/remote job | ROS acceptance | non-root、host device/networkなし |

既存環境のpackageを不足dependencyの供給元にしない。必要dependencyが隔離環境へ導入できない場合は
実装を止め、既存環境へ代替installしない。

### 12.2 core

- Python 3.8互換のsyntaxを維持する。
- Python 3.8と3.11をCI対象にする。
- NumPy、SciPy、PyYAML、Pillowを直接依存にする。
- legacy `gym==0.19.0`へ依存しない。
- ROS 2とplot出力はoptional install extraにする。ただしROS 2 adapterの実装と専用CIは
  初期releaseの必須条件とする。GUIは初期release対象外とする。
- venv、pip cache、wheel cache、build、test outputは`STATE_ROOT`へ置き、user siteを無効にする。
- core jobでROS環境変数、`/opt/ros/foxy`、PyPI版`rclpy`へ依存しない。

### 12.3 ROS 2

- ROS 2 Foxy adapterはchecksum/digest固定したUbuntu 20.04、system Python 3.8の
  `bwrap` rootfsまたは専用remote jobで検証する。
- core testとROS workspace testを分ける。
- coreデータ型をROS messageへ直接依存させない。
- ROS jobはFoxyのapt packageとclean colcon overlayを使用し、PyPI版`rclpy`へ依存しない。
- colconの`build/install/log`、`ROS_HOME`、bag、temporary fileを`STATE_ROOT`へ置く。
- job開始時に`ROS_DISTRO=foxy`、Python 3.8、`sys.executable`、RMW、non-default domain、
  localhost-only、host device passthroughなしをassertする。

### 12.4 Phase依存関係

```text
P0 -> P1 -> P2 -> P3
P0 + P1 + P2  -> P4
P1 + P3       -> P5
P2 + P3 + P4 + P5 -> P6
P0 ... P6     -> P7
P7 + explicit approval -> P8
```

Phase 5のdecoder実装はPhase 1後に並行着手できるが、localizer replayの受入にはPhase 3を
必要とする。Phase 4の統計artifact生成はPhase 0後に始められるが、生成modelのcore統合には
Phase 1を必要とする。Phase 0の環境保全gateだけは並行省略できず、すべての後続Phaseに優先する。
Phase 8はPhase 7で完成したrelease candidateの公開だけを行い、明示承認なしでは開始しない。

## 13. マイルストーンと概算

単独の実装担当者を想定した概算であり、実機再試験は含まない。

| マイルストーン | 内容 | 目安 |
| --- | --- | ---: |
| M0 | 環境隔離、非破壊guard、rollback、golden、manifest | 2から4日 |
| M1 | package骨格と共通契約 | 1から2日 |
| M2 | Level 1 LiDAR | 2から4日 |
| M3 | 車両・localizer・Controller統合 | 3から5日 |
| M4 | Level 2実測分布 | 3から5日 |
| M5 | Level 3 replay | 2から4日 |
| M6 | ROS 2・UDP・Safety | 3から5日 |
| M7 | CI、acceptance、文書、release candidate | 2から3日 |
| M8 | 独立repositoryの統合承認・公開 | 承認後0.5から1日 |

全体目安は18から32営業日とする。既存コードの移植範囲とデータ公開方法によって変動する。
これはM7までのrelease candidate完成までの見積りである。M8は明示承認後に別途実施する。
M0が完了するまではM1以降へ着手しない。

## 14. リスクと対策

| リスク | 影響 | 対策 |
| --- | --- | --- |
| 元repositoryや共有`.git`を変更する | 現行実験と利用者作業の破損 | 独立repository、write guard、前後manifest、変更検出時停止 |
| Conda、既存venv、system packageが競合する | 現行Python/ROS環境の破損 | `STATE_ROOT`専用venvとpinned sandbox、global command禁止 |
| hostにDocker/Python 3.11がない | clean-room jobをlocal実行不能 | verified `bwrap` rootfsまたは専用remote runner、host導入禁止 |
| 実機ROS graphとtopicが衝突する | 実機nodeの誤動作 | non-default domain、localhost-only、run namespace、preflight |
| UDPが外部interfaceへ送信される | 実機・LANへの意図しない通信 | loopback validation、ephemeral port、network namespace、socket監査 |
| raw、map、goldenを上書きする | 再現根拠の喪失 | read-only mount、出力path拒否、前後checksum、更新command分離 |
| cleanupが利用者fileを削除する | データ消失 | `STATE_ROOT`、ownership marker、run UUID、realpath検証に限定 |
| 利用者の並行変更を自動復元する | 利用者作業の消失 | 差分検出時は停止・保存し、自動reset/revertしない |
| 3系統のLiDAR実装が再び乖離する | 実機とsimの契約不一致 | core ScanFrameを単一正本にする |
| 72から360 beamsで処理が重くなる | real-time budget超過 | benchmarkし、必要ならsphere tracingを採用 |
| 実測欠測を一般化しすぎる | 誤ったセンサ性能主張 | 環境限定profileとして版管理 |
| raw dataがgit-ignoreされている | clean環境でLevel 3不能 | manifest、artifact、curated fixtureを用意 |
| rosbagのtopicがQoSで欠落している | 完全replay不能 | CSVを正本、bagを補助入力にする |
| 地図YAMLが絶対path | 他PCで読込不能 | 相対path化とclean clone test |
| PGM 205をfree扱いする | ray castとlocalizerが地図を誤解 | unknown policyを明示してtest |
| `+pi`重複を既存testが要求する | 新geometryへの移行阻害 | legacyとnew profileを分離 |
| 一様dropoutで二重計上する | 有効率が実測より低下 | 後方maskと非後方欠測を別model化 |
| source時刻と受信時刻が混在する | Safety ageとreplayが不正 | clock domainとage種別を型で分離 |
| rolling scan歪みが未実装 | 旋回時の再現性不足 | 初期版の制限に明記し将来Phaseへ分離 |
| Sensitivity modeが取得側で固定されない | 再収録条件が変わる | 将来実験前にPi Agentのmode選択を検証 |
| 0.132 sと7.6 Hzを別設定にする | schedulerが仕様からずれる | 周期を正本にしてrateを導出 |
| 後方±30度の端点解釈が変わる | 59/60/61 binで実装が分岐 | 離散index集合をDG-06とtestで固定 |
| raw CSVの改行が変換される | checksumと再現性が失われる | rawはbyte-identical、派生物は別path |
| UDP packetのendianやchunk処理が乖離する | 実機bridgeとの互換性喪失 | protocol fixtureとloopback testを固定 |

## 15. 判断ゲート

| ID | 判断内容 | 期限 | 暫定方針 |
| --- | --- | --- | --- |
| DG-00 | source、project、stateの非破壊境界を承認するか | Phase 0最初 | 本書1.1と6章を採用 |
| DG-01 | 完全CSVをGit LFSか外部artifactのどちらで配布するか | Phase 0 | 外部artifact + checksum |
| DG-02 | release candidateの統合先とmerge可否 | Phase 8開始前 | 独立repositoryのみ、明示承認までmergeしない |
| DG-03 | unknown cellをrayでoccupied扱いにするか | Phase 2開始時 | occupied扱い |
| DG-04 | `DEGRADED -> FAULT`状態契約の承認 | Phase 1終了時 | 本書9.7の契約を採用 |
| DG-05 | rolling scanを初期版へ含めるか | Phase 4終了時 | 含めない |
| DG-06 | 後方±30度の離散端点と60 bin集合 | Phase 2開始前 | observed `0..30, 331..359` |
| DG-07 | block bootstrapとLOO許容帯の承認 | sampler実装前 | 8 scans、境界wrapなし、本書11.4の算式 |
| DG-08 | local/remote isolation backendの固定 | ISO-005開始前 | verified `bwrap` + checksum rootfs、remote fallback |

本計画書はPhase 0でchecksum固定し、その後は`SOURCE_ROOT`側を変更しない。判断変更は、理由、日付、
影響するtest、profile version、承認者を`PROJECT_ROOT/docs/decisions/DG-<ID>.md`へADRとして追記する。

## 16. Definition of Done

次をすべて満たした時点で初期版完成とする。

- `SOURCE_ROOT`全entryのcanonical manifest、Git manifest、branch、HEAD、index、tracked diff、statusが
  activeな承認済みbaselineと一致し、全baseline versionと承認ADRを追跡できる。
- 本計画書のPhase 0 checksumが固定され、以後の判断変更が`PROJECT_ROOT`のADRだけに記録されている。
- 既存`gym_env`、Conda `base`、system/user packages、ROS install、既存ROS workspaceが変更されていない。
- raw、map、参照文書のfile集合、size、SHA-256が前後一致し、source側の追加・削除fileが0件である。
- 全build、cache、temporary、ROS log、bag、report、run outputが許可した`STATE_ROOT`内だけにある。
- `PROJECT_ROOT`のclean checkoutと検証済みartifactをread-only mountし、`SOURCE_ROOT`を
  mountしないclean-roomで全処理が成功する。
- 専用venv、rootfs、remote job artifactと`STATE_ROOT`を除去するだけで新しい実行環境を撤去でき、
  host packageの復元操作を必要としない。
- rollback drillのmerge前後で新packageのlegacy結果がgoldenと一致し、revert後のtree hashが
  bootstrap `main`と一致し、隔離source baseline runnerもgoldenを再現する。
- 起動したprocess、ROS node、DDS process、publisher、UDP listenerが残存しない。
- clean環境へinstallできる。
- 現在の`f1tenth_gym`を実行時に参照しない。
- generic `N=beam_count`契約、A1の360非重複契約、legacyの72-beam契約がtestで固定されている。
- 新packageのlegacy 27 runsがPhase 0 goldenと一致する。
- 取付TF、後方60-bin index契約、range、無効値が実機契約と一致する。
- Level 1、Level 2、Level 3をprofileで切り替えられる。
- 実測分布の出典、環境、raw checksum、dedup規則、generator versionを追跡できる。
- 3 Controllerが共通runnerで動く。
- nominal schedulerのevent数と位相誤差が受入値内にある。
- 保存5試行を再生できる。
- replay結果が4合格・1件N/Aになる。
- 二周期scanとtimeoutの安全遷移testが成功する。
- UDP protocol fixtureと60秒loopback testが成功する。
- ROS message、clock、QoS、TF、bag記録率の受入testが成功する。
- unit、integration、statistical、acceptance testが成功する。
- hardware出力backend、entry point、launch、device access経路が存在しない。
- ROSはnon-default domain、localhost-only、run namespace、host device passthroughなしで受入testに成功する。
- UDPはloopback以外のbindと送信を拒否し、外部datagramが0件である。
- reportが実測、仮想、未検証事項を区別する。
- README、architecture、data dictionaryが完成している。

## 17. 最初の実装バックログ

| ID | タスク | 依存 | 完了条件 |
| --- | --- | --- | --- |
| ISO-001 | bootstrap transactionとbaseline固定 | DG-00 | journal、全source/Git/environment canonical manifest成功 |
| ISO-002 | 独立`PROJECT_ROOT`/`STATE_ROOT`作成 | ISO-001 | source外、非共有Git、bootstrap tag、ownership検査成功 |
| ISO-003 | write/output boundary guard実装 | ISO-002 | source/raw/golden/path escape拒否test成功 |
| ISO-005 | isolation backend検証 | ISO-003, DG-08 | `bwrap`またはremoteでnon-root、device/network分離成功 |
| ISO-004 | bootstrap legacy sandbox固定 | ISO-003, ISO-005 | state内rootfs/env、host package前後一致 |
| ISO-006 | read-only source/data bundle生成 | ISO-003 | copy前後source hash一致、raw read-only化 |
| DT-001 | 対象5試行と地図のmanifest生成 | ISO-006 | byte size、row数、checksumが記録される |
| DT-002 | legacy goldenの固定 | ISO-004, ISO-006 | 隔離実行で27 runと4/1 replayを保存 |
| ISO-007 | 環境保全gateとrollback rehearsal | ISO-003, ISO-004, ISO-005, DT-001, DT-002 | 全baseline一致、残存process/socket 0 |
| ISO-008 | version付きbaseline再承認（条件付き） | ISO-007 | 明示承認、canonical diff、ADR、新active version固定 |
| DT-003 | package skeleton作成 | ISO-007 | isolated clean install成功 |
| ENV-001 | 製品用Python環境とlock固定 | DT-003 | py38/py311 test、host package不変 |
| ENV-002 | 製品用core/ROS sandbox固定 | DT-003, ISO-005 | rootfs/image checksum、Foxy smoke成功 |
| DT-004 | `ScanGeometry`実装 | ENV-001 | generic N、A1 360、legacy 72 test成功 |
| DT-005 | `ScanFrame`実装 | DT-004 | invariant test成功 |
| DT-006 | SensorProfile loader実装 | DT-004 | A1/legacy YAML validation成功 |
| PG-001 | Phase 1完了gate | ISO-007, ENV-001, ENV-002, DT-003, DT-005, DT-006 | Phase 1条件と環境保全gate成功 |
| DT-007 | 地図asset相対path化 | PG-001, DT-001 | clean環境でload成功 |
| DT-008 | unknown cell policy修正 | DT-007 | 205 sentinel test成功 |
| DT-009 | 360 beam ray caster実装 | PG-001, DT-005, DT-008 | Level 1 scan test成功 |
| DT-010 | rear mask共通化 | PG-001, DT-005, DG-06 | exact 60-index test成功 |
| DT-011 | extrinsic適用 | PG-001, DT-005 | x=0.25の幾何test成功 |
| DT-012 | Level 1 benchmark | DT-009, DT-010, DT-011 | p99がscan budget内 |
| PG-002 | Phase 2完了gate | DT-008, DT-009, DT-010, DT-011, DT-012 | Phase 2条件と環境保全gate成功 |
| DT-013 | Bicycle/Actuator Model移植 | PG-002 | model unit test成功 |
| DT-014 | Controller APIと3 Controller移植 | DT-013 | 共通schema test成功 |
| DT-015 | Grid Map Localizer移植 | PG-002, DT-009 | fixed-scan test成功 |
| DT-016 | event schedulerとrunner実装 | PG-002, DT-006, DT-009, DT-010, DT-011, DT-013, DT-014, DT-015 | 9-run smoke成功 |
| PG-003 | Phase 3完了gate | DT-013, DT-014, DT-015, DT-016 | Phase 3条件と環境保全gate成功 |
| DT-017 | Phase 5 empirical artifact生成 | ISO-007, DT-001, DG-07 | 818 scan集計とsemantic hash一致 |
| DT-018 | empirical scheduler実装 | PG-002, DT-017 | 通常/fault周期test成功 |
| DT-019 | empirical validity実装 | PG-002, DT-010, DT-017 | LOO統計test成功 |
| PG-004 | Phase 4完了gate | DT-018, DT-019 | Phase 4条件と環境保全gate成功 |
| DT-020 | CSV replay source実装 | PG-001, DT-005, DT-001 | 5試行decode成功 |
| DT-021 | localizer replay接続 | PG-003, DT-015, DT-020 | 4合格・1 N/A |
| PG-005 | Phase 5完了gate | DT-020, DT-021 | Phase 5条件と環境保全gate成功 |
| DT-022 | ROS LaserScan adapter | PG-003, PG-004, PG-005, DT-005, DT-016 | message/clock/QoS test成功 |
| DT-023 | ROS TFとscan status実装 | DT-011, DT-022 | TF/session契約成功 |
| DT-024 | Safety状態機械と2 watchdog実装 | DT-023, DG-04 | DEGRADED/FAULT/neutral test成功 |
| DT-025 | UDP protocol loopback | PG-004, PG-005, DT-005 | 3 packets復元と異常系test成功 |
| DT-026 | ROS/Safety acceptance | ENV-002, PG-004, PG-005, DT-022, DT-024, DT-025 | 隔離60秒bag、fault、I/O監査成功 |
| PG-006 | Phase 6完了gate | DT-026 | Phase 6条件と環境保全gate成功 |
| DT-027 | Legacy compatibility facade | PG-006, DT-002, DT-005, DT-007, DT-016 | 27-run golden比較成功 |
| REL-001 | clean-room pre-release acceptance | ISO-007, PG-004, PG-005, PG-006, DT-027 | rollback drillを除くpre-release acceptance/DoD成功、`RC_SHA`/RC tag固定 |
| REL-002 | merge/revert rollback drill | REL-001 | merge前後legacy一致、revert tree/source baseline一致 |
| REL-003 | 最終Definition of Done gate | REL-002 | 全DoDとrelease証跡が成功 |
| REL-004 | 独立repository `main`統合 | REL-003, DG-02 | 固定`RC_SHA`統合とPhase 8全完了条件成功 |

最初の着手範囲は`ISO-001`から`ISO-007`、`DT-001`、`DT-002`だけとする。環境隔離、
データ正本、legacy基準、rollbackを先に固定し、`ISO-007`が成功するまで`DT-003`以降の
機能実装へ進まない。
