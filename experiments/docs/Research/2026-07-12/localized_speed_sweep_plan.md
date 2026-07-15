# Localized Speed Sweep Plan

## 目的

`localized + Pure Pursuit`, `localized + MPC`, `localized + MPPI` を同じ条件で比較し、速度を上げたときにどの controller がどこから崩れるかを確認する。

今の `1.0 m/s` 条件では 3 手法とも 1 周完了しており、差が小さい。したがって次に必要なのは、条件を少し厳しくして差が出る領域を探すことである。

## 対象

比較対象:

- `localized + Pure Pursuit`
- `localized + MPC`
- `localized + MPPI`

共通条件:

- map: `homur_oval`
- localizer: `map_localizer`
- 1 周評価: `--lap-target 1`

## 基本方針

- まずは速度だけを変える
- localizer 設定は固定する
- controller ごとの `target_speed` / `vgain` を整理して比較する
- 条件を一度に増やしすぎない

## 速度スイープの狙い

確認したいこと:

1. どの速度まで 1 周完了できるか
2. 速度を上げたときに localization 誤差がどう増えるか
3. tracking 誤差が controller ごとにどう増えるか
4. controller ごとの崩れ方がどう違うか

## 最初の速度候補

段階的に上げる。

- `1.0 m/s`
- `1.5 m/s`
- `2.0 m/s`
- `2.5 m/s`

必要ならその後:

- `3.0 m/s`

最初は 0.5 刻みで十分である。

## 変更するパラメータ

### Pure Pursuit

- `controller.vgain`

必要なら:

- `controller.tlad`

### MPC

- `controller.target_speed`

必要なら:

- `controller.horizon`
- `controller.position_weight`
- `controller.heading_weight`

### MPPI

- `controller.target_speed`

必要なら:

- `controller.horizon`
- `controller.num_samples`
- `controller.temperature`

## 実験の進め方

### フェーズ 1: 速度別 config を作る

候補:

- `homur_oval_localized_pure_pursuit_v1_0.yaml`
- `homur_oval_localized_pure_pursuit_v1_5.yaml`
- `homur_oval_localized_pure_pursuit_v2_0.yaml`
- `homur_oval_localized_mpc_v1_0.yaml`
- `homur_oval_localized_mpc_v1_5.yaml`
- `homur_oval_localized_mpc_v2_0.yaml`
- `homur_oval_localized_mppi_v1_0.yaml`
- `homur_oval_localized_mppi_v1_5.yaml`
- `homur_oval_localized_mppi_v2_0.yaml`

最初は手作業で増やしてよい。

### フェーズ 2: 各条件で 1 周 run を取る

実行例:

```bash
./gym_env/bin/python experiments/run_localized_controller_experiment.py \
  --config experiments/configs/<config_name>.yaml \
  --no-render \
  --lap-target 1 \
  --max-steps 5000
```

### フェーズ 3: 各 run のレポートを出す

```bash
./gym_env/bin/python experiments/plot_run_results.py \
  experiments/results/<log_file>.csv
```

### フェーズ 4: localized 比較レポートを更新する

```bash
./gym_env/bin/python experiments/compare_localized_runs.py
```

### フェーズ 5: 結果を速度ごとに整理する

必要なら速度ごとに別 Markdown を残す。

## 主に見る指標

### localization 側

- `mean_xy_error_m`
- `max_xy_error_m`
- `mean_theta_error_rad`
- `max_theta_error_rad`

### tracking 側

- `lap_count`
- `lap_time_s`
- `mean_cross_track_m`
- `max_cross_track_m`

### 補助

- `steps`
- `done`
- `lap_target_reached`

## 判定の観点

### 1. 完走限界

- どの controller が最も高い速度まで 1 周完了できるか

### 2. localization 劣化

- 速度が上がったとき、どの controller が localization 誤差を増やしやすいか

### 3. tracking 劣化

- 速度が上がったとき、どの controller が cross-track error を増やしにくいか

### 4. 実用性

- 同じ速度で走れたとして、計算時間が許容範囲か

## 注意点

- localizer が同じでも、controller が違うと走行軌跡が変わるため、localization 誤差も変わる
- controller の差と localization の差は完全には分離できない
- 最初は localizer 設定を固定し、controller 側だけで比較する
- 速度を上げすぎる前に 1.5 / 2.0 m/s で傾向を確認する

## 推奨する次の実務タスク

1. localized 3 手法の `1.5 m/s` config を作る
2. 3 条件を走らせる
3. localized 比較レポートを更新する
4. 問題なければ `2.0 m/s` へ進む

## 期待する出口

この速度スイープの出口は以下である。

- どの controller が localized 条件で安定か分かる
- どの速度域で差が出るか分かる
- 次に localizer 改善へ進むべきか、controller 調整へ進むべきか判断できる
