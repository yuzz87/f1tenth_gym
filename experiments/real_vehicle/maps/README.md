# Real Vehicle Maps

実機RPLIDARとエンコーダ`/odom`から作成した占有格子地図を保存する。

各地図は同名のサブディレクトリに、少なくとも次を保存する。

```text
map_name/map_name.yaml
map_name/map_name.pgm
map_name/map_name.posegraph
map_name/map_name.data
map_name/metadata.yaml
```

生成には`integration_ws/scripts/save_real_map.sh map_name`を使用する。
地図作成時の可動障害物、実験場所、開始位置など、地図ファイルから判断できない条件は
同じディレクトリへMarkdownで追記する。
