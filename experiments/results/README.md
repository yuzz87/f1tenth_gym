# Experiment Results Policy

`experiments/results/` is split into three roles:

- `raw/`: temporary outputs from one run, such as timestamped `csv`, per-run `svg`, and `_view.md`.
- `runs/`: grouped experiment batches when one study needs multiple related runs.
- `reports/`: curated comparison tables and representative figures that should be reviewed and can be committed.

## Push policy

Commit these:

- files under `experiments/results/reports/`
- small summary markdown that captures conclusions
- representative figures used in discussion or reports

Do not commit these:

- timestamped run `csv`
- auto-generated per-run `_view.md`
- bulk per-run `svg`
- temporary comparisons that can be regenerated

## Naming

- batch folder: `YYYY-MM-DD_topic`
- report file: `topic_summary.md`, `topic_comparison.md`
- representative figure: `topic_summary.svg`

## Practical rule

If a file is reproducible from code and config, keep it local.
If a file is the conclusion you want to show later, put it in `reports/`.
