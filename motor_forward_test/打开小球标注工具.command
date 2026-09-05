#!/bin/zsh

cd "$(dirname "$0")"
exec .venv-xanylabeling/bin/xanylabeling \
  --filename dataset_source/ball_pipe_seed_annotation_pipe/raw/slow_left \
  --labels steel_ball \
  --autosave
