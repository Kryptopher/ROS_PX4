#!/usr/bin/env bash
set -euo pipefail

SESSION="${DDS_SESSION:-px4_dds_mission}"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "No tmux session named $SESSION is running."
  exit 0
fi

echo "Stopping hardware DDS mission panes so logs can close..."

pane_ids="$(tmux list-panes -t "$SESSION":0 -F '#{pane_id}' 2>/dev/null || true)"
for pane_id in $pane_ids; do
  tmux send-keys -t "$pane_id" C-c 2>/dev/null || true
done

sleep 2
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo
echo "Hardware DDS mission stopped. Logs are under ~/logs/."
latest_log="$(ls -td "$HOME"/logs/* 2>/dev/null | head -1 || true)"
if [[ -n "$latest_log" ]]; then
  echo "Newest log run: $latest_log"
fi
