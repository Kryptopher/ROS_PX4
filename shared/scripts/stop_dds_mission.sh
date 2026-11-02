#!/usr/bin/env bash
set -euo pipefail

SESSION="${DDS_SESSION:-px4_dds_mission}"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "No tmux session named $SESSION is running."
  exit 0
fi

if [[ -t 0 && "${1:-}" != "--yes" ]]; then
  echo "WARNING: this stops the Offboard stream immediately, without a Position mode handover." >&2
  echo "If the drone is flying in Offboard, PX4 triggers its Offboard-loss failsafe." >&2
  echo "In flight, use Shutdown / Save Logs in the mission UI instead." >&2
  read -r -p "Type yes if the drone is landed or the pilot has control: " answer
  if [[ "$answer" != "yes" ]]; then
    echo "Not stopping."
    exit 1
  fi
fi

echo "Stopping hardware DDS mission panes so logs can close..."

logger_pids="$(pgrep -f '/zed_px4_bridge/flight_logger( |$)' || true)"
pane_ids="$(tmux list-panes -t "$SESSION":0 -F '#{pane_id}' 2>/dev/null || true)"
for pane_id in $pane_ids; do
  tmux send-keys -t "$pane_id" C-c 2>/dev/null || true
done

if [[ -n "$logger_pids" ]]; then
  # The logger flushes encoder_angles.csv first, then copies the PX4 ULog and
  # generates metadata, plots, and the summary. Give that cleanup time to
  # finish instead of destroying its pane after a fixed two-second delay.
  for _ in $(seq 1 120); do
    logger_running=false
    for logger_pid in $logger_pids; do
      if kill -0 "$logger_pid" 2>/dev/null; then
        logger_running=true
        break
      fi
    done
    if [[ "$logger_running" == "false" ]]; then
      break
    fi
    sleep 1
  done

  for logger_pid in $logger_pids; do
    if kill -0 "$logger_pid" 2>/dev/null; then
      echo "Warning: flight logger did not finish within 120 seconds; closing tmux." >&2
      break
    fi
  done
else
  sleep 2
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true

echo
echo "Hardware DDS mission stopped. Logs are under ~/logs/."
latest_log="$(ls -td "$HOME"/logs/* 2>/dev/null | head -1 || true)"
if [[ -n "$latest_log" ]]; then
  echo "Newest log run: $latest_log"
  if [[ -f "$latest_log/encoder_angles.csv" ]]; then
    encoder_rows="$(( $(wc -l < "$latest_log/encoder_angles.csv") - 1 ))"
    echo "Encoder angle CSV: $latest_log/encoder_angles.csv ($encoder_rows samples)"
  fi
fi
