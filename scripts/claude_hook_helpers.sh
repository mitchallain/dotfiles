#!/usr/bin/env bash

# Shared helper functions for Claude Code hook scripts

# Validate that stdin is available and contains valid JSON
# Returns: 0 on success, 1 on failure
# Sets global variable: input (the JSON content)
validate_stdin_json() {
  # Check if stdin is available and not a terminal
  if [ -t 0 ]; then
    echo "Error: This script expects JSON data from Claude Code via stdin"
    return 1
  fi

  # Read all of stdin into a variable safely
  input=$(cat)

  # Validate JSON input
  if ! echo "$input" | jq -e . >/dev/null 2>&1; then
    echo "Error: Invalid JSON input"
    return 1
  fi

  return 0
}

# Get tmux pane information if running in tmux
# Returns: tmux info string (empty if not in tmux or on error)
get_tmux_info() {
  if [ -n "$TMUX_PANE" ]; then
    tmux display-message -p -t "${TMUX_PANE}" '#{session_name}:#{window_index}:#{window_name}' 2>/dev/null
  fi
}

# Write a debug log entry
# Args: $1 = log file path, $2 = message
log_debug() {
  local log_file="$1"
  local message="$2"
  echo "$message" >> "$log_file"
}

# Initialize a debug log file with timestamp
# Args: $1 = log file path
init_debug_log() {
  local log_file="$1"
  echo "$(date) - Script running" > "$log_file"
}

# Get appropriate notification icon based on context
# Args: $1 = notification type ("info"|"warning"|"error"|"success"|default)
# Returns: icon name for notify-send
get_notification_icon() {
  local notification_type="$1"

  case "$notification_type" in
    "info")
      echo "dialog-information"
      ;;
    "warning")
      echo "dialog-warning"
      ;;
    "error")
      echo "dialog-error"
      ;;
    "success")
      echo "emblem-default"
      ;;
    *)
      echo "terminal"
      ;;
  esac
}

# Default notification expire time in milliseconds
# 10 seconds = 10000ms
NOTIFICATION_EXPIRE_TIME=10000
