#!/usr/bin/env bash

# Source helper functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/claude_hook_helpers.sh"

# Validate stdin and get JSON input
if ! validate_stdin_json; then
  exit 1
fi

# Initialize debug log
debug_log="/tmp/claude_hook_notification.log"
init_debug_log "$debug_log"
log_debug "$debug_log" "Notification received"
log_debug "$debug_log" "$input"

# Extract notification details
notification_type=$(echo "$input" | jq -r '.type')
title=$(echo "$input" | jq -r '.title')
message=$(echo "$input" | jq -r '.message')
reference_id=$(echo "$input" | jq -r '.reference_id')

log_debug "$debug_log" "Type: $notification_type"
log_debug "$debug_log" "Title: $title"
log_debug "$debug_log" "Message: $message"
log_debug "$debug_log" "Reference ID: $reference_id"

# Get tmux pane info if running in tmux
tmux_info=$(get_tmux_info)
if [ -n "$tmux_info" ]; then
  message="$message ($tmux_info)"
  log_debug "$debug_log" "Tmux info: $tmux_info"
fi

# Get appropriate icon and send the notification
icon=$(get_notification_icon "$notification_type")
notify-send "$title" "$message" --icon="$icon" --expire-time="$NOTIFICATION_EXPIRE_TIME"

exit 0
