#!/usr/bin/env bash

# Check if stdin is available and not a terminal
if [ -t 0 ]; then
  echo "Error: This script expects JSON data from Claude via stdin"
  exit 1
fi

# Read all of stdin into a variable safely
input=$(cat)

# Minimal debug log
debug_log="/tmp/claude_notifications_debug.log"
echo "$(date) - Notification received" > "$debug_log"
echo "$input" >> "$debug_log"

# Validate JSON input
if ! echo "$input" | jq -e . >/dev/null 2>&1; then
  echo "Error: Invalid JSON input" >> "$debug_log"
  exit 1
fi

# Extract notification details
notification_type=$(echo "$input" | jq -r '.type')
title=$(echo "$input" | jq -r '.title')
message=$(echo "$input" | jq -r '.message')
reference_id=$(echo "$input" | jq -r '.reference_id')

echo "Type: $notification_type" >> "$debug_log"
echo "Title: $title" >> "$debug_log"
echo "Message: $message" >> "$debug_log"
echo "Reference ID: $reference_id" >> "$debug_log"

# Customize the notification icon based on notification type
icon="terminal"
case "$notification_type" in
  "info")
    icon="dialog-information"
    ;;
  "warning")
    icon="dialog-warning"
    ;;
  "error")
    icon="dialog-error"
    ;;
  "success")
    icon="emblem-default"
    ;;
  *)
    icon="terminal"
    ;;
esac

# Send the notification
notify-send "$title" "$message" --icon="$icon"

# Exit with success
exit 0
