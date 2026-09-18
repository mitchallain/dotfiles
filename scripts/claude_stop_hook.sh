#!/usr/bin/env bash

# Source helper functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/claude_hook_helpers.sh"

# Validate stdin and get JSON input
if ! validate_stdin_json; then
  exit 1
fi

# Initialize debug log
debug_log="/tmp/claude_hook_stop.log"
init_debug_log "$debug_log"

# Get the path to the conversation transcript
transcript_path=$(echo "$input" | jq -r '.transcript_path')
log_debug "$debug_log" "Transcript path: $transcript_path"

# Extract summary from transcript if available
if [ -f "$transcript_path" ]; then
  # Get the content of the very first user prompt in the session
  # Transcripts are JSONL, find the first line with "type":"user"
  user_line=$(grep -m 1 '"type":"user"' "$transcript_path")

  # Extract the prompt content
  summary=$(echo "$user_line" | jq -r '.message.content' 2>/dev/null | cut -c 1-100)
  log_debug "$debug_log" "Extracted summary: $summary"
else
  log_debug "$debug_log" "Transcript file not found: $transcript_path"
fi

# Fallback if transcript missing or summary extraction failed
if [ -z "$summary" ] || [ "$summary" = "null" ]; then
  summary="Session complete"
  log_debug "$debug_log" "Using fallback summary"
fi

# Get tmux pane info if running in tmux
tmux_info=$(get_tmux_info)
if [ -n "$tmux_info" ]; then
  log_debug "$debug_log" "Tmux info: $tmux_info"
fi

# Send the notification for normal completion
notification_title="ClaudeCode Finished"
notification_message="Prompt starting with '$summary...' is complete."

if [ -n "$tmux_info" ]; then
  notification_message="$notification_message ($tmux_info)"
fi

icon=$(get_notification_icon "success")
notify-send "$notification_title" "$notification_message" --icon="$icon" --expire-time="$NOTIFICATION_EXPIRE_TIME"

exit 0
