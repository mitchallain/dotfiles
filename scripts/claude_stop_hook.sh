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

# Check if transcript path exists
if [ ! -f "$transcript_path" ]; then
  log_debug "$debug_log" "Error: Transcript file not found: $transcript_path"
  icon=$(get_notification_icon "error")
  notify-send "ClaudeCode Error" "Transcript file not found" --icon="$icon" --expire-time="$NOTIFICATION_EXPIRE_TIME"
  exit 1
fi

# Get the content of the very first user prompt in the session
# Transcripts are JSONL, find the first line with "type":"user"
user_line=$(grep -m 1 '"type":"user"' "$transcript_path")

# Extract the prompt content
summary=$(echo "$user_line" | jq -r '.message.content' 2>/dev/null | cut -c 1-100)

# If summary extraction fails, provide a fallback
if [ -z "$summary" ] || [ "$summary" = "null" ]; then
  summary="Unknown prompt"
  log_debug "$debug_log" "Failed to extract summary from transcript"
else
  log_debug "$debug_log" "Successfully extracted summary: $summary"
fi

# Check if the session ended with a 429 error
# Look for either:
# 1. An assistant message with isApiErrorMessage=true containing "429"
# 2. A system api_error with status 429
# Exclude the stop_hook_summary line
last_lines=$(tail -n 10 "$transcript_path" | grep -v '"subtype":"stop_hook_summary"')

# Check for assistant API error message
is_429_error=$(echo "$last_lines" | tail -n 1 | jq -r 'select(.type=="assistant" and .isApiErrorMessage==true and (.message.content[0].text | contains("429"))) | "true"' 2>/dev/null)

# If not found, check for system api_error
if [ "$is_429_error" != "true" ]; then
  is_429_error=$(echo "$last_lines" | tail -n 1 | jq -r 'select(.type=="system" and .subtype=="api_error" and .error.status==429) | "true"' 2>/dev/null)
fi

if [ "$is_429_error" = "true" ]; then
  log_debug "$debug_log" "Session ended with 429 error - blocking stop and continuing"

  # Get tmux pane info for notification
  tmux_info=$(get_tmux_info)
  notification_message="Rate limit hit, automatically retrying..."
  if [ -n "$tmux_info" ]; then
    notification_message="$notification_message ($tmux_info)"
  fi

  # Send notification to user
  icon=$(get_notification_icon "warning")
  notify-send "ClaudeCode Rate Limited (429)" "$notification_message" --icon="$icon" --expire-time="$NOTIFICATION_EXPIRE_TIME"

  # Block the stop and tell Claude to continue
  # Output JSON to prevent Claude from stopping
  jq -n \
    --arg reason "Continue" \
    --arg sysmsg "🔄 Rate limit encountered (429). Automatically continuing..." \
    '{
      "decision": "block",
      "reason": $reason,
      "systemMessage": $sysmsg
    }'

  log_debug "$debug_log" "Sent block decision with Continue prompt"
  exit 0
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
