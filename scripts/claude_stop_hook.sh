#!/usr/bin/env bash

# Check if stdin is available and not a terminal
if [ -t 0 ]; then
  echo "Error: This script expects JSON data from Claude Code via stdin"
  exit 1
fi

# Read all of stdin into a variable safely
input=$(cat)

# Validate JSON input
if ! echo "$input" | jq -e . >/dev/null 2>&1; then
  echo "Error: Invalid JSON input"
  exit 1
fi

# Minimal debug log
debug_log="/tmp/claude_notify_debug.log"
echo "$(date) - Script running" > "$debug_log"

# Get the path to the conversation transcript
transcript_path=$(echo "$input" | jq -r '.transcript_path')
echo "Transcript path: $transcript_path" >> "$debug_log"

# Check if transcript path exists
if [ ! -f "$transcript_path" ]; then
  echo "Error: Transcript file not found: $transcript_path" >> "$debug_log"
  notify-send "ClaudeCode Error" "Transcript file not found" --icon=terminal
  exit 1
fi

# Get the content of the very first user prompt in the session
# Transcripts are JSONL, so we read the first line.
first_line=$(head -n 1 "$transcript_path")

# Extract the prompt content - message.content is a string, not an array
summary=$(echo "$first_line" | jq -r '.message.content' 2>/dev/null | cut -c 1-100)

# If summary extraction fails, provide a fallback
if [ -z "$summary" ]; then
  summary="Unknown prompt"
  echo "Failed to extract summary from transcript" >> "$debug_log"
else
  echo "Successfully extracted summary" >> "$debug_log"
fi

# Send the notification
notify-send "ClaudeCode Finished" "Prompt starting with '$summary...' is complete." --icon=terminal
