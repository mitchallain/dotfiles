#!/bin/bash

# Custom statusline combining PS1 prompt format with token tracking
# Format: username@laptop shortened-path (git-branch) [model] | tokens | percentage%

# Configurable threshold (default: 160K tokens = 80% of 200K context)
THRESHOLD=${CLAUDE_AUTO_COMPACT_THRESHOLD:-160000}

# Read JSON input from stdin if available
if [ ! -t 0 ]; then
    input=$(cat)
else
    input=""
fi

# Function from bashrc to shorten directory paths
_dir_chomp () {
    local IFS=/ c=1 n d
    local p=(${1/#$HOME/\\~}) r=${p[*]}
    local s=${#r}
    while ((s>$2&&c<${#p[*]}-1))
    do
        d=${p[c]}
        n=1;[[ $d = .* ]]&&n=2
        ((s-=${#d}-n))
        p[c++]=${d:0:n}
    done
    echo "${p[*]}"
}

# Git branch parser from bashrc
parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
}

# Extract short model name from input JSON or ANTHROPIC_MODEL env var
model_short=""
if [ -n "$input" ]; then
    # Try to get model from JSON input
    MODEL_DISPLAY=$(echo "$input" | jq -r '.model.display_name' 2>/dev/null)
    if [ -n "$MODEL_DISPLAY" ] && [ "$MODEL_DISPLAY" != "null" ]; then
        model_short="$MODEL_DISPLAY"
    fi
fi

# Fallback to ANTHROPIC_MODEL env var if no JSON input
if [ -z "$model_short" ] && [ -n "$ANTHROPIC_MODEL" ]; then
    # Extract the model name, e.g., "claude-sonnet-4-5" from the full identifier
    model_short=$(echo "$ANTHROPIC_MODEL" | sed -n 's/.*\.\(claude-[^:]*\).*/\1/p')
fi

# Shorten model name regardless of source
if [ -n "$model_short" ]; then
    # Remove version numbers and "anthropic.", simplify model names
    model_short=$(echo "$model_short" | sed 's/-[0-9]\{8\}.*//; s/anthropic\.//; s/claude-sonnet/sonnet/; s/claude-haiku/haiku/; s/claude-opus/opus/; s/Claude Sonnet/sonnet/; s/Claude Haiku/haiku/; s/Claude Opus/opus/')
fi

# Calculate tokens from transcript
TOTAL_TOKENS=0
TOKEN_DISPLAY=""
PERCENTAGE_DISPLAY=""

if [ -n "$input" ]; then
    SESSION_ID=$(echo "$input" | jq -r '.session_id' 2>/dev/null)
    if [ -n "$SESSION_ID" ] && [ "$SESSION_ID" != "null" ]; then
        TRANSCRIPT_PATH=$(find ~/.claude/projects -name "${SESSION_ID}.jsonl" 2>/dev/null | head -1)
        if [ -f "$TRANSCRIPT_PATH" ]; then
            # Estimate tokens (rough approximation: 1 token per 4 characters)
            TOTAL_CHARS=$(wc -c < "$TRANSCRIPT_PATH" 2>/dev/null || echo 0)
            TOTAL_TOKENS=$((TOTAL_CHARS / 4))

            # Calculate percentage
            PERCENTAGE=$((TOTAL_TOKENS * 100 / THRESHOLD))
            if [ $PERCENTAGE -gt 100 ]; then
                PERCENTAGE=100
            fi

            # Format token count with K notation
            if [ $TOTAL_TOKENS -ge 1000 ]; then
                TOKEN_DISPLAY=$(echo "scale=1; $TOTAL_TOKENS / 1000" | bc 2>/dev/null || echo "$((TOTAL_TOKENS / 1000))")"K"
            else
                TOKEN_DISPLAY="$TOTAL_TOKENS"
            fi

            PERCENTAGE_DISPLAY="${PERCENTAGE}%"
        fi
    fi
fi

# Build the status line
echo -n "$USER@laptop $(_dir_chomp "$(pwd)" 20)$(parse_git_branch)"
if [ -n "$model_short" ]; then
    echo -n " [$model_short]"
fi

# Add token count and percentage if available
if [ -n "$TOKEN_DISPLAY" ]; then
    echo -n " | $TOKEN_DISPLAY | $PERCENTAGE_DISPLAY"
fi
