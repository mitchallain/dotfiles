#!/usr/bin/env bash

# Custom statusline combining PS1 prompt format with token tracking
# Format: username@laptop shortened-path (git-branch) [model] | tokens | percentage%
# Uses context_window fields provided by Claude Code

# Read JSON input from stdin if available
if [ ! -t 0 ]; then
    input=$(cat)
else
    input=""
fi

# Function from bashrc to shorten directory paths
_dir_chomp () {
    local IFS=/ c=1 n d
    local tilde='~'
    local p=(${1/#$HOME/$tilde}) r=${p[*]}
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
    model_short=$(echo "$model_short" | sed 's/-[0-9]\{8\}.*//; s/anthropic\.//; s/claude-sonnet/sonnet/; s/claude-haiku/haiku/; s/claude-opus/opus/; s/Claude Sonnet/sonnet/; s/Claude Haiku/haiku/; s/Claude Opus/opus/; s/ (1M context)/ 1M/')
fi

# ============================================================================
# Token Calculation using context_window fields
# ============================================================================
# Claude Code now provides context_window data directly:
#   - context_window.context_window_size: Total token capacity (e.g., 200000)
#   - context_window.used_percentage: Current usage percentage (e.g., 24)
# ============================================================================

TOKEN_DISPLAY=""
PERCENTAGE_DISPLAY=""

if [ -n "$input" ]; then
    # Extract context window data from JSON input
    WINDOW_SIZE=$(echo "$input" | jq -r '.context_window.context_window_size' 2>/dev/null)
    USED_PERCENTAGE=$(echo "$input" | jq -r '.context_window.used_percentage' 2>/dev/null)

    if [ -n "$WINDOW_SIZE" ] && [ "$WINDOW_SIZE" != "null" ] && [ -n "$USED_PERCENTAGE" ] && [ "$USED_PERCENTAGE" != "null" ]; then
        # Calculate actual token usage from percentage
        TOTAL_TOKENS=$((WINDOW_SIZE * USED_PERCENTAGE / 100))

        # Format current usage with K notation (e.g., "42.5K")
        if [ $TOTAL_TOKENS -ge 1000 ]; then
            CURRENT_DISPLAY=$(echo "scale=1; $TOTAL_TOKENS / 1000" | bc 2>/dev/null || echo "$((TOTAL_TOKENS / 1000))")"K"
        else
            CURRENT_DISPLAY="$TOTAL_TOKENS"
        fi

        # Format total budget (200K)
        BUDGET_DISPLAY=$(echo "scale=0; $WINDOW_SIZE / 1000" | bc 2>/dev/null || echo "$((WINDOW_SIZE / 1000))")"K"

        # Display format: Current/Total | Percentage
        # Example: 42.5K/200K | 24%
        TOKEN_DISPLAY="${CURRENT_DISPLAY}/${BUDGET_DISPLAY}"
        PERCENTAGE_DISPLAY="${USED_PERCENTAGE}%"
    fi
fi

# Nix shell indicator
NIX_INDICATOR=""
if [ -n "$IN_NIX_SHELL" ]; then
    NIX_INDICATOR=" ❄"
fi

# Build the status line
echo -n "$USER@laptop $(_dir_chomp "$(pwd)" 20)$(parse_git_branch)${NIX_INDICATOR}"
if [ -n "$model_short" ]; then
    echo -n " [$model_short]"
fi

# Add token count and percentage if available
if [ -n "$TOKEN_DISPLAY" ]; then
    echo -n " | $TOKEN_DISPLAY | $PERCENTAGE_DISPLAY"
fi
