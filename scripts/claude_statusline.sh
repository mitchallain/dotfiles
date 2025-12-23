#!/bin/bash

# Custom statusline combining PS1 prompt format with token tracking
# Format: username@laptop shortened-path (git-branch) [model] | tokens | percentage%
# Uses actual API token usage from transcript files for accurate context tracking

# ============================================================================
# Context Budget Configuration
# ============================================================================
# Claude Code manages a 200K token context window with auto-compact protection.
# Auto-compact buffer is reserved space that:
#   1. Prevents performance degradation as context fills
#   2. Provides working space for the summarization/compaction operation
#   3. Triggers auto-compact before hitting hard limits
#
# When usage reaches THRESHOLD (155K), auto-compact activates to condense
# conversation history while preserving important information.
# ============================================================================

TOTAL_BUDGET=200000        # Total context window size
AUTO_COMPACT_BUFFER=45000  # Reserved space for compaction operation and safety margin
THRESHOLD=$((TOTAL_BUDGET - AUTO_COMPACT_BUFFER))  # Auto-compact triggers at 155K

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

# ============================================================================
# Token Calculation from Transcript
# ============================================================================
# Each API response in the transcript contains usage data with four fields:
#
#   1. cache_read_input_tokens: System context (MCP tools, CLAUDE.md, instructions)
#      These ARE in the context window, just cached for efficiency. Reused each turn.
#
#   2. cache_creation_input_tokens: New tokens added to cache when system context changes
#      This is an amortized cost metric, not a direct context usage measure.
#
#   3. input_tokens: Fresh user input for this turn (not previously cached)
#
#   4. output_tokens: Assistant's response for this turn
#
# Context window at any point = cached system context + all conversation history
#
# Calculation approach:
#   - Get most recent API call's cache_read (system context size)
#   - Sum all output_tokens (entire conversation history)
#   - Add most recent input_tokens (current user message)
#   - Total = cache_read + sum(outputs) + latest_input
# ============================================================================

TOTAL_TOKENS=0
TOKEN_DISPLAY=""
PERCENTAGE_DISPLAY=""

if [ -n "$input" ]; then
    SESSION_ID=$(echo "$input" | jq -r '.session_id' 2>/dev/null)
    if [ -n "$SESSION_ID" ] && [ "$SESSION_ID" != "null" ]; then
        TRANSCRIPT_PATH=$(find ~/.claude/projects -name "${SESSION_ID}.jsonl" 2>/dev/null | head -1)
        if [ -f "$TRANSCRIPT_PATH" ]; then
            # Calculate total tokens using actual API usage data
            TOTAL_TOKENS=$(cat "$TRANSCRIPT_PATH" | jq -s '
                # Extract all API responses with usage data
                map(select(.message.usage)) as $responses |

                # Get the most recent cached system context size
                ($responses | last | .message.usage.cache_read_input_tokens // 0) as $system_context |

                # Sum all conversation outputs (assistant responses across all turns)
                ($responses | map(.message.usage.output_tokens) | add // 0) as $conversation_history |

                # Get the latest user input
                ($responses | last | .message.usage.input_tokens // 0) as $latest_input |

                # Total context = system + conversation + current input
                $system_context + $conversation_history + $latest_input
            ' 2>/dev/null || echo 0)

            # ================================================================
            # Format Display
            # ================================================================
            # Calculate percentage relative to auto-compact threshold (155K)
            # When this reaches 100%, auto-compact will trigger
            # ================================================================

            PERCENTAGE=$((TOTAL_TOKENS * 100 / THRESHOLD))
            if [ $PERCENTAGE -gt 100 ]; then
                PERCENTAGE=100
            fi

            # Format current usage with K notation (e.g., "42.5K")
            if [ $TOTAL_TOKENS -ge 1000 ]; then
                CURRENT_DISPLAY=$(echo "scale=1; $TOTAL_TOKENS / 1000" | bc 2>/dev/null || echo "$((TOTAL_TOKENS / 1000))")"K"
            else
                CURRENT_DISPLAY="$TOTAL_TOKENS"
            fi

            # Format auto-compact threshold (155K)
            THRESHOLD_DISPLAY=$(echo "scale=0; $THRESHOLD / 1000" | bc 2>/dev/null || echo "$((THRESHOLD / 1000))")"K"

            # Format total budget (200K)
            BUDGET_DISPLAY=$(echo "scale=0; $TOTAL_BUDGET / 1000" | bc 2>/dev/null || echo "$((TOTAL_BUDGET / 1000))")"K"

            # Display format: Current/Threshold/Total | Percentage
            # Example: 42.5K/155K/200K | 27%
            TOKEN_DISPLAY="${CURRENT_DISPLAY}/${THRESHOLD_DISPLAY}/${BUDGET_DISPLAY}"
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
