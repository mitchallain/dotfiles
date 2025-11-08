#!/bin/bash

# Replicate the PS1 prompt format from bashrc with model name added
# Format: username@laptop shortened-path (git-branch) [model]

# Function from bashrc to shorten directory paths
_dir_chomp () {
    local IFS=/ c=1 n d
    local p=(${1/#$HOME/\~}) r=${p[*]}
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

# Extract short model name from ANTHROPIC_MODEL env var
model_short=""
if [ -n "$ANTHROPIC_MODEL" ]; then
    # Extract the model name, e.g., "claude-sonnet-4-5" from the full identifier
    model_short=$(echo "$ANTHROPIC_MODEL" | sed -n 's/.*\.\(claude-[^:]*\).*/\1/p')
    # Simplify further: remove version numbers and "anthropic."
    model_short=$(echo "$model_short" | sed 's/-[0-9]\{8\}.*//; s/anthropic\.//; s/claude-sonnet/sonnet/; s/claude-haiku/haiku/; s/claude-opus/opus/')
fi

# Build the status line
echo -n "$USER@laptop $(_dir_chomp "$(pwd)" 20)$(parse_git_branch)"
if [ -n "$model_short" ]; then
    echo -n " [$model_short]"
fi
