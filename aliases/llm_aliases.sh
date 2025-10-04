#!/usr/bin/env bash

markdown_prompt="Formatting re-enabled. You are a helpful assistant. Please use markdown formatting in your responses."
commando_prompt="Formatting re-enabled. You are a gritty, tactical commando, \
                who also knows everything about systems programming and \
                Unix-like systems. You understand the importance of writing \
                stable, reliable software for defense systems, in order to \
                maintain Western military superiority and deter conflicts. You \
                take your character very seriously and only respond in character. "
commando_prompt_o3="Respond as a gritty, tactical commando to this build process output."

# temporary hack to pipe ninja to runphase out

# if llm is executable
if [ -x "$(command -v llm)" ]; then
    if [ -x "$(command -v nix)" ] && [ -f ~/.aliases/nix_aliases.sh ]; then
        if [ -x "$(command -v glow)" ]; then
            splain () {
                llm -s "$markdown_prompt" "Please explain this error" < /tmp/ninja-out.txt | sglow
            }
        else
            splain () {
                llm "Please explain this error" < /tmp/ninja-out.txt
            }
        fi

        compiler_commando () {
            llm -s "$commando_prompt_o3" \
                < /tmp/nix_runphase_out.txt | sglow
        }

    fi

    sglow() {
        # local accumulator=""
        #
        # # Read from stdin, pass through to stdout, and accumulate
        # while IFS= read -r line; do
        #     echo "$line"
        #     accumulator+="$line"$'\n'
        # done < /dev/stdin
        #
        # # After input completes, pipe accumulated output to glow
        # # TODO: decide between TUI and pager (-t vs -p)
        # echo "$accumulator" | glow -t -

        local tmp
        tmp=$(mktemp) || { echo "Failed to create temp file"; return 1; }

        # Tee writes input to both stdout and the temp file
        tee "$tmp"

        glow -t - < "$tmp"

        rm "$tmp"
    }
fi

if [ -x "$(command -v claude)" ]; then
    clint () {
        claude -p "you are a linter. please look at the changes vs. \
        master and report any issues related to typos. report the filename and \
        line number on one line, and a description of the issue on the second \
        line. do not return any other text."
    }

    flaude () {
        claude -p "concisely explain the root cause of this build error" < /tmp/ninja-out.txt
    }
fi

alias finja="ninja |& tee /tmp/ninja-out.txt"


