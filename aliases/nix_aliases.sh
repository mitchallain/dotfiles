#!/bin/bash

export DEBUGINFOD_URLS="http://127.0.0.1:1949"

# NOTE: leaving this for posterity, but now using systemctl service
# defined in ./systemd/user/nixseparatedebuginfod.service
anix-start-debuginfod () {
    # check if nixseparatedebuginfod is running
    pid=$(pgrep nixseparatedebu)
    if [ -z "$pid" ]; then
        nixseparatedebuginfod >/dev/null 2>&1 &
        pid=$(pgrep nixseparatedebu)
        disown
        echo "nixseparatedebuginfod started with PID $pid"
    else
        echo "nixseparatedebuginfod is already running with PID $pid"
    fi
}

anix-log-debuginfod () {
    journalctl -f --user-unit nixseparatedebuginfod.service
}

anix-get-store-path () {
    if [ -t 0 ] || [ $# -eq 0 ]; then
        echo "usage: 'echo \$manyPaths | anix-get-store-path <name of input>'"
        return
    fi
    read -r log
    echo "$log" | grep -oP "\S*$1\S*"
}

# pipe a bunch of nix store paths to this with the name of input you care about
# and this will run nix log on that store path
# e.g., echo $buildInputs | anix-get-log fusionator
anix-get-log() {
    store_path=$(anix-get-store-path "$1")

    # if its a valid store path it will begin with "/nix/store"
    if [[ "$store_path" == "/nix/store"* ]]; then
        nix log "$store_path"
    else
        echo "Error: invalid store path - must begin with /nix/store"
    fi
}


# check the build log of a particular buildInput by name
anix-build-input-log () {
    # if no args, then just print the buildInputs
    if [ -z "${buildInputs}" ]; then
        echo "error: buildInputs not set, are you in a nix dev shell?"
    elif [ $# -eq 0 ]; then
        echo "error: provide an input name as an argument"
        return
    # if buildInputs is set, then pipe it to anix-get-log
    else
        echo "$buildInputs" | anix-get-log "$1"
    fi
}


showPhaseFooter2 ()
{
    local phase="$1";
    local startTime="$2";
    local endTime="$3";
    local delta=$(( endTime - startTime ));
    local H=$((delta/3600));
    local M=$((delta%3600/60));
    local S=$((delta%60));
    echo -n "$phase completed in ";
    (( H > 0 )) && echo -n "$H hours ";
    (( M > 0 )) && echo -n "$M minutes ";
    echo "$S seconds"
}

# nixpkgs made me do this
runPhase2() {
    local status=0
    local curPhase="$*"
    if [[ "$curPhase" = unpackPhase && -n "${dontUnpack:-}" ]]; then return; fi
    if [[ "$curPhase" = patchPhase && -n "${dontPatch:-}" ]]; then return; fi
    if [[ "$curPhase" = configurePhase && -n "${dontConfigure:-}" ]]; then return; fi
    if [[ "$curPhase" = buildPhase && -n "${dontBuild:-}" ]]; then return; fi
    if [[ "$curPhase" = checkPhase && -z "${doCheck:-}" ]]; then return; fi
    if [[ "$curPhase" = installPhase && -n "${dontInstall:-}" ]]; then return; fi
    if [[ "$curPhase" = fixupPhase && -n "${dontFixup:-}" ]]; then return; fi
    if [[ "$curPhase" = installCheckPhase && -z "${doInstallCheck:-}" ]]; then return; fi
    if [[ "$curPhase" = distPhase && -z "${doDist:-}" ]]; then return; fi

    if [[ -n $NIX_LOG_FD ]]; then
        echo "@nix { \"action\": \"setPhase\", \"phase\": \"$curPhase\" }" >&"$NIX_LOG_FD"
    fi

    showPhaseHeader "$curPhase"
    dumpVars

    local startTime=$(date +"%s")

    # trap handler for phase run errors
    trap 'status=1; trap - ERR' ERR

    # Evaluate the variable named $curPhase if it exists, otherwise the
    # function named $curPhase.
    # eval uses a subshell, set errtrace to pass ERR trap handler
    eval "set -o errtrace; ${!curPhase:-$curPhase}"

    local endTime=$(date +"%s")

    showPhaseFooter2 "$curPhase" "$startTime" "$endTime"

    if [ "$curPhase" = unpackPhase ]; then
        # make sure we can cd into the directory
        [ -n "${sourceRoot:-}" ] && chmod +x "${sourceRoot}"

        cd "${sourceRoot:-.}"
    fi

    return $status
}

runPhases () {
    if [ -z "${IN_NIX_SHELL:-}" ]; then
        echo "error: IN_NIX_SHELL is unset, please run from a devshell"
    fi

    local status=0
    toplevel=$(git rev-parse --show-toplevel)
    for phase in "$@"; do
        # if arg doesn't end in -Phase, append it
        actphase=${phase}
        if [[ "$phase" != *Phase ]]; then
            actphase="${phase}Phase"
        fi

        # if previous failure, skip this phase
        if [ "$status" -ne 0 ]; then
            echo "Skipping '$actphase'..."
            continue
        fi

        # if build phase, step into $(git rev-parse --show-toplevel)/build
        if [ "$actphase" = "cleanPhase" ]; then
            echo "Entering $toplevel"
            cd "$toplevel" || (echo "Failed to enter $toplevel" && return 1)
            rm -rf build
            continue
        elif [ "$actphase" = "buildPhase" ] || [ "$actphase" = "checkPhase" ] \
            || [ "$actphase" = "installPhase" ] || [ "$actphase" = "fixupPhase" ]; then
            echo "Entering $toplevel/build"
            if [ "$actphase" = "installPhase" ] && [ -n "${prefix:-}" ]; then
                echo "Removing $prefix directory"
                rm -rf "$prefix"
            fi
            cd "$toplevel/build" || (echo "Failed to enter $toplevel/build" && return 1)
        else
            cd "$toplevel" || (echo "Failed to enter $toplevel" && return 1)
        fi

        # run phase and capture exit status
        runPhase2 "$actphase"
        status=$?

        if [ "$status" -ne 0 ]; then
            alert "'$actphase' failed!"
        fi
    done

    # if all successful, alert
    if [ "$status" -eq 0 ]; then
        alert "Nix phase(s) succeeded!"
    fi

    return $status

}

# useful in combination with nix-direnv
# Copied from "direnv hook bash" output:
_direnv_hook_enabled=1
_direnv_hook() {
    if [ $_direnv_hook_enabled == "1" ]; then
        eval "$(direnv export bash)"
    fi
};

direnv-stop() {
    pushd "$(pwd)" > /dev/null
    cd
    _direnv_hook_enabled=0
    eval "$(direnv export bash)"
    popd > /dev/null
}
direnv-start() {
    echo "direnv: enabling shell hook"
    _direnv_hook_enabled=1
}

if command -v fzf &> /dev/null; then
    anix-build-input-cd () {
        # if no args, then just print the buildInputs
        if [ -z "${buildInputs}" ] || [ -z "${propagatedBuildInputs}" ]; then
            echo "error: buildInputs not set, are you in a nix dev shell?"
        elif [ $# -ne 0 ]; then
            cd "$(echo "$buildInputs $propagatedBuildInputs" | anix-get-store-path "$1")" || return
        else
            cd "$(echo "$buildInputs $propagatedBuildInputs" | tr ' ' '\n' | fzf --preview 'ls -AFGhlp {}')"
        fi
    }

    anix-native-build-input-cd () {
        # if no args, then just print the buildInputs
        if [ -z "${nativeBuildInputs}" ]; then
            echo "error: nativeBuildInputs not set, are you in a nix dev shell?"
        elif [ $# -ne 0 ]; then
            cd "$(echo "$nativeBuildInputs" | anix-get-store-path "$1")" || return
        else
            cd "$(echo "$nativeBuildInputs" | tr ' ' '\n' | fzf --preview 'ls -AFGhlp {}')"
        fi
    }

    # try to print some helpful info about the store path
    anix-store-explore () {
        ls /nix/store/ | fzf --preview 'if [[ /nix/store/{} == *.drv ]]; then nix derivation show /nix/store/{}; else ls -AFGhlp /nix/store/{}/; fi' \
            | xargs -I {} echo "/nix/store/{}"
    }

    anix-store-cd () {
        cd "$(anix-store-explore)" || exit
    }
fi

anix-show-derivation () {
    storepath=$(anix-store-explore)
    # if ends in .drv then show it
    # else get the derivation using nix-store --query --deriver
    if [[ "$storepath" == *.drv ]]; then
        drvpath="$storepath"
    else
        echo "Given path is not a derivation, using nix-store --query --deriver to get drv path"
        drvpath=$(nix-store --query --deriver "$storepath")
    fi

    echo "nix derivation show $drvpath | jq -C | less"
    nix derivation show "$drvpath" | jq -C | less
}

anix-get-locked-ref () {
    echo "Getting locked ref for '$1' with 'nix flake metadata --json | jq '.locks.nodes.\"$1\"'"
    jq_filter=".locks.nodes.\"$1\""
    nix flake metadata --json . | jq -C "$jq_filter"
}
