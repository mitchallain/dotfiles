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
    journalctl -fu --user nixseparatedebuginfod.service
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

anix-build-input-cd () {
    # if no args, then just print the buildInputs
    if [ -z "${buildInputs}" ]; then
        echo "error: buildInputs not set, are you in a nix dev shell?"
    elif [ $# -eq 0 ]; then
        echo "error: provide an input name as an argument"
        return
    # if buildInputs is set, then pipe it to anix-get-store-path
    else
        cd "$(echo "$buildInputs" | anix-get-store-path "$1")" || return
    fi
}

anix-native-build-input-cd () {
    # if no args, then just print the buildInputs
    if [ -z "${nativeBuildInputs}" ]; then
        echo "error: nativeBuildInputs not set, are you in a nix dev shell?"
    elif [ $# -eq 0 ]; then
        echo "error: provide an input name as an argument"
        return
    # if set, then pipe it to anix-get-store-path
    else
        cd "$(echo "$nativeBuildInputs" | anix-get-store-path "$1")" || return
    fi
}

# show the shell script for a phase by first checking for the bash variable
# and then for a bash function, similar to how nixpkgs runPhase works
# e.g., eval '${!configurePhase:-$configurePhase}'
showPhase () {
    command=$1
    if [ -n "${!1}" ]; then
        echo "$1=\"${!1}\""
        command=${!1}
    fi

    # if command is a bash function, then
    if [ -n "$(type -a "$command" 2>/dev/null)" ]; then
        type -a "$command"
    else
        echo "$1 is not a bash variable or bash function"
    fi
}

# TODO: this doesn't stop if a phase fails, and runPhase doesn't give useful
# exit codes
runPhases () {
    # if arg doesn't end in Phase, sub it
    for phase in "$@"; do
        actphase=${phase}
        if [[ "$phase" != *Phase ]]; then
            actphase="${phase}Phase"
        fi
        runPhase "$actphase"
    done
    alert "Nix phase(s) completed!"

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



