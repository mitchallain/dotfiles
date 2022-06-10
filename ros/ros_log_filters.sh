#!/bin/bash
# ros_log_filters.sh
#
# various bash functions for filtering ros log files


# roslognotopics   : remove the '[topics: /rosout, /scan, ...]' string from
#                  : rosout.log files using sed
# -----------------:------------------------------------------------------------
roslognotopics () {
    sed -E 's/\[topics: ((\/\w+)+(, |\]))+//g'
}

# roslogshortpath  : shorten absolute path component from rosout.log files using
#                  : sed
# -----------------:------------------------------------------------------------
roslogshortpath () {
    sed -E 's/(\/\S+)+\/(\S+(\.cpp|\.py))/\2/g'
}

# roslogcolor      : uses perl to add ansi escaped colors to lines containing
#                  : log severities
#                  : https://automationrhapsody.com/coloured-log-files-linux
# -----------------:------------------------------------------------------------
roslogcolor () {
    perl -pe 's/^.*FATAL.*$/\e[1;37m$&\e[0m/g; s/^.*ERROR.*$/\e[1;31m$&\e[0m/g; s/^.*WARN.*$/\e[0;33m$&\e[0m/g; s/^.*INFO.*$/\e[0;36m$&\e[0m/g; s/^.*DEBUG.*$/\e[0;37m$&\e[0m/g'
}


# roslogomit       : shorten absolute path component from rosout.log files using
#                  : sed
# -----------------:------------------------------------------------------------
roslogomit () {
    grep -v "$1" --color=always
}

# cgrep           : grep color
# -----------------:------------------------------------------------------------
cgrep () {
    grep --color=always "$@"
}

# ncgrep           : grep no color
# -----------------:------------------------------------------------------------
ncgrep () {
    grep --color=never "$@"
}

roslogtimekey () {
    cat "$1" | grep --color=never "${@:2}" | roslognotopics | roslogshortpath \
        | roslogomit "sicknav.cpp" | roslogcolor | less -SNR
}
