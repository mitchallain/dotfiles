#!/bin/bash
#  ---------------------------------------------------------------------------
#
#  Description:  This file holds default arguments, aliases, and custom functions
#
#  Sections:
#  1.  Environment Configuration
#  2.  Default Options and Useful Aliases
#  3.  ROS and Robotics Development
#  4.  Specialized Tools
#  4.  Searching
#  5.  Process Management
#  6.  Networking
#  7.  System Operations & Information
#  9.  Reminders and Notes
#
#  ---------------------------------------------------------------------------

#   ----------------------------------------
#   2. DEFAULT OPTIONS AND USEFUL ALIASES
#   ----------------------------------------

alias cp='cp -iv'                                   # prompt before overwrite (-i) and verbose (-v)
alias mv='mv -iv'                                   # prompt before overwrite (-i) and verbose (-v)
alias nt='nautilus'                                 # nautilus file browser shortcut
alias mkdir='mkdir -pv'                             # make parents (-p) and verbose (-v)

# show dotfiles (-A), classify (-F), no-group (-G), human readable sizes (-h)
# long listing (-l), append / to directories (-p), sort by mtime newest first (-t)
alias ll='ls -AFGhlp'
alias llt='ls -AFGhlpt'

# clear screen (-c), exit if only one screen (-F), raw control chars (-R)
# truncate long lines (-S), no termcap init (-X)
alias less='less -cFRSX'

alias cd..='cd ../'                                 # Go back 1 directory level (for fast typers)
alias ..='cd ../'                                   # Go back 1 directory level
alias ...='cd ../../'                               # Go back 2 directory levels
alias .3='cd ../../../'                             # Go back 3 directory levels
alias .4='cd ../../../../'                          # Go back 4 directory levels
alias .5='cd ../../../../../'                       # Go back 5 directory levels
alias .6='cd ../../../../../../'                    # Go back 6 directory levels
alias ~="cd ~"                                      # ~:            Go Home
alias c='clear'                                     # c:            Clear terminal display

# ch               : clear screen and tmux history if in tmux
# -----------------:-----------------------------------------------------------------
ch() {
  clear
  clear
  # if [ -z "$TMUX" ]; then
  #   tmux clear-history
  # fi
}

alias which='type -a'                  # which:        Find executables
alias path='echo -e ${PATH//:/\\n}'    # path:         Echo all executable Paths
alias fix_stty='stty sane'             # fix_stty:     Restore terminal settings when screwed up
mcd () { mkdir -p "$1" && cd "$1"; }   # mcd:          Makes new Dir and jumps inside
alias DT='tee /tmp/termout.txt'        # DT:           Pipe content to temporary file

alias xclipc="xclip -selection c"
alias xclipcr="tr -d '\n' | xclip -selection c"
alias xclipv="xclip -selection clipboard -o"

## Git aliases
# note git <alias> type commands can be defined in .gitconfig
alias gitl="git log"
alias gits="git status"


# Print a compact git status, log, and diff for the provided directory
# Args:
#     $1: directory to check
gitstat () {
    echo -e "\033[32m$(basename "$1") \033[33m($(cd "$1" && git branch --show-current))\033[00m"
    printf '%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
    echo -e "\033[34m$(cd "$1" && git log -1 --pretty="%h - %>(12)%ad - %B" | tr -d '\n' |\
        sed "s/\(.\{$((COLUMNS - 3))\}\).*/\1.../")\033[00m"
    (cd "$1" && git status -s)
    echo ""
}


# bd - back to parent directory
# https://github.com/vigneshwaranr/bd
if [ -x "$(command -v bd)" ]; then
    alias bd=". bd -si"
fi

## COMMAND ALTERNATIVES
## Replace some common commands with fancier alternatives from this century

# bat - better cat
# when installed via debian, executable is batcat
# must be linked with `ln -s /usr/bin/batcat ~/.local/bin/bat`
if [ -x "$(command -v bat)" ]; then
    alias cat="bat"
fi

# exa - better ls
if [ -x "$(command -v exa)" ]; then
    alias lx="exa -lga --icons -s name"
fi

## BASH FUNCTIONS
## --------------

# Moves all arguments to the Ubuntu trash
# Example     : 'trash file1 file2'
# Args:
#     $@: files to move to trash
trash () { command mv "$@" ~/.local/share/Trash ; }


# Open file(s) in the background using xdg-open
#
# Args:
#    $@: files to open
#
# Example: 'xopen file1 file2'
xopen() {
    for var in "$@"; do
        (xdg-open "$var" > /dev/null 2>&1 &)
    done
}


# Capture the last 10000 lines of the tmux pane and save to a file
#
# Args:
#    $1: tmux session
#    $2: tmux window
#    $3: tmux pane
#    $4: file to save to
#
# Example: 'savetmuxbuffer ROS bringup 0 /tmp/bringup.log'
savetmuxbuffer () {
    /usr/bin/tmux send-keys -t "$1":"$2"."$3" C-b ":capture-pane -e -S 10000" C-m
    /usr/bin/tmux send-keys -t "$1":"$2"."$3" C-b ":save-buffer $4" C-m
}


# List n most recently modified files in directory
# Args:
#    $1: number of files to list
lr () {
    if [ -z "$1" ]; then
        echo "Usage: lr <number of files>"
        return
    fi
    ll -t | head -n "$1"
}

# Search manpage given in arg 1 for term in arg 2 (case insensitive)
# Displays paginated & colored result with two lines per hit
# Args:
#     $1: manpage to search
#     $2: term to search for
#
# Example     : 'mans ls -l'
mans () {
    # pipe manpage to less, start less with search term
    man "$1" | less -R -p "$2"
}


# View a csv file in the console
# Args:
#    $1: csv file to view
#
# Example: 'colview test.csv'
colview () {
    column -n -s, -tn < "$1" | less -#2 -N -S
}

# Move file at arg 1 back to path arg 2, then git mv from 2 to 1
# Args:
#    $1: file to move
#    $2: destination path
#
# Example:
#   'mvgitmv /home/user/file1 /home/user/file2'
unmvgitmv () {
    mv "$1" "$2"
    git mv "$2" "$1"
}

# ----------------------------------------
# 3. ROS AND ROBOTICS DEVELOPMENT
# ----------------------------------------

# Preferred pylint config
alias pylintrc="pylint --rcfile=~/.pylintrc"

# #   ROS aliases
alias rosenv='printenv | grep -i ROS'
alias rosgdb="rosrun --prefix 'gdb -ex run --args' "
alias rosprofile="rosrun --prefix 'valgrind --tool=callgrind' "
alias rosmemcheck="rosrun --prefix 'valgrind --tool=memcheck' "

# Set the logger level for a ROS node
# Args:
#    $1: node name
#    $2: logger name
#    $3: log level
#
# Example: 'rosloglvl ultra_workspace_manager rospy debug'
rosloglvl () {
    rosservice call "$2"/set_logger_level "logger='ros.$1'"$'\r'"level='$3'"
}

# Reset any overlays by sourcing the default ROS environment, then source
# the workspace containing the current directory, if any.
sws () {
    # shellcheck source=/opt/ros/noetic/setup.bash
    source /opt/ros/"$ROS_DISTRO"/setup.bash;
    local wspath
    wspath="$(catkin locate 2>/dev/null)"

    # check if in a workspace and setup.bash exists
    if [ -n "$wspath" ] && [ -f "$wspath"/devel/setup.bash ]; then
        # shellcheck source=/home/mallain/catkin_ws/devel/setup.bash
        source "$wspath"/devel/setup.bash;
    fi
}

# Update 'rostopic list' every 1 second, or n seconds where n is arg 1
# Args:
#   $1: (optional: default 1) period in seconds
#
# Example: 'rostopicw 2'
rostopicw () {
    local period=${1:-1}
    watch -n "$period" "rostopic list"
}

# Update 'rosnode list' every 1 second, or n seconds where n is arg 1
# Args:
#  $1: (optional: default 1) period in seconds
#
# Example: 'rosnodew 2'
rosnodew () {
    local period=${1:-"1"}
    watch -n "$period" "rosnode list"
}

# Replace rosmaster hostname with arg 1, assume default port
# Args:
#     $1: hostname
#     $2: (optional, default '11311') port
#
# Example: 'rosmaster robot'
rosmaster () {
    local port=${2:-"11311"}
    export ROS_MASTER_URI=http://$1:$port
}

# Search package names and paths for pattern
# Args:
#    $1: pattern to search for
#
# Example: 'rosgrep sensor'
rosgrep () {
    rospack list | grep "$1";
}

# Republish a Quaternion as Vector3 with Euler angles in degrees on /eulerify/<topic>
# Args:
#     $1: Quaternion topic to republish, or quaternion field in a message
#     e.g. '/<topic>/pose/orientation'
#
# Example: 'roseulerify /pose/pose/orientation'
roseulerify () {
    rosrun topic_tools transform "$1" /eulerify/"$1" geometry_msgs/Vector3 \
        'map(lambda x: x * 57.296, tf.transformations.euler_from_quaternion([m.x, m.y, m.z, m.w]))' --import tf
}

# Parse ros wiki url from package.xml and open in chrome
# Args:
#    $1: package name
#
# Example: 'roswiki robot_localization'
roswiki () {
    local pkgxml
    pkgxml=$(rospack find "$1")/package.xml
    local url
    url=$(cat "$pkgxml" | grep http | cut -d ">" -f 2 | cut -d "<" -f 1)

    # open in chrome in the background, suppress output
    google-chrome "$url" > /dev/null 2>&1 &
}

# Build package name from first arg and execute tests, printing results
# Args:
#   $1: package name
catkin_test() {
    local wspath
    wspath=$(catkin locate)
    catkin build --no-deps --catkin-make-args run_tests -- "$1" > /dev/null \
        && catkin_test_results "$wspath"/build/"$1"
}

# Build current package and execute tests, printing results
catkin_test_this() {
    local pkgname
    pkgname=$(catkin list --this -u)
    catkin_test "$pkgname"
}

# Source containing workspace without clearing ros env, i.e. overlay ws
catkin_source() {
    local wspath
    wspath=$(catkin locate 2>/dev/null)
    if [ -n "$wspath" ] && [ -f "$wspath"/devel/setup.bash ]; then
        # shellcheck source=/home/mallain/catkin_ws/devel/setup.bash
        source "$wspath"/devel/setup.bash
    else
        echo "Not in a catkin workspace"
    fi
}

# compdb is a tool that manipulates compilation databases generated by CMake
if [[ -n $(command -v compdb) ]]; then
    # Collect all translation dbs from all packages in the workspace and link to src
    # This may require a clean build to ensure all translation dbs are generated
    # but that is left to the user to decide
    # Args:
    #     $@: additional arguments to pass to catkin build
    #
    # Example: 'catkin clean -y && wscompdb'
    link_compdb() {
        local wspath
        wspath=$(catkin locate 2>/dev/null)
        if [ -n "$wspath" ]; then
            local packages
            packages=$(catkin list -u)
            local dbs=""
            for pack in $packages
            do
                if [ -f "$wspath"/build/"$pack"/compile_commands.json ]; then
                    dbs="$dbs -p $wspath/build/$pack"
                else
                    echo "No translation db for $pack"
                fi
            done
            # shellcheck disable=SC2086
            compdb $dbs list > "$wspath"/src/compile_commands.json \
                && echo "Compilation database created at $wspath/src/compile_commands.json"
        else
            echo "Not in a catkin workspace"
        fi
    }

    build_compdb() {
        catkin build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
    }
fi

# Play a rosbag with simulated time
rosbag_simtime() {
    rosparam set /use_sim_time true
    rosbag play "$1" -l --clock
}

# Try to find the package path using rospack and catkin tools. If the package
#     is not found, return the package name.
# Args:
#     $1  ROS package name
rospackpath() {
    if rospack find "$1" > /dev/null 2>&1; then
        rospack find "$1"
    elif catkin locate "$1" > /dev/null 2>&1; then
        # otherwise, use catkin locate (local workspace package search)
        catkin locate "$1"
    else
        echo "$1"
    fi
}

# entr - automatic execution of commands using kqueue or inotify
if [ -x "$(command -v entr)" ]; then

    # Re-execute all pytests within a ROS package when any test file is updated.
    # Args:
    #     $1      ROS package name
    #     ${@:2}  Additional args are passed to pytest
    entr_pytest_rospack() {
        local packpath
        packpath="$(rospackpath "$1")"
        find "$packpath"/test -name "test*.py" | entr -c pytest "${@:2}" "$packpath"
    }

    # Re-execute a pytest when the test*.py file is updated
    # Args:
    #     $1      Path to a pytest test*.py file
    #     ${@:2}  Additional args are passed to pytest
    entr_pytest() {
        echo "$1" | entr -c pytest "${@:2}" "$1"
    }

    # Execute catkin test when any test*.py or test*.launch file is updated.
    # Args:
    #     $1      ROS Package name
    #     ${@:2}  Additional args are passed to catkin test
    entr_test_rospack() {
        local packpath
        packpath="$(rospackpath "$1")"
        find "$packpath"/test -name "test*.py" -o -name "test*.launch" | entr -c catkin test "${@:2}" "$1"
    }
fi

# fzf dependent functions and aliases
if [ -x "$(command -v fzf)" ] && [ -f ~/.aliases/fzf_functions.sh ]; then
    # shellcheck source=/home/mallain/.aliases/fzf_functions.sh
    . ~/.aliases/fzf_functions.sh
fi

# -------------------------------
# 4. SPECIALIZED TOOLS
# -------------------------------

# DOCKER ALIASES
alias docker_clean_images='docker rmi $(docker images -a --filter=dangling=true -q)'
alias docker_clean_ps='docker rm $(docker ps --filter=status=exited --filter=status=created -q)'

# LAUNCH OBSIDIAN FLATPAK WITHOUT NETWORK
alias obsidian='flatpak run --unshare=network md.obsidian.Obsidian >/dev/null 2>&1 &'

# MERMAID DIAGRAM GENERATION
# alias mmdc="/home/mallain/dev/node_modules/.bin/mmdc"


# Use text in clipboard as input to mmdc to generate mermaid diagrams
# Args:
#    $@: any args to pass to mmdc, must include output path (-o PATH)
#
# Example: 'mmdclip -w 1000 -o test.svg'
mmdclip() {
  xclip -selection clipboard -o | mmdc -i /dev/stdin -c ~/.mermaid.json "$@"
}

# Extract most known archives with one command
# Args:
#    $1: archive file
#
# Example: 'extract archive.tar.gz'
extract () {
  if [ -f "$1" ] ; then
    case "$1" in
      *.tar.bz2)   tar xjf "$1"     ;;
      *.tar.gz)    tar xzf "$1"     ;;
      *.bz2)       bunzip2 "$1"     ;;
      *.rar)       unrar e "$1"     ;;
      *.gz)        gunzip "$1"      ;;
      *.tar)       tar xf "$1"      ;;
      *.tar.xz)    tar xf "$1"      ;;
      *.tar.zst)   tar xaf "$1"     ;;
      *.tbz2)      tar xjf "$1"     ;;
      *.tgz)       tar xzf "$1"     ;;
      *.zip)       unzip "$1"       ;;
      *.Z)         uncompress "$1"  ;;
      *.7z)        7z x "$1"        ;;
      *)     echo "'$1' cannot be extracted via extract()" ;;
    esac
  else
    echo "'$1' is not a valid file"
  fi
}

# ---------------------------
# 4. SEARCHING
# ---------------------------


# ---------------------------
# 5. PROCESS MANAGEMENT
# ---------------------------

# findPid: find out the pid of a specified process
# -----------------------------------------------------
#     Note that the command name can be specified via a regex
#     E.g. findPid '/d$/' finds pids of all processes with names ending in 'd'
#     Without the 'sudo' it will only find processes of the current user
# -----------------------------------------------------
find_pid () { lsof -t -c "$@" ; }

# memHogsTop, memHogsPs:  Find memory hogs
# -----------------------------------------------------
alias mem_hogs_top='top -l 1 -o rsize | head -20'
alias mem_hogs_ps='ps wwaxm -o pid,stat,vsize,rss,time,command | head -10'

#   cpuHogs:  Find CPU hogs
#   -----------------------------------------------------
alias cpu_hogs='ps wwaxr -o pid,stat,%cpu,time,command | head -10'

#   #   my_ps: List processes owned by my user:
#   ------------------------------------------------------------
my_ps() { ps $@ -u $USER -o pid,%cpu,%mem,start,time,bsdtime,command ; }

#   ---------------------------
#   6. NETWORKING
#   ---------------------------

    # alias myip='curl ip.appspot.com'                    # myip:         Public facing IP Address
    # alias netCons='lsof -i'                             # netCons:      Show all open TCP/IP sockets
    # alias flushDNS='dscacheutil -flushcache'            # flushDNS:     Flush out the DNS Cache
    # alias lsock='sudo /usr/sbin/lsof -i -P'             # lsock:        Display open sockets
    # alias lsockU='sudo /usr/sbin/lsof -nP | grep UDP'   # lsockU:       Display only open UDP sockets
    # alias lsockT='sudo /usr/sbin/lsof -nP | grep TCP'   # lsockT:       Display only open TCP sockets
    # alias ipInfo0='ipconfig getpacket en0'              # ipInfo0:      Get info on connections for en0
    # alias ipInfo1='ipconfig getpacket en1'              # ipInfo1:      Get info on connections for en1
    # alias openPorts='sudo lsof -i | grep LISTEN'        # openPorts:    All listening connections
    # alias showBlocked='sudo ipfw list'                  # showBlocked:  All ipfw rules inc/ blocked IPs

# #   ii:  display useful host related informaton
# #   -------------------------------------------------------------------
#     ii() {
#         echo -e "\nYou are logged on ${RED}$HOST"
#         echo -e "\nAdditionnal information:$NC " ; uname -a
#         echo -e "\n${RED}Users logged on:$NC " ; w -h
#         echo -e "\n${RED}Current date :$NC " ; date
#         echo -e "\n${RED}Machine stats :$NC " ; uptime
#         echo -e "\n${RED}Current network location :$NC " ; scselect
#         echo -e "\n${RED}Public facing IP Address :$NC " ;myip
#         #echo -e "\n${RED}DNS Configuration:$NC " ; scutil --dns
#         echo
#     }

# prints an excerpt from the art of the command line
if [ -x "$(command -v pandoc)" ] && [ -x "$(command -v xmlstarlet)" ]; then
    function taocl() {
      curl -s https://raw.githubusercontent.com/jlevy/the-art-of-command-line/master/README.md |
        sed '/cowsay[.]png/d' |
        pandoc -f markdown -t html |
        xmlstarlet fo --html --dropdtd |
        xmlstarlet sel -t -v "(html/body/ul/li[count(p)>0])[$RANDOM mod last()+1]" |
        xmlstarlet unesc | fmt -80 | iconv -t US
    }
fi

# test terminal color scheme
# \033[ is the control sequence introducer
# attributes are separated by semicolons and terminated by m to Select Graphic Rendition
# empty attributes are interpreted as 0 (reset/normal)
function termcolors() {
    printf "          "
    for b in 0 1 2 3 4 5 6 7; do printf "  4%sm " "$b"; done
    echo
    for f in "" 30 31 32 33 34 35 36 37; do
        for s in "" "1;"; do
            printf "%4sm" "${s}${f}"
            printf " \033[%sm%s\033[0m" "$s$f" "gYw "
            for b in 0 1 2 3 4 5 6 7; do
                printf " \033[4%s;%sm%s\033[0m" "$b" "$s$f" " gYw "
            done
            echo
         done
    done
}

# shorter cmake -G ninja
alias nmake="cmake -G Ninja"
