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

#   -------------------------------
#   1. ENVIRONMENT CONFIGURATION
#   -------------------------------

#   Change Prompt
#   ------------------------------------------------------------
    # SEE bashrc for powerline invocation
    parse_git_branch() {
        git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
    }
    export PS1="\u@\h \[\033[32m\]\W\[\033[33m\]\$(parse_git_branch)\[\033[00m\] $ "
    #export PS1="\u@\h:\W\\$ "

#   Set Paths
#   ------------------------------------------------------------
#    export PATH="$PATH:/usr/local/bin/"

#   Set Default Editor (change 'Nano' to the editor of your choice)
#   ------------------------------------------------------------
    export EDITOR=/usr/bin/vi

#   Set default blocksize for ls, df, du
#   from this: http://hints.macworld.com/comment.php?mode=view&cid=24491
#   ------------------------------------------------------------
    export BLOCKSIZE=1k

#   Add color to terminal
#   (this is all commented out as I use Mac Terminal Profiles)
#   from http://osxdaily.com/2012/02/21/add-color-to-the-terminal-in-mac-os-x/
#   ------------------------------------------------------------
#   export CLICOLOR=1
#   export LSCOLORS=ExFxBxDxCxegedabagacad


#   ----------------------------------------
#   2. DEFAULT OPTIONS AND USEFUL ALIASES
#   ----------------------------------------

alias cp='cp -iv'                                   # prompt before overwrite (-i) and verbose (-v)
alias mv='mv -iv'                                   # prompt before overwrite (-i) and verbose (-v)
alias nt='nautilus'                                 # nautilus file browser shortcut
alias mkdir='mkdir -pv'                             # make parents (-p) and verbose (-v)

# show dotfiles (-A), classify (-F), no-group (-G), human readable sizes (-h)
# long listing (-l), append / to directories (-p), sort by mtime newest first (-t)
alias ll='ls -AFGhlpt'

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

alias which='type -a'                               # which:        Find executables
alias path='echo -e ${PATH//:/\\n}'                 # path:         Echo all executable Paths
alias fix_stty='stty sane'                          # fix_stty:     Restore terminal settings when screwed up
mcd () { mkdir -p "$1" && cd "$1"; }                # mcd:          Makes new Dir and jumps inside
alias DT='tee /tmp/termout.txt'                     # DT:           Pipe content to temporary file

alias xclipc="xclip -selection c"
alias xclipcr="tr -d '\n' | xclip -selection c"
alias xclipv="xclip -selection clipboard -o"

## Git aliases
# note git <alias> type commands can be defined in .gitconfig
alias gitl="git log"
alias gits="git status"

# bd - back to parent directory
# https://github.com/vigneshwaranr/bd
alias bd=". bd -si"

## COMMAND ALTERNATIVES
## Replace some common commands with fancier alternatives from this century

# bat - better cat
if [ -x "$(command -v bat)" ]; then
  alias cat="bat"
fi

# exa - better ls
if [ -x "$(command -v exa)" ]; then
  alias lx="exa -lga --icons -s name"
fi

## BASH FUNCTIONS
## --------------

# trash       : alternative to rm which moves all arguments to the Ubuntu trash
#             : TODO: platform-agnostic
# ------------:-----------------------------------------------------------------
trash () { command mv "$@" ~/.local/share/Trash ; }

# xopen       : xdg-open wrapper which supports multiple files/urls and runs in
#             : bg by default
# ------------:-----------------------------------------------------------------
xopen() {
  for var in "$@"; do
    (xdg-open "$var" > /dev/null 2>&1 &)
  done
}

# savetmuxbuffer  : captures the last 10000 lines from tmux and saves to a file
#                 : requires three args: session, window, and pane
# ----------------:-------------------------------------------------------------
# Example         : 'savetmuxbuffer ROS bringup 0'
savetmuxbuffer () {
  /usr/bin/tmux send-keys -t $1:$2.$3 C-b ":capture-pane -e -S 10000" C-m
  /usr/bin/tmux send-keys -t $1:$2.$3 C-b ":save-buffer ~/buffer.txt" C-m
}

# lr          : List n most recently modified files in directory
# ------------:-----------------------------------------------------------------
# Example     : 'lr 5'
lr () {
  ll -t | head -n $1
}

# mans        : Search manpage given in arg 1 for term in arg 2 (case insensitive)
#             : Displays paginated & colored result with two lines per hit
# ------------:-----------------------------------------------------------------
# Example     : 'mans mplayer codec'
mans () {
  man $1 | grep -iC2 --color=always $2 | less
}

# colview     : view argument as formatted csv file in the console
# ------------:-----------------------------------------------------------------
# Example     : 'colview test.csv'
colview () {
  column -n -s, -tn < $1 | less -#2 -N -S
}

# unmvgitmv   : move a file at arg 1 back to path arg 2, then git mv from 2 to 1
#             : for when you forget to git mv
# ------------:-----------------------------------------------------------------
unmvgitmv () {
  mv $1 $2
  git mv $2 $1
}

# ----------------------------------------
# 3. ROS AND ROBOTICS DEVELOPMENT
# ----------------------------------------

# Preferred pylint config
alias pylint="pylint --rcfile=~/.pylintrc"

# #   ROS aliases
alias rosenv='printenv | grep ROS'                              # rosenv:  print the ROS path variables
alias rosgdb="rosrun --prefix 'gdb -ex run --args' "
alias rosprofile="rosrun --prefix 'valgrind --tool=callgrind' "
alias rosmemcheck="rosrun --prefix 'valgrind --tool=memcheck' "

# rosloglvl   : set logger in arg 1 at node in arg 2 to level in arg 3
# ------------:-----------------------------------------------------------------
# Example     : 'rosloglvl ultra_workspace_manager rospy debug' (confirm?)
rosloglvl () {
    rosservice call $2/set_logger_level "logger='ros.$1'"$'\r'"level='$3'"
}

# sws         : reset ROS environment, find containing workspace and source it
# ------------:-----------------------------------------------------------------
sws () {
    source /opt/ros/$ROS_DISTRO/setup.bash;
    local wspath=`catkin locate`;
    source $wspath/devel/setup.bash;
}

# rostopicw   : update rostopic list every 1 second, or n seconds where n is arg 1
# ------------:-----------------------------------------------------------------
rostopicw () {
  local period=${1:-1}
  watch -n $period "rostopic list"
}

# rostopicw   : update rosnode list every 1 second, or n seconds where n is arg 1
# ------------:-----------------------------------------------------------------
rosnodew () {
  local period=${1:-"1"}
  watch -n $period "rosnode list"
}

# rosmaster   : replace rosmaster hostname with arg 1, assume default port
# ------------:-----------------------------------------------------------------
rosmaster () {
  export ROS_MASTER_URI=http://$1:11311
}

# rosgrep     : find package
#             : todo replace with fuzzy package finder
# ------------:-----------------------------------------------------------------
rosgrep () {
  rospack list | grep "$1";
}

# roseulerify : republish a transform as Vector3 with rpy components
#             : TODO better way?
# ------------:-----------------------------------------------------------------
roseulerify () {
  rosrun topic_tools transform $1 /eulerify$1 geometry_msgs/Vector3 'map(lambda x: x * 57.296, tf.transformations.euler_from_quaternion([m.x, m.y, m.z, m.w]))' --import tf
}

# roswiki     : parse ros wiki url from package.xml and open in chrome
# ------------:-----------------------------------------------------------------
roswiki () {
  roscd $1; URL=$(cat package.xml | grep http | cut -d ">" -f 2 | cut -d "<" -f 1); /opt/google/chrome/chrome $URL
}

# catkin_test : build package name from first arg and execute tests, printing results
#             : results are printed ...
#             : TODO is this redundant with 'catkin test --this'?
# ------------:-----------------------------------------------------------------
catkin_test() {
  local wspath=`catkin locate`
  catkin build --no-deps --catkin-make-args run_tests -- $1 > /dev/null && catkin_test_results $wspath/build/$1
  #    catkin build --no-deps --catkin-make-args run_tests -- $1 && catkin_test_results $wspath/build/$1
}

# catkin_test_this : build current package and execute tests, printing results
# -----------------:-----------------------------------------------------------------
catkin_test_this() {
  local pkgname=`catkin list --this -u`
  catkin_test $pkgname
}

# catkin_source    : source containing ws *without* clearing ros env
#                  : i.e., this will overlay the current workspace
# -----------------:-----------------------------------------------------------------
catkin_source() {
  source `catkin locate`/devel/setup.bash
}

# catkin_tidy      : uses compdb tool to collect all built packages' translation dbs
#                  : and links to src directory for editor to run clang-tidy against
#                  : TODO check compdb installation first
# -----------------:-----------------------------------------------------------------
catkin_tidy() {
  catkin build $1 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  local wspath=`catkin locate`
  # local translation_db=$wspath/build/$1/compile_commands.json
  local packages=`catkin list -u`
  local dbs=""
  for pack in $packages
  do
    if [ -f $wspath/build/$pack/compile_commands.json ]; then
      local db="-p $wspath/build/$pack "
      dbs="$dbs$db"
    fi
  done
  compdb $dbs list > $wspath/src/compile_commands.json

  # old, todo remove, linking individual packages, before I found compdb
  #if [ -f $translation_db ]; then
  #    ln -fs $wspath/build/$1/compile_commands.json $wspath/src/compile_commands.json
  #else
  #    echo "$translation_db does not exist"
  #fi
}

rosbag_simtime() {
  rosparam set /use_sim_time true
  rosbag play $1 -l --clock
}

# Private alias definitions for work - stored separately
if [ -f ~/.aliases/private_aliases.sh ]; then
  . ~/.aliases/private_aliases.sh
fi

# fzf dependent functions and aliases
if [ -x "$(command -v fzf)" ] && [ -f ~/.aliases/fzf_functions.sh ]; then
  . ~/.aliases/fzf_functions.sh
fi

# MAC OS X Aliases - TO DO
# ------------------------
# ql () { qlmanage -p "$*" >& /dev/null; }    # ql:           Opens any file in MacOS Quicklook Preview

# -------------------------------
# 4. SPECIALIZED TOOLS
# -------------------------------

# DOCKER ALIASES
alias docker_clean_images='docker rmi $(docker images -a --filter=dangling=true -q)'
alias docker_clean_ps='docker rm $(docker ps --filter=status=exited --filter=status=created -q)'

# MERMAID DIAGRAM GENERATION
alias mmdc="/home/mallain/dev/node_modules/.bin/mmdc"

# mmdclip          : use text in clipboard to generate mermaid diagrams
#                  : need to provide at least output path (-o PATH)
#                  : can provide any other args (see mmdc -h)
# -----------------:-----------------------------------------------------------------
# Example          : 'mmdclip -w 1000 -o test.svg'
mmdclip() {
  xclip -o | mmdc -i /dev/stdin -c ~/.mermaid.json $@
}

# extract:  Extract most know archives with one command
# ---------------------------------------------------------
extract () {
  if [ -f $1 ] ; then
    case $1 in
      *.tar.bz2)   tar xjf $1     ;;
      *.tar.gz)    tar xzf $1     ;;
      *.bz2)       bunzip2 $1     ;;
      *.rar)       unrar e $1     ;;
      *.gz)        gunzip $1      ;;
      *.tar)       tar xf $1      ;;
      *.tbz2)      tar xjf $1     ;;
      *.tgz)       tar xzf $1     ;;
      *.zip)       unzip $1       ;;
      *.Z)         uncompress $1  ;;
      *.7z)        7z x $1        ;;
      *)     echo "'$1' cannot be extracted via extract()" ;;
    esac
  else
    echo "'$1' is not a valid file"
  fi
}

# ---------------------------
# 4. SEARCHING
# ---------------------------

alias qfind="find . -name "                 # qfind:    Quickly search for file
ff () { /usr/bin/find . -name "$@" ; }      # ff:       Find file under the current directory
ffs () { /usr/bin/find . -name "$@"'*' ; }  # ffs:      Find file whose name starts with a given string
ffe () { /usr/bin/find . -name '*'"$@" ; }  # ffe:      Find file whose name ends with a given string

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
function taocl() {
    curl -s https://raw.githubusercontent.com/jlevy/the-art-of-command-line/master/README.md |
        sed '/cowsay[.]png/d' |
        pandoc -f markdown -t html |
        xmlstarlet fo --html --dropdtd |
        xmlstarlet sel -t -v "(html/body/ul/li[count(p)>0])[$RANDOM mod last()+1]" |
        xmlstarlet unesc | fmt -80 | iconv -t US
}

