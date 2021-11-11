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
    export PS1="\u@\h:\W\\$ "

#   Set Paths
#   ------------------------------------------------------------
    export PATH="$PATH:/usr/local/bin/"

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

    alias cp='cp -iv'                           # Preferred 'cp' implementation
    alias mv='mv -iv'                           # Preferred 'mv' implementation
    alias nt='nautilus'                         # nautilus shortcut
    alias mkdir='mkdir -pv'                     # Preferred 'mkdir' implementation
    alias ll='ls -FGlAhp'                       # Preferred 'ls' implementation
    alias less='less -FSRXc'                    # Preferred 'less' implementation
    cd() { builtin cd "$@"; ll; }               # Always list directory contents upon 'cd'
    alias cd..='cd ../'                         # Go back 1 directory level (for fast typers)
    alias ..='cd ../'                           # Go back 1 directory level
    alias ...='cd ../../'                       # Go back 2 directory levels
    alias .3='cd ../../../'                     # Go back 3 directory levels
    alias .4='cd ../../../../'                  # Go back 4 directory levels
    alias .5='cd ../../../../../'               # Go back 5 directory levels
    alias .6='cd ../../../../../../'            # Go back 6 directory levels
    alias ~="cd ~"                              # ~:            Go Home
    alias c='clear'                             # c:            Clear terminal display
    alias which='type -all'                     # which:        Find executables
    alias path='echo -e ${PATH//:/\\n}'         # path:         Echo all executable Paths
    alias show_options='shopt'                  # Show_options: display bash options settings
    alias fix_stty='stty sane'                  # fix_stty:     Restore terminal settings when screwed up
    alias cic='set completion-ignore-case On'   # cic:          Make tab-completion case-insensitive
    mcd () { mkdir -p "$1" && cd "$1"; }        # mcd:          Makes new Dir and jumps inside
    ql () { qlmanage -p "$*" >& /dev/null; }    # ql:           Opens any file in MacOS Quicklook Preview
    alias DT='tee /tmp/termout.txt'             # DT:           Pipe content to temporary file
    trash () { command mv "$@" ~/.local/share/Trash ; }     # trash:        Moves a file to the Ubuntu trash

    alias bnet="nmcli con up id 'Bastian Guest'"
    alias xclipc="xclip -selection c"
    alias xclipcr="tr -d '\n' | xclip -selection c"
    alias xclipv="xclip -selection clipboard -o"

    ## Git aliases
    alias gitl="git log --pretty=format:'%C(yellow)%h %Cred%ad %Cblue%an%Cgreen%d %Creset%s' --date=short"
    alias gits="git status --porcelain"

    # Pipe to this, displays up to newline
    alias wut="sed '/^\s*$/q'"

    # xopen
    # -----
    # xopen is my preferred xdg-open wrapper which supports multiple files/urls and runs in bg by default
    xopen() {
        for var in "$@"; do
            (xdg-open "$var" > /dev/null 2>&1 &)
        done
    }

    # tmux scrollback helper
    savetmuxbuffer () {
        /usr/bin/tmux send-keys -t $1:$2.$3 C-b ":capture-pane -e -S 10000" C-m
        /usr/bin/tmux send-keys -t $1:$2.$3 C-b ":save-buffer ~/buffer.txt" C-m
    }

#   lrr:  Full Recursive Directory Listing
#   ------------------------------------------
#   alias lrr='ls -R | grep ":$" | sed -e '\''s/:$//'\'' -e '\''s/[^-][^\/]*\//--/g'\'' -e '\''s/^/   /'\'' -e '\''s/-/|/'\'' | less'

#   lr: List n most recently modified files in directory
#   ----------------------------------------------------
    lr() {
        ll -t | head -n $1
    }

#   mans:   Search manpage given in agument '1' for term given in argument '2' (case insensitive)
#           displays paginated result with colored search terms and two lines surrounding each hit.            Example: mans mplayer codec
#   --------------------------------------------------------------------
    mans () {
        man $1 | grep -iC2 --color=always $2 | less
    }

#   colview: view formatted csv files from the console
#   ------------------------------------------------------------
    colview () {
        column -n -s, -tn < $1 | less -#2 -N -S
    }

    #if ! [ -x "$(command -v bat)" ]; then
    #    alias cat="bat"
    #fi

    unmovegitmove () {
        mv $1 $2
        git mv $2 $1
    }

#   ----------------------------------------
#   3. ROS AND ROBOTICS DEVELOPMENT
#   ----------------------------------------

    # Preferred pylint config
    alias pylint="pylint --rcfile=~/.pylintrc"

    # #   ROS aliases
    alias rosenv='printenv | grep ROS'                              # rosenv:  print the ROS path variables

    alias rosgdb="rosrun --prefix 'gdb -ex run --args' "
    alias rosprofile="rosrun --prefix 'valgrind --tool=callgrind' "
    alias rosmemcheck="rosrun --prefix 'valgrind --tool=memcheck' "

    alias listbranches="find . -type d -name '.git' -exec echo {} \; -exec git -C {} branch \;"

    rosloglvl () {
        rosservice call $2/set_logger_level "logger='ros.$1'"$'\r'"level='$3'"
    }

    # sws
    # ---
    # reset ROS environment, find containing workspace and source it
    sws () {
        source /opt/ros/$ROS_DISTRO/setup.bash;
        local wspath=`catkin locate`;
        source $wspath/devel/setup.bash;
    }

    # sitetime
    # --------
    # print time in various important locales
    sitetime() {
    : ${WORLDCLOCK_FORMAT:='+%Y-%m-%d %H:%M:%S %Z'}

      LCL=`date "$WORLDCLOCK_FORMAT"`
      FFC=`env TZ=US/Eastern date "$WORLDCLOCK_FORMAT"`
      HTS=`env TZ=Asia/Tokyo date "$WORLDCLOCK_FORMAT"`

      echo "Local                     $LCL"
      echo "-------------------------------------------------"
      echo "U3-1-FFC Frankfort, ID    $FFC"
      echo "U3-2-HTS Kanda, JP        $HTS"
    }


#  TODO: the following do not belong in my bash_aliases, but to robot workspace specific alias files...
#   -------------------
#   MR - Mobile Robot
#   -------------------
    alias mrsrc="source ~/catkin_ws/devel/setup.bash"


#   -----------------------------
#   LMR - Little Mobile Robot
#   -----------------------------
    alias lemurnet="nmcli con up id 'Lemur WiFi Static'";
    alias lemurstate="rostopic echo /lmr/state | grep pack -A 23"
    alias lemursrc="source /opt/ros/$ROS_DISTRO/setup.bash && \
                    source ~/lmr_ws/devel/setup.bash && \
                    export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/lmr_ws/src/lemur-BMR/lemur_simulator/lemur_gazebo/worlds"

#   -------------------
#   ULTRA
#   -------------------
    alias ultranet="nmcli con up id 'ULTRA_2_Wireless Static'";
    alias aualsrc="source /home/$USER/.ros_env_hooks/.ultra";
    alias ultrasrc="aualsrc"
    alias ultravcs="wstool merge $ULTRA_REPO/utilities/ultra_$ROS_DISTRO.rosinstall -t $ULTRA_WS/src"

#   ultrasync: rsync ultra repo to given hostname
#   --------------------------------------------------------------
    ultrasync () {
        rsync -avE $ULTRA_REPO bmr@$1:~/catkin_ws/src/
    }

    # transfer_to: transfer working directory to the specified robot
    # --------------------------------------------------------------
    transfer_to ()
    {
      src_dir=$(pwd)
      dest_dir="`echo ${src_dir%/*} | sed "s/${USER}/bmr/g"`"
      rsync -avE $src_dir bmr@$1:$dest_dir
    }

    # transfer_from: transfer working directory from the specified robot to the local machine
    # --------------------------------------------------------------
    transfer_from ()
    {
      src_dir="`pwd | sed "s/${USER}/bmr/g"`/"
      dest_dir="$(pwd)"
      rsync -avE bmr@$1:$src_dir $dest_dir
    }


    # Vision WS
    alias vissrc="source ~/vision_ws/devel/setup.bash"


    # ros_start: starts the ros system
    # --------------------------------------------------------------
    # Note: The ros_start command assumes that there is only one configuration
    # file located in the home directory
    ros_start ()
    {
      ${ULTRA_WS}/src/ultra-2-BMR/utilities/launcher.sh $(find ~/ultra* -print -quit)
    }

    # ros_exit: shuts down the ros system
    # --------------------------------------------------------------
    ros_exit ()
    {
      echo 'Killing ROS Tmux session...'
      tmux kill-session -t ROS
      tmux kill-session -t VENV
    }

    # ros_restart: restarts the ros system
    # --------------------------------------------------------------
    ros_restart ()
    {
      ros_exit
      ros_start
    }


#   rostopicw: update list every $1 seconds
#   --------------------------------------------------------------
    rostopicw () {
        watch -n $1 "rostopic list"
    }

#   rosnodew: update list every $1 seconds
#   --------------------------------------------------------------
    rosnodew () {
        watch -n $1 "rosnode list"
    }

#   rosmaster: change master URI to machine name or IP in first arg
#   --------------------------------------------------------------
    rosmaster () {
        export ROS_MASTER_URI=http://$1:11311
    }

    rosgrep () {
        rospack list | grep "$1";
    }

    roseulerify () {
        rosrun topic_tools transform $1 /eulerify$1 geometry_msgs/Vector3 'map(lambda x: x * 57.296, tf.transformations.euler_from_quaternion([m.x, m.y, m.z, m.w]))' --import tf
    }

#   roswiki: parse ros wiki url from package.xml and open in chrome
#   --------------------------------------------------------------
    roswiki () {
        roscd $1; URL=$(cat package.xml | grep http | cut -d ">" -f 2 | cut -d "<" -f 1); /opt/google/chrome/chrome $URL
    }

#   catkin_test: build package name from first arg and execute tests, printing results
#   --------------------------------------------------------------
    catkin_test() {
        local wspath=`catkin locate`
        catkin build --no-deps --catkin-make-args run_tests -- $1 > /dev/null && catkin_test_results $wspath/build/$1
    #    catkin build --no-deps --catkin-make-args run_tests -- $1 && catkin_test_results $wspath/build/$1
    }

#   catkin_test_this:
#   --------------------------------------------------------------
    catkin_test_this() {
        local pkgname=`catkin list --this -u`
        catkin_test $pkgname
    }

#   catkin_source: source containing ws *without* clearing ros env
#   --------------------------------------------------------------
    catkin_source() {
        source `catkin locate`/devel/setup.bash
    }

#   catkin_tidy: uses compdb tool to collect all built packages translation dbs and links to src
#                directory for editor or IDE to run clang-tidy against
#   --------------------------------------------------------------
    catkin_tidy() {
        catkin build $1 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        local wspath=`catkin locate`
#        local translation_db=$wspath/build/$1/compile_commands.json
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

#   bastian_jupyter: launch your jupyter workspace
#   ------------------------------------------------------------
    bastian_jupyter () {
        cd /home/mallain/dev/ultra-jupyter-BMR && conda activate && jupyter notebook
    }



#   -------------------------------
#   4. SPECIALIZED TOOLS
#   -------------------------------

    # DOCKER ALIASES
    alias docker_clean_images='docker rmi $(docker images -a --filter=dangling=true -q)'
    alias docker_clean_ps='docker rm $(docker ps --filter=status=exited --filter=status=created -q)'

    # MERMAID DIAGRAM GENERATION
    alias mmdc="/home/mallain/dev/node_modules/.bin/mmdc"
    # make mermaid diagram from clipboard selection
    mmdclip() {
        xclip -o | mmdc -i /dev/stdin -c ~/.mermaid.json $@
    }

    # tm - create new tmux session, or switch to existing one. Works from within tmux too. (@bag-man)
    # `tm` will allow you to select your tmux session via fzf.
    # `tm irc` will attach to the irc session (if it exists), else it will create it.

    tm() {
      [[ -n "$TMUX" ]] && change="switch-client" || change="attach-session"
      if [ $1 ]; then
        tmux $change -t "$1" 2>/dev/null || (tmux new-session -d -s $1 && tmux $change -t "$1"); return
      fi
      session=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | fzf --exit-0) &&  tmux $change -t "$session" || echo "No sessions found."
    }


#   -------------------------------
#   3. FILE AND FOLDER MANAGEMENT
#   -------------------------------

# zipf () { zip -r "$1".zip "$1" ; }          # zipf:         To create a ZIP archive of a folder
# alias numFiles='echo $(ls -1 | wc -l)'      # numFiles:     Count of non-hidden files in current dir
# alias make1mb='mkfile 1m ./1MB.dat'         # make1mb:      Creates a file of 1mb size (all zeros)
# alias make5mb='mkfile 5m ./5MB.dat'         # make5mb:      Creates a file of 5mb size (all zeros)
# alias make10mb='mkfile 10m ./10MB.dat'      # make10mb:     Creates a file of 10mb size (all zeros)

#   extract:  Extract most know archives with one command
#   ---------------------------------------------------------
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


#   ---------------------------
#   4. SEARCHING
#   ---------------------------

    alias qfind="find . -name "                 # qfind:    Quickly search for file
    ff () { /usr/bin/find . -name "$@" ; }      # ff:       Find file under the current directory
    ffs () { /usr/bin/find . -name "$@"'*' ; }  # ffs:      Find file whose name starts with a given string
    ffe () { /usr/bin/find . -name '*'"$@" ; }  # ffe:      Find file whose name ends with a given string

#   ---------------------------
#   5. PROCESS MANAGEMENT
#   ---------------------------

#   findPid: find out the pid of a specified process
#   -----------------------------------------------------
#       Note that the command name can be specified via a regex
#       E.g. findPid '/d$/' finds pids of all processes with names ending in 'd'
#       Without the 'sudo' it will only find processes of the current user
#   -----------------------------------------------------
    find_pid () { lsof -t -c "$@" ; }

#   memHogsTop, memHogsPs:  Find memory hogs
#   -----------------------------------------------------
    alias mem_hogs_top='top -l 1 -o rsize | head -20'
    alias mem_hogs_ps='ps wwaxm -o pid,stat,vsize,rss,time,command | head -10'

#   cpuHogs:  Find CPU hogs
#   -----------------------------------------------------
    alias cpu_hogs='ps wwaxr -o pid,stat,%cpu,time,command | head -10'

#   topForever:  Continual 'top' listing (every 10 seconds)
#   -----------------------------------------------------
    alias topForever='top -l 9999999 -s 10 -o cpu'

#   ttop:  Recommended 'top' invocation to minimize resources
#   ------------------------------------------------------------
#       Taken from this macosxhints article
#       http://www.macosxhints.com/article.php?story=20060816123853639
#   ------------------------------------------------------------
    alias ttop="top -R -F -s 10 -o rsize"

#   my_ps: List processes owned by my user:
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

#   ---------------------------------------
#   9. FUZZY SEARCHING
#   ---------------------------------------
#   see https://github.com/junegunn/fzf/wiki/Examples#searching-file-contents

#   --------------
#   9.1 FUZZY GIT
#   --------------

    ### https://github.com/wfxr/forgit#custom-options
    # ga:     interactive git add selector
    # glo:    interactive git log viewer
    # gi:     interactive .gitignore generator
    # gd:     interactive git diff viewer
    # grh:    interactive interactive git reset HEAD &lt;file> selector
    # gcf:    interactive git checkout &lt;file> selector
    # gss:    interactive git stash viewer
    # gclean: interactive git clean selector
    [ -f ~/.forgit/forgit.plugin.zsh ] && source ~/.forgit/forgit.plugin.zsh

    # fbr: checkout git branch (including remote branches), sorted by most recent commit, limit 30 last branches
    # ----------------------------------------------------------------------------------------------------------
    fbr() {
      local branches branch
      branches=$(git for-each-ref --count=30 --sort=-committerdate refs/ --format="%(refname:short)") &&
      branch=$(echo "$branches" |
               fzf-tmux -d $(( 2 + $(wc -l <<< "$branches") )) +m) &&
      git checkout $(echo "$branch" | sed "s/.* //" | sed "s#remotes/[^/]*/##")
    }

    # fco - checkout git branch/tag
    # ----------------------------------------------------------------------------------------------------------
    fco() {
      local tags branches target
      branches=$(
        git --no-pager branch --all \
          --format="%(if)%(HEAD)%(then)%(else)%(if:equals=HEAD)%(refname:strip=3)%(then)%(else)%1B[0;34;1mbranch%09%1B[m%(refname:short)%(end)%(end)" \
        | sed '/^$/d') || return
      tags=$(
        git --no-pager tag | awk '{print "\x1b[35;1mtag\x1b[m\t" $1}') || return
      target=$(
        (echo "$branches"; echo "$tags") |
        fzf --no-hscroll --no-multi -n 2 \
            --ansi) || return
      git checkout $(awk '{print $2}' <<<"$target" )
    }


    # fco_preview - checkout git branch/tag, with a preview showing the commits between the tag/branch and HEAD
    # ----------------------------------------------------------------------------------------------------------
    fco_preview() {
      local tags branches target
      branches=$(
        git --no-pager branch --all \
          --format="%(if)%(HEAD)%(then)%(else)%(if:equals=HEAD)%(refname:strip=3)%(then)%(else)%1B[0;34;1mbranch%09%1B[m%(refname:short)%(end)%(end)" \
        | sed '/^$/d') || return
      tags=$(
        git --no-pager tag | awk '{print "\x1b[35;1mtag\x1b[m\t" $1}') || return
      target=$(
        (echo "$branches"; echo "$tags") |
        fzf --no-hscroll --no-multi -n 2 \
            --ansi --preview="git --no-pager log -150 --pretty=format:%s '..{2}'") || return
      git checkout $(awk '{print $2}' <<<"$target" )
    }

    # fshow - git commit browser
    # ----------------------------------------------------------------------------------------------------------
    fshow() {
      git log --graph --color=always \
          --format="%C(auto)%h%d %s %C(black)%C(bold)%cr" "$@" |
      fzf --ansi --no-sort --reverse --tiebreak=index --bind=ctrl-s:toggle-sort \
          --bind "ctrl-m:execute:
                    (grep -o '[a-f0-9]\{7\}' | head -1 |
                    xargs -I % sh -c 'git show --color=always % | less -R') << 'FZF-EOF'
                    {}
    FZF-EOF"
    }

    alias glNoGraph='git log --color=always --format="%C(auto)%h%d %s %C(black)%C(bold)%cr% C(auto)%an" "$@"'
    _gitLogLineToHash="echo {} | grep -o '[a-f0-9]\{7\}' | head -1"
    _viewGitLogLine="$_gitLogLineToHash | xargs -I % sh -c 'git show --color=always % | diff-so-fancy'"

    # fcoc_preview - checkout git commit with previews
    # ----------------------------------------------------------------------------------------------------------
    fcoc_preview() {
      local commit
      commit=$( glNoGraph |
        fzf --no-sort --reverse --tiebreak=index --no-multi \
            --ansi --preview="$_viewGitLogLine" ) &&
      git checkout $(echo "$commit" | sed "s/ .*//")
    }

    # fshow_preview - git commit browser with previews
    # ----------------------------------------------------------------------------------------------------------
    fshow_preview() {
        glNoGraph |
            fzf --no-sort --reverse --tiebreak=index --no-multi \
                --ansi --preview="$_viewGitLogLine" \
                    --header "enter to view, alt-y to copy hash" \
                    --bind "enter:execute:$_viewGitLogLine   | less -R" \
                    --bind "alt-y:execute:$_gitLogLineToHash | xclip"
    }


#   ---------------------
#   9.2 FUZZY CHROME
#   ---------------------

    # fbook - fuzzy browse chrome bookmarks
    alias fbook="/home/mallain/bin/fbook.rb"

    # fhist
    # -------
    # search chrome browser history
    export -f xopen
    fhist() {
      local cols sep google_history open
      cols=$(( COLUMNS / 3 ))
      sep='{::}'

      if [ "$(uname)" = "Darwin" ]; then
        google_history="$HOME/Library/Application Support/Google/Chrome/Default/History"
        open=open
      else
        google_history="$HOME/.config/google-chrome/Default/History"
        open=xopen
      fi
      \cp -f "$google_history" /tmp/h
      sqlite3 -separator $sep /tmp/h \
        "select substr(title, 1, $cols), url
         from urls order by last_visit_time desc" |
      awk -F $sep '{printf "%-'$cols's  \x1b[36m%s\x1b[m\n", $1, $2}' |
      fzf --ansi --multi | sed 's#.*\(https*://\)#\1#' | xargs bash -c 'xopen "$@"' _ {}
    }


    fman() {
        man -k . | fzf --prompt='Man> ' | awk '{print $1}' | xargs -r man
    }

    # fnotes
    # ------
    # fuzzy full-text search markdown notes and open the exact line number match in sublime
    fnotes() {
      # rg --files-with-matches --no-messages "$1" | fzf --preview "highlight -O ansi -l {} 2> /dev/null | rg --colors 'match:bg:yellow' --ignore-case --pretty --context 10 '$1' || rg --ignore-case --pretty --context 10 '$1' {}"
      bdir=$pwd
      ndir="/home/mallain/Dropbox/BMR-MA/Notable/notes"
      cd $ndir > /dev/null 2>&1

      # VSCODE DEFAULT
      # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && cat -n {1}" | cut -d':' -f1-2 | xargs -d '\n' code -g

      # SUBLIME DEFAULT
      ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && cat -n {1}" | cut -d':' -f1-2 | xargs -d '\n' subl

      # ag --nobreak --noheading . | fzf -m --delimiter=: --preview "echo {1} | cut -d':' -f1 | xargs -d '\n' -r cat" | cut -d':' -f1
      # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "rg --pretty --context 10 -F "{3}" {1}" | cut -d':' -f1
      # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "rg -n {2} --context 10 {1}" | cut -d':' -f1
      # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && cat -n {1} | rg --context 10 --pretty {2} " | cut -d':' -f1
      # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && cat {1} | tail -n +\$({2} + 10)" | cut -d':' -f1
      # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && echo \"{3..}\" | xargs rg -F \"{3..}\" {1}"
      # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && rg -F \"\$(echo {3..})\" {1}"
      cd $bdir > /dev/null 2>&1
    }

    # using ripgrep combined with preview
    # find-in-file - usage: fif <searchTerm>
    fif() {
        if [ ! "$#" -gt 0 ]; then echo "Need a string to search for!"; return 1; fi
        rg --files-with-matches --no-messages "$1" | fzf --preview "highlight -O ansi -l {} 2> /dev/null | rg --colors 'match:bg:yellow' --ignore-case --pretty --context 10 '$1' || rg --ignore-case --pretty --context 10 '$1' {}"
    }

    fcd() {
        local dir
        dir=$(find ${1:-.} -path '*/\.*' -prune \
                      -o -type d -print 2> /dev/null | fzf +m --preview="tree -L 2 {}") &&
        cd "$dir"
    }

    fsubld() {
        local dirs
        dirs=$(find ${1:-.} -path '*/\.*' -prune \
                      -o -type d -print 2> /dev/null | fzf -m --preview="tree -L 2 {}") &&
        for var in $dirs; do
            (subl "$var" > /dev/null 2>&1 &)
        done
    }

    # fopen
    # -----
    # open one or more files with xdg-open
    fopen() {
        IFS=$'\n' files=($(fzf-tmux --query="$1" --multi --select-1 --exit-0))
        [[ -n "$files" ]] && xopen "${files[@]}"
    }

    # Modified version where you can press
    #   - CTRL-O to open subl
    #   - CTRL-E or Enter key to open with the $EDITOR
    fedit () {
        IFS=$'\n' out=("$(fzf-tmux --query="$1" --exit-0 --expect=ctrl-o,ctrl-e)")
        key=$(head -1 <<< "$out")
        file=$(head -2 <<< "$out" | tail -1)
        if [ -n "$file" ]; then
            [ "$key" = ctrl-o ] && subl "$file" || ${EDITOR:-vim} "$file"
        fi
    }

    # fkill - kill processes - list only the ones you can kill. Modified the earlier script.
    fkill() {
        local pid
        if [ "$UID" != "0" ]; then
            pid=$(ps -f -u $UID | sed 1d | fzf -m | awk '{print $2}')
        else
            pid=$(ps -ef | sed 1d | fzf -m | awk '{print $2}')
        fi

        if [ "x$pid" != "x" ]
        then
            echo $pid | xargs kill -${1:-9}
        fi
    }

    alias bd=". bd -si"

    # fuzzy grep open via ag with line number
    fvim() {
        local file
        local line

        read -r file line <<<"$(ag --nobreak --noheading $@ | fzf -0 -1 | awk -F: '{print $1, $2}')"

        if [[ -n $file ]]
        then
            vim $file +$line
        fi
    }

    # prints an excerpt from the art of the command line
    function taocl() {
        curl -s https://raw.githubusercontent.com/jlevy/the-art-of-command-line/master/README.md |
        sed '/cowsay[.]png/d' |
        pandoc -f markdown -t html |
        xmlstarlet fo --html --dropdtd |
        xmlstarlet sel -t -v "(html/body/ul/li[count(p)>0])[$RANDOM mod last()+1]" |
        xmlstarlet unesc | fmt -80 | iconv -t US
    }

