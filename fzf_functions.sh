#!/bin/bash
# fzf_functions.sh
#
# bash functions using the fzf fuzzy finder
#
# TODO
# - check if fzf is installed and exit this file early
#

# fwifi
# -----
# fuzzy wifi switcher using nmcli
fwifi () {
  local ssid
  ssid=$(
    nmcli -f 'bssid,signal,bars,freq,ssid' --color yes device wifi |
    fzf \
      --with-nth=1.. \
      --ansi \
      --height=40% \
      --reverse \
      --cycle \
      --inline-info \
      --header-lines=1 \
      | awk '{print $1}' ) || return
  [[ -z "$ssid" ]] && return
  nmcli -a device wifi connect $ssid
}

# fwhich
# ------
# fuzzy search all bash functions
# getting pretty meta bruh
# never could get preview of function source working
# see https://github.com/junegunn/fzf/issues/1337
fwhich () {
  local bashfns
  bashfn=$(
    typeset -F |
    fzf \
      --with-nth=3 \
      --preview 'source ~/.bashrc; declare -f {3}' \
      | awk '{print $3}' ) || return
  which $bashfn
}
# tm               : create new tmux session, or switch to existing one with fzf
#                  : works from within tmux too (@bag-man)
#                  : (optional) with an arg that names an existing session,
#                  : switch to that session, else create it
# -----------------:-----------------------------------------------------------------
tm() {
  [[ -n "$TMUX" ]] && change="switch-client" || change="attach-session"
  if [ $1 ]; then
    tmux $change -t "$1" 2>/dev/null || (tmux new-session -d -s $1 && tmux $change -t "$1"); return
  fi
  session=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | fzf --exit-0) &&  tmux $change -t "$session" || echo "No sessions found."
}

# FUZZY GIT
# --------------

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

# fbr              : checkout git branch (including remote branches),
#                  : sorted by most recent commit, limit 30 last branches
# -----------------:-----------------------------------------------------------------
fbr() {
  local branches branch
  branches=$(git for-each-ref --count=30 --sort=-committerdate refs/ --format="%(refname:short)") &&
  branch=$(echo "$branches" |
           fzf-tmux -d $(( 2 + $(wc -l <<< "$branches") )) +m) &&
  git checkout $(echo "$branch" | sed "s/.* //" | sed "s#remotes/[^/]*/##")
}

# fco              : checkout git branch/tag
# -----------------:-----------------------------------------------------------------
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


# fco_preview      : checkout git branch/tag, with a preview showing the commits
#                  : between the tag/branch and HEAD
# -----------------:-----------------------------------------------------------------
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

# fshow            : git commit browser
# -----------------:-----------------------------------------------------------------
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

# fcoc_preview     : checkout git commit with previews
# -----------------:-----------------------------------------------------------------
fcoc_preview() {
  local commit
  commit=$( glNoGraph |
    fzf --no-sort --reverse --tiebreak=index --no-multi \
    --ansi --preview="$_viewGitLogLine" ) &&
    git checkout $(echo "$commit" | sed "s/ .*//")
}

# fshow_preview - git commit browser with previews
# -----------------:-----------------------------------------------------------------
fshow_preview() {
  glNoGraph |
    fzf --no-sort --reverse --tiebreak=index --no-multi \
    --ansi --preview="$_viewGitLogLine" \
    --header "enter to view, alt-y to copy hash" \
    --bind "enter:execute:$_viewGitLogLine   | less -R" \
    --bind "alt-y:execute:$_gitLogLineToHash | xclip"
}

# ---------------------
# FUZZY CHROME
# ---------------------

# fbook - fuzzy browse chrome bookmarks
alias fbook="$HOME/bin/fbook.rb"

# fhist            : search chrome browser history
# -----------------:-----------------------------------------------------------------
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

# fman             : global man page search
# -----------------:------------------------------------------------------------
fman() {
  man -k . | fzf --prompt='Man> ' | awk '{print $1}' | xargs -r man
}

# fnotes           : fuzzy full-text search markdown notes and open in sublime
#                  : TODO test against obsidian notes
# -----------------:------------------------------------------------------------
fnotes() {
  bdir=$pwd
  ndir="/home/mallain/Dropbox/BMR-MA/Notable/notes"
  cd $ndir > /dev/null 2>&1

  # VSCODE DEFAULT
  # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && cat -n {1}" | cut -d':' -f1-2 | xargs -d '\n' code -g

  # SUBLIME DEFAULT
  ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && cat -n {1}" | cut -d':' -f1-2 | xargs -d '\n' subl

  cd $bdir > /dev/null 2>&1
}

# fthis            : fuzzy full-text search current directory and open in editor
# -----------------:------------------------------------------------------------
fthis() {
  # VSCODE DEFAULT
  # ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && cat -n {1}" | cut -d':' -f1-2 | xargs -d '\n' code -g

  # SUBLIME DEFAULT
  ag --nobreak --noheading . | fzf -m --delimiter=":" --preview "set -x && cat -n {1}" | cut -d':' -f1-2 | xargs -d '\n' subl
}

# fif              : find-in-file - usage: fif <searchTerm>
# -----------------:------------------------------------------------------------
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

# fsubld           : open directory in sublime
# -----------------:-----------------------------------------------------------------
fsubld() {
  local dirs
  dirs=$(find ${1:-.} -path '*/\.*' -prune \
    -o -type d -print 2> /dev/null | fzf -m --preview="tree -L 2 {}") &&
  for var in $dirs; do
    (subl "$var" > /dev/null 2>&1 &)
  done
}

# fopen            : open one or more files with xdg-open
#                  : use tab to multi-select in fzf view
# -----------------:-----------------------------------------------------------------
fopen() {
  IFS=$'\n' files=($(fzf-tmux --query="$1" --multi --select-1 --exit-0))
  [[ -n "$files" ]] && xopen "${files[@]}"
}

# fedit            : Select a file from fuzzy listing to edit
#                  :   - CTRL-O to open subl
#                  :   - CTRL-E or Enter key to open with the $EDITOR
# -----------------:-----------------------------------------------------------------
fedit () {
  IFS=$'\n' out=("$(fzf-tmux --query="$1" --exit-0 --expect=ctrl-o,ctrl-e)")
  key=$(head -1 <<< "$out")
  file=$(head -2 <<< "$out" | tail -1)
  if [ -n "$file" ]; then
    [ "$key" = ctrl-o ] && subl "$file" || ${EDITOR:-vim} "$file"
  fi
}

# fe               : Open selected file(s) with vim in multiple tabs
#                  :   - Bypass fuzzy finder if there's only one match (--select-1)
#                  :   - Exit if there's no match (--exit-0)
# -----------------:-----------------------------------------------------------------
fe() {
  IFS=$'\n' files=($(fzf-tmux --query="$1" --multi --select-1 --exit-0))
  # [[ -n "$files" ]] && ${EDITOR:-vim} "${files[@]}"

  # use vim over $EDITOR so that -p option can be passed reliably
  [[ -n "$files" ]] && vim -p  "${files[@]}"
}

# fkill            : kill processes - list only the ones you can kill
# -----------------:-----------------------------------------------------------------
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


# ---------------------
# FUZZY ROS
# ---------------------
frostopic() {
  local topic=$(rostopic list | fzf --preview "rostopic info {}")
  [[ -z "$topic" ]] && return
  rostopic echo $@ $topic
}

frosnode() {
  local node=$(rosnode list | fzf --preview "rosnode info -q {}")
  [[ -z "$node" ]] && return
  rosnode info -q $@ $node
}

frospack() {
  local pack=$(rospack list | fzf --preview "cat {2}/package.xml" | awk '{print $2}')
  cd $pack
}

frosparam() {
  local param=$(rosparam list | fzf --preview "rosparam get {}")
  [[ -z "$param" ]] && return
  rosparam get $param
}

