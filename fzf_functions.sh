# fzf_functions.sh
#
# bash functions using the fzf fuzzy finder
#
# TODO
# - check if fzf is installed and exit file
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
  # echo $ssid
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


