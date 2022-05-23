#!/bin/bash

sudo add-apt-repository ppa:zeal-developers/ppa

sudo apt update

sudo apt install \
tmux \              # session-based terminal multiplexer
zeal                # offline documentation browser

# fzf - fuzzy command line finder
git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install

# gitbatch - multiple git repository management tool
gitbatchv = "0.5.0"
cd ~/Downloads
curl -OL https://github.com/isacikgoz/gitbatch/releases/download/vi${gitbatchv}/gitbatch_${gitbatchv}_linux_amd64.tar.gz
tar xzf gitbatch_${gitbatchv}_linux_amd64.tar.gz
sudo mv gitbatch /usr/local/bin

# albert - alfred clone for linux (spotlight replacement)
# https://albertlauncher.github.io/installing/
# https://software.opensuse.org/download.html?project=home:manuelschneid3r&package=albert
echo 'deb http://download.opensuse.org/repositories/home:/manuelschneid3r/xUbuntu_18.04/ /' | sudo tee /etc/apt/sources.list.d/home:manuelschneid3r.list
curl -fsSL https://download.opensuse.org/repositories/home:manuelschneid3r/xUbuntu_18.04/Release.key | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/home_manuelschneid3r.gpg > /dev/null
sudo apt update
sudo apt install albert

# bat - cat alternative
# https://github.com/sharkdp/bat#installation
sudo apt install bat
mkdir -p ~/.local/bin
ln -s /usr/bin/batcat ~/.local/bin/bat

# exa - ls alternative with extended attributes
sudo apt install exa


## NEOVIM
sudo apt install neovim

sh -c 'curl -fLo "${XDG_DATA_HOME:-$HOME/.local/share}"/nvim/site/autoload/plug.vim --create-dirs \
       https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim'





