#!/bin/bash

DF_DIR="~/dotfiles"

ln -s $DF_DIR/tmux.conf ~/.tmux.conf
ln -s $DF_DIR/vimrcs/vimrc ~/.vimrc
ln -s $DF_DIR/vimrcs ~/.vimrcs
ln -s $DF_DIR/nvim ~/.config/nvim

