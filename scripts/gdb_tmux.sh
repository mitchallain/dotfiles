#!/bin/sh

tmux new-window "gdb --args $*"
