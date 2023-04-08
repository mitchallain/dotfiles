# ~/.bash_profile: executed by the command interpreter for login shells.

# Because of this file's existence, neither ~/.bash_login nor ~/.profile
# will be sourced.

# See /usr/share/doc/bash/examples/startup-files for examples.
# The files are located in the bash-doc package.

# source .profile if it exists
if [ -r "$HOME/.profile" ]; then
. "$HOME/.profile"
fi

# The following sources ~/.bashrc in the interactive login case,
# because .bashrc isn't sourced for interactive login shells:
case "$-" in
    *i*)
        if [ -r "$HOME/.bashrc" ]; then
            . "$HOME/.bashrc"
        fi
esac

# include .bashrc if it exists
# if [ -r "$HOME/.bashrc" ]; then
# . "$HOME/.bashrc"
# fi

if [ -r "$HOME/.cargo/env" ]; then
    . "$HOME/.cargo/env"
fi
