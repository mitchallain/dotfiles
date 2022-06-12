# executed only by login bash shells
# login shells are Mac OS X default

# MAC OS X Only
# See ~/.bashrc for os conditional definitions

if [ -n "$BASH_VERSION" ]; then
    # include .bashrc if it exists
    if [ -f "$HOME/.bashrc" ]; then
	. "$HOME/.bashrc"
    fi
fi

