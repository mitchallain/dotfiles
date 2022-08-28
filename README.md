# dotfiles

This repository tracks my bash shell configuration and the configuration of many
tools that I use with Ubuntu (18.04).

This repository uses the awesome
[dotbot](https://github.com/anishathalye/dotbot) tool to "bootstrap" my
dotfiles. I primarily use it for creating and updating the symlinks from my home
folder to the `~/.dotfiles` folder containing this repo, rather than using it to
set up new systems.

## `private` submodule

The private directory submodule is hosted separately, and contains configuration
specific to my work. The `~/.bashrc` file checks the existence of most of these
files prior to sourcing them and as such, should succeed without the private
directory present.

## References

Credit to Nathaniel Landau's
[awesome article "MY MAC OSX BASH PROFILE"](https://natelandau.com/my-mac-osx-bash_profile/)
which provided much of the inspiration for my `~/.bashrc`, despite being geared
towards Mac OS X.
