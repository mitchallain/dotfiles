# Get default config with cmake-format --dump-config python

with section("format"):
    # If a statement is wrapped to more than one line, than dangle the closing
    # parenthesis on its own line.
    dangle_parens = True

    # If an argument group contains more than this many sub-groups (parg or kwarg
    # groups) then force it to a vertical layout.
    max_subgroups_hwrap = 3

    # If a positional argument group contains more than this many arguments, then
    # force it to a vertical layout.
    max_pargs_hwrap = 3

