#!/bin/bash

# VSCode can't always elaborate setting variables with the ${env:NAME} syntax,
# so it can't be programmatically directed to the non-internal rust-analyzer.
# This is problematic when the nightly version/nix version is moved forward
# and the hashed path changes, or a user wants to use a specific version on
# the path. This shell script shims the analyzer using the which command.

exec $(which rust-analyzer) "$@"
