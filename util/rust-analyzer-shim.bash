#!/bin/bash

# VSCode can't always elaborate setting variables with the ${env:NAME} syntax,
# so it can't be programmatically directed to the non-internal rust-analyzer.
# This is problematic when the nightly version/nix version is moved forward
# and the hashed path changes, or a user wants to use a specific version on
# the path. This shell script shims the analyzer using the which command.

if ! command -v rust-analyzer; then
    echo "Rust Analyzer Shim Script Error. There is no rust-analyzer on the path."
    echo "Did you start VS code from the Nix environemnt? OR Is the rust cross compiler installed?"
    exit 1
fi

if ! which rust-analyzer | grep -q "nix"; then
    echo "Rust Analyzer Shim Script Error. rust-analyzer is on path but is not in the Nix store."
    echo "This script is intended to support the Nix environment on Linux, Mac, and WSL2"
    echo "If you are not using the Nix env, please modify the project .vscode/settings.json to set your own rust language server"
    exit 1
fi

exec $(which rust-analyzer) "$@"
