{
  description = "The firmware repository for the SSL A-Team.";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";

    uv2nix = {
      url = "github:pyproject-nix/uv2nix";
      inputs.pyproject-nix.follows = "pyproject-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    pyproject-nix = {
      url = "github:pyproject-nix/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils, uv2nix, pyproject-nix }:
    flake-utils.lib.eachSystem [
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
        "x86_64-linux" ]
    (system:
      let
        overlays = [ (import rust-overlay) ];

        pkgs = import nixpkgs {
          inherit system overlays;
        };

        python = pkgs.python312;

        # Load the uv workspace from pyproject.toml + uv.lock at the repo root.
        workspace = uv2nix.lib.workspace.loadWorkspace { workspaceRoot = ./.; };

        # Build a Nix overlay from the resolved lockfile.
        # sourcePreference = "wheel" avoids building packages from source where
        # pre-built wheels are available (faster, and avoids needing build tools
        # for packages like numpy that ship compiled wheels).
        overlay = workspace.mkPyprojectOverlay { sourcePreference = "wheel"; };

        # Some packages in the lockfile declare setuptools/wheel as their PEP 517
        # build backend but uv2nix doesn't resolve those as runtime deps.  This
        # overlay bridges them from nixpkgs so the pyproject-nix resolver can find
        # them when walking the dependency graph.
        buildToolOverlay = _final: _prev: {
          setuptools = python.pkgs.setuptools;
          wheel      = python.pkgs.wheel;
        };

        # The workspace root is a virtual package (package = false in pyproject.toml)
        # — there is no Python source to build or install.  Override its derivation
        # to be a no-op so pyproject-nix doesn't try to invoke setuptools on an
        # empty source tree.
        virtualRootOverlay = _final: prev: {
          ateam-firmware-scripts = prev.ateam-firmware-scripts.overrideAttrs (_old: {
            dontBuild   = true;
            installPhase = "true";
          });
        };

        pythonSet = (pkgs.callPackage pyproject-nix.build.packages {
          inherit python;
        }).overrideScope (
          pkgs.lib.composeManyExtensions [
            buildToolOverlay
            overlay
            virtualRootOverlay
          ]
        );

        # The virtualenv that will be injected into the dev shell.
        pythonEnv = pythonSet.mkVirtualEnv "ateam-firmware-env" workspace.deps.default;

        packageName = "ateam-firmware";

      in {
        devShell = pkgs.mkShell {
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang}/lib/libclang.so";

          shellHook = ''
          export LIBCLANG_PATH="${pkgs.libclang.lib}/lib"
          '';

          buildInputs = with pkgs; [
            # C/C++ build utils
            gnumake
            cmake
            gcc-arm-embedded-14

            # programmers
            dfu-util
            openocd
            probe-rs

            # userpsace clang and link vars for bindgen subtargets
            clang

            # Rust Embedded
            (rust-bin.selectLatestNightlyWith (toolchain: toolchain.default.override {
              extensions = [ "rust-src" "rust-analyzer" ];
              targets = [ "thumbv7em-none-eabihf" "thumbv6m-none-eabi" ];
            }))

            # Python environment managed by uv2nix (see pyproject.toml + uv.lock)
            pythonEnv

            # uv is included so developers can add/update dependencies
            uv
          ];
        };
      }
    );
}
