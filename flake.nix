{
  description = "The firmware repository for the SSL A-Team.";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils }:
    flake-utils.lib.eachSystem [
        # "aarch64-linux" # TODO unsupported for clang_multi, find pkgsCross package
        # "aarch64-darwin" # TODO same as above ^^^
        # "i686-linux" # gcc10 doesn't seem to have support in the Nix repos
        # "x86_64-darwin"
        "x86_64-linux" ]
    (system: 
      let 
        overlays = [ (import rust-overlay) ];

        pkgs = import nixpkgs {
          inherit system overlays; 
        };

        python = "python39";

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
            gcc-arm-embedded-10

            # programmers
            openocd
            probe-run

            # userpsace clang and link vars for bindgen subtargets
            clang_multi

            # Rust Embedded
            (rust-bin.nightly.latest.default.override {
              extensions = [ "rust-src" ];
              targets = [ "thumbv7em-none-eabihf" "thumbv6m-none-eabi" ];
            })
            rust-analyzer

            # Python
            (pkgs.${python}.withPackages
              (ps: with ps; [ numpy matplotlib ]))
          ];
        };
      }
    );
}
