{
  description = "The firmware repository for the SSL A-Team.";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils }:
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

        python = "python312";

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
            gcc-arm-embedded-12

            # programmers
            openocd
            probe-rs

            # userpsace clang and link vars for bindgen subtargets
            clang

            # Rust Embedded
            (rust-bin.nightly."2024-12-10".default.override {
              extensions = [ "rust-src" "rust-analyzer" ];
              targets = [ "thumbv7em-none-eabihf" "thumbv6m-none-eabi" ];
            })

            # Python
            (pkgs.${python}.withPackages
              (ps: with ps; [ numpy matplotlib ]))
          ];
        };
      }
    );
}
