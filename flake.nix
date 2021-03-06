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
        # "i686-linux" # gcc10 doesn't seem to have support in the Nix repos
        "x86_64-darwin"
        "x86_64-linux" ]
    (system: 
      let 
        overlays = [ (import rust-overlay) ];
        # pkgs = nixpkgs.legacyPackages.${system};
        pkgs = import nixpkgs {
          inherit system overlays;
        };
        python = "python39";
        packageName = "ateam-firmware";

      in {
        devShell = pkgs.mkShell {

          buildInputs = with pkgs; [
            # C/C++ build utils
            gnumake
            cmake
            gcc-arm-embedded-10
            openocd

            # Rust Embedded
            rust-bin.stable.latest.default
            #rust-bin.stable.latest.default.override {
            #  extensions = [ "rust-src" ];
            #  targets = [ "thumbv7em-none-eabihf" ];
            #}

            # Python
            (pkgs.${python}.withPackages
              (ps: with ps; [ numpy matplotlib ]))
          ];
        };
      }
    );
}
