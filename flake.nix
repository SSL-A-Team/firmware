{
  description = "The firmware repository for the SSL A-Team.";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:  
      let 
        pkgs = nixpkgs.legacyPackages.${system};
        python = "python39";
        packageName = "ateam-firmware";

      in {
        devShell = pkgs.mkShell {
          buildInputs = with pkgs; [
            gnumake
            cmake
            gcc-arm-embedded-10
            openocd
            (pkgs.${python}.withPackages
              (ps: with ps; [ numpy matplotlib ]))
          ];
        };
      }
    );
}
