{
  description = "The firmware repository for the SSL A-Team.";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:  
      let pkgs = nixpkgs.legacyPackages.${system};

      packageName = "ateam-firmware";

      in {
	# packages.${system} = with pkgs; [ gnumake cmake gcc-arm-embedded openocd ];
        # defaultPackage = self.packages.${system}.${packageName};

        devShell = pkgs.mkShell {
          buildInputs = with pkgs; [ gnumake cmake gcc-arm-embedded-10 openocd ];
        };
      }
    );
}
