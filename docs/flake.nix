# This flake offers a shell environment with Python and UV. To access it:
#
#     nix develop
{
  description = "A Python dev environment using UV.";
  
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            python3        # Python 3
            uv             # Python dependency manager
            figlet         # ASCII art welcome messages
          ];
          shellHook = ''
            #!/usr/bin/env bash
            
            # Create and activate the virtual environment
            export VIRTUAL_ENV_DISABLE_PROMPT=1
            test -d .venv || ${pkgs.uv}/bin/uv venv .venv
            source .venv/bin/activate

            # Create a fancy welcome message
            REPO_NAME=$(basename "$PWD")
            PROPER_REPO_NAME=$(echo "$REPO_NAME" | awk '{print toupper(substr($0,1,1)) tolower(substr($0,2))}')
            figlet "$PROPER_REPO_NAME"
            echo "Welcome to the $PROPER_REPO_NAME development environment on ${system}!"
            echo 

            # Install Python packages from requirements.txt using uv
            # This allows flexibility to use the latest PyPI packages
            # Note: This makes the environment less deterministic
            echo "- Installing pip packages with uv..."
            if uv pip install --upgrade pip --quiet && \
              uv pip install -r requirements.txt --quiet; then
                package_count=$(uv pip list --format=freeze | wc -l)
                echo "- Done. $package_count pip packages installed."
            else
                echo "Warning: An error occurred during uv pip setup."
            fi
          '';
        };
      }
    );
}
