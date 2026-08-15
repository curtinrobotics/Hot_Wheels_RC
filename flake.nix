{
  description = "Hot Wheels RC ESP32 firmware development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    arduino-nix.url = "github:bouk/arduino-nix";
    treefmt-nix.url = "github:numtide/treefmt-nix";
    pre-commit-hooks.url = "github:cachix/pre-commit-hooks.nix";
    arduino-index = {
      url = "github:bouk/arduino-indexes";
      flake = false;
    };

    # Additional Arduino package indexes (pinned via flake.lock)
    # - Espressif's official Arduino-ESP32 core
    # - Bluepad32's Arduino-ESP32 fork (needed for the Bluepad32 library)
    arduino-esp32-index = {
      url = "github:espressif/arduino-esp32?ref=gh-pages";
      flake = false;
    };
    esp32-bluepad32-index = {
      url = "github:ricardoquesada/esp32-arduino-lib-builder?ref=master";
      flake = false;
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      arduino-nix,
      treefmt-nix,
      pre-commit-hooks,
      arduino-index,
      arduino-esp32-index,
      esp32-bluepad32-index,
      ...
    }:
    let
      # arduino-nix expects each "package" entry in a package index JSON to have a
      # top-level `tools` field. The Bluepad32 index intentionally omits it (it
      # depends on Espressif's tools instead), so we add `tools: []` as a shim.
      esp32Bluepad32IndexPatched =
        let
          raw = builtins.fromJSON (
            builtins.readFile (esp32-bluepad32-index + "/bluepad32_files/package_esp32_bluepad32_index.json")
          );
          patchedPackages = map (pkg: if pkg ? tools then pkg else pkg // { tools = [ ]; }) raw.packages;
        in
        builtins.toFile "package_esp32_bluepad32_index.patched.json" (
          builtins.toJSON (raw // { packages = patchedPackages; })
        );

      overlays = [
        arduino-nix.overlay
        (arduino-nix.mkArduinoPackageOverlay (arduino-index + "/index/package_index.json"))
        (arduino-nix.mkArduinoLibraryOverlay (arduino-index + "/index/library_index.json"))

        # ESP32 package indexes requested for this project
        (arduino-nix.mkArduinoPackageOverlay (arduino-esp32-index + "/package_esp32_index.json"))
        (arduino-nix.mkArduinoPackageOverlay esp32Bluepad32IndexPatched)
      ];
    in
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = (import nixpkgs) {
          inherit system overlays;
        };

        globalExcludes = [
          ".direnv/**"
          "Hot_Wheels_Electronics/libraries/**"
        ];

        markdownlintConfig = pkgs.writeText "hot-wheels.markdownlint-cli2.jsonc" (
          builtins.toJSON {
            config = {
              MD010.code_blocks = false;
              MD013 = false;
            };
          }
        );

        markdownlintExcludeArgs = map (pattern: "!${pattern}") globalExcludes;

        markdownlint = pkgs.writeShellApplication {
          name = "markdownlint";
          runtimeInputs = [ pkgs.markdownlint-cli2 ];
          text = ''
            if [ "$#" -eq 0 ]; then
              set -- "**/*.md" ${pkgs.lib.escapeShellArgs markdownlintExcludeArgs}
            fi
            exec markdownlint-cli2 --config ${markdownlintConfig} "$@"
          '';
        };

        markdownFormat = pkgs.writeShellApplication {
          name = "markdown-format";
          runtimeInputs = [
            pkgs.markdownlint-cli2
            pkgs.prettier
          ];
          text = ''
            markdownlint-cli2 --config ${markdownlintConfig} --fix "$@" || true
            prettier --write "$@"
            markdownlint-cli2 --config ${markdownlintConfig} "$@"
          '';
        };

        markdownlintCheck =
          pkgs.runCommand "markdownlint-check" { nativeBuildInputs = [ markdownlint ]; }
            ''
              cd ${self}
              markdownlint
              touch "$out"
            '';

        treefmtCommon = {
          projectRootFile = "flake.nix";
          settings.global.excludes = globalExcludes;
        };

        treefmt = treefmt-nix.lib.evalModule pkgs (
          treefmtCommon
          // {
            programs = {
              clang-format.enable = true;
              nixfmt.enable = true;
              prettier = {
                enable = true;
                excludes = [ "*.md" ];
              };
              shellcheck.enable = true;
              shfmt.enable = true;
              statix.enable = true;
              deadnix.enable = true;
            };

            settings = treefmtCommon.settings // {
              formatter.clang-format.includes = [
                "*.c"
                "*.cc"
                "*.cpp"
                "*.h"
                "*.hh"
                "*.hpp"
                "*.ino"
              ];
              formatter.markdown = {
                command = markdownFormat;
                includes = [ "*.md" ];
              };
            };
          }
        );

        preCommit = pre-commit-hooks.lib.${system}.run {
          src = self;
          hooks = {
            nix-flake-check = {
              enable = true;
              name = "nix flake check";
              entry = "nix flake check";
              language = "system";
              pass_filenames = false;
            };
          };
        };

        arduinoCli =
          let
            bluepadPlatform = arduino-nix.latestVersion pkgs.arduinoPackages.platforms."esp32-bluepad32".esp32;
            hasBluepad32Library = pkgs.arduinoLibraries ? Bluepad32;
            bluepadLibrary =
              if hasBluepad32Library then arduino-nix.latestVersion pkgs.arduinoLibraries.Bluepad32 else null;
          in
          pkgs.wrapArduinoCLI {
            packages = [ bluepadPlatform ];
            libraries = pkgs.lib.optionals hasBluepad32Library [ bluepadLibrary ];
          };

        fqbn = "esp32-bluepad32:esp32:esp32";

        sketchSource = builtins.path {
          path = ./Hot_Wheels_arduino_firmware;
          name = "Hot_Wheels_arduino_firmware";
        };

        arduinoCompileDatabase =
          pkgs.runCommand "hot-wheels-arduino-compile-database"
            {
              nativeBuildInputs = [
                arduinoCli
                pkgs.jq
                python3
              ];
            }
            ''
              export HOME="$TMPDIR/home"
              mkdir -p "$HOME" "$out/build" "$TMPDIR/sketch/Hot_Wheels_arduino_firmware"
              cp -R ${sketchSource}/. "$TMPDIR/sketch/Hot_Wheels_arduino_firmware/"

              arduino-cli compile \
                --only-compilation-database \
                --build-path "$out/build" \
                --fqbn ${fqbn} \
                --build-property compiler.cpp.extra_flags=-I${sketchSource} \
                "$TMPDIR/sketch/Hot_Wheels_arduino_firmware"

              jq --arg sourceRoot ${sketchSource} '
                walk(
                  if type == "string"
                  then gsub($sourceRoot; "@HOT_WHEELS_WORKSPACE_ROOT@")
                  else .
                  end
                )
                |
                . as $commands
                | first(
                    $commands[]
                    | select(.file | endswith("/sketch/Hot_Wheels_arduino_firmware.ino.cpp"))
                  ) as $sketch_command
                | $commands + [($sketch_command | .file = "@HOT_WHEELS_WORKSPACE_SKETCH@")]
              ' "$out/build/compile_commands.json" > "$out/compile_commands.json"
            '';

        python3 = pkgs.python3.withPackages (ps: [ ps.pyserial ]);

        firmware =
          pkgs.runCommand "hot-wheels-firmware"
            {
              nativeBuildInputs = [
                arduinoCli
                python3
              ];
            }
            ''
              export TMPDIR="''${TMPDIR:-/tmp}"
              export HOME="$TMPDIR/hot-wheels-arduino-home"
              mkdir -p "$HOME" "$out"
              arduino-cli compile \
                --fqbn ${fqbn} \
                --output-dir "$out" \
                ${self}/Hot_Wheels_arduino_firmware/Hot_Wheels_arduino_firmware.ino
            '';

        firmwareCheck = pkgs.writeShellApplication {
          name = "firmware-check";
          runtimeInputs = [
            arduinoCli
            python3
          ];
          text = ''
            export TMPDIR="''${TMPDIR:-/tmp}"
            export HOME="$TMPDIR/hot-wheels-arduino-home"
            mkdir -p "$HOME"
            exec arduino-cli compile \
              --fqbn ${fqbn} \
              Hot_Wheels_arduino_firmware/Hot_Wheels_arduino_firmware.ino
          '';
        };

        flash = pkgs.writeShellApplication {
          name = "flash";
          runtimeInputs = [
            arduinoCli
            python3
          ];
          text = ''
            if [ "$#" -ne 1 ]; then
              echo "Usage: flash <serial-port>" >&2
              exit 2
            fi
            if [ ! -d result ]; then
              echo "No ./result build found; run 'nix build' first." >&2
              exit 1
            fi

            export TMPDIR="''${TMPDIR:-/tmp}"
            export HOME="$TMPDIR/hot-wheels-arduino-home"
            mkdir -p "$HOME"
            exec arduino-cli upload \
              --input-dir result \
              --port "$1" \
              --fqbn esp32-bluepad32:esp32:esp32 \
              --verify
          '';
        };

        monitor = pkgs.writeShellApplication {
          name = "monitor";
          runtimeInputs = [
            arduinoCli
            python3
          ];
          text = ''
            if [ "$#" -ne 1 ]; then
              echo "Usage: monitor <serial-port>" >&2
              exit 2
            fi

            export TMPDIR="''${TMPDIR:-/tmp}"
            export HOME="$TMPDIR/hot-wheels-arduino-home"
            mkdir -p "$HOME"
            exec arduino-cli monitor \
              --port "$1" \
              --config baudrate=115200
          '';
        };

      in
      {
        packages = {
          arduino-cli = arduinoCli;
          default = firmware;
          firmware-check = firmwareCheck;
          inherit
            firmware
            flash
            markdownlint
            monitor
            ;
        };

        devShells.default = pkgs.mkShell {
          # Some ESP32 tooling (upload/monitor helpers, esptool usage, etc.) uses
          # Python and expects `pyserial` (`import serial`). Provide it
          # explicitly to make the shell work out-of-the-box.
          #
          # Using python-withPackages ensures `python3`/`python` in the shell
          # refers to this interpreter (not the system one).

          packages = [
            arduinoCli
            python3
            flash
            markdownlint
            monitor
            pkgs.nixd
            pkgs.nixfmt
          ];

          shellHook = preCommit.shellHook + ''
            output="$PWD/.direnv/arduino-intellisense/compile_commands.json"
            tmp="$output.tmp"
            mkdir -p "$(dirname "$output")"
            ${pkgs.jq}/bin/jq \
              --arg root "$PWD/Hot_Wheels_arduino_firmware" \
              --arg sketch "$PWD/Hot_Wheels_arduino_firmware/Hot_Wheels_arduino_firmware.ino" \
              'walk(
                if type == "string"
                then gsub("@HOT_WHEELS_WORKSPACE_ROOT@"; $root)
                else .
                end
              )
              | map(if .file == "@HOT_WHEELS_WORKSPACE_SKETCH@" then .file = $sketch else . end)' \
              <${arduinoCompileDatabase}/compile_commands.json >"$tmp"
            if ! cmp -s "$tmp" "$output" 2>/dev/null; then
              mv "$tmp" "$output"
            else
              rm "$tmp"
            fi
            echo "Arduino dev shell ready."
            echo "Try: arduino-cli board listall | grep -i bluepad"
          '';
        };

        formatter = treefmt.config.build.wrapper;

        checks = {
          formatting = treefmt.config.build.check self;
          inherit
            arduinoCompileDatabase
            firmware
            markdownlintCheck
            ;
        };
      }
    );
}
