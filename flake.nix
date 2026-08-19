{
  description = "Python ros2_controller for Agimus";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [
          "humble"
          "jazzy"
        ];
        rosOverrideAttrs.agimus-pytroller = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
              ./agimus_pytroller_py
              ./CMakeLists.txt
              ./controller_plugins.xml
              ./include
              ./package.xml
              ./README.md
              ./src
            ];
          };
        };
      }
    );
}
