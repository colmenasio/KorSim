# KorSim

KorSim is a hybrid Godot + native C++ project for flight simulation. The workspace contains a Godot project under `sim/` and a native GDExtension engine module under `engine/`.

## Repository structure

- `engine/`
  - Native C++ extension code for the Godot module.
  - `SConstruct` builds the extension using SCons and Godot C++ bindings.
  - `godot-cpp/` contains the Godot C++ binding generator and build support.
  - `src/` contains the extension sources, including the six degree-of-freedom model.
- `sim/`
  - Godot project files and scenes.
  - `project.godot` is the main Godot project configuration.
  - `bin/` contains the built `korsim_engine.gdextension` plugin.

## Key components

- `engine/src/` - native engine implementation, registration, and simulation model.
- `engine/src/model/six_dof/` - six degree-of-freedom flight model code.
- `sim/scenes/` - Godot scenes for aircraft, camera, UI, and debug tools.
- `sim/script/singleton/` - project singleton scripts and simulation entry points.

## Requirements

- Godot Engine 4.6 (or compatible with the project configuration)
- Python 3 for SCons
- SCons
- `eigen3` development headers
- A C++ compiler that supports C++17

## Build

From the repository root:

```bash
cd engine
scons
```

This builds the native extension into `sim/bin/` as a Godot GDExtension library.

## Run

Open `sim/project.godot` in Godot and run the project. The project is configured to load the native extension from `sim/bin/korsim_engine.gdextension`.

## Notes

- The native build relies on `godot-cpp` support in `engine/godot-cpp/`.
- Ensure `eigen3` headers are installed and available at `/usr/include/eigen3` or update `engine/SConstruct` accordingly.
- The project contains an in-repo Godot project and a custom simulation engine module.

## Development tips

- Rebuild the extension after changing C++ sources.
- Keep the Godot project open in the editor to refresh resources and input mappings.
- Use the `sim/` folder for asset, scene, and script changes.
