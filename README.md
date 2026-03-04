# urdf-step-assembler

Convert a URDF visual model into a single STEP assembly (AP242/AP214) with one component per link.

The tool preserves URDF link-frame semantics for CAD assembly:
- each link becomes one STEP component
- each component origin is the URDF link frame origin
- component placement is from URDF forward kinematics at joint=0
- visual meshes are applied in link-local coordinates (`visual.origin` + `mesh.scale`)

## Why this tool

This project is for URDF-to-CAD assembly export when origin/frame correctness matters more than watertight solids.
It is designed to produce a standard STEP assembly with named components that can be opened in major CAD applications.

## Features

- URDF parsing with `yourdfpy`
- Forward kinematics at default pose (all joints = 0)
- All visuals per link are supported (`visuals[*]`)
- `package://`, relative, and absolute mesh URI resolution
- FBX URI fallback to same-name STL (`.stl`/`.STL`/case-insensitive match)
- Case-insensitive STL filename matching on Linux filesystems
- STEP assembly export via OpenCascade XCAF (no FreeCAD dependency)
- Optional debug JSON dump of `T_world_link`

## Requirements

- Python `>=3.10,<3.13`
- [uv](https://docs.astral.sh/uv/)
- OS/runtime capable of installing `cadquery-ocp` wheels

ROS 2 is **not** required to run this tool.

## Install

```bash
uv sync
```

For development/tests:

```bash
uv sync --group dev
```

## Quick Start

Run on the included sample URDF:

```bash
uv run urdf-step-assembler \
  --urdf ./examples/sample_robot/sample_robot.urdf \
  --out ./build/sample_robot_assembly.step
```

Expected outputs:
- `./build/sample_robot_assembly.step`
- `./build/urdf_assembly_frames.json` (default debug frames file)

## Usage

```bash
uv run urdf-step-assembler --urdf <path/to/model.urdf> [options]
```

### Main options

- `--urdf` (required): input URDF file
- `--out` (default `./build/urdf_assembly.step`): output STEP path
- `--base` (default `auto`): base link (`auto`/`root` uses URDF root link)
- `--mesh-root` (default `.`): root for package/fallback mesh lookup
- `--skip-missing`: skip missing/unloadable meshes instead of failing
- `--global-scale` (default `1.0`): multiplies all translations and mesh scales
- `--frames-json` (default `./build/urdf_assembly_frames.json`): debug transform output
- `--no-frames-json`: disable frames JSON output
- `--schema` (`AP242` default, or `AP214`): STEP schema selection

## Mesh Resolution Rules

For mesh `filename` in URDF visuals:

- `package://<pkg>/<relpath>`:
  - tries `<mesh-root>/<pkg>/<relpath>`
  - tries `<mesh-root>/<relpath>`
  - tries common workspace/package fallbacks
  - optionally tries `ament_index_python` if available
- relative path:
  - first relative to URDF directory
  - then relative to `--mesh-root`
- absolute path:
  - used directly

### FBX handling

FBX is not imported directly into OCCT in this tool.

If a URI ends with `.fbx`, resolver attempts the same basename with STL extensions/casing and directory scan (case-insensitive stem match). If no STL is found:
- fail with clear diagnostics (default), or
- continue when `--skip-missing` is set

## Coordinate/Assembly Semantics

- World frame is the selected base link frame.
- If `--base auto` (default), URDF root link is at world origin.
- Each link part geometry is built in link-local frame.
- Link instance placement in assembly is `T_world_link`.

This ensures CAD component origins match URDF link frame origins.

## Smoke Test

```bash
./scripts/smoke_test.sh
```

This runs conversion on `examples/sample_robot` and verifies output files are non-empty.

## Tests

```bash
uv sync --group dev
uv run pytest -q
```

Current tests cover:
- mesh path resolver behavior (`package://`, case variants, FBX->STL fallback)
- sample robot integration conversion
- CLI failure paths (missing `--urdf`, missing files, missing mesh without skip)

## Project Layout

```text
src/urdf_step_assembler/   Python package + CLI
examples/sample_robot/     Public sample URDF + meshes
tests/                     Pytest suite
scripts/smoke_test.sh      Manual smoke runner
```

## License

This project is licensed under the terms in `LICENSE.txt`.
