# Changelog

## 0.2.0

- Added `--mesh-mode` options for `faceted`, `solid`, and `auto`.
- Added `--repair-mesh` (`none`/`basic`/`aggressive`) for optional pre-solid mesh repair.
- Added `--decimate-max-faces` to simplify meshes and reduce STEP output size.
- Improved `auto` behavior to fall back to faceted when solid transform fails.
- Changed default `--global-scale` to `1000.0` (URDF meters to millimeter-sized STEP geometry).
- Added global scale to CLI summary output.
- Added tests for repair and decimation-related paths.
- Updated README with production recommendations and advanced option caveats.

## 0.1.0

- Initial OSS release: URDF visual mesh to STEP assembly converter.
