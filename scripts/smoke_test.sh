#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URDF_PATH="$ROOT_DIR/examples/sample_robot/sample_robot.urdf"
OUT_STEP="$ROOT_DIR/build/sample_robot_assembly.step"
OUT_FRAMES="$ROOT_DIR/build/sample_robot_assembly_frames.json"

mkdir -p "$ROOT_DIR/build"

echo "[smoke] running urdf-step-assembler on sample_robot..."
uv run urdf-step-assembler \
  --urdf "$URDF_PATH" \
  --out "$OUT_STEP" \
  --mesh-root "$ROOT_DIR" \
  --frames-json "$OUT_FRAMES"

test -s "$OUT_STEP"
test -s "$OUT_FRAMES"

echo "[smoke] PASS"
echo "[smoke] step:   $OUT_STEP"
echo "[smoke] frames: $OUT_FRAMES"
