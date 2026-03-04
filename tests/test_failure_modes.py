import subprocess
import sys
from pathlib import Path


def test_cli_requires_urdf_argument() -> None:
    cmd = [sys.executable, "-m", "urdf_step_assembler.cli"]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)

    assert result.returncode == 2
    assert "--urdf" in result.stderr


def test_bad_urdf_path_returns_error(tmp_path: Path) -> None:
    out_step = tmp_path / "bad.step"
    cmd = [
        sys.executable,
        "-m",
        "urdf_step_assembler.cli",
        "--urdf",
        str(tmp_path / "does_not_exist.urdf"),
        "--out",
        str(out_step),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)

    assert result.returncode == 2
    assert "ERROR: URDF does not exist" in result.stderr


def test_missing_mesh_without_skip_missing_fails(tmp_path: Path) -> None:
    urdf_path = tmp_path / "broken.urdf"
    out_step = tmp_path / "broken.step"
    urdf_path.write_text(
        """<?xml version="1.0"?>
<robot name="broken_robot">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="meshes/not_here.stl"/>
      </geometry>
    </visual>
  </link>
</robot>
""",
        encoding="utf-8",
    )

    cmd = [
        sys.executable,
        "-m",
        "urdf_step_assembler.cli",
        "--urdf",
        str(urdf_path),
        "--out",
        str(out_step),
        "--mesh-root",
        str(tmp_path),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)

    assert result.returncode == 3
    assert "ERROR: Missing mesh files detected" in result.stderr
