import subprocess
import sys
from pathlib import Path

import trimesh


def test_repair_basic_closes_simple_hole_for_solid_mode(tmp_path: Path) -> None:
    mesh_path = tmp_path / "meshes" / "open_box.stl"
    mesh_path.parent.mkdir(parents=True, exist_ok=True)

    mesh = trimesh.creation.box(extents=[0.1, 0.1, 0.1])
    mesh.update_faces([i for i in range(len(mesh.faces) - 1)])
    mesh.remove_unreferenced_vertices()
    mesh.export(str(mesh_path), file_type="stl")

    urdf_path = tmp_path / "open_box.urdf"
    urdf_path.write_text(
        """<?xml version="1.0"?>
<robot name="open_box_robot">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="meshes/open_box.stl"/>
      </geometry>
    </visual>
  </link>
</robot>
""",
        encoding="utf-8",
    )

    out_no_repair = tmp_path / "no_repair.step"
    cmd_no_repair = [
        sys.executable,
        "-m",
        "urdf_step_assembler.cli",
        "--urdf",
        str(urdf_path),
        "--out",
        str(out_no_repair),
        "--mesh-root",
        str(tmp_path),
        "--mesh-mode",
        "solid",
        "--repair-mesh",
        "none",
        "--no-frames-json",
    ]
    result_no_repair = subprocess.run(
        cmd_no_repair, capture_output=True, text=True, check=False
    )
    assert result_no_repair.returncode == 3
    assert "not watertight" in result_no_repair.stderr

    out_basic_repair = tmp_path / "basic_repair.step"
    cmd_basic_repair = [
        sys.executable,
        "-m",
        "urdf_step_assembler.cli",
        "--urdf",
        str(urdf_path),
        "--out",
        str(out_basic_repair),
        "--mesh-root",
        str(tmp_path),
        "--mesh-mode",
        "solid",
        "--repair-mesh",
        "basic",
        "--no-frames-json",
    ]
    result_basic_repair = subprocess.run(
        cmd_basic_repair, capture_output=True, text=True, check=False
    )

    assert result_basic_repair.returncode == 0, (
        result_basic_repair.stderr + "\n" + result_basic_repair.stdout
    )
    assert out_basic_repair.exists()
    assert out_basic_repair.stat().st_size > 0
