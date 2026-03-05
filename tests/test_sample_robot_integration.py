import json
import subprocess
import sys
from pathlib import Path


def test_sample_robot_generates_step_and_frames(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    urdf = repo_root / "examples" / "sample_robot" / "sample_robot.urdf"
    out_step = tmp_path / "sample_robot.step"
    out_frames = tmp_path / "sample_robot_frames.json"

    cmd = [
        sys.executable,
        "-m",
        "urdf_step_assembler.cli",
        "--urdf",
        str(urdf),
        "--out",
        str(out_step),
        "--frames-json",
        str(out_frames),
        "--mesh-root",
        str(repo_root),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)

    assert result.returncode == 0, result.stderr + "\n" + result.stdout
    assert out_step.exists()
    assert out_step.stat().st_size > 0
    assert out_frames.exists()
    assert out_frames.stat().st_size > 0

    payload = json.loads(out_frames.read_text(encoding="utf-8"))
    assert payload["link_count"] == 2
    assert "base_link" in payload["frames"]
    assert "link1" in payload["frames"]


def test_sample_robot_generates_step_in_solid_mode(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    urdf = repo_root / "examples" / "sample_robot" / "sample_robot.urdf"
    out_step = tmp_path / "sample_robot_solid.step"

    cmd = [
        sys.executable,
        "-m",
        "urdf_step_assembler.cli",
        "--urdf",
        str(urdf),
        "--out",
        str(out_step),
        "--mesh-root",
        str(repo_root),
        "--mesh-mode",
        "solid",
        "--no-frames-json",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)

    assert result.returncode == 0, result.stderr + "\n" + result.stdout
    assert out_step.exists()
    assert out_step.stat().st_size > 0
    assert "Mesh mode requested: solid" in result.stdout
    assert "Meshes exported as solids: 2" in result.stdout


def test_sample_robot_generates_step_with_decimation_flag(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    urdf = repo_root / "examples" / "sample_robot" / "sample_robot.urdf"
    out_step = tmp_path / "sample_robot_decimated.step"

    cmd = [
        sys.executable,
        "-m",
        "urdf_step_assembler.cli",
        "--urdf",
        str(urdf),
        "--out",
        str(out_step),
        "--mesh-root",
        str(repo_root),
        "--mesh-mode",
        "faceted",
        "--decimate-max-faces",
        "8",
        "--no-frames-json",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)

    assert result.returncode == 0, result.stderr + "\n" + result.stdout
    assert out_step.exists()
    assert out_step.stat().st_size > 0
    assert "Mesh decimate max faces: 8" in result.stdout
