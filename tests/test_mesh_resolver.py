from pathlib import Path

from urdf_step_assembler.cli import MeshResolver


def _make_resolver(tmp_path: Path) -> MeshResolver:
    urdf_path = tmp_path / "robot" / "model.urdf"
    urdf_path.parent.mkdir(parents=True, exist_ok=True)
    urdf_path.write_text("<robot name='dummy'/>", encoding="utf-8")
    return MeshResolver(urdf_path=urdf_path, mesh_root=tmp_path)


def test_resolve_relative_case_insensitive_stl(tmp_path: Path) -> None:
    resolver = _make_resolver(tmp_path)
    rel_mesh = tmp_path / "robot" / "meshes" / "base_link.STL"
    rel_mesh.parent.mkdir(parents=True, exist_ok=True)
    rel_mesh.write_text("solid x\nendsolid x\n", encoding="utf-8")

    resolved, attempts = resolver.resolve("meshes/base_link.stl")

    assert resolved == rel_mesh
    assert attempts


def test_resolve_package_uri(tmp_path: Path) -> None:
    resolver = _make_resolver(tmp_path)
    mesh = tmp_path / "sample_description" / "meshes" / "part.STL"
    mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.write_text("solid x\nendsolid x\n", encoding="utf-8")

    resolved, _ = resolver.resolve("package://sample_description/meshes/part.stl")

    assert resolved == mesh


def test_resolve_fbx_maps_to_stl(tmp_path: Path) -> None:
    resolver = _make_resolver(tmp_path)
    mesh = tmp_path / "sample_description" / "meshes" / "wheel.STL"
    mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.write_text("solid x\nendsolid x\n", encoding="utf-8")

    resolved, attempts = resolver.resolve("package://sample_description/meshes/wheel.fbx")

    assert resolved == mesh
    assert any(str(p).lower().endswith("wheel.stl") for p in attempts)


def test_resolve_missing_returns_none(tmp_path: Path) -> None:
    resolver = _make_resolver(tmp_path)

    resolved, attempts = resolver.resolve("package://sample_description/meshes/missing.fbx")

    assert resolved is None
    assert attempts
