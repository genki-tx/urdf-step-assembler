#!/usr/bin/env python3
"""
Convert a ROS2 URDF visual model into a single STEP assembly (AP242/AP214).

Smoke test (local uv environment):
  1) uv sync
  2) uv run urdf-step-assembler --urdf ./path/to/model.urdf
  3) test -s ./build/urdf_assembly.step
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import trimesh
from yourdfpy import URDF

from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_GTransform,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_Sewing,
)
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.RWStl import RWStl
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Controller
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Compound, TopoDS_Face, TopoDS_Shape
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.gp import gp_GTrsf, gp_Pnt, gp_Trsf


DEFAULT_PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = DEFAULT_PROJECT_ROOT / "build/urdf_assembly.step"
DEFAULT_MESH_ROOT = DEFAULT_PROJECT_ROOT
DEFAULT_FRAMES_JSON = DEFAULT_PROJECT_ROOT / "build/urdf_assembly_frames.json"


@dataclass
class MissingMesh:
    link: str
    visual_index: int
    source_uri: str
    attempted_paths: List[str]
    reason: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a URDF visual model to a STEP assembly with one component per link."
        )
    )
    parser.add_argument("--urdf", required=True, help="URDF file path")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="Output STEP file path")
    parser.add_argument(
        "--base",
        default="auto",
        help='Base link frame for assembly placement ("auto"/"root" = URDF root)',
    )
    parser.add_argument(
        "--mesh-root",
        default=str(DEFAULT_MESH_ROOT),
        help=(
            "Base path used for package:// and fallback mesh resolution "
            "(e.g. . or ./sample_description)"
        ),
    )
    parser.add_argument(
        "--skip-missing",
        action="store_true",
        help="Skip missing meshes (otherwise fail clearly)",
    )
    parser.add_argument(
        "--global-scale",
        type=float,
        default=1000.0,
        help=(
            "Uniform scale multiplier for all translations and mesh scales "
            "(default 1000 for URDF meters -> STEP millimeter-sized geometry)"
        ),
    )
    parser.add_argument(
        "--frames-json",
        default=str(DEFAULT_FRAMES_JSON),
        help="Write debug JSON of T_world_link transforms to this path",
    )
    parser.add_argument(
        "--no-frames-json",
        action="store_true",
        help="Disable writing debug frame JSON",
    )
    parser.add_argument(
        "--schema",
        choices=["AP242", "AP214"],
        default="AP242",
        help="STEP schema",
    )
    parser.add_argument(
        "--mesh-mode",
        choices=["faceted", "solid", "auto"],
        default="faceted",
        help=(
            "Mesh-to-BRep mode: faceted=triangle surface face, "
            "solid=watertight solid only, auto=try solid then fallback faceted"
        ),
    )
    parser.add_argument(
        "--repair-mesh",
        choices=["none", "basic", "aggressive"],
        default="none",
        help=(
            "Attempt automatic mesh repair before solid conversion. "
            "Only used for --mesh-mode solid/auto."
        ),
    )
    parser.add_argument(
        "--decimate-max-faces",
        type=int,
        default=0,
        help=(
            "If >0, simplify each mesh to at most this many triangle faces before export. "
            "Useful to reduce STEP size."
        ),
    )
    return parser.parse_args()


def expand_path(path_str: str) -> Path:
    return Path(path_str).expanduser().resolve(strict=False)


def infer_workspace_src(urdf_path: Path) -> Optional[Path]:
    for parent in [urdf_path.parent, *urdf_path.parents]:
        candidate = parent / "src"
        if candidate.is_dir():
            return candidate
    return None


class MeshResolver:
    def __init__(self, urdf_path: Path, mesh_root: Path):
        self.urdf_path = urdf_path
        self.urdf_dir = urdf_path.parent
        self.mesh_root = mesh_root
        self.workspace_src = infer_workspace_src(urdf_path)

    def _case_insensitive_file(self, path: Path) -> Optional[Path]:
        if path.is_file():
            return path
        parent = path.parent
        if not parent.is_dir():
            return None
        target = path.name.lower()
        for entry in parent.iterdir():
            if entry.is_file() and entry.name.lower() == target:
                return entry
        return None

    def _fbx_to_stl(self, path: Path) -> Tuple[Optional[Path], List[Path]]:
        attempted: List[Path] = []
        stl_like = path.with_suffix(".stl")
        attempted.append(stl_like)
        direct = self._case_insensitive_file(stl_like)
        if direct is not None:
            return direct, attempted

        parent = path.parent
        if parent.is_dir():
            stem_lower = path.stem.lower()
            for entry in parent.iterdir():
                if entry.is_file() and entry.suffix.lower() == ".stl":
                    if entry.stem.lower() == stem_lower:
                        attempted.append(entry)
                        return entry, attempted
        return None, attempted

    def _candidate_paths_from_uri(self, mesh_uri: str) -> List[Path]:
        uri = mesh_uri.strip()
        if not uri:
            return []

        if uri.startswith("package://"):
            rest = uri[len("package://") :]
            if "/" not in rest:
                return []
            package_name, rel = rest.split("/", 1)
            rel_path = Path(rel)
            roots: List[Path] = []

            if self.mesh_root.name == package_name:
                roots.append(self.mesh_root)
            roots.append(self.mesh_root / package_name)
            roots.append(self.mesh_root)

            if self.workspace_src is not None:
                roots.append(self.workspace_src / package_name)

            default_pkg_flat = DEFAULT_PROJECT_ROOT / package_name
            default_pkg_src = DEFAULT_PROJECT_ROOT / "src" / package_name
            roots.append(default_pkg_flat)
            roots.append(default_pkg_src)

            try:
                from ament_index_python.packages import get_package_share_directory

                share_dir = Path(get_package_share_directory(package_name))
                roots.append(share_dir)
            except Exception:
                pass

            candidates: List[Path] = []
            seen: set[str] = set()
            for root in roots:
                candidate = (root / rel_path).expanduser().resolve(strict=False)
                key = str(candidate)
                if key not in seen:
                    seen.add(key)
                    candidates.append(candidate)
            return candidates

        path = Path(uri).expanduser()
        candidates = []
        if path.is_absolute():
            candidates.append(path.resolve(strict=False))
            return candidates

        # Standard URDF-relative path.
        candidates.append((self.urdf_dir / path).resolve(strict=False))

        # Optional mesh_root-relative fallback.
        candidates.append((self.mesh_root / path).resolve(strict=False))

        seen = set()
        unique: List[Path] = []
        for c in candidates:
            key = str(c)
            if key not in seen:
                seen.add(key)
                unique.append(c)
        return unique

    def resolve(self, mesh_uri: str) -> Tuple[Optional[Path], List[Path]]:
        candidates = self._candidate_paths_from_uri(mesh_uri)
        attempted: List[Path] = []

        for candidate in candidates:
            if candidate.suffix.lower() == ".fbx":
                stl_path, stl_attempts = self._fbx_to_stl(candidate)
                attempted.extend(stl_attempts)
                if stl_path is not None:
                    return stl_path, attempted
                continue

            attempted.append(candidate)
            resolved = self._case_insensitive_file(candidate)
            if resolved is not None:
                return resolved, attempted

        return None, attempted


def _shape_from_triangulation(path: Path, triangulation) -> TopoDS_Shape:
    if (
        triangulation is None
        or triangulation.NbNodes() <= 0
        or triangulation.NbTriangles() <= 0
    ):
        raise RuntimeError(f"Failed to parse triangulation: {path}")

    face = TopoDS_Face()
    builder = BRep_Builder()
    builder.MakeFace(face, triangulation)
    if face.IsNull():
        raise RuntimeError(f"Failed to build OCCT face from triangulation: {path}")
    return face


def load_triangle_mesh(path: Path, decimate_max_faces: int) -> trimesh.Trimesh:
    try:
        tri_mesh = trimesh.load(str(path), force="mesh")
    except Exception as exc:
        raise RuntimeError(f"Failed to load mesh with trimesh: {path}") from exc

    if tri_mesh is None or not hasattr(tri_mesh, "faces"):
        raise RuntimeError(f"Unsupported mesh object: {path}")
    if len(tri_mesh.faces) == 0:
        raise RuntimeError(f"Mesh has no triangles: {path}")
    if tri_mesh.faces.shape[1] != 3:
        raise RuntimeError(f"Non-triangle faces are not supported: {path}")

    if decimate_max_faces > 0 and len(tri_mesh.faces) > decimate_max_faces:
        try:
            simplified = tri_mesh.simplify_quadric_decimation(face_count=decimate_max_faces)
            if (
                simplified is not None
                and hasattr(simplified, "faces")
                and len(simplified.faces) > 0
                and simplified.faces.shape[1] == 3
            ):
                tri_mesh = simplified
        except Exception:
            # Keep original mesh if decimation is unavailable or fails.
            pass
    return tri_mesh


def repair_triangle_mesh(mesh: trimesh.Trimesh, repair_mode: str) -> trimesh.Trimesh:
    mode = repair_mode.lower()
    if mode == "none":
        return mesh

    repaired = mesh.copy()
    # Run trimesh cleanup pipeline first; this catches common index/normal issues.
    repaired.process(validate=True)
    repaired.remove_unreferenced_vertices()
    repaired.merge_vertices()

    try:
        trimesh.repair.fix_normals(repaired, multibody=True)
    except Exception:
        pass
    try:
        trimesh.repair.fix_inversion(repaired, multibody=True)
    except Exception:
        pass
    try:
        trimesh.repair.fill_holes(repaired)
    except Exception:
        pass

    if mode == "basic":
        return repaired

    if mode != "aggressive":
        raise ValueError(f"Unsupported repair mode: {repair_mode}")

    if repaired.is_watertight:
        return repaired

    # Aggressive fallback: convex hull guarantees a closed volume but can alter shape.
    try:
        hull = repaired.convex_hull
        if (
            hull is not None
            and hasattr(hull, "faces")
            and len(hull.faces) > 0
            and hull.is_watertight
            and abs(float(hull.volume)) > 1.0e-18
        ):
            hull.process(validate=True)
            hull.remove_unreferenced_vertices()
            hull.merge_vertices()
            return hull
    except Exception:
        pass

    return repaired


def read_mesh_shape_faceted(path: Path, decimate_max_faces: int) -> TopoDS_Shape:
    # Fast path for standard STL.
    if path.suffix.lower() == ".stl" and decimate_max_faces <= 0:
        triangulation = RWStl.ReadFile_s(str(path))
        if (
            triangulation is not None
            and triangulation.NbNodes() > 0
            and triangulation.NbTriangles() > 0
        ):
            return _shape_from_triangulation(path, triangulation)

    # Fallback: load via trimesh (e.g. DAE or odd STL), export temp STL, then read with RWStl.
    tri_mesh = load_triangle_mesh(path, decimate_max_faces=decimate_max_faces)

    tmp_path: Optional[Path] = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
            tmp_path = Path(tmp.name)
        tri_mesh.export(str(tmp_path), file_type="stl")
        triangulation = RWStl.ReadFile_s(str(tmp_path))
        return _shape_from_triangulation(path, triangulation)
    finally:
        if tmp_path is not None and tmp_path.exists():
            tmp_path.unlink()


def read_mesh_shape_solid(
    path: Path,
    require_watertight: bool,
    repair_mode: str,
    decimate_max_faces: int,
) -> TopoDS_Shape:
    tri_mesh = load_triangle_mesh(path, decimate_max_faces=decimate_max_faces)
    tri_mesh = repair_triangle_mesh(tri_mesh, repair_mode)
    if require_watertight and not tri_mesh.is_watertight:
        raise RuntimeError(
            f"Mesh is not watertight after repair='{repair_mode}', cannot convert to solid: {path}"
        )

    vertices = np.asarray(tri_mesh.vertices, dtype=float)
    faces = np.asarray(tri_mesh.faces, dtype=np.int64)

    sewing = BRepBuilderAPI_Sewing(1.0e-6, True, True, True, False)
    for idx in faces:
        p0 = vertices[int(idx[0])]
        p1 = vertices[int(idx[1])]
        p2 = vertices[int(idx[2])]

        polygon = BRepBuilderAPI_MakePolygon()
        polygon.Add(gp_Pnt(float(p0[0]), float(p0[1]), float(p0[2])))
        polygon.Add(gp_Pnt(float(p1[0]), float(p1[1]), float(p1[2])))
        polygon.Add(gp_Pnt(float(p2[0]), float(p2[1]), float(p2[2])))
        polygon.Close()
        if not polygon.IsDone():
            raise RuntimeError(f"Failed to build triangle polygon from mesh: {path}")

        make_face = BRepBuilderAPI_MakeFace(polygon.Wire(), True)
        if not make_face.IsDone():
            raise RuntimeError(f"Failed to build triangle face from mesh: {path}")
        sewing.Add(make_face.Face())

    sewing.Perform()
    sewed = sewing.SewedShape()
    if sewed.IsNull():
        raise RuntimeError(f"Sewing mesh triangles failed: {path}")

    solid_compound, solid_builder = create_empty_compound()
    solid_count = 0
    explorer = TopExp_Explorer(sewed, TopAbs_SHELL)
    while explorer.More():
        shell = TopoDS.Shell_s(explorer.Current())
        make_solid = BRepBuilderAPI_MakeSolid()
        make_solid.Add(shell)
        if make_solid.IsDone():
            solid = make_solid.Solid()
            if not solid.IsNull():
                solid_builder.Add(solid_compound, solid)
                solid_count += 1
        explorer.Next()

    if solid_count == 0:
        raise RuntimeError(f"No closed shell/solid could be produced from mesh: {path}")
    if solid_compound.IsNull():
        raise RuntimeError(f"Generated solid compound is null: {path}")
    return solid_compound


def read_mesh_shape(
    path: Path,
    mesh_mode: str,
    repair_mode: str,
    decimate_max_faces: int,
) -> Tuple[TopoDS_Shape, str]:
    mode = mesh_mode.lower()
    if mode == "faceted":
        return read_mesh_shape_faceted(path, decimate_max_faces=decimate_max_faces), "faceted"
    if mode == "solid":
        return (
            read_mesh_shape_solid(
                path,
                require_watertight=True,
                repair_mode=repair_mode,
                decimate_max_faces=decimate_max_faces,
            ),
            "solid",
        )
    if mode == "auto":
        try:
            return (
                read_mesh_shape_solid(
                    path,
                    require_watertight=True,
                    repair_mode=repair_mode,
                    decimate_max_faces=decimate_max_faces,
                ),
                "solid",
            )
        except Exception:
            return (
                read_mesh_shape_faceted(path, decimate_max_faces=decimate_max_faces),
                "faceted",
            )
    raise ValueError(f"Unsupported mesh mode: {mesh_mode}")


def matrix_to_gp_gtrsf(matrix: np.ndarray) -> gp_GTrsf:
    gtrsf = gp_GTrsf()
    for r in range(3):
        for c in range(4):
            gtrsf.SetValue(r + 1, c + 1, float(matrix[r, c]))
    return gtrsf


def matrix_to_gp_trsf(matrix: np.ndarray) -> gp_Trsf:
    trsf = gp_Trsf()
    trsf.SetValues(
        float(matrix[0, 0]),
        float(matrix[0, 1]),
        float(matrix[0, 2]),
        float(matrix[0, 3]),
        float(matrix[1, 0]),
        float(matrix[1, 1]),
        float(matrix[1, 2]),
        float(matrix[1, 3]),
        float(matrix[2, 0]),
        float(matrix[2, 1]),
        float(matrix[2, 2]),
        float(matrix[2, 3]),
    )
    return trsf


def transform_shape_general(shape: TopoDS_Shape, matrix: np.ndarray) -> TopoDS_Shape:
    gtrsf = matrix_to_gp_gtrsf(matrix)
    transformer = BRepBuilderAPI_GTransform(shape, gtrsf, True)
    if not transformer.IsDone():
        raise RuntimeError("Failed to apply visual affine transform to mesh shape")
    return transformer.Shape()


def create_empty_compound() -> Tuple[TopoDS_Compound, BRep_Builder]:
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    return compound, builder


def visual_local_matrix(origin: np.ndarray, mesh_scale: np.ndarray, global_scale: float) -> np.ndarray:
    local = np.array(origin, dtype=float, copy=True)
    local[:3, 3] *= global_scale
    scale = np.array(mesh_scale, dtype=float, copy=True) * global_scale
    scale_matrix = np.eye(4, dtype=float)
    scale_matrix[0, 0] = scale[0]
    scale_matrix[1, 1] = scale[1]
    scale_matrix[2, 2] = scale[2]
    return local @ scale_matrix


def get_mesh_scale(mesh_scale_value: Optional[Sequence[float]]) -> np.ndarray:
    if mesh_scale_value is None:
        return np.ones(3, dtype=float)
    arr = np.array(mesh_scale_value, dtype=float).reshape(-1)
    if arr.size == 1:
        return np.array([arr[0], arr[0], arr[0]], dtype=float)
    if arr.size != 3:
        raise ValueError(f"Unsupported mesh scale size: {arr.size}")
    return arr


def set_zero_joint_cfg(urdf: URDF) -> None:
    cfg = np.array(urdf.cfg, dtype=float).reshape(-1)
    if cfg.size > 0:
        urdf.update_cfg(np.zeros_like(cfg))


def compute_link_transforms(urdf: URDF, base_link: str, global_scale: float) -> Dict[str, np.ndarray]:
    transforms: Dict[str, np.ndarray] = {}
    identity = np.eye(4, dtype=float)
    for link in urdf.robot.links:
        if link.name == base_link:
            tf = identity.copy()
        else:
            tf, _ = urdf.scene.graph.get(frame_from=base_link, frame_to=link.name)
            tf = np.array(tf, dtype=float, copy=True)
        tf[:3, 3] *= global_scale
        transforms[link.name] = tf
    return transforms


def schema_value(schema: str) -> str:
    upper = schema.upper()
    if upper == "AP214":
        return "AP214IS"
    return "AP242DIS"


def build_step_assembly(args: argparse.Namespace) -> int:
    urdf_path = expand_path(args.urdf)
    out_path = expand_path(args.out)
    mesh_root = expand_path(args.mesh_root)
    frames_json_path = None if args.no_frames_json else expand_path(args.frames_json)

    if not urdf_path.is_file():
        print(f"ERROR: URDF does not exist: {urdf_path}", file=sys.stderr)
        return 2
    if args.global_scale <= 0.0:
        print("ERROR: --global-scale must be > 0", file=sys.stderr)
        return 2
    if args.decimate_max_faces < 0:
        print("ERROR: --decimate-max-faces must be >= 0", file=sys.stderr)
        return 2

    resolver = MeshResolver(urdf_path=urdf_path, mesh_root=mesh_root)
    urdf = URDF.load(str(urdf_path), load_meshes=False, build_scene_graph=True)
    set_zero_joint_cfg(urdf)

    root_link = urdf.base_link
    if args.base.lower() in {"auto", "root"}:
        base_link = root_link
    else:
        base_link = args.base
    if base_link not in urdf.link_map:
        print(f"ERROR: base link '{base_link}' not found in URDF", file=sys.stderr)
        return 2

    transforms = compute_link_transforms(urdf, base_link=base_link, global_scale=args.global_scale)

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("urdf_to_step_doc"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    root_shape, root_builder = create_empty_compound()
    root_label = shape_tool.AddShape(root_shape, True)
    TDataStd_Name.Set_s(root_label, TCollection_ExtendedString(f"{base_link}_assembly"))
    # Keep a reference to avoid lint/static warnings for the builder.
    _ = root_builder

    missing_meshes: List[MissingMesh] = []
    total_meshes = 0
    links_with_visuals = 0
    solid_meshes = 0
    faceted_meshes = 0

    for link in urdf.robot.links:
        link_shape, link_builder = create_empty_compound()
        exported_meshes_in_link = 0

        visuals = link.visuals or []
        for visual_index, visual in enumerate(visuals):
            if visual.geometry is None or visual.geometry.mesh is None:
                continue

            mesh = visual.geometry.mesh
            mesh_uri = mesh.filename or ""
            resolved_mesh, attempted = resolver.resolve(mesh_uri)
            if resolved_mesh is None:
                missing_meshes.append(
                    MissingMesh(
                        link=link.name,
                        visual_index=visual_index,
                        source_uri=mesh_uri,
                        attempted_paths=[str(p) for p in attempted],
                    )
                )
                continue

            try:
                shape, mode_used = read_mesh_shape(
                    resolved_mesh,
                    args.mesh_mode,
                    args.repair_mesh,
                    args.decimate_max_faces,
                )
            except Exception as exc:
                missing_meshes.append(
                    MissingMesh(
                        link=link.name,
                        visual_index=visual_index,
                        source_uri=mesh_uri,
                        attempted_paths=[str(resolved_mesh)],
                        reason=str(exc),
                    )
                )
                continue

            origin = (
                np.eye(4, dtype=float)
                if visual.origin is None
                else np.array(visual.origin, dtype=float)
            )
            mesh_scale = get_mesh_scale(mesh.scale)
            local_matrix = visual_local_matrix(origin, mesh_scale, args.global_scale)

            try:
                transformed = transform_shape_general(shape, local_matrix)
                link_builder.Add(link_shape, transformed)
            except Exception as exc:
                if args.mesh_mode == "auto" and mode_used == "solid":
                    try:
                        fallback_shape = read_mesh_shape_faceted(
                            resolved_mesh,
                            decimate_max_faces=args.decimate_max_faces,
                        )
                        transformed = transform_shape_general(fallback_shape, local_matrix)
                        link_builder.Add(link_shape, transformed)
                        mode_used = "faceted"
                    except Exception as fallback_exc:
                        missing_meshes.append(
                            MissingMesh(
                                link=link.name,
                                visual_index=visual_index,
                                source_uri=mesh_uri,
                                attempted_paths=[str(resolved_mesh)],
                                reason=(
                                    f"solid transform failed: {exc}; "
                                    f"faceted fallback failed: {fallback_exc}"
                                ),
                            )
                        )
                        continue
                else:
                    missing_meshes.append(
                        MissingMesh(
                            link=link.name,
                            visual_index=visual_index,
                            source_uri=mesh_uri,
                            attempted_paths=[str(resolved_mesh)],
                            reason=str(exc),
                        )
                    )
                    continue

            total_meshes += 1
            exported_meshes_in_link += 1
            if mode_used == "solid":
                solid_meshes += 1
            else:
                faceted_meshes += 1

        if exported_meshes_in_link > 0:
            links_with_visuals += 1

        part_label = shape_tool.AddShape(link_shape, False)
        TDataStd_Name.Set_s(part_label, TCollection_ExtendedString(link.name))

        placement = TopLoc_Location(matrix_to_gp_trsf(transforms[link.name]))
        instance_label = shape_tool.AddComponent(root_label, part_label, placement)
        TDataStd_Name.Set_s(instance_label, TCollection_ExtendedString(link.name))

    if missing_meshes and not args.skip_missing:
        print("ERROR: Missing mesh files detected:", file=sys.stderr)
        for miss in missing_meshes:
            print(
                f"  - link='{miss.link}' visual={miss.visual_index} uri='{miss.source_uri}'",
                file=sys.stderr,
            )
            if miss.attempted_paths:
                print("    attempted:", file=sys.stderr)
                for attempted in miss.attempted_paths:
                    print(f"      {attempted}", file=sys.stderr)
            if miss.reason:
                print(f"    reason: {miss.reason}", file=sys.stderr)
        print("Use --skip-missing to ignore missing meshes.", file=sys.stderr)
        return 3

    out_path.parent.mkdir(parents=True, exist_ok=True)
    STEPControl_Controller.Init_s()
    Interface_Static.SetCVal_s("write.step.schema", schema_value(args.schema))
    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    writer.SetColorMode(False)
    writer.SetLayerMode(False)
    writer.SetPropsMode(False)
    shape_tool.UpdateAssemblies()

    if not writer.Transfer(doc, STEPControl_AsIs):
        print("ERROR: STEP transfer failed", file=sys.stderr)
        return 4
    status = writer.Write(str(out_path))
    if int(status) != int(IFSelect_RetDone):
        print(f"ERROR: STEP write failed (status={int(status)})", file=sys.stderr)
        return 4

    if frames_json_path is not None:
        frames_json_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "base_link": base_link,
            "urdf_root_link": root_link,
            "global_scale": args.global_scale,
            "link_count": len(urdf.robot.links),
            "frames": {name: mat.tolist() for name, mat in transforms.items()},
        }
        with frames_json_path.open("w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

    print(f"Base link: {base_link}")
    print(f"URDF root link: {root_link}")
    print(f"Number of links: {len(urdf.robot.links)}")
    print(f"Links with visuals exported: {links_with_visuals}")
    print(f"Meshes processed: {total_meshes}")
    print(f"Global scale: {args.global_scale}")
    print(f"Mesh mode requested: {args.mesh_mode}")
    print(f"Mesh repair mode: {args.repair_mesh}")
    print(f"Mesh decimate max faces: {args.decimate_max_faces}")
    print(f"Meshes exported as solids: {solid_meshes}")
    print(f"Meshes exported as faceted surfaces: {faceted_meshes}")
    print(f"STEP output: {out_path}")
    if frames_json_path is not None:
        print(f"Frames JSON: {frames_json_path}")

    if missing_meshes:
        print("Missing meshes (skipped):")
        for miss in missing_meshes:
            print(f"  - link='{miss.link}' visual={miss.visual_index} uri='{miss.source_uri}'")
            for attempted in miss.attempted_paths:
                print(f"    {attempted}")
            if miss.reason:
                print(f"    reason: {miss.reason}")
    else:
        print("Missing meshes: none")

    return 0


def main() -> int:
    args = parse_args()
    return build_step_assembly(args)


if __name__ == "__main__":
    sys.exit(main())
