#!/usr/bin/env python3
"""Build the mesh assets shipped with the rebot_b601 module, for one arm variant.

Pulls the Seeed description package for the chosen variant (dm or rs) at a
pinned commit and writes, under src/rebot_b601/assets/<variant subdir>/:

  meshes/<link>.stl     decimated binary STL collision mesh per link (< 150 KB)
  models/<link>.glb     per-link visual GLB, URDF <visual> parts merged, flat
                        per-part colours from the URDF material, or guessed from
                        the part filename for packages without materials (< 300 KB)
  primitives.json       axis-aligned bounding box of the ORIGINAL collision
                        mesh per link, in the link frame, metres; plus
                        triangle counts and provenance
  ATTRIBUTION.md        source, commit, licence, file list

Re-runnable. Usage:

  .venv/bin/python tools/build_assets.py [--variant dm|rs] [--source DIR] [--out DIR]

With --source, meshes/URDF already present in DIR are used as-is; anything the
URDF references that is missing there (typically the visual meshes) is
downloaded from GitHub into DIR. Without --source a cache directory under
~/.cache is used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO_ROOT / "src" / "rebot_b601" / "assets"

STL_CAP_BYTES = 150 * 1024
GLB_CAP_BYTES = 300 * 1024

# Measured on the built GLBs: a decimated, vertex-shared mesh costs about 18 bytes per
# face once positions, normals and indices are packed. The cap loop below shrinks an
# overshoot, so erring generous here just means using the byte budget we paid for.
GLB_BYTES_PER_FACE = 18

# The smallest triangle count a shell can be decimated to and still be a solid. Sampling
# the RS shells: cut to 48 faces they keep 84-100% of their volume, at 32 that is 50-90%,
# and by 8 faces it is 0-40% -- a collapsed sliver. Shells smaller than this are kept whole.
MIN_COMPONENT_FACES = 48

# The DM package names its finishes in the part filenames instead of declaring URDF
# materials. Ordered: the first matching token wins, so the more specific names come first.
COLOR_RULES: list[tuple[str, str]] = [
    ("travel_stop_yellow", "#F2C200"),
    ("seeed_yellow", "#F2C200"),
    ("finger_black", "#202020"),
    ("hardware_black", "#202020"),
    ("matte_black", "#202020"),
    ("carriage_grey", "#8A8A8A"),
    ("anodized_grey", "#8A8A8A"),
    ("rack_metal", "#C0C0C0"),
    ("silver_trim", "#C0C0C0"),
    ("metal", "#C0C0C0"),
]
FALLBACK_COLOR = "#8A8A8A"


@dataclass
class Variant:
    """One arm variant's upstream description package and its output layout."""

    name: str
    source_repo: str
    source_commit: str
    source_subdir: str
    urdf_name: str
    out_subdir: str  # "" for dm, "rs" for rs
    links: dict[str, str]  # collision mesh stem -> URDF link name
    rename: dict[str, str]  # vendor link name -> bundled link name (GLB filenames, primitives' urdf_link)
    blurb: str  # one line for ATTRIBUTION.md
    color_rules: list[tuple[str, str]]  # filename token -> hex, for packages without URDF materials
    licence: list[str]  # the licence paragraph for ATTRIBUTION.md

    @property
    def raw_base(self) -> str:
        owner_repo = self.source_repo.removeprefix("https://github.com/")
        return f"https://raw.githubusercontent.com/{owner_repo}/{self.source_commit}/{self.source_subdir}/"


# Output STLs and primitives.json are keyed by the collision mesh stem (as shipped by
# Seeed); GLBs by the bundled link name (that is what get_kinematics / Get3DModels
# address), which is the URDF link name after `rename`.
VARIANTS: dict[str, Variant] = {
    "dm": Variant(
        name="dm",
        source_repo="https://github.com/Seeed-Projects/reBot-DevArm",
        source_commit="b0acdcfc47843de16a9f018c6ab1de1d31649fdc",
        source_subdir="Rebot_Arm_description/DM",
        urdf_name="ReBot_Arm_DM.urdf",
        out_subdir="",
        links={
            "base_link": "base_link",
            "link1": "link1",
            "link2": "link2",
            "link3": "link3",
            "link4": "link4",
            "link5": "link5",
            "link6": "link6",
            "gripper_base": "end_link",
            "left_finger": "finger_left_link",
            "right_finger": "finger_right_link",
        },
        rename={},
        blurb="Seeed Studio reBot-DevArm B601-DM description package",
        color_rules=COLOR_RULES,
        licence=[
            "The upstream repository licenses its hardware design files (including these",
            "meshes and the URDF) under the CERN Open Hardware Licence Version 2 - Weakly",
            "Reciprocal, and its software under the Apache License 2.0:",
            "",
            "- Hardware files (meshes, URDF): `SPDX-License-Identifier: CERN-OHL-W-2.0`",
            "- Code: `SPDX-License-Identifier: Apache-2.0`",
        ],
    ),
    "rs": Variant(
        name="rs",
        source_repo="https://github.com/Seeed-Projects/reBotArm_control_py",
        source_commit="76512eab38ba54f11830e7cdfcba68e11629f902",
        source_subdir="urdf/RS",
        urdf_name="ReBot_Arm_RS.urdf",
        out_subdir="rs",
        links={
            "base_link": "base_link",
            "link1": "link1",
            "link2": "link2",
            "link3": "link3",
            "link4": "link4",
            "link5": "link5",
            "link6": "link6",
            "gripper_end": "gripper_end",
            "gripper_left": "gripper_left",
            "gripper_right": "gripper_right",
        },
        rename={
            "gripper_end": "end_link",
            # STLs and primitives are keyed by the vendor mesh stem, GLBs by the renamed link.
            # arm_3d_models asks for the DM finger names, so RS must produce the same filenames.
            "gripper_left": "finger_left_link",
            "gripper_right": "finger_right_link",
        },
        blurb="Seeed Studio reBotArm_control_py B601-RS description package (urdf/RS)",
        color_rules=[],  # the RS URDF declares its finishes as <material> elements
        licence=[
            "The upstream repository ships no LICENSE file at the pinned commit, and no",
            "SPDX header appears in the URDF or the meshes. Seeed Studio publishes the",
            "sibling reBot-DevArm description package under the CERN Open Hardware Licence",
            "Version 2 - Weakly Reciprocal for hardware files and the Apache License 2.0",
            "for code, so these RS hardware files are treated the same way:",
            "",
            "- Hardware files (meshes, URDF): `SPDX-License-Identifier: CERN-OHL-W-2.0`",
            "  (assumed; upstream has not stated one for this package)",
            "- Code: `SPDX-License-Identifier: Apache-2.0`",
        ],
    ),
}

V: Variant = VARIANTS["dm"]  # replaced in main() from --variant


# --------------------------------------------------------------------------- utils


def log(msg: str) -> None:
    print(msg, file=sys.stderr)


def hex_to_rgba(h: str) -> list[int]:
    h = h.lstrip("#")
    return [int(h[i : i + 2], 16) for i in (0, 2, 4)] + [255]


def color_for(filename: str) -> tuple[str, str]:
    """Return (rule_token, hex) for a visual part filename."""
    stem = Path(filename).stem.lower()
    for token, hexc in V.color_rules:
        if token in stem:
            return token, hexc
    return "fallback", FALLBACK_COLOR


def rgba_to_hex(rgba: str) -> str:
    r, g, b = (int(round(float(v) * 255)) for v in rgba.split()[:3])
    return f"#{r:02X}{g:02X}{b:02X}"


def color_for_part(part: VisualPart) -> tuple[str, str]:
    """Name-token rule first (keeps the DM GLBs byte-identical), then the URDF material, then grey."""
    token, hexc = color_for(part.rel)
    if token != "fallback":
        return token, hexc
    if part.material_rgba:
        return f"material:{part.material}", rgba_to_hex(part.material_rgba)
    return "fallback", FALLBACK_COLOR


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(rel: str, dest: Path) -> bool:
    """Download the variant's raw_base/rel to dest if dest does not exist. Returns success."""
    if dest.exists() and dest.stat().st_size > 0:
        return True
    url = V.raw_base + rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    try:
        log(f"  download {url}")
        with urllib.request.urlopen(url, timeout=60) as r:
            data = r.read()
    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as e:
        log(f"  FAILED {rel}: {e}")
        return False
    dest.write_bytes(data)
    return True


def rpy_to_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """URDF origin (xyz, rpy fixed-axis roll/pitch/yaw) -> 4x4."""
    r, p, y = rpy
    Rx = trimesh.transformations.rotation_matrix(r, [1, 0, 0])
    Ry = trimesh.transformations.rotation_matrix(p, [0, 1, 0])
    Rz = trimesh.transformations.rotation_matrix(y, [0, 0, 1])
    T = Rz @ Ry @ Rx
    T[:3, 3] = xyz
    return T


def parse_origin(el: ET.Element | None) -> np.ndarray:
    if el is None:
        return np.eye(4)
    xyz = [float(v) for v in el.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in el.get("rpy", "0 0 0").split()]
    return rpy_to_matrix(xyz, rpy)


def urdf_rel_to_source(fn: str) -> str:
    """'../meshes/visual/x.stl' (relative to the package's urdf/ dir) -> 'meshes/visual/x.stl'."""
    fn = fn.replace("package://", "")
    parts = [p for p in Path(fn).parts if p not in ("..", ".")]
    if "meshes" in parts:
        parts = parts[parts.index("meshes") :]
    return "/".join(parts)


# --------------------------------------------------------------------------- URDF


@dataclass
class VisualPart:
    rel: str  # path relative to the package source root, e.g. meshes/visual/x.stl
    origin: np.ndarray
    scale: np.ndarray
    material: str | None
    material_rgba: str | None


@dataclass
class LinkSpec:
    urdf_link: str
    stem: str
    collision_rel: str
    collision_origin: np.ndarray
    collision_scale: np.ndarray
    visuals: list[VisualPart] = field(default_factory=list)


def parse_urdf(urdf_path: Path) -> dict[str, LinkSpec]:
    root = ET.parse(urdf_path).getroot()
    materials = {
        m.get("name"): m.find("color").get("rgba")
        for m in root.iter("material")
        if m.get("name") and m.find("color") is not None
    }
    by_urdf = {v: k for k, v in V.links.items()}
    specs: dict[str, LinkSpec] = {}
    for link in root.iter("link"):
        name = link.get("name")
        if name not in by_urdf:
            continue
        stem = by_urdf[name]
        col = link.find("collision")
        if col is None:
            raise SystemExit(f"URDF link {name} has no <collision>")
        cmesh = col.find("geometry/mesh")
        spec = LinkSpec(
            urdf_link=name,
            stem=stem,
            collision_rel=urdf_rel_to_source(cmesh.get("filename")),
            collision_origin=parse_origin(col.find("origin")),
            collision_scale=np.array([float(v) for v in cmesh.get("scale", "1 1 1").split()]),
        )
        for vis in link.findall("visual"):
            vmesh = vis.find("geometry/mesh")
            if vmesh is None:
                continue
            mat = vis.find("material")
            mat_name = mat.get("name") if mat is not None else None
            inline = mat.find("color") if mat is not None else None
            spec.visuals.append(
                VisualPart(
                    rel=urdf_rel_to_source(vmesh.get("filename")),
                    origin=parse_origin(vis.find("origin")),
                    scale=np.array([float(v) for v in vmesh.get("scale", "1 1 1").split()]),
                    material=mat_name,
                    material_rgba=inline.get("rgba") if inline is not None else materials.get(mat_name),
                )
            )
        specs[stem] = spec
    missing = set(V.links) - set(specs)
    if missing:
        raise SystemExit(f"URDF is missing expected links: {sorted(missing)}")
    return specs


# --------------------------------------------------------------------------- mesh ops


def load_mesh(path: Path) -> trimesh.Trimesh:
    m = trimesh.load(str(path), force="mesh", process=False)
    if not isinstance(m, trimesh.Trimesh) or len(m.faces) == 0:
        raise ValueError(f"{path} did not load as a triangle mesh")
    return m


def detect_unit_scale(meshes: dict[str, trimesh.Trimesh]) -> tuple[float, str]:
    """Return (scale_to_metres, note). Source is expected in metres."""
    longest = max(float(m.extents.max()) for m in meshes.values())
    link2 = float(meshes["link2"].extents.max()) if "link2" in meshes else longest
    if longest > 50.0:
        return 0.001, (f"source meshes look like millimetres (largest extent {longest:.1f}); scaled by 0.001 to metres")
    note = f"source meshes are in metres (largest extent {longest:.3f} m, link2 longest extent {link2:.3f} m)"
    if not (0.20 <= link2 <= 0.40):
        note += " -- WARNING: link2 extent outside the expected 0.26-0.30 m band"
    return 1.0, note


def _quadric(mesh: trimesh.Trimesh, face_count: int) -> trimesh.Trimesh:
    face_count = max(int(face_count), 4)
    out = mesh.simplify_quadric_decimation(face_count=face_count)
    out.remove_unreferenced_vertices()
    if len(out.faces) == 0:
        raise ValueError("decimation produced an empty mesh")
    return out


def components_of(mesh: trimesh.Trimesh) -> list[trimesh.Trimesh]:
    """The mesh's disconnected shells, largest first.

    merge_vertices first: STL stores three unshared vertices per triangle, so
    without welding every triangle is its own component and quadric decimation
    has no shared edge to collapse -- it just deletes triangles, which is how
    the collision meshes turned into loose shards. Shells under 4 faces cannot
    bound a volume (these packages leave hundreds of 1- and 2-triangle slivers),
    so they are dropped rather than spent budget on.
    """
    mesh.merge_vertices()
    comps = [c for c in mesh.split(only_watertight=False, repair=False) if len(c.faces) >= 4]
    comps.sort(key=lambda c: len(c.faces), reverse=True)
    return comps


def _decimate_components(mesh: trimesh.Trimesh, target_faces: int) -> tuple[trimesh.Trimesh, str]:
    """Decimate shell by shell so small parts survive instead of dissolving.

    These are CAD assemblies of many closed shells (fasteners, bosses, cable
    glands). Decimated as one mesh with a proportional face budget, a tiny shell
    gets a share of a few triangles and collapses into loose rubble. So every
    shell is first taken down to its viability floor, and only what is left of
    the budget is shared out across the shells in proportion to their detail.
    """
    comps = components_of(mesh)
    if len(comps) <= 1:
        return _quadric(mesh, target_faces), "quadric"

    # The floor is measured, not assumed: some shells refuse to decimate that far
    # (fast_simplification stops once no edge collapse is valid) and a 2148-face
    # shell that bottoms out at 1840 would otherwise blow the budget silently.
    floors = [(c, c if len(c.faces) <= MIN_COMPONENT_FACES else _quadric(c, MIN_COMPONENT_FACES)) for c in comps]

    # Largest shell first, so when the floors do not all fit (link2 has 143 shells
    # against a 3062-face STL budget) what falls off the end is the fine detail.
    kept: list[tuple[trimesh.Trimesh, trimesh.Trimesh]] = []
    spent = 0
    for c, floor in floors:
        if spent + len(floor.faces) <= target_faces:
            kept.append((c, floor))
            spent += len(floor.faces)
    if not kept:  # target under a single shell's floor: nothing sensible to keep apart
        return _quadric(mesh, target_faces), "quadric"

    surplus = target_faces - spent
    headroom = sum(len(c.faces) - len(floor.faces) for c, floor in kept)
    out = []
    for c, floor in kept:
        extra = surplus * (len(c.faces) - len(floor.faces)) // headroom if headroom else 0
        out.append(_quadric(c, len(floor.faces) + extra) if extra else floor)
    if len(kept) < len(comps):
        log(f"    kept {len(kept)} of {len(comps)} shells within the {target_faces}-face budget")
    return trimesh.util.concatenate(out), "components"


def decimate_to_faces(mesh: trimesh.Trimesh, target_faces: int) -> tuple[trimesh.Trimesh, str]:
    """Decimate to about target_faces. Returns (mesh, method)."""
    try:
        if len(mesh.faces) <= target_faces:
            out, method = mesh, "unchanged"
        else:
            out, method = _decimate_components(mesh, target_faces)
        # Sweep the slivers once more: decimation sheds the odd orphan triangle of its
        # own, and those loose shards are what the 3D view was showing.
        comps = components_of(out)
        return (trimesh.util.concatenate(comps) if comps else out), method
    except Exception as e:  # noqa: BLE001 - any failure means: fall back to the hull
        log(f"  decimation failed ({type(e).__name__}: {e}); using convex hull")
        return mesh.convex_hull, "convex_hull"


def stl_bytes(mesh: trimesh.Trimesh) -> bytes:
    return mesh.export(file_type="stl")  # trimesh's STL export is binary


def decimate_collision(mesh: trimesh.Trimesh) -> tuple[trimesh.Trimesh, str, bytes]:
    """Decimate a collision mesh until its binary STL is under STL_CAP_BYTES."""
    # Binary STL: 84-byte header + 50 bytes/triangle.
    budget = (STL_CAP_BYTES - 84) // 50 - 8
    method = "unchanged"
    out = mesh
    for _ in range(8):
        out, method = decimate_to_faces(mesh, budget)
        data = stl_bytes(out)
        if len(data) < STL_CAP_BYTES or method == "convex_hull":
            break
        budget = int(budget * 0.9)
    if len(data) >= STL_CAP_BYTES:
        log("  still over the STL cap after decimation; using convex hull")
        out, method = mesh.convex_hull, "convex_hull"
        data = stl_bytes(out)
    return out, method, data


def build_glb(parts: list[tuple[str, trimesh.Trimesh, str]], link_name: str) -> tuple[bytes, dict]:
    """parts: [(part_name, mesh_in_link_frame, hex_colour)]. Decimates jointly
    until the GLB is under GLB_CAP_BYTES. Returns (glb_bytes, report)."""
    total_faces = sum(len(m.faces) for _, m, _ in parts)
    budget = min(total_faces, GLB_CAP_BYTES // GLB_BYTES_PER_FACE)
    methods: dict[str, str] = {}
    data = b""
    for _ in range(10):
        scene = trimesh.Scene()
        for name, m, hexc in parts:
            share = max(MIN_COMPONENT_FACES, int(budget * len(m.faces) / max(total_faces, 1)))
            dm, method = decimate_to_faces(m, share)
            methods[name] = method
            dm = dm.copy()
            dm.visual = trimesh.visual.TextureVisuals(
                material=trimesh.visual.material.PBRMaterial(
                    name=name,
                    baseColorFactor=hex_to_rgba(hexc),
                    metallicFactor=0.2,
                    roughnessFactor=0.7,
                )
            )
            scene.add_geometry(dm, node_name=name, geom_name=name)
        data = scene.export(file_type="glb")
        if len(data) < GLB_CAP_BYTES:
            break
        # Proportional correction, so a small overshoot costs a small trim, not 15%.
        budget = int(budget * GLB_CAP_BYTES * 0.97 / len(data))
    faces_out = sum(len(g.faces) for g in scene.geometry.values())
    return data, {
        "faces_original": total_faces,
        "faces_decimated": faces_out,
        "methods": methods,
    }


# --------------------------------------------------------------------------- main


def main() -> int:
    global V

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", choices=list(VARIANTS), default="dm", help="arm variant to build (default dm)")
    ap.add_argument(
        "--source",
        type=Path,
        default=None,
        help="directory holding the Seeed description package (URDF + meshes/). Missing "
        "files are downloaded into it. Default: ~/.cache/viam-rebot-b601/seeed-<variant>-<commit>",
    )
    ap.add_argument("--out", type=Path, default=None, help=f"output dir (default {DEFAULT_OUT}/<variant subdir>)")
    args = ap.parse_args()

    V = VARIANTS[args.variant]
    source: Path = args.source or (
        Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
        / "viam-rebot-b601"
        / f"seeed-{V.name}-{V.source_commit[:12]}"
    )
    source.mkdir(parents=True, exist_ok=True)
    out: Path = args.out or (DEFAULT_OUT / V.out_subdir)
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    (out / "models").mkdir(parents=True, exist_ok=True)

    log(f"source: {source}")
    log(f"out:    {out}")

    # URDF: accept <pkg>/ReBot_Arm_*.urdf or <pkg>/urdf/ReBot_Arm_*.urdf.
    urdf_path = source / V.urdf_name
    if not urdf_path.exists():
        urdf_path = source / "urdf" / V.urdf_name
        if not urdf_path.exists() and not fetch(f"urdf/{V.urdf_name}", urdf_path):
            raise SystemExit("could not obtain the URDF")
    for aux in ("LICENSE", "README.md"):
        fetch(aux, source / aux)
    specs = parse_urdf(urdf_path)

    # Collision meshes (required).
    for spec in specs.values():
        p = source / spec.collision_rel
        if not p.exists():
            # Case-insensitive match for .stl/.STL already on disk.
            cand = [q for q in p.parent.glob("*") if q.name.lower() == p.name.lower()] if p.parent.exists() else []
            if cand:
                spec.collision_rel = str(cand[0].relative_to(source))
            elif not fetch(spec.collision_rel, p):
                raise SystemExit(f"could not obtain collision mesh {spec.collision_rel}")

    # Visual meshes (optional; skipped with a report on failure).
    skipped_visuals: list[str] = []
    for spec in specs.values():
        for v in spec.visuals:
            if not fetch(v.rel, source / v.rel):
                skipped_visuals.append(v.rel)

    # Load collision meshes, decide units.
    raw: dict[str, trimesh.Trimesh] = {s: load_mesh(source / spec.collision_rel) for s, spec in specs.items()}
    unit_scale, unit_note = detect_unit_scale(raw)
    log(f"units: {unit_note}")

    primitives: dict = {
        "units": "m",
        "frame": "URDF link frame (collision <origin> applied)",
        "bbox_source": "original (undecimated) collision mesh",
        "variant": V.name,
        "source_repo": V.source_repo,
        "source_commit": V.source_commit,
        "source_subdir": V.source_subdir,
        "unit_scale_applied": unit_scale,
        "unit_note": unit_note,
        "links": {},
        "triangles": {},
        "decimation": {},
    }
    file_rows: list[tuple[str, int, str]] = []  # (relpath, bytes, provenance)
    fallbacks: list[str] = []

    # ---- collision STLs + primitives
    for stem, spec in specs.items():
        mesh = raw[stem].copy()
        T = spec.collision_origin.copy()
        S = np.diag(list(spec.collision_scale * unit_scale) + [1.0])
        mesh.apply_transform(T @ S)
        lo, hi = mesh.bounds
        center = ((lo + hi) / 2.0).tolist()
        size = (hi - lo).tolist()
        dec, method, data = decimate_collision(mesh)
        dest = out / "meshes" / f"{stem}.stl"
        dest.write_bytes(data)
        if method == "convex_hull":
            fallbacks.append(f"meshes/{stem}.stl: convex hull")
        primitives["links"][stem] = {
            "urdf_link": V.rename.get(spec.urdf_link, spec.urdf_link),
            "center": [round(c, 6) for c in center],
            "size": [round(s, 6) for s in size],
        }
        primitives["triangles"][stem] = {"original": len(mesh.faces), "decimated": len(dec.faces)}
        primitives["decimation"][stem] = method
        file_rows.append((f"meshes/{stem}.stl", len(data), f"decimated from {spec.collision_rel}"))
        log(
            f"  {stem:13s} faces {len(mesh.faces):6d} -> {len(dec.faces):5d} ({method}), "
            f"{len(data) / 1024:6.1f} KB, center {np.round(center, 4).tolist()} size {np.round(size, 4).tolist()}"
        )

    # ---- visual GLBs
    glb_report: dict = {}
    for spec in specs.values():
        out_link = V.rename.get(spec.urdf_link, spec.urdf_link)
        parts: list[tuple[str, trimesh.Trimesh, str]] = []
        for v in spec.visuals:
            if v.rel in skipped_visuals:
                continue
            m = load_mesh(source / v.rel)
            # STL stores 3 unshared vertices per triangle; share them so the
            # GLB indexes vertices instead of repeating them (geometry unchanged).
            m.merge_vertices()
            m.apply_transform(v.origin @ np.diag(list(v.scale * unit_scale) + [1.0]))
            _token, hexc = color_for_part(v)
            parts.append((Path(v.rel).stem, m, hexc))
        if not parts:
            log(f"  {out_link}: no visual parts available, skipping GLB")
            fallbacks.append(f"models/{out_link}.glb: not built (no visual meshes)")
            continue
        data, rep = build_glb(parts, out_link)
        dest = out / "models" / f"{out_link}.glb"
        dest.write_bytes(data)
        hulls = [n for n, m in rep["methods"].items() if m == "convex_hull"]
        if hulls:
            fallbacks.append(f"models/{out_link}.glb: convex hull for {hulls}")
        rep["parts"] = {
            Path(v.rel).stem: {"source": v.rel, "color": color_for_part(v)[1], "urdf_material": v.material}
            for v in spec.visuals
            if v.rel not in skipped_visuals
        }
        rep["bytes"] = len(data)
        glb_report[out_link] = rep
        file_rows.append((f"models/{out_link}.glb", len(data), f"merged {len(parts)} visual part(s)"))
        log(
            f"  {out_link:17s} faces {rep['faces_original']:6d} -> {rep['faces_decimated']:5d}, "
            f"{len(data) / 1024:6.1f} KB"
        )

    primitives["visual_models"] = glb_report
    primitives["skipped_visual_meshes"] = skipped_visuals
    prim_path = out / "primitives.json"
    prim_path.write_text(json.dumps(primitives, indent=2) + "\n")
    file_rows.append(("primitives.json", prim_path.stat().st_size, "generated"))

    # ---- ATTRIBUTION.md
    src_files = sorted(
        {spec.collision_rel for spec in specs.values()}
        | {v.rel for spec in specs.values() for v in spec.visuals if v.rel not in skipped_visuals}
    )
    lines = [
        "# Asset attribution",
        "",
        f"The meshes in this directory are derived from the {V.blurb}.",
        "",
        f"- Source repository: {V.source_repo}",
        f"- Pinned commit: `{V.source_commit}`",
        f"- Package path: `{V.source_subdir}/` (URDF `urdf/{V.urdf_name}`, meshes under `meshes/`)",
        f"- Rebuilt with `tools/build_assets.py --variant {V.name}` (trimesh {trimesh.__version__})",
        "",
        "## Licence",
        "",
        *V.licence,
        "",
        "The meshes are redistributed unmodified apart from decimation (triangle count",
        "reduction) and format conversion (binary STL; visual parts merged per link",
        "into GLB with flat colours). Geometry, units (metres) and link frames are",
        "unchanged from the source. A copy of the upstream LICENSE text, where the",
        "source package ships one, accompanies it; this notice is the required",
        "attribution.",
        "",
        "## Units and frames",
        "",
        f"- {unit_note}.",
        "- Every mesh is expressed in its URDF link frame, with the URDF `<origin>`",
        "  of the visual/collision element applied.",
        "",
        "## Generated files",
        "",
        "| File | Size | Notes |",
        "|---|---:|---|",
    ]
    for rel, nbytes, note in file_rows:
        lines.append(f"| `{rel}` | {nbytes / 1024:.1f} KB | {note} |")
    lines += ["", "## Upstream source files used", ""]
    for f in src_files:
        p = source / f
        digest = sha256_of(p)[:16] if p.exists() else "missing"
        lines.append(f"- `{V.source_subdir}/{f}` (sha256 {digest}...)")
    if skipped_visuals:
        lines += ["", "## Skipped (download failed)", ""] + [f"- `{f}`" for f in skipped_visuals]
    lines.append("")
    (out / "ATTRIBUTION.md").write_text("\n".join(lines))

    # ---- summary
    log("")
    log("== files ==")
    for rel, nbytes, _ in file_rows:
        log(f"  {rel:32s} {nbytes / 1024:7.1f} KB")
    log(f"  {'ATTRIBUTION.md':32s} {(out / 'ATTRIBUTION.md').stat().st_size / 1024:7.1f} KB")
    log("== bounding boxes (m, original collision mesh, link frame) ==")
    for stem, e in primitives["links"].items():
        log(f"  {stem:13s} center {e['center']}  size {e['size']}")
    log(f"== units == {unit_note}")
    log("== fallbacks ==")
    for f in fallbacks or ["none"]:
        log(f"  {f}")
    if skipped_visuals:
        log("== skipped visual meshes ==")
        for f in skipped_visuals:
            log(f"  {f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
