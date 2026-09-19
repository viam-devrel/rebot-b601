# Asset attribution

The meshes in this directory are derived from the Seeed Studio reBotArm_control_py B601-RS description package (urdf/RS).

- Source repository: https://github.com/Seeed-Projects/reBotArm_control_py
- Pinned commit: `76512eab38ba54f11830e7cdfcba68e11629f902`
- Package path: `urdf/RS/` (URDF `urdf/ReBot_Arm_RS.urdf`, meshes under `meshes/`)
- Rebuilt with `tools/build_assets.py --variant rs` (trimesh 5.1.0)

## Licence

The upstream repository ships no LICENSE file at the pinned commit, and no
SPDX header appears in the URDF or the meshes. Seeed Studio publishes the
sibling reBot-DevArm description package under the CERN Open Hardware Licence
Version 2 - Weakly Reciprocal for hardware files and the Apache License 2.0
for code, so these RS hardware files are treated the same way:

- Hardware files (meshes, URDF): `SPDX-License-Identifier: CERN-OHL-W-2.0`
  (assumed; upstream has not stated one for this package)
- Code: `SPDX-License-Identifier: Apache-2.0`

The meshes are redistributed unmodified apart from decimation (triangle count
reduction) and format conversion (binary STL; visual parts merged per link
into GLB with flat colours). Geometry, units (metres) and link frames are
unchanged from the source. A copy of the upstream LICENSE text, where the
source package ships one, accompanies it; this notice is the required
attribution.

## Units and frames

- source meshes are in metres (largest extent 0.327 m, link2 longest extent 0.327 m).
- Every mesh is expressed in its URDF link frame, with the URDF `<origin>`
  of the visual/collision element applied.

## Generated files

| File | Size | Notes |
|---|---:|---|
| `meshes/base_link.stl` | 146.8 KB | decimated from meshes/shared/base_link.STL |
| `meshes/link1.stl` | 146.5 KB | decimated from meshes/shared/link1.STL |
| `meshes/link2.stl` | 145.1 KB | decimated from meshes/shared/link2.STL |
| `meshes/link3.stl` | 142.2 KB | decimated from meshes/shared/link3.STL |
| `meshes/link4.stl` | 141.1 KB | decimated from meshes/shared/link4.STL |
| `meshes/link5.stl` | 142.6 KB | decimated from meshes/shared/link5.STL |
| `meshes/link6.stl` | 147.3 KB | decimated from meshes/shared/link6.STL |
| `meshes/gripper_end.stl` | 135.4 KB | decimated from meshes/shared/gripper_end.STL |
| `meshes/gripper_left.stl` | 148.3 KB | decimated from meshes/shared/gripper_left.STL |
| `meshes/gripper_right.stl` | 148.3 KB | decimated from meshes/shared/gripper_right.STL |
| `models/base_link.glb` | 297.6 KB | merged 1 visual part(s) |
| `models/link1.glb` | 299.8 KB | merged 1 visual part(s) |
| `models/link2.glb` | 296.4 KB | merged 4 visual part(s) |
| `models/link3.glb` | 288.5 KB | merged 6 visual part(s) |
| `models/link4.glb` | 280.8 KB | merged 2 visual part(s) |
| `models/link5.glb` | 280.1 KB | merged 3 visual part(s) |
| `models/link6.glb` | 291.0 KB | merged 1 visual part(s) |
| `models/end_link.glb` | 279.8 KB | merged 3 visual part(s) |
| `models/finger_left_link.glb` | 291.1 KB | merged 2 visual part(s) |
| `models/finger_right_link.glb` | 291.1 KB | merged 2 visual part(s) |
| `primitives.json` | 10.0 KB | generated |

## Upstream source files used

- `urdf/RS/meshes/shared/base_link.STL` (sha256 af1df69639fb1f10...)
- `urdf/RS/meshes/shared/gripper_end.STL` (sha256 d52ed4bb37754fa9...)
- `urdf/RS/meshes/shared/gripper_left.STL` (sha256 1f604d582dc23814...)
- `urdf/RS/meshes/shared/gripper_right.STL` (sha256 b6b5c4df5b71c24e...)
- `urdf/RS/meshes/shared/link1.STL` (sha256 60860556dd91b15e...)
- `urdf/RS/meshes/shared/link2.STL` (sha256 f3653afa47f7f2ee...)
- `urdf/RS/meshes/shared/link3.STL` (sha256 3d6502345c14bd2d...)
- `urdf/RS/meshes/shared/link4.STL` (sha256 431e5e7365ba31df...)
- `urdf/RS/meshes/shared/link5.STL` (sha256 e8888083f0cf597e...)
- `urdf/RS/meshes/shared/link6.STL` (sha256 29a7809bde57f21b...)
- `urdf/RS/meshes/visual/cnc2.STL` (sha256 fa41ae81c68a7ce3...)
- `urdf/RS/meshes/visual/cnc3.STL` (sha256 4018cd61ac7d6dd9...)
- `urdf/RS/meshes/visual/cnc4.STL` (sha256 6a85571eaedbbf06...)
- `urdf/RS/meshes/visual/cnc5.STL` (sha256 1772159cd66b600c...)
- `urdf/RS/meshes/visual/cnc7.STL` (sha256 2d6088744c7d5195...)
- `urdf/RS/meshes/visual/cnc_left.STL` (sha256 8f8ed951fad05dd5...)
- `urdf/RS/meshes/visual/cnc_right.STL` (sha256 d7be82297465675b...)
- `urdf/RS/meshes/visual/motor_2_3.STL` (sha256 509d445fa6cade76...)
- `urdf/RS/meshes/visual/motor_4.STL` (sha256 2251e479b009b2c1...)
- `urdf/RS/meshes/visual/motor_5.STL` (sha256 87a9380b8441116d...)
- `urdf/RS/meshes/visual/motor_6.STL` (sha256 a488f0f5d5727b20...)
- `urdf/RS/meshes/visual/motor_7.STL` (sha256 4e019eae9e44376b...)
- `urdf/RS/meshes/visual/pla2_black.STL` (sha256 3334ffafff11e43f...)
- `urdf/RS/meshes/visual/pla2_green.STL` (sha256 3c1445d686ec4563...)
- `urdf/RS/meshes/visual/pla3_black_without_seeed_badge.STL` (sha256 3cc986a724bb7e22...)
- `urdf/RS/meshes/visual/pla3_green.STL` (sha256 7aee9689f19b6cb6...)
- `urdf/RS/meshes/visual/pla3_seeed_badge_with_counters.STL` (sha256 24f7fb2daafafc40...)
- `urdf/RS/meshes/visual/pla3_seeed_wordmark_backing.STL` (sha256 b837592ca2910576...)
- `urdf/RS/meshes/visual/pla5_green.STL` (sha256 d8810ec3fe0a9cb5...)
- `urdf/RS/meshes/visual/pla7_green.STL` (sha256 00b3c3d51f6f756a...)
- `urdf/RS/meshes/visual/pla_left.STL` (sha256 0e774ab0bf297579...)
- `urdf/RS/meshes/visual/pla_right.STL` (sha256 43556446abbbecd8...)
