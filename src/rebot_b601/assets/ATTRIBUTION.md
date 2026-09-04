# Asset attribution

The meshes in this directory are derived from the Seeed Studio reBot-DevArm
B601-DM description package.

- Source repository: https://github.com/Seeed-Projects/reBot-DevArm
- Pinned commit: `b0acdcfc47843de16a9f018c6ab1de1d31649fdc`
- Package path: `Rebot_Arm_description/DM/` (URDF `urdf/ReBot_Arm_DM.urdf`, meshes under `meshes/`)
- Rebuilt with `tools/build_assets.py` (trimesh 5.1.0)

## Licence

The upstream repository licenses its hardware design files (including these
meshes and the URDF) under the CERN Open Hardware Licence Version 2 - Weakly
Reciprocal, and its software under the Apache License 2.0:

- Hardware files (meshes, URDF): `SPDX-License-Identifier: CERN-OHL-W-2.0`
- Code: `SPDX-License-Identifier: Apache-2.0`

The meshes are redistributed unmodified apart from decimation (triangle count
reduction) and format conversion (binary STL; visual parts merged per link
into GLB with flat colours). Geometry, units (metres) and link frames are
unchanged from the source. A copy of the upstream LICENSE text accompanies the
source package; this notice is the required attribution.

## Units and frames

- source meshes are in metres (largest extent 0.321 m, link2 longest extent 0.321 m).
- Every mesh is expressed in its URDF link frame, with the URDF `<origin>`
  of the visual/collision element applied.

## Generated files

| File | Size | Notes |
|---|---:|---|
| `meshes/base_link.stl` | 149.6 KB | decimated from meshes/collision/base_link.STL |
| `meshes/link1.stl` | 149.6 KB | decimated from meshes/collision/link1.STL |
| `meshes/link2.stl` | 149.6 KB | decimated from meshes/collision/link2.STL |
| `meshes/link3.stl` | 149.6 KB | decimated from meshes/collision/link3.STL |
| `meshes/link4.stl` | 149.6 KB | decimated from meshes/collision/link4.STL |
| `meshes/link5.stl` | 149.6 KB | decimated from meshes/collision/link5.STL |
| `meshes/link6.stl` | 149.6 KB | decimated from meshes/collision/link6.STL |
| `meshes/gripper_base.stl` | 149.6 KB | decimated from meshes/collision/gripper_base.stl |
| `meshes/left_finger.stl` | 149.6 KB | decimated from meshes/collision/left_finger.stl |
| `meshes/right_finger.stl` | 149.6 KB | decimated from meshes/collision/right_finger.stl |
| `models/base_link.glb` | 195.3 KB | merged 3 visual part(s) |
| `models/link1.glb` | 120.7 KB | merged 3 visual part(s) |
| `models/link2.glb` | 196.3 KB | merged 4 visual part(s) |
| `models/link3.glb` | 196.5 KB | merged 4 visual part(s) |
| `models/link4.glb` | 195.6 KB | merged 4 visual part(s) |
| `models/link5.glb` | 195.0 KB | merged 3 visual part(s) |
| `models/link6.glb` | 194.2 KB | merged 2 visual part(s) |
| `models/end_link.glb` | 194.7 KB | merged 3 visual part(s) |
| `models/finger_left_link.glb` | 87.9 KB | merged 4 visual part(s) |
| `models/finger_right_link.glb` | 88.0 KB | merged 4 visual part(s) |
| `primitives.json` | 13.5 KB | generated |

## Upstream source files used

- `Rebot_Arm_description/DM/meshes/collision/base_link.STL` (sha256 14d98e291f5eb20c...)
- `Rebot_Arm_description/DM/meshes/collision/gripper_base.stl` (sha256 5e7b0473acaf31fc...)
- `Rebot_Arm_description/DM/meshes/collision/left_finger.stl` (sha256 cf8a3acbbfbcaf15...)
- `Rebot_Arm_description/DM/meshes/collision/link1.STL` (sha256 33fc9ebe2654cc27...)
- `Rebot_Arm_description/DM/meshes/collision/link2.STL` (sha256 3dc33c484f02966f...)
- `Rebot_Arm_description/DM/meshes/collision/link3.STL` (sha256 783383016dbe0a25...)
- `Rebot_Arm_description/DM/meshes/collision/link4.STL` (sha256 95364d6a7008b62f...)
- `Rebot_Arm_description/DM/meshes/collision/link5.STL` (sha256 19ea62cabf41394d...)
- `Rebot_Arm_description/DM/meshes/collision/link6.STL` (sha256 6315a8ae1c03a280...)
- `Rebot_Arm_description/DM/meshes/collision/right_finger.stl` (sha256 e4b5ebef66e0b2b3...)
- `Rebot_Arm_description/DM/meshes/shared/colored_left_finger_carriage_grey.stl` (sha256 f650b231f1d366e0...)
- `Rebot_Arm_description/DM/meshes/shared/colored_left_finger_travel_stop_yellow.stl` (sha256 2fdf7c97d0388fe3...)
- `Rebot_Arm_description/DM/meshes/shared/colored_right_finger_carriage_grey.stl` (sha256 efa43d88c9bed0a6...)
- `Rebot_Arm_description/DM/meshes/shared/colored_right_finger_travel_stop_yellow.stl` (sha256 1d28609e4761077b...)
- `Rebot_Arm_description/DM/meshes/visual/colored_base_link_hardware_black.stl` (sha256 12deea0cd6f358ba...)
- `Rebot_Arm_description/DM/meshes/visual/colored_base_link_matte_black.stl` (sha256 a692eb4c625b6fa5...)
- `Rebot_Arm_description/DM/meshes/visual/colored_base_link_silver_trim.stl` (sha256 da6fefa2ff02dd66...)
- `Rebot_Arm_description/DM/meshes/visual/colored_gripper_base_hardware_black.stl` (sha256 9e89b784b7db552d...)
- `Rebot_Arm_description/DM/meshes/visual/colored_gripper_base_metal.stl` (sha256 ac9032a968febcca...)
- `Rebot_Arm_description/DM/meshes/visual/colored_gripper_base_seeed_yellow.stl` (sha256 5632650c7506d5ea...)
- `Rebot_Arm_description/DM/meshes/visual/colored_left_finger_finger_black.stl` (sha256 5473c38c7de9d4c8...)
- `Rebot_Arm_description/DM/meshes/visual/colored_left_finger_rack_metal.stl` (sha256 938e863507227a7d...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link1_anodized_grey.stl` (sha256 8ba2520c9321fc55...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link1_hardware_black.stl` (sha256 4825fcb6b569439a...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link1_matte_black.stl` (sha256 e4f57546af71031f...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link2_anodized_grey.stl` (sha256 a0ceb3fddf2f65db...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link2_hardware_black.stl` (sha256 f50c02f1568b636e...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link2_matte_black.stl` (sha256 f12463e6f38763bd...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link2_seeed_yellow.stl` (sha256 56a5cd3550a46626...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link3_anodized_grey.stl` (sha256 b8687a7fdd6aef7d...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link3_hardware_black.stl` (sha256 e983eb537d06b742...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link3_matte_black.stl` (sha256 7dd5ecba918916ca...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link3_seeed_yellow.stl` (sha256 39ab75600959b4ef...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link4_anodized_grey.stl` (sha256 4ce8aff6c90e347c...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link4_hardware_black.stl` (sha256 9e26076a2565f53c...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link4_matte_black.stl` (sha256 faeffab64bf462b2...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link4_seeed_yellow.stl` (sha256 380299025b1e0ebb...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link5_anodized_grey.stl` (sha256 b1b4caeba17d30bb...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link5_hardware_black.stl` (sha256 75adf40d6c770d6d...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link5_matte_black.stl` (sha256 c4555b72ccfb8cd0...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link6_hardware_black.stl` (sha256 e4dedf8865e4cbf0...)
- `Rebot_Arm_description/DM/meshes/visual/colored_link6_matte_black.stl` (sha256 4d122a7978946ba0...)
- `Rebot_Arm_description/DM/meshes/visual/colored_right_finger_finger_black.stl` (sha256 caf83c51fb2d98cb...)
- `Rebot_Arm_description/DM/meshes/visual/colored_right_finger_rack_metal.stl` (sha256 0ae753bbf754c48a...)
