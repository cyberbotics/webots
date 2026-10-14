# OneRobotics A1

The [OneRobotics A1](http://www.onerobot.com/) is a compact fixed-base robotic arm with seven revolute joints. This
model represents the independent right-arm revision published by OneRobotics.

The model was converted from the published
[`a1_r.urdf`](https://github.com/katazen/onerobot_h1/blob/ecf530911284ba0e559f7a24dc222fd8e60d31ed/source/h1_reach/h1_reach/assets/urdf/A1_2026/a1_r.urdf).
The visual geometry, kinematics, joint limits, masses, centers of mass, and inertia matrices come from source commit
[`ecf530911284ba0e559f7a24dc222fd8e60d31ed`](https://github.com/katazen/onerobot_h1/commit/ecf530911284ba0e559f7a24dc222fd8e60d31ed).
The eight visual meshes contain **33,882 triangles** and are modified derivatives of the published STL files,
in their original metre-based coordinate frames. Source STL hashes are retained in the OBJ headers. Link1 derives
from omichel's `link1_simplified.obj`, shared for [PR #7022](https://github.com/cyberbotics/webots/pull/7022) on
September 28, 2026 (reference SHA-256: `3cff3274186248660ce70dd362e9bb39acb09bb7424cb39410130179c60ce88c`).

The visual meshes simplify internal mechanisms and close deep joint cavities. Links3, 4 and 6 use a 0.25 mm
reconstruction grid with selected source-measured blind recesses; this grid spacing is not a maximum surface-error
guarantee. Link5 instead uses source-exterior surgery: narrow +/-0.03 mm casing cuts, local seam bridges and a
surface intersection join. Buried surfaces and internal mechanisms, including the bottom attachment, are removed.
Joint cavities are intentionally closed; source screw and top-slot walls are retained with local blind floors.
Cavity depths and some small interface openings intentionally differ from the source CAD.

The meshes include UV coordinates and explicit smooth/sharp normals. Link5 has newly generated dominant-face-axis
planar UV charts at one metre per texture repeat, with an offset of 0.5; these UVs were not present in the source STL.
Chart boundaries can introduce texture seams. Link5 floating-point text has changed; the installed native Assimp
and Blender float32 imports preserve position, normal and UV bits at every ordered face corner, rather than raw
double values. Assimp checks cover raw import and the full Mesh postprocess with PP_RVC_FLAGS=1000.
Localized, source-guided normal corrections remain at the Link3 lower rim, Link4 cap holes and Link4 rear rim.
The PROTO uses Webots' `Plastic` PBR appearance and casts shadows on all links.

The eight OBJ files total **3,196,692 bytes**, compared with **9,483,046 bytes / 57,904 triangles** in the PR-head
baseline: 66.29% fewer bytes and 41.49% fewer triangles. This is approximately 2.97 times smaller by bytes, not a
10-fold reduction. The final index compaction saves 67,510 bytes from the 3,264,202-byte runtime-tested revision.
It only reorders attribute tables and remaps positive indices; numeric tokens, ordered face corners, winding,
provenance and triangle counts are preserved. All eight meshes pass the unchanged strict geometry validator.
Native Assimp raw/full Mesh imports (PP_RVC_FLAGS=1000) and Blender imports preserve exact float32 position,
normal and UV bits at ordered face corners, including signed zeros.

The runtime-tested revision passed a 16-second desktop simulation on October 14, 2026: all seven joints moved,
four settled-pose checks passed, five screenshots were captured, and Webots exited with code 0 without detected
runtime errors. The final index-compacted files were not rerun on the desktop; their exact importer equivalence
to that revision supports reuse of its runtime evidence. The five runtime images were reviewed as a contact sheet
and robot closeups; this is a limited still-image review, not visual acceptance or a check of occluded surfaces,
fine texture seams or temporal flicker. Runtime and native Assimp checks use the installed **R2025a** engine/library
with preserved **R2025b** source headers. Native R2025b runtime and general forward-version compatibility remain
unverified.

The robot assets are © 2026 OneRobotics and licensed under the
[Creative Commons Attribution 4.0 International license](https://creativecommons.org/licenses/by/4.0/).

## OneRoboticsA1 PROTO

Derived from [`Robot`](https://cyberbotics.com/doc/reference/robot).

```proto
OneRoboticsA1 {
  SFVec3f    translation       0 0 0
  SFRotation rotation          0 0 1 0
  SFString   name              "OneRobotics A1"
  SFString   controller        "<generic>"
  MFString   controllerArgs    []
  SFString   window            "<generic>"
  SFString   customData        ""
  SFBool     supervisor        FALSE
  SFBool     synchronization   TRUE
  SFBool     selfCollision     FALSE
  MFNode     endEffectorSlot   []
  SFBool     staticBase        TRUE
}
```

### Field Summary

- `translation`: position of the arm base.
- `rotation`: orientation of the arm base.
- `name`: name of the `Robot` node.
- `controller`: controller used by the arm.
- `controllerArgs`: arguments passed to the controller.
- `window`: robot window shown for the arm.
- `customData`: application-specific robot data.
- `supervisor`: whether the controller has supervisor privileges.
- `synchronization`: whether the controller is synchronized with the simulation.
- `selfCollision`: whether collisions between the arm's links are enabled.
- `endEffectorSlot`: nodes attached to the Link7 flange frame.
- `staticBase`: whether the arm base is fixed to the static environment.

## Devices

Each joint has one `RotationalMotor` named `joint1-a1_r` through `joint7-a1_r` and one `PositionSensor` using the
same name followed by `_sensor`.

| Joint | Minimum position [rad] | Maximum position [rad] | Maximum velocity [rad/s] | Maximum torque [N m] |
| --- | ---: | ---: | ---: | ---: |
| `joint1-a1_r` | -1.04 | 3.14 | 2.6179938779914944 | 26.859 |
| `joint2-a1_r` | -3.14 | 0.26 | 2.6179938779914944 | 26.859 |
| `joint3-a1_r` | -2.76 | 2.76 | 2.6179938779914944 | 26.859 |
| `joint4-a1_r` | -1.92 | 1.92 | 12.566370614359172 | 5.975 |
| `joint5-a1_r` | -2.23 | 2.23 | 12.566370614359172 | 5.975 |
| `joint6-a1_r` | -1.57 | 1.57 | 12.566370614359172 | 5.975 |
| `joint7-a1_r` | -2.76 | 2.76 | 12.566370614359172 | 5.975 |

## Known Limitations

- This independent-arm revision does not include a gripper. Attach an end effector using `endEffectorSlot`.
- The Link7 mass and inertia are explicitly marked as a placeholder in the source URDF and should be replaced when
  measured flange data becomes available.
- Collision geometry uses axis-aligned boxes for stable and efficient contact simulation; it is less detailed than the
  visual meshes.
