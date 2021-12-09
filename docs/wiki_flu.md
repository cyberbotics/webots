# New axis system for geometries, devices, and PROTOs

Starting from the R2022a version, all geometries, devices, and PROTOs that come with Webots use the FLU axis orientation (x-**F**orward, y-**L**eft, and z-**U**p, see [#3073](https://github.com/cyberbotics/webots/pull/3073)).
We introduced the update to be consistent with ROS (see [REP 103](https://www.ros.org/reps/rep-0103.html)) and other robotics systems.
If your Webots world uses Webots geometries, devices, and PROTOs then you will probably need to adapt your world.
We also introduced a backward compability mechanism (see [#3619](https://github.com/cyberbotics/webots/pull/3619)) to adapt worlds automatically.
However, the mechanism has limited capabilities and it is likely you will need to assist the world adaptation.
Therefore, it is important to understand how Webots nodes are affected by the update and how they should be rotated to preserve the old behavior:

| Node | Required rotation to preserve the old behavior | Strategy code |
|---|:---:|:---:|
| [Cylinder](reference/cylinder.md) | (-$\pi/2$, 0, 0) | B1 |
| [Capsule](reference/capsule.md) | (-$\pi/2$, 0, 0) | B1 |
| [ElevationGrid](reference/elevationgrid.md) | (-$\pi/2$, 0, 0) | B1 |
| [Cone](reference/cone.md) | (-$\pi/2$, 0, 0) | B1 |
| [Plane](reference/plane.md) | (-$\pi/2$, 0, 0) | B1 |
| [Camera](reference/camera.md) | (-$\pi/2$, 0, $\pi/2$) | A2 |
| [Lidar](reference/lidar.md) | (-$\pi/2$, 0, $\pi/2$) | A2 |
| [Radar](reference/radar.md) | (-$\pi/2$, 0, $\pi/2$) | A2 |
| [Viewpoint](reference/viewpoint.md) | (-$\pi/2$, 0, $\pi/2$) | A1 |
| [Track](reference/track.md) | (-$\pi/2$, 0, $\pi/2$) | A2 |
| [Pen](reference/camera.md) | (-$\pi/2$, 0, 0) | A2 |
| [Emitter](reference/emitter.md) | (-$\pi/2$, 0, -$\pi/2$) | A2 |
| [Receiver](reference/receiver.md) | (-$\pi/2$, 0, -$\pi/2$) | A2 |
| [Connector](reference/connector.md) | (-$\pi/2$, 0, -$\pi/2$) | A2 |
| [TouchSensor](reference/touchsensor.md) | (-$\pi/2$, 0, -$\pi/2$) | A2 |
| Webots PROTOs | (-$\pi/2$, 0, $\pi/2$), but there are numerous exceptions to this rule as we haven't followed any specific convention | C |

Note that the backward compability mechanism adapts the nodes in the internal tree structure.
Therefore, the saved worlds will be overriden with the updated structure, but the PROTO files adapter exclusively at the runtime.
A user should update the PROTO files manually.   

# New axis system recommendations

As of Webots R2022a, we recommend the FLU (x-**F**orward, y-**L**eft, and z-**U**p) axis orientation for objects and ENU (x-**E**ast, y-**N**orth, and z-**U**p, see [Axes conventions
](https://en.wikipedia.org/wiki/Axes_conventions)) for worlds.
Once the world and PROTOs behave work properly in Webots R2022a then you can change object axis system to FLU and the world axis system to ENU.

## Convert PROTOs to FLU

If your PROTO file does not contain JavaScript or Lua code and it follows the RUB axis system (x-**R**ight, y-**U**p, z-**B**ack) then you can use the script:
```
python scripts/converter/convert_proto.py /path/to/proto/file.proto
```

## Convert worlds to ENU

We have have a quite robust Python script that converts NUE worlds to ENU axis system:
```
python scripts/converter/convert_nue_to_enu.py /path/to/world/file.wbt
```


## Introduction

From the 2022a version, **the axis convention of the global coordinate system changed from NUE to ENU**. _ENU_ means **East** along the **X**-positive axis, **North** along the **Y**-positive axis and **Up** along the **Z**-positive axis. It is the most widely used axis convention in robotics, including ROS. _NUE_ means **North** along the **X**-positive axis, **Up** along the **Y**-positive axis and **East** along the **Z**-positive axis.
In addition, **the object's axis system of Webots is now FLU** (_x-**F**orward, y-**L**eft, z-**U**p_). All the protos and worlds included in the Webots package have been converted according to these conventions.

If you open a world in the version 2022a, a backward compatibility algorithm will rotate it for you. However, you may see that some parts are turned upside down!
This wiki will explain to you how to convert your protos and worlds.

## Backward compatibility

If your world or proto does not contain too many objects, you should keep your world in **NUE** (see _CoordinateSystem_ in [worldInfo](https://www.cyberbotics.com/doc/reference/worldinfo)) and simply rotate the objects as wanted.

If your world or proto is too complex to be handled by hand, use the script to convert it to ENU or FLU (see [Meet Webots new conventions](#meet-webots-new-conventions)), or refer to the [conversion process](#conversion-process) below.
## Meet Webots new conventions

You may want to update your world from _NUE_ to _ENU_ or your proto to _FLU_ to meet the new Webots convention.
### World conversion NUE to ENU

You can use the script `python scripts/converter/convert_nue_to_enu_rub_to_flu.py`. This script will convert your world from _NUE_ to _ENU_. However, since the rotation depends on the proto itself, it could be needed to rotate some part by hand. Also, we advise you to check the differences in a file comparator. You can find detailed explication into the script (dependencies, usage, limitations and conversion process).
Simple usage:
```
python3 convert_nue_to_enu_rub.py /your_path_to_your_projects/worlds/my_world.wbt
```

### Proto conversion to FLU

If your proto is not too complex, you can directly rotate it by using Webots interface and save it. You should keep your world in `NUE` (see [CoordinateSystem]{worldInfo}) and simply rotate the geometries and sensors. Indeed, the geometries and sensors were Y-UP and are now Z-up. Basically, you need to rotate them to an angle of `pi/2` on the x-axis.

If your proto is _RUB_ (x-**R**ight, y-**U**p, z-**B**ack) and does not contain javascript, you can convert it to _FLU_ using the script `python scripts/converter/convert_nue_to_enu_rub_to_flu.py`
## Conversion process

Otherwise, to convert your proto or world manually, open your `.proto` or `.wbt` file in your text editor. 
You can use the script `python3 scripts/converter/convert.py` and apply the wanted rotations to convert your proto. This script takes the clipboard content, applies transforms, and pastes the transformed VRML to the clipboard. To set the correct transformation, change the variable `ROTATION` into the script. Follow the conversion process bellow.

Conversion process:
- replace `R2021b` by `R2022a`
- remove the `coordinateSystem ENU` line
- rotate the geometries such as `Cylinder`, `Capsule`, `Cone`, `Plane`
- convert the fields `translation`, `axis`, `anchor`, `location`, `direction`
- convert the fields `rotation`
- convert the fields `size`, `frameSize`, `stepSize`, `palletSize`
- convert the fields `centerOfMass`
- convert the fields `point` of the `IndexFaceSet`, `corners` and `path`
- adjust the viewpoint with your mouse into the Webots interface.

## Examples

### Protos

### Worlds

Here is an example of a world converted from an NUE world with its objects in RUB to an ENU world with its objects in FLU using the script `convert_nue_to_enu_rub.py`.


