# New axis system for geometries, devices, and PROTOs

Starting from the R2022a version, all geometries, devices, and PROTOs that come with Webots use the FLU axis orientation (x-**F**orward, y-**L**eft, and z-**U**p, see [#3073](https://github.com/cyberbotics/webots/pull/3073)).
We introduced the update to be consistent with ROS (see [REP 103](https://www.ros.org/reps/rep-0103.html)) and other robotics systems.
If your Webots world uses Webots geometries, devices, and PROTOs then you will probably need to adapt your world.

[add image example of a 2021b world with the axis and a 2022a world with the axis]

# How to adapt your world or PROTO to Webots R2022a

## Backward compatibility mechanism
We introduced a backward compatibility mechanism (see [#3619](https://github.com/cyberbotics/webots/pull/3619)) to adapt worlds automatically.
This mechanism will be launched when you open a world in the R2022a version.
Nevertheless, the mechanism has limited capabilities and it is likely you will need to assist the world adaptation. 

* If your world or PROTO does not contain too many objects, you should simply adapt them as wanted using the interface and save it.

* If your world or PROTO is too complex to be handled by hand, you should directly use the script to convert it to ENU/FLU (see [automatic conversion to ENU/FLU](#automatic-conversion-to-ENU/FLU)).

In any case, it is important to understand how Webots nodes are affected by the update and how they should be rotated to preserve the old behavior:

| Node | Required rotation to preserve the old behavior | Strategy code |
|---|:---:|:---:|
| [Cylinder](reference/cylinder.md) | (-$\frac{\pi}{2}$, 0, 0) | B1 |
| [Capsule](reference/capsule.md) | (-$\frac{\pi}{2}$, 0, 0) | B1 |
| [ElevationGrid](reference/elevationgrid.md) | (-$\frac{\pi}{2}$, 0, 0) | B1 |
| [Cone](reference/cone.md) | (-$\frac{\pi}{2}$, 0, 0) | B1 |
| [Plane](reference/plane.md) | (-$\frac{\pi}{2}$, 0, 0) | B1 |
| [Camera](reference/camera.md) | (-$\frac{\pi}{2}$, 0, $\frac{\pi}{2}$) | A2 |
| [Lidar](reference/lidar.md) | (-$\frac{\pi}{2}$, 0, $\frac{\pi}{2}$) | A2 |
| [Radar](reference/radar.md) | (-$\frac{\pi}{2}$, 0, $\frac{\pi}{2}$) | A2 |
| [Viewpoint](reference/viewpoint.md) | (-$\frac{\pi}{2}$, 0, $\frac{\pi}{2}$) | A1 |
| [Track](reference/track.md) | (-$\frac{\pi}{2}$, 0, $\frac{\pi}{2}$) | A2 |
| [Pen](reference/camera.md) | (-$\frac{\pi}{2}$, 0, 0) | A2 |
| [Emitter](reference/emitter.md) | (-$\frac{\pi}{2}$, 0, -$\frac{\pi}{2}$) | A2 |
| [Receiver](reference/receiver.md) | (-$\frac{\pi}{2}$, 0, -$\frac{\pi}{2}$) | A2 |
| [Connector](reference/connector.md) | (-$\frac{\pi}{2}$, 0, -$\frac{\pi}{2}$) | A2 |
| [TouchSensor](reference/touchsensor.md) | (-$\frac{\pi}{2}$, 0, -$\frac{\pi}{2}$) | A2 |
| Webots PROTOs | (-$\frac{\pi}{2}$, 0, $\frac{\pi}{2}$), but there are numerous exceptions to this rule as we haven't followed any specific convention | C |

## Automatic conversion to ENU/FLU

This script `scripts/converter/convert_nue_to_enu_rub_to_flu.py` intends to help you to adapt your world or PROTOs from an old version to the new version of Webots.
You can find detailed explication into the script (dependencies, usage, limitations and conversion process).

[add image example of a "broken" RUB (2021b) to FLU (R2022a) complex proto]
[add image example of a "broken" NUE (2021b) to ENU (R2022a) complex proto]

This script will convert your world from _NUE_ to _ENU_ and the objects contained in the world from _RUB_ to _FLU_. It could be needed to rotate some parts by hand. 

Simple usage:
```
python3 convert_nue_to_enu_rub_to_flu.py /your_path_to_your_projects/worlds/my_world.wbt
```

If your PROTO is _RUB_ (x-**R**ight, y-**U**p, z-**B**ack) and does not contain JavaScript or Lua code, you can convert it to _FLU_ using the same script. You may have to rotate some parts of your PROTOs by hand.

**Notes:** check that the conversion went successfully by displaying the different rendering `View/Optional rendering`. In addition, we advise you to check the differences of your `.wbt` or `.proto` before and after the script in a text comparator.
# New axis system recommendations

As of Webots R2022a, we recommend the FLU (x-**F**orward, y-**L**eft, and z-**U**p) axis orientation for objects and ENU (x-**E**ast, y-**N**orth, and z-**U**p, see [Axes conventions
](https://en.wikipedia.org/wiki/Axes_conventions)) for worlds.
Once the world and PROTOs have the desired behavior in Webots R2022a then you can change the object axis system to FLU and the world axis system to ENU.

## Convert PROTOs to FLU

If your PROTO file does not contain JavaScript or Lua code and it follows the RUB axis system (x-**R**ight, y-**U**p, z-**B**ack) then you can use the script:
```
python scripts/converter/convert_PROTO.py /path/to/PROTO/file.PROTO
```

## Convert worlds to ENU

We have a quite robust Python script that converts NUE worlds to ENU axis system:
```
python scripts/converter/convert_nue_to_enu.py /path/to/world/file.wbt
```


