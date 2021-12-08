# Convert protos and world to FLU

## Introduction

From the 2022a version, the axis convention of the global coordinate system changed from NUE to ENU. "ENU" means East along the X-positive axis, North along the Y-positive axis and Up along the Z-positive axis. It is the most widely used axis convention in robotics, including ROS. "NUE" means North along the X-positive axis, Up along the Y-positive axis and East along the Z-positive axis.
In addition, the object's axis system of Webots is now FLU (x-Forward, y-Left, z-Up). All the protos and worlds included in the Webots package have been converted according to these conventions.

If you open a project in the version 2022a, a backward compatibility algorithm will rotate your world for you. However, you may see that some parts are turned upside down!
This wiki will explain to you how to convert your protos and worlds. Try first method 1 or/and 2, and if it is not working, method 3.

## World conversion

### Method 1: using Webots interface

If your world does not contain too many objects, you should keep your world in `NUE` (see [CoordinateSystem]{worldInfo}) and simply rotate the objects as wanted.

### Method 2: using the script

If your world is too complex to be handled by hand, you can use the script `python scripts/converter/convert_nue_to_enu_rub.py`. This script will convert your world from `NUE` to `ENU` and from `RUB` (x-Right, y-Up, z-Back) to `FLU`. However, since the rotation depends on the proto itself, it could be needed to rotate some part by hand. Also, we advise you to check the differences in a file comparator. You can find detailed explication into the script (dependencies, usage, limitations and conversion process).
Simple usage:
```python3 convert_nue_to_enu_rub.py /your_path_to_your_projects/worlds/my_world.wbt```

## Proto conversion

### Method 1: using Webots interface

If your proto is not too complex, you can directly rotate it by using Webots interface and save it. You should keep your world in `NUE` (see [CoordinateSystem]{worldInfo}) and simply rotate the geometries and sensors. Indeed, the geometries and sensors were Y-UP and are now Z-up. Basically, you need to rotate them to an angle of `pi/2` on the x-axis.

## Method 3: Manually

Otherwise, to convert your proto manually, open your `.proto` or `.wbt` file in your text editor. 
You can use the script `python3 scripts/converter/convert.py` and apply the wanted rotations to convert your proto. This script takes the clipboard content, applies transforms, and pastes the transformed VRML to the clipboard. To set the correct transformation, change the variable `ROTATION` into the script. Follow the conversion process bellow.

Conversion process:
    - replace `R2021b` by `R2022a`
    - remove the `coordinateSystem ENU` line
    - rotate the geometries such as `Cylinder`, `Capsule`, `Cone`, `Plane`
    - convert the fields 'translation', 'axis', 'anchor', 'location', 'direction'
    - convert the fields 'rotation'
    - convert the fields 'size', 'frameSize', 'stepSize', 'palletSize'
    - convert the fields 'centerOfMass'
    - convert the fields 'point' of the 'IndexFaceSet', 'corners' and 'path'
    - adjust the viewpoint with your mouse into the Webots interface.

## Examples

### Protos

### Worlds

Here is an example of a world converted from an NUE world with its objects in RUB to an ENU world with its objects in FLU.


