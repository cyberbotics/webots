## Webots Telemetry System

The Webots telemetry sytem sends anonymous data to the Webots developers in order to help them improving the software.
It can be turned off or on the first time you start Webots and from the Webots [preferences](preferences.md#general).
This data is mainly used to detect and fix crashes on some specific hardware.

### Sent Informations
The following table describes all the data sent:

| Data sent                 | Description                                                                                                                                                     | Value Example                  |
| ------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------ |
| id                        | A random id                                                                                                                                                     | 1                              |
| operation                 | Defines the operation associated to this data (currently only `trial` when a world is going to be loaded and `success` when it is successfully loaded are used) | success                        |
| file                      | The current world file (only the filename and not the path is sent and only if the world is located in the Webots installation directory)                       | khepera3_gripper.wbt           |
| version                   | The version of Webots                                                                                                                                           | R2019a revision 1              |
| os                        | The operating system used                                                                                                                                       | Linux 4.4.0-142-generic x86_64 |
| glVendor                  | The GPU vendor                                                                                                                                                  | NVIDIA Corporation             |
| glRenderer                | The GPU model                                                                                                                                                   | GeForce GTX 970/FPCIe/FSSE2    |
| glVersion                 | The OpenGL version used                                                                                                                                         | 3.3.0 NVIDIA 396.54            |
| textureQuality            | The texture quality option set in the [preferences](preferences.md#opengl)                                                                                      | 0                              |
| disableCameraAntiAliasing | The camera anti-aliasing option set in the [preferences](preferences.md#opengl)                                                                                 | false                          |
| disableShadows            | The shadows option set in the [preferences](preferences.md#opengl)                                                                                              | false                          |
| GTAO                      | The ambient occlusion option set in the [preferences](preferences.md#opengl)                                                                                    | 2                              |
| SMAA                      | The main 3D view anti-aliasing option set in the [preferences](preferences.md#opengl)                                                                           | true                           |


To be as transparent as possible, all the telemetry raw data is available [here](https://cyberbotics.com/telemetry).
