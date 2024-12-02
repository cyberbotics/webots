## Computer Peripherals

You may interface various computer peripherals with Webots in order to interact with the simulation.

### Joystick

You can use the [Joystick API](../reference/joystick.md) to get the state of the buttons and value of the axes in your controller.
You may also (if your joystick supports it) use force feedback.
You simply need to plug your peripherals and if needed install its driver.
A wide range of gamepad and steering wheel joysticks are supported.

> **Note**: On Mac OS X, only the first 12 buttons and first 2 axes of the joystick are taken into account.

### Virtual Reality Headset

On Windows you can use a virtual reality headset to view the simulation.
We use SteamVR to interface the headset with Webots, you will therefore need to install [Steam](http://store.steampowered.com/about/) and then [SteamVR](steam://run/250820).
SteamVR is currently still unstable on Linux and Mac OS X, this is why we decided to support virtual reality headset only on Windows for now, but this might change in the near future.
We have tested Webots with the [HTC Vive](https://www.vive.com/) and [Oculus Rift](https://www.oculus.com/), but it might work with other virtual reality headsets too.

#### HTC Vive

You don't need to install anything else than SteamVR to use the HTC Vive, simply run the [Room Setup](https://support.steampowered.com/kb_article.php?ref=2001-UXCM-4439#room-setup) before starting Webots.

#### Oculus Rift

In addition to SteamVR you will need to install the [Oculus SDK](https://developer.oculus.com/downloads/package/oculus-sdk-for-windows) too.

> **Note**: If you are using a development version of the Oculus Rift you will need to install the [Oculus Legacy runtime](https://developer.oculus.com/downloads/package/oculus-runtime-for-windows) instead.
