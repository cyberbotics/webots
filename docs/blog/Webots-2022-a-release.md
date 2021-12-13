# Version R2022a Released

<p id="publish-data">By Benjamin Délèze - 17th December 2021</p>

---

It's that time of year again! Today we're happy to announce the release of Webots R2022a!
And it's packed with some new features, improvements and, of course, bug fixes.

Here we are going to present some of the main new features, but for a comprehensive list of changes please refer to the [ChangeLog](../reference/changelog-r2022.md).


## Conversion to FLU/ENU

### What is FLU/ENU

**FLU and ENU are coordinate axis convention.**

Before, Webots was using **NUE** as the global coordinate system and we switched it to be new **ENU** by default.

**NUE** means _North_ along the _X_-positive axis, _Up_ along the _Y_-positive axis and _East_ along the _Z_-positive axis.

**ENU** means _East_ along the _X_-positive axis, _North_ along the _Y_-positive axis and _Up_ along the _Z_-positive axis


In addition, **the object's axis system of Webots is now FLU** (_x-**F**orward, y-**L**eft, z-**U**p_). Whereas before there was no real prefered object's axis system.

### Why have we done these changes

ENU and FLU are the most widely used axis conventions in robotics, including ROS and ROS2.

More and more people are using Webots with ROS2 and we changed the axis convention of Webots to make it easier for them !

### What we have done

We worked hard to convert all PROTOS and worlds distributed with Webots for you, so that they are ready to use in R2022a.

A mechanism of backward compatibility has been implemented directly in Webots. Unfortunately, it was not possible to make it work in every cases. It will try to convert worlds but cannot convert Protos.

We also wrote a guide to help you convert your own protos and worlds to be able to use them in Webots R2022a and benefits from the new features and bug fixes.

We know it is a major change and it can be painful to convert your worlds and PROTOS to the new standard. But it is a worthwhile effort, especially if you work with ROS.

---

## Addition of the Skin node


---

## Improving user interface

We made several improvement to enhance the user experience:

  - The zooming speed have been adjusted to make the navigation more pleasant.
  - Rotating objects directly in the scene is now easier than ever.
  - We added a warning when attempting to add a node to an already started simulation.


---

## ROS 2 & Webots

---

## Extra Goodies

The performance on [Lidar](../guide/lidar) point cloud generation has been greatly improved.

[ContactProperties](../guide/contactproperties) now supports a new type of friction: **rolling friction**.

**Go and [download](https://cyberbotics.com/#download) Webots R2022a today, so you don't miss out on all these great new features!**

---

## Acknowledgements

The current release includes contributions from [Ayk Borstelmann](https://github.com/aykborstelmann),[CoreCode](https://github.com/core-code), [dzyGIT](https://github.com/dzywater), [Justin-Fisher](https://github.com/Justin-Fisher), [KNTRL9](https://github.com/KNTRL9), [Laurent Lessieux](https://github.com/llessieux),[maciejlichon](https://github.com/maciejlichon),[Oleg Elifantiev](https://github.com/Olegas), [ShuffleWire](https://github.com/ShuffleWire), [Stefan Arndt](https://github.com/stef264), [Snow1](https://github.com/lixk28), [Thomas Feldmeier](https://github.com/Thomas-Feldmeier)and [Tim Clephas](https://github.com/Timple), .
Special thanks go to these contributors and the many other members of our community that have contributed by reporting issues, bugs or provided support and moderation in our [Discord](https://discord.com/invite/nTWbN9m) channel.
