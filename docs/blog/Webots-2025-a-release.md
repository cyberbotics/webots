# Version R2025a Released

<p id="publish-data">By Name Surname - Day Month Year</p>

---

It is that time of the year again!
Today we are happy to announce the release of Webots R2023b!
This new version is packed with some new features, improvements and, of course, bug fixes.

Here we are going to present some of the main new features, but for a comprehensive list of changes please refer to the [Change Log](../reference/changelog-r2025.md).

## New Robot

A new robot model has been added to the Webots library.


## Introducing the W3D Format

Webots 2025a introduces the new `w3d` format, replacing the outdated `x3d` format to simplify simulation workflows.

This update aligns Webots, WebotsJS, and webots.cloud, offering a unified and consistent export system for various elements like sensors, actuators, and visual components.

The transition also removes VRML support, replacing `vrmlField` with `w3dField` for cleaner integration and simplified export processes.

Additionally, the Surveyor robot is enhanced with tracks and an updated controller. These changes make the Webots ecosystem more robust, scalable, and easier to use.

---

## Improvement to the Supervisor API

A new feature enhances the Supervisor API by allowing direct access to internal fields of a PROTO hierarchy.

This includes new functions such as `wb_supervisor_proto_get_field` and `wb_supervisor_proto_get_number_of_fields`, enabling introspection and manipulation of nested PROTO parameters.

The update also propagates these changes across all supported programming languages, such as Python, C++, Java, and Matlab, ensuring consistency.

Additionally, several existing API methods were renamed for clarity, minimizing the impact on existing controllers. This improvement streamlines simulation control and introspection, making it easier for developers to interact with complex PROTO structures.

All the changes are available in the [Supervisor API](https://cyberbotics.com/doc/reference/supervisor?version=develop).

---

## Improved Rotating Lidar Angle Range in Webots

To improve consistency and compatibility with ROS 2, the angle range of rotating Lidars in Webots has been updated. Previously providing data in `[0, 2π]`, the angle range now spans `[-π, +π]`. This change ensures better alignment with ROS 2 standards and provides a uniform experience across Lidar implementations.

---

## Improved Speed and Accuracy of Box-Plane Collisions

Contact handling has been significantly improved for better accuracy and performance. 

The update ensures that the `maxContacts` deepest contacts are always returned, fixing issues with oscillations caused by inconsistent contact selection in edge cases.

The new implementation is faster when handling common scenarios, such as objects settling on a plane, and provides comparable performance in other cases.

This change improves stability and reliability in physics simulations.

---

## Extra Goodies

- Webots is now compatible with Ubuntu 24.04 "Noble Numbat" and macOS 14 "Sonoma".

**Go and [download Webots R2025a](https://cyberbotics.com/#download) today, so you do not miss out on all these great new features!**

---

## Acknowledgements

The current release includes contributions from [Daniel Dias](https://github.com/ad-daniel), [Kimberly McGuire](https://github.com/knmcguire), [Chandan Bharadwaj](https://github.com/Chandan-Bharadwaj), [Songyang Zhao](https://github.com/songyangZhao), [ShuffleWire](https://github.com/ShuffleWire), [Jakub Delicat](https://github.com/delihus), [Léo Duggan](https://github.com/Jean-Eudes-le-retour), [nilsjor](https://github.com/nilsjor), [Simon Gene Gottlieb](https://github.com/SGSSGene), [Tsubasaya](https://github.com/Minimerl), [Dean Brettle](https://github.com/brettle), [Chirath Pansilu](https://github.com/ChirathPansilu), [fparat](https://github.com/fparat), [Gaël Écorchard](https://github.com/galou), [Ian Burwell](https://github.com/IanBurwell), [Stephan Kunz](https://github.com/stepkun), [Kode Creer](https://github.com/kodecreer), [Toshiharu Tabuchi](https://github.com/toshiharutf), [Justin Beri](https://github.com/justinberi), [DrakerDG](https://github.com/DrakerDG), [Jolon Behrent](https://github.com/JolonB), [Darko Lukić](https://github.com/lukicdarkoo), [Miloš Nikolić](https://github.com/MNikoliCC), [Angel Ayala](https://github.com/angel-ayala), [Sebastian Ohl](https://github.com/sebastianohl), [GnSight](https://github.com/ftyghome) and [tantan](https://github.com/naos080415).

Special thanks go to these contributors and the many other members of our community who have contributed by reporting issues, bugs or provided support and moderation in our [Discord](https://discord.com/invite/nTWbN9m) channel.

The development of Webots is also partially supported by several European research projects, including [OpenDR](https://opendr.eu) and [OPTIMA](https://optima-hpc.eu), the [SimGait](https://simgait.org) Swiss national research project and many other private and academic partners.

