# Version R2025a Released

<p id="publish-data">By Name Surname - Day Month Year</p>

---

It is that time of the year again!
Today we are happy to announce the release of Webots R2023b!
This new version is packed with some new features, improvements and, of course, bug fixes.

Here we are going to present some of the main new features, but for a comprehensive list of changes please refer to the [Change Log](../reference/changelog-r2025.md).

## Improvement to the Supervisor API

---

In Webots 2025a, a new feature enhances the Supervisor API by allowing direct access to internal fields of a PROTO hierarchy.

This includes new functions such as `wb_supervisor_proto_get_field` and `wb_supervisor_proto_get_number_of_fields`, enabling introspection and manipulation of nested PROTO parameters.

The update also propagates these changes across all supported programming languages, such as Python, C++, Java, and Matlab, ensuring consistency.

Additionally, several existing API methods were renamed for clarity, minimizing the impact on existing controllers. This improvement streamlines simulation control and introspection, making it easier for developers to interact with complex PROTO structures.

All the changes are available in the [Supervisor API](https://cyberbotics.com/doc/reference/supervisor?version=develop).

---

