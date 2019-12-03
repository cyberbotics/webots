# Webots R2020 Change Log

## [Webots R2020a](../blog/Webots-2019-a-release.md)
Released on XXX YYth, 2019.

  - New Features
    - Improved the [Background](background.md) node:
      - Added the `Background.luminosity` field which specifies the light contribution of the [Background](background.md) node. Open this field in the `TexturedBackground` and the `TexturedBackgroundLight` PROTO nodes.
      - Dropped the support of the equirectangular projection in textures to improve loading time.
      - Dropped the `Cubemap` node to improve consistency.
      - Deprecated non-HDR backgrounds.
      - Restored the `Background.*Url` fields, and support only `JPEG` and `PNG` format there.
      - Introduced the `Background.*IrradianceUrl` fields to define an `HDR` irradiance map.
      - Added image tools to help with `HDR` format and equirectangular projections.
      - Added new HDR background: `entrance_hall`
    - Added new appearance: `DryMud`
    - Added new functions to add force and torque to [Solid](solid.md) nodes: `wb_supervisor_node_add_force`, `wb_supervisor_node_add_relative_force` and `wb_supervisor_node_add_torque`.
    - Added function to import and remove nodes from SFNode fields: `wb_supervisor_field_remove_sf`, `wb_supervisor_field_import_sf_node` and `wb_supervisor_field_import_sf_node_from_string`.
    - Improved the Webots online 3D viewer: `webots.min.js`
      - Improved support of the Webots rendering pipeline: supported the Bloom post-processing effect.
      - Added support for the `ImageTexture.filtering` field.
      - Improved the console log history. Added a button to clear the console.
    - Improved [robotbenchmark](https://robotbenchmark.net) worlds.
      - Improved overall graphics quality (using the PBR materials and the HDR backgrounds).
      - Improved `humanoid_sprint` benchmark metrics.
    - Replaced the [Viewpoint](viewpoint.md) `followOrientation` field by a `followType` field for more flexibility.
  - Enhancements
    - Improved the [Sick LD MRS](../guide/lidar-sensors.md#sick-ld-mrs) PROTO to support the following types: `400001`, `400102`, `400001S01`, `400102S01` and `800001S01`.
    - Split the Webots and controller libraries to avoid possible conflicts with external libraries.
    - **Improved the `wb_supervisor_simulation_reset` to avoid restartign the controllers.**
    - Set the [ABB IRB 4600/40](../guide/irb4600-40.md) root node to [Robot](robot.md) instead of [Solid](solid.md) to be able to insert it everywhere.
    - Windows/Linux: Move the `Check for updates...` menu from `Tools` to `Help` for consistency with other applications.
  - New Samples
    - Added a `complete_apartment` world.
