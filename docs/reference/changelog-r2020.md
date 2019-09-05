# Webots R2020 Change Log

## [Webots R2020a](../blog/Webots-2019-a-release.md)
Released on XXX YYth, 2019.

  - New Features
    - Background:
      - Added the `Background.luminosity` field which specifies the light contribution of the [Background](background.md) node. Open this field in the `TexturedBackground` and the `TexturedBackgroundLight` PROTO nodes.
      - Dropped the support of the equirectangular projection in textures to improve loading time.
      - Dropped the `Cubemap` node to improve consistency. Restore the `Background.*Url` fields.
      - Added a Python script to split equirectangular textures to a cubemap (6 textures).
      - Added new HDR background: `entrance_hall`
    - Added new appearance: `DryMud`
    - Improved the Webots online 3D viewer: `webots.min.js`
      - Improved support of the Webots rendering pipeline: supported the Bloom post-processing effect.
      - Added support for the `ImageTexture.filtering` field.
      - Improved the console log history. Added a button to clear the console.
    - Improved graphics quality of the [robotbenchmark](https://robotbenchmark.net) worlds.
    - Replaced the [Viewpoint](viewpoint.md) `followOrientation` field by a `followType` field for more flexibility.
  - New Samples
    - Added a `complete_apartment` world.
