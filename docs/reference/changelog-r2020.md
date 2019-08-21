# Webots R2020 Change Log

## [Webots R2020a](../blog/Webots-2019-a-release.md)
Released on XXX YYth, 2019.

  - New Features
    - Added the `Background.luminosity` field which specifies the light contribution of the [Background](background.md) node. Open this field in the `TexturedBackground` and the `TexturedBackgroundLight` PROTO nodes.
    - Added new appearance: `DryMud`
    - Added new HDR background: `entrance_hall`
    - Improved the Webots online 3D viewer: `webots.min.js`
      - Improved support of the Webots rendering pipeline: supported the Bloom post-processing effect.
      - Added support for the `ImageTexture.filtering` field.
      - Improved the console log history. Added a button to clear the console.
    - Improved [robotbenchmark](https://robotbenchmark.net) worlds.
      - Improved overall graphics quality (using the PBR materials and the HDR backgrounds).
      - Improved `humanoid_sprint` benchmark metrics.
    - Replaced the [Viewpoint](viewpoint.md) `followOrientation` field by a `followType` field for more flexibility.
  - New Samples
    - Added a `complete_apartment` world.
