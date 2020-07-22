## Lens

```
Lens {
  SFVec2f center                 0.5 0.5   # [0, 1] [0, 1]
  SFVec2f radialCoefficients     0 0       # any vector
  SFVec2f tangentialCoefficients 0 0       # any vector
}
```

### Description

The [Lens](#lens) node allows the user to simulate the camera image distortion due to the camera lens.
A Brown's distortion model with two coefficients both for the radial and tangential distortions is used to simulate image distortion.

### Field Summary

- The `center` field defines the distortion center.
Its value should be between 0 and 1.

- The `radialCoefficients` field defines the first and second coefficients of the radial distortion of the Brown's distortion model.

- The `tangentialCoefficients` field defines the first and second coefficients of the tangential distortion of the Brown's distortion model.
The value of the tangential coefficients are typically smaller than the value of the radial coefficients for standard cameras.
