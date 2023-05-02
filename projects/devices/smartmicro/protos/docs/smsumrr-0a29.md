The `Smartmicro UMRR-0a type 29` radar has an horizontal field of view of 36 degrees, a vertical field of view of 8 degrees and a maximum range of 160 meters.

```
SmsUmrr-0a29 {
  SFVec3f    translation    0 0 0
  SFRotation rotation       0 0 1 0
  SFString   name           "Sms UMRR 0a29"
  SFBool     occlusion      FALSE
  SFFloat    cellSpeed      0.0
  SFFloat    angularNoise   0.0
}
```

The `occlusion` field can be used to set whether occlusions between the targets and the radar should be checked.

The `cellSpeed` field can be used to set the minimum radial distance between two targets to be detected as distinct objects.

The `angularNoise` field can be used to set the angular noise.
