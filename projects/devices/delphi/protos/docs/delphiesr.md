The `Delphi ESR` is a radar which can see cars up to 174 meters ahead, has a vertical field of view of 4.5 degrees and an horizontal field of view of up to 90 degrees.
The sensor can operate in two modes: the long range mode uses an horizontal field of view of 20 degrees and a maximum range of 174 meters, the medium range mode uses a field of view of 90 degrees and a maximum range of 60 meters.

```
DelphiESR {
  SFVec3f    translation    0 0 0
  SFRotation rotation       0 0 1 0
  SFString   name           "Delphi ESR"
  SFBool     occlusion      FALSE
  SFBool     longRangeMode  FALSE
}
```

The `occlusion` field can be used to set whether occlusions between the targets and the radar should be checked.

The `longRangeMode` field can be used to set the radar mode.
