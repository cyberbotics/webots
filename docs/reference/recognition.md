## Recognition

```
Recognition {
  SFFloat  maxRange       100     # [0, inf)
  SFInt32  maxObjects     -1      # {-1, [0, inf)}
  SFBool   occlusion      TRUE    # {TRUE, FALSE}
  SFColor  frameColor     1 0 0   # any color
  SFInt32  frameThickness 1       # [0, inf)
  SFBool   segmentation   FALSE   # {TRUE, FALSE}
}
```

### Description

The [Recognition](#recognition) node provides a [Camera](camera.md) device with object recognition capability.
When a [Camera](camera.md) device has a [Recognition](#recognition) node in its `recognition` field, it is able to recognize which objects are present in the camera image.
Only [Solids](solid.md) whose `recognitionColors` field is not empty can be recognized by the camera.

Additionally, the [Recognition](#recognition) also provides the `segmentation` functionality to generate segmentation ground truth images displaying the recognized objects.
In the segmentation image, each pixel will be colored using the first item of the `recognitionColors` of the corresponding object rendered from the [Camera](camera.md) device.
The segmentation image can be used as ground truth data, i.e. validated data, given that it will classify exactly the recognized objects.
An example of segmentation image is shown in [the following figure](#recognition-segmentation-image): on the left you have the [Camera](camera.md) image and on the right the corresponding segmentation image.
The pixels corresponding to the cereal boxes, that have an empty `recognitionColors` field, and to the background are not classified and rendered in black.

%figure "Recognition Segmentation Image"

![recognition_segmentation_image.png](images/recognition_segmentation_image.png)

%end

### Field Summary

- The `maxRange` field defines the maximum distance at which an object can be recognized.
Objects farther than `maxRange` are not recognized.

- The `maxObjects` field defines the maximum number of objects detected by the camera.
`-1` means no limit.
If more objects are visible to the camera, only the `maxObjects` biggest ones (considering pixel size) are recognized.

- The `occlusion` field defines if occlusions between the camera and the object should be checked.
Disabling the occlusion can be useful to allow the camera to see through thin or transparent objects that may hide the center of the object we are interested in, but it can lead to recognized objects that are not really visible to the camera.
Additionally, it will slightly speed up the simulation.

- The `frameColor` field defines the color used to frame the objects recognized by the camera in its overlay.

- The `frameThickness` field defines the thickness in pixels of the frames in the camera overlay.
0 means no object frame in the camera overlay.

- The `segmentation` field defines if a segmentation ground truth image is generated based on the [Solid](solid.md).`recognitionColors` field value.
Background and objects with empty `recognitionColors` field are rendered in black.
