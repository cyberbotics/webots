## Recognition

```
Recognition {
  SFFloat  maxRange          100     # [0, inf)
  SFInt32  maxObjects        -1      # {-1, [0, inf)}
  SFInt32  occlusion         1       # {0, 1, 2}
  SFColor  frameColor        1 0 0   # any color
  SFInt32  frameThickness    1       # [0, inf)
  SFBool   segmentation      FALSE   # {TRUE, FALSE}
}
```

### Description

The [Recognition](#recognition) node provides a [Camera](camera.md) device with object recognition capability.
When a [Camera](camera.md) device has a [Recognition](#recognition) node in its `recognition` field, it is able to recognize which objects are present in the camera image.
Only [Solids](solid.md) whose `recognitionColors` field is not empty can be recognized by the camera.

Defining the [Solid.boundingObject](solid.md) of the object might help computing a more precise and tight fitting recognized size.

Additionally, the [Recognition](#recognition) also provides the `segmentation` functionality to generate segmentation ground truth images displaying the recognized objects.
In the segmentation image, each pixel will be colored using the first item of the `recognitionColors` of the corresponding object rendered from the [Camera](camera.md) device.
The segmentation image can be used as ground truth data, i.e. validated data, given that it will classify exactly the recognized objects.
An example of segmentation image is shown in [the following figure](#recognition-segmentation-image): on the left you have the [Camera](camera.md) image and on the right the corresponding segmentation image.
The pixels corresponding to the cereal boxes, that have an empty `recognitionColors` field, and to the background are not classified and rendered in black.

%figure "Recognition Segmentation Image"

![recognition_segmentation_image.png](images/recognition_segmentation_image.png)

%end

> **note**: A current known limitation of the object recognition functionality applies to large objects, like floors, that extend all around the device that might only be partially detected.


### Field Summary

- The `maxRange` field defines the maximum distance at which an object can be recognized.
Objects farther than `maxRange` are not recognized.

- The `maxObjects` field defines the maximum number of objects detected by the camera.
`-1` means no limit.
If more objects are visible to the camera, only the `maxObjects` biggest ones (considering pixel size) are recognized.

- The `occlusion` field defines if occlusions between the camera and the object should be checked and the accuracy that will be used.
If the `occlusion` field is set to `0`, then the occlusion computation will be disabled.
Disabling the occlusion can be useful to allow the camera to see through thin or transparent objects that may hide the object we are interested in, but it can lead to recognized objects that are not really visible to the camera.
If the `occlusion` field is set to `1`, only the center of the object is taken into account to compute if the object is visible or not.
Otherwise, if the accuracy is set to `2`, the outbound of the object is used to compute if the object is visible.
Note that increasing the `occlusion` field value decreases the simulation speed.

- The `frameColor` field defines the color used to frame the objects recognized by the camera in its overlay.

- The `frameThickness` field defines the thickness in pixels of the frames in the camera overlay.
0 means no object frame in the camera overlay.

- The `segmentation` field defines if a segmentation ground truth image is generated based on the [Solid](solid.md).`recognitionColors` field value.
Background and objects with empty `recognitionColors` field are rendered in black.
