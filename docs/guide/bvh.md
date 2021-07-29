## BVH Utility Functions

### BVH Format

The Biovision Hierarchy animation file format (.bvh) was developed to capture and distribute motion capture data, typically of humans performing actions such as walking, running, etc.
This file format was developed by the Biovision company (now defunct).
There are several freely available human motion capture data resources in BVH format, which can be used for making animations.
This BVH utility is provided to enable reading BVH files and animate human models defined using the [Skin](../reference/skin.md) node.

The BVH file define a skeleton in the form of hierarchy of bones, and a time series of joint angles for each joint in the skeleton.
Note that this skeleton is not necessarily the same as the skeleton associated with a [Skin](../reference/skin.md) node, since they come from different sources.
This library provides functions to adapt and re-target BVH motion data to [Skin](../reference/skin.md).

### BVH Utility Functions

#### `wbu_bvh_read_file`
#### `wbu_bvh_cleanup`

%tab-component "language"

%tab "C"

```c
#include "bvh_reader.h"

WbBvhDataRef wbu_bvh_read_file(const char *filename);
void wbu_bvh_cleanup(WbBvhDataRef motion);
```

%tab-end

%end

##### Description

*obtaining and releasing a BVH file handle*

The `wbu_bvh_read_file` function parses the hierarchical skeleton data and motion data in the specified BVH file.
The `filename` can be specified either with an absolute path or a path relative to the controller directory.
The function returns a `WbBvhDataRef` object, which is a reference to the data read from the BVH file.
This object is NULL if an error occurred when reading the BVH file, for example when the file is corrupted or the path is incorrect.

The `wbu_bvh_cleanup` function frees all the memory associated with the `WbBvhDataRef` object.
After this function is called the corresponding `WbBvhDataRef` object can no longer be used.

<!-- -->

> **Note** [C++, Java, Python]:  In object-oriented languages there is no `wbu_bvh_cleanup` function because it is automatically called by the destructor.

---

#### `wbu_bvh_get_joint_count`
#### `wbu_bvh_get_joint_name`
#### `wbu_bvh_get_frame_count`
#### `wbu_bvh_set_scale`

%tab-component "language"

%tab "C"

```c
#include "bvh_reader.h"

int wbu_bvh_get_joint_count(WbBvhDataRef ref);
const char* wbu_bvh_get_joint_name(WbBvhDataRef ref, int joint_index);
int wbu_bvh_get_frame_count(WbBvhDataRef ref);
void wbu_bvh_set_scale(WbBvhDataRef ref, double scale);
```

%tab-end

%end

##### Description

*query joints and frames in the BVH file and set scale for translation data*

The `wbu_bvh_get_joint_count` function returns the number of joints defined in the loaded BVH file.

The `wbu_bvh_get_joint_name` function returns the name of the joint indexed by the number `joint_index` in the loaded BVH file.

The `wbu_bvh_get_frame_count` function returns the number of frames of motion data in the loaded BVH file. Each frame corresponds to one entry of time-series data.


The `wbu_bvh_set_scale` function sets the scale factor, which is used to scale the translation data form the BVH animation file.
The default scale factor is 1.0.
Note that this value does not affect the rotation data.

---

#### `wbu_bvh_step`
#### `wbu_bvh_goto_frame`
#### `wbu_bvh_reset`

%tab-component "language"

%tab "C"

```c
#include "bvh_reader.h"

bool wbu_bvh_step(WbBvhDataRef ref);
bool wbu_bvh_goto_frame(WbBvhDataRef ref, int frame_number);
bool wbu_bvh_reset(WbBvhDataRef ref);
```

%tab-end

%end

##### Description

*read next frame, read a specific frame, or jump to first frame*

The `wbu_bvh_step` function reads the joint angles and translation data in the next frame.
This function is typically called at the beginning of the main loop.
If the current frame is the last frame in the BVH file, calling this function reads the first frame again.
Returns TRUE on success.

The function `wbu_bvh_goto_frame` reads a specific frame, whose index is the argument `frame_number`.
Returns TRUE on success.
Note that if the argument `frame_number` is greater than the number of frames in the file, it leads to an error, and the function returns FALSE.

The function `wbu_bvh_reset` jumps to the first frame.
Returns TRUE on success.

``` c
void main() {
  WbBvhDataRef ref = wbu_bvh_read_file(filename);
  do {
    wbu_bvh_step(ref);
    // Read the joint angles and do something
  } while (condition);
}
```

---

#### `wbu_bvh_get_joint_rotation`
#### `wbu_bvh_get_root_translation`

%tab-component "language"

%tab "C"

```c
#include "bvh_reader.h"

double *wbu_bvh_get_joint_rotation(WbBvhDataRef ref, int joint_index);
double *wbu_bvh_get_root_translation(WbBvhDataRef ref);
```

%tab-end

%end

##### Description

*get the joint rotation for a specific joint and the BVH object translation*

The `wbu_bvh_get_joint_rotation` function returns the rotation of the joint specified by `joint_index`.
The rotation is returned as an array of double.

The `wbu_bvh_get_root_translation` function returns the translation of the BVH object.
The translation is returned as an array of double.
This is typically used to set the translation of the [Skin](../reference/skin.md) node that is being animated by this BVH file.
The values can wither be used directly, or scaled using the `wbu_bvh_set_scale` function.

``` c
void main() {
  WbBvhDataRef ref = wbu_bvh_read_file(filename);
  int i = 0;
  do {
    wbu_bvh_step(ref);
    for ( i = 0; i &lt joint_count; ++i){
      double *rotation = wbu_bvh_get_joint_rotation(ref, i);
      double axis_x = rotation[0];
      double axis_y = rotation[1];
      double axis_z = rotation[2];
      double angle  = rotation[3];
      // Do something with the joint angles
    }
    double *position = wbu_bvh_get_root_translation(ref);
    double x = position[0];
    double y = position[1];
    double z = position[2];
    // Do something with the translation
  } while (condition);
}
```

---

#### `wbu_bvh_adapt_skeleton`

%tab-component "language"

%tab "C"

```c
#include "bvh_reader.h"

void wbu_bvh_adapt_skeleton(WbBvhDataRef ref, int num_joints, const char** joint_name_list);
```

%tab-end

%end

##### Description

*adapt the skeleton specified in the BVH*

The `wbu_bvh_adapt_skeleton` function adapts the skeleton defined in the BVH file to be able to re-target the BVH motion data to the [Skin](#skin) Node.
This is necessary because the skeleton of the [Skin](#skin) node will generally be different from the skeleton defined in the BVH file.
The difference can be in the joint hierarchy and in the number of joints.
Therefore, firstly, the extra joints in the BVH file must be removed.
Further, since the joint rotations are specified as relative to parent joints, they must be recomputed in case a joint is removed.
This computation is done by this function.

The arguments to this function are the number of bones and the list of bones names in the [Skin](../reference/skin.md) node.
They can be obtained by calling the [`wb_skin_get_bone_count`](../reference/skin.md#wb_skin_get_bone_count) and [`wb_skin_get_bone_name`](../reference/skin.md#wb_skin_get_bone_name) functions.
Note that if the nomenclature of the joints in the [Skin](../reference/skin.md) node is different from that of the BVH file, the names have to be manually translated.
Please refer to the advanced sample project in ["WEBOTS\_HOME/projects/humans/skin_animation"]({{ url.github_tree }}/projects/humans/skin_animation/) directory for an example of how this is done.
