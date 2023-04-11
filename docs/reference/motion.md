## Motion

### Motion Functions

#### `wbu_motion_new`
#### `wbu_motion_delete`

%tab-component "language"

%tab "C"

```c
#include <webots/utils/motion.h>

WbMotionRef wbu_motion_new(const char *filename);
void wbu_motion_delete(WbMotionRef motion);
```

%tab-end

%tab "C++"

```cpp
#include <webots/utils/Motion.hpp>

namespace webots {
  class Motion {
    Motion(const std::string &fileName);
    virtual ~Motion();
    bool isValid() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Motion

class Motion:
    def __init__(self, fileName):
    def __del__(self):
    def isValid(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Motion;

public class Motion {
  public Motion(String fileName);
  protected void finalize();
  public boolean isValid();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
motion = wbu_motion_new('filename')
wbu_motion_delete(motion)
```

%tab-end

%end

##### Description

*obtaining and releasing a motion file handle*

The `wbu_motion_new` function allows to read a motion file specified by the `filename` parameter.
The `filename` can be specified either with an absolute path or a path relative to the controller directory.
If the file can be read, if its syntax is correct and if it contains at least one pose and one joint position, then the `wbu_motion_new` function returns a `WbMotionRef` that can be used as parameter in further `wbu_motion_*` function calls.
If an error occurred, an error message is printed to Webots' console, and `NULL` is returned.
Motions are created in *stopped mode*, the `wbu_motion_play` function must be called to start the playback.

The `wbu_motion_delete` function frees all the memory associated with the `WbMotionRef`.
This `WbMotionRef` can no longer be used afterwards.

> **Note** [C++, Java, Python]: The constructor and destructor of the Motion class are used instead of the `wbu_motion_new` and `wbu_motion_delete` functions.
In these languages, an error condition can be detected by calling the `isValid` function after the constructor.
If the `isValid` function yields `false` then the `Motion` object should be explicitly deleted.
See example below.

```cpp
Motion *walk = new Motion(filename);
if (! walk->isValid()) {
  cerr << "could not load file: " << filename << endl;
  delete walk;
}
```

---

#### `wbu_motion_play`
#### `wbu_motion_stop`
#### `wbu_motion_set_loop`
#### `wbu_motion_set_reverse`

%tab-component "language"

%tab "C"

```c
#include <webots/utils/motion.h>

void wbu_motion_play(WbMotionRef motion);
void wbu_motion_stop(WbMotionRefmotion);
void wbu_motion_set_loop(WbMotionRef motion, bool loop);
void wbu_motion_set_reverse(WbMotionRefmotion, bool reverse);
```

%tab-end

%tab "C++"

```cpp
#include <webots/utils/Motion.hpp>

namespace webots {
  class Motion {
    virtual void play();
    virtual void stop();
    virtual void setLoop(bool loop);
    virtual void setReverse(bool reverse);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Motion

class Motion:
    def play(self):
    def stop(self):
    def setLoop(self, loop):
    def setReverse(self, reverse):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Motion;

public class Motion {
  public void play();
  public void stop();
  public void setLoop(boolean loop);
  public void setReverse(boolean reverse);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wbu_motion_play(motion)
wbu_motion_stop(motion)
wbu_motion_set_loop(motion, loop)
wbu_motion_set_reverse(motion, reverse)
```

%tab-end

%end

##### Description

*Controlling motion files playback*

The `wbu_motion_play` function starts the playback of the specified motion.
This function registers the motion to the playback system, but the effective playback happens in the background and is activated as a side effect of calling the `wb_robot_step` function.
If you want to play a file and wait for its termination you can do it with this simple function:

```c
void my_motion_play_sync(WbMotionRef motion)
{
  wbu_motion_play(motion);
  do {
    wb_robot_step(TIME_STEP);
  }
  while (! wbu_motion_is_over(motion));
}
```

Several motion files can be played simultaneously by the same robot, however if two motion files have common joints, the behavior is undefined.

Note that the steps of the `wb_robot_step` function and the pose intervals in the motion file can differ.
In this case Webot computes intermediate joint positions by linear interpolation.

The `wbu_motion_stop` function interrupts the playback of the specified motion but preserves the current position.
After interruption the playback can be resumed with a `wbu_motion_play` function call.

The `wbu_motion_set_loop` function sets the *loop mode* of the specified motion.
If the *loop mode* is `true`, the motion repeats when it reaches either the end or the beginning (*reverse mode*) of the file.
The *loop mode* can be used, for example, to let a robot repeat a series of steps in a walking sequence.
Note that the loop mode can be changed while the motion is playing.

The `wbu_motion_set_reverse` function sets the *reverse mode* of the specified motion.
If the *reverse mode* is `true`, the motion file plays backwards.
For example, by using the *reverse mode*, it may be possible to turn a forwards walking motion into a backward walking motion.
The *reverse mode* can be changed while the motion is playing, in this case, the motion will go back from its current position.

By default, the *loop mode* and *reverse mode* of motions are `false`.

---

#### `wbu_motion_is_over`
#### `wbu_motion_get_duration`
#### `wbu_motion_get_time`
#### `wbu_motion_set_time`

%tab-component "language"

%tab "C"

```c
#include <webots/utils/motion.h>

bool wbu_motion_is_over(WbMotionRef motion);
int wbu_motion_get_duration(WbMotionRefmotion);
int wbu_motion_get_time(WbMotionRef motion);
void wbu_motion_set_time(WbMotionRefmotion, int t);
```

%tab-end

%tab "C++"

```cpp
#include <webots/utils/Motion.hpp>

namespace webots {
  class Motion {
    bool isOver() const;
    int getDuration() const;
    int getTime() const;
    virtual void setTime(int time);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Motion

class Motion:
    def isOver(self):
    def getDuration(self):
    def getTime(self):
    def setTime(self, time):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Motion;

public class Motion {
  public boolean isOver();
  public int getDuration();
  public int getTime();
  public void setTime(int time);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
over = wbu_motion_is_over(motion)
duration = wbu_motion_get_duration(motion)
time = wbu_motion_get_time(motion)
wbu_motion_set_time(motion, time)
```

%tab-end

%end

##### Description

*controlling the playback position*

The `wbu_motion_is_over` function returns `true` when the playback position has reached the end of the motion file.
That is when the last pose has been sent to the [Motor](motor.md) nodes using the `wb_motor_set_position` function.
But this does not mean that the motors have yet reached the specified positions; they may be slow or blocked by obstacles, robots, walls, the floor, etc.
If the motion is in *loop mode*, this function returns always `false`.
Note that the `wbu_motion_is_over` funciton depends on the *reverse mode*.
The `wbu_motion_is_over` function returns `true` when *reverse mode* is `true` and the playback position is at the beginning of the file or when *reverse mode* is `false` and the playback position is at the end of the file.

The `wbu_motion_get_duration` function returns the total duration of the motion file in milliseconds.

The `wbu_motion_get_time` function returns the current playback position in milliseconds.

The `wbu_motion_set_time` function allows to change the playback position in time.
This allows the user to skip forwards or backwards.
Note that, the time position can be changed whether the motion is playing or stopped.
The minimum value is 0 (beginning of the motion), and the maximum value is the value returned by the `wbu_motion_get_duration` function (end of the motion).
The time position is expressed in milliseconds.
