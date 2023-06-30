## Driver Library

The [driver](#driver-library) library provides all the usual functionalities available to a human driving his own car.
All the functions included in this library are explained below.

### Driver Library Functions

#### `Constructor`
#### `wbu_driver_init`
#### `wbu_driver_cleanup`
#### `wbu_driver_step`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

void wbu_driver_init();
void wbu_driver_cleanup();
int wbu_driver_step();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    Driver();
    virtual ~Driver();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def __init__(self):
    def __del__(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public Driver();
  protected void finalize();
  // ...
}
```

%tab-end

%tab "ROS"

> In ROS, car library initialization and cleanup are implicit.

%tab-end

%end

##### Description

*Initialise, clean and run a driver step*

These functions are the equivalent of the init, cleanup and step function of any regular [Robot](../reference/robot.md) controller.
As a reminder, the `init` function should be called at the very beginning of any controller program, the `cleanup` function at the end of the controller program just before exiting and the `step` function should be called in the main loop to run one simulation step.
Unlike the robot step, the driver step does not have any argument, the default time step of the world being used.

---

#### `wbu_driver_set_steering_angle`
#### `wbu_driver_get_steering_angle`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

void wbu_driver_set_steering_angle(double steering_angle);
double wbu_driver_get_steering_angle();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    void setSteeringAngle(double steeringAngle);
    double getSteeringAngle();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def setSteeringAngle(self, steeringAngle):
    def getSteeringAngle(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public void setSteeringAngle(double steeringAngle);
  public double getSteeringAngle();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/steering_angle` | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |
| `/automobile/set_steering_angle` | `service` | `webots_ros::set_float` | |

%tab-end

%end

##### Description

*Set and get the steering angle*

The `wbu_driver_set_steering_angle` function is used to steer the car, it steers the front wheels according to the Ackermann geometry (left and right wheels are not steered with the exact same angle).
The angle is set in radians, a positive angle steers right and a negative angle steers left.
The formulas used in order to compute the right and left angles are the following (`trackFront` and `wheelbase` are the parameters of the [Car](https://webots.cloud/run?url={{ url.github_tree }}/projects/vehicles/protos/abstract/Car.proto) PROTO):


```c
angle_right = atan(1 / (cot(steering_angle) - trackFront / (2 * wheelbase)));
angle_left = atan(1 / (cot(steering_angle) + trackFront / (2 * wheelbase)));
```

The `wbu_driver_get_steering_angle` function returns the current steering angle.
**Note**: When the steering angle of the left and right wheels is imposed directly using the [`wbu_car_set_[right/left]_steering_angle`](car-library.md#wbu_car_set_right_steering_angle) function no update is made to the overall steering angle, in other worlds, calling this function will return zero or the last value set.

---

#### `wbu_driver_set_cruising_speed`
#### `wbu_driver_get_target_cruising_speed`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

void wbu_driver_set_cruising_speed(double speed);
double wbu_driver_get_target_cruising_speed();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    void setCruisingSpeed(double speed);
    double getTargetCruisingSpeed();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def setCruisingSpeed(self, speed):
    def getTargetCruisingSpeed(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public void setCruisingSpeed(double speed);
  public double getTargetCruisingSpeed();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/set_cruising_speed` | `service` | `webots_ros::set_float` | |
| `/automobile/get_cruising_speed` | `service` | `webots_ros::get_float` | |

%tab-end

%end

##### Description

*Set and get the target cruising speed*

The `wbu_driver_set_cruising_speed` function activates the control in cruising speed of the car, the rotational speed of the wheels is forced (respecting the geometric differential constraint) in order for the car to move at the speed given in argument of the function (in kilometers per hour).
When the control in cruising speed is activated, the speed is directly applied to the wheel without any engine model simulation, therefore any call to functions like `wbu_driver_get_rpm` will raise an error.
The acceleration of the car is computed using the `time0To100` field of the [Car](https://webots.cloud/run?url={{ url.github_tree }}/projects/vehicles/protos/abstract/Car.proto) PROTO.

The `wbu_driver_get_target_cruising_speed` function simply returns the target cruising speed (argument of the last call to the `wbu_driver_set_cruising_speed` function).

---

#### `wbu_driver_get_current_speed`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

double wbu_driver_get_current_speed();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    double getCurrentSpeed();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def getCurrentSpeed(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public double getCurrentSpeed();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/current_speed` | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |

%tab-end

%end

##### Description

*Get the current speed*

This function returns the current speed of the car (in kilometers per hour).
The estimated speed is computed using the rotational speed of the actuated wheels and their respective radius.

---

#### `wbu_driver_set_throttle`
#### `wbu_driver_get_throttle`


%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

void wbu_driver_set_throttle(double throttle);
double wbu_driver_get_throttle();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    void setThrottle(double throttle);
    double getThrottle();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def setThrottle(self, throttle):
    def getThrottle(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public void setThrottle(double throttle);
  public double getThrottle();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/set_throttle` | `service` | `webots_ros::set_float` | |
| `/automobile/throttle` | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |

%tab-end

%end

##### Description

*Set and get the throttle*

The `wbu_driver_set_throttle` function is used in order to control the car in torque, it sets the state of the throttle.
The argument should be between 0.0 and 1.0, 0 means that 0% of the output torque of the engine is sent to the wheels and 1.0 means that 100% of the output torque of the engine is sent to the wheels.
For more information about how the output torque of the engine is computed see section [Engine models](#engine-models).

The `wbu_driver_get_throttle` function simply returns the state of the throttle (argument of the last call to the `wbu_driver_set_throttle` function).

---

#### `wbu_driver_set_brake_intensity`
#### `wbu_driver_get_brake_intensity`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

void wbu_driver_set_brake_intensity(double intensity);
double wbu_driver_get_brake_intensity();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    void setBrakeIntensity(double intensity);
    double getBrakeIntensity();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def setBrakeIntensity(self, intensity):
    def getBrakeIntensity(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public void setBrakeIntensity(double intensity);
  public double getBrakeIntensity();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/set_brake_intensity` | `service` | `webots_ros::set_float` | |
| `/automobile/brake_intensity` | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |

%tab-end

%end

##### Description

*Set and get the brake intensity*

The `wbu_driver_set_brake_intensity` function brakes the car by increasing the `dampingConstant` coefficient of the rotational joints of each of the four wheels.
The argument should be between 0.0 and 1.0, 0 means that no damping constant is added on the joints (no breaking), 1 means that the parameter `brakeCoefficient` of the [Car](https://webots.cloud/run?url={{ url.github_tree }}/projects/vehicles/protos/abstract/Car.proto) PROTO is applied on the `dampingConstant` of each joint (the value will be linearly interpolated between 0 and `brakeCoefficient` for any arguments between 0 and 1).

The `wbu_driver_get_brake_intensity` function simply returns the current brake intensity (argument of the last call to the `wbu_driver_set_brake_intensity` function).

---

#### `wbu_driver_set_indicator`
#### `wbu_driver_get_indicator`
#### `wbu_driver_set_hazard_flashers`
#### `wbu_driver_get_hazard_flashers`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

typedef enum {
  OFF,
  RIGHT,
  LEFT
} WbuDriverIndicatorState;

void wbu_driver_set_indicator(WbuDriverIndicatorState state);
WbuDriverIndicatorState wbu_driver_get_indicator();
void wbu_driver_set_hazard_flashers(bool state);
bool wbu_driver_get_hazard_flashers();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    typedef enum {INDICATOR_OFF, INDICATOR_RIGHT, INDICATOR_LEFT} IndicatorState;

    void setIndicator(IndicatorState state);
    IndicatorState getIndicator();
    void setHazardFlashers(bool state);
    bool getHazardFlashers();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    INDICATOR_OFF, INDICATOR_RIGHT, INDICATOR_LEFT

    def setIndicator(self, state):
    def getIndicator(self):
    def setHazardFlashers(self, state):
    def getHazardFlashers(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public final static int INDICATOR_OFF, INDICATOR_RIGHT, INDICATOR_LEFT;

  public void setIndicator(int state);
  public int getIndicator();
  public void setHazardFlashers(boolean state);
  public boolean getHazardFlashers();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/set_indicator` | `service` | `webots_ros::set_bool` | |
| `/automobile/get_indicator` | `service` | `webots_ros::get_bool` | |
| `/automobile/set_hazard_flashers` | `service` | `webots_ros::set_bool` | |
| `/automobile/get_hazard_flashers` | `service` | `webots_ros::get_bool` | |

%tab-end

%end

##### Description

*Set and get the indicator state*

The `wbu_driver_set_indicator` function allows the user to set (using the `WbuDriverIndicatorState` enum) if the indicator should be on only for the right side of the car, the left side of the car or should be off.
The `wbu_driver_get_indicator` function allows the user to get the indicator state.

%figure "WbuDriverIndicatorState enumeration"

| ENUM    | Value |
| ------- | ----- |
| `OFF`   | 0     |
| `RIGHT` | 1     |
| `LEFT`  | 2     |

%end

The `wbu_driver_set_hazard_flashers` function allows the user to switch the hazard flashers on (indicator on both side of the car) or off.
The `wbu_driver_get_hazard_flashers` function allows the user to get the state of the hazard flashers.

---

#### `wbu_driver_set_dipped_beams`
#### `wbu_driver_set_antifog_lights`
#### `wbu_driver_get_dipped_beams`
#### `wbu_driver_get_antifog_lights`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

void wbu_driver_set_dipped_beams(bool state);
void wbu_driver_set_antifog_lights(bool state);
bool wbu_driver_get_dipped_beams();
bool wbu_driver_get_antifog_lights();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    void setDippedBeams(bool state);
    void setAntifogLights(bool state);
    bool getDippedBeams();
    bool getAntifogLights();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def setDippedBeams(self, state):
    def setAntifogLights(self, state):
    def getDippedBeams(self):
    def getAntifogLights(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public void setDippedBeams(boolean state);
  public void setAntifogLights(boolean state);
  public boolean getDippedBeams();
  public boolean getAntifogLights();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/set_dipped_beam` | `service` | `webots_ros::set_bool` | |
| `/automobile/set_antifog_light` | `service` | `webots_ros::set_bool` | |
| `/automobile/get_antifog_light` | `service` | `webots_ros::get_bool` | |
| `/automobile/get_dipped_beam` | `service` | `webots_ros::get_bool` | |

%tab-end

%end

##### Description

*Set and get the lights*

The `wbu_driver_set_dipped_beams` and `wbu_driver_set_antifog_lights` functions are used to enable or disable the dipped beams and the anti-fog lights.

The `wbu_driver_get_dipped_beams` and `wbu_driver_get_antifog_lights` functions return the state of the dipped beams or the anti-fog lights.

---

#### `wbu_driver_get_rpm`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

double wbu_driver_get_rpm();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    double getRpm();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def getRpm(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public double getRpm();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/rpm` | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |

%tab-end

%end

##### Description

*Get the motor rpm*

This function returns the estimation of the engine rotation speed.

> **Note**: If the control in cruising speed is enabled, this function returns an error because there is no engine model when control in cruising speed is enabled.

---

#### `wbu_driver_set_gear`
#### `wbu_driver_get_gear`
#### `wbu_driver_get_gear_number`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

void wbu_driver_set_gear(int gear);
int wbu_driver_get_gear();
int wbu_driver_get_gear_number();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    void setGear(int gear);
    int getGear();
    int getGearNumber();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    def setGear(self, gear):
    def getGear(self):
    def getGearNumber(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public void setGear(int gear);
  public int getGear();
  public int getGearNumber();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/set_gear` | `service` | `webots_ros::set_int` | |
| `/automobile/get_gear` | `service` | `webots_ros::get_int` | |
| `/automobile/get_gear_number` | `service` | `webots_ros::get_int` | |

%tab-end

%end

##### Description

*Get and set the gear*

The `wbu_driver_set_gear` function sets the engaged gear.
An argument of `-1` is used in order to engage the reverse gear, an argument of `0` is used in order to disengaged the gearbox.
Any other arguments than `0` and `-1` should be between 1 and the number of coefficients set in the `gearRatio` parameter of the [Car](https://webots.cloud/run?url={{ url.github_tree }}/projects/vehicles/protos/abstract/Car.proto) PROTO.

The `wbu_driver_get_gear` function returns the currently engaged gear.

The `wbu_driver_get_gear_number` function simply returns the number of available gears (including the reverse gear).

---

#### `wbu_driver_get_control_mode`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

typedef enum {
  UNDEFINED_CONTROL_MODE = -1,
  SPEED = 0,
  TORQUE
} WbuDriverControlMode;

WbuDriverControlMode wbu_driver_get_control_mode();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    typedef enum {SPEED, TORQUE} ControlMode;

    ControlMode getControlMode();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    SPEED, TORQUE

    def getControlMode(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public final static int SPEED, TORQUE;

  public int getControlMode();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/get_control_mode` | `service` | `webots_ros::get_int` | |

%tab-end

%end

##### Description

*Get the control mode*

This `wbu_driver_get_control_mode` returns the current control mode of the car.

%figure "WbuDriverControlMode enumeration"

| ENUM     | Value |
| -------- | ----- |
| `SPEED`  | 0     |
| `TORQUE` | 1     |

%end

---

#### `wbu_driver_set_wiper_mode`
#### `wbu_driver_get_wiper_mode`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/driver.h>

typedef enum {
  DOWN,
  SLOW,
  NORMAL,
  FAST
} WbuDriverWiperMode;

void wbu_driver_set_wiper_mode(WbuDriverWiperMode mode);
WbuDriverWiperMode wbu_driver_get_wiper_mode();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Driver {
    typedef enum {DOWN, SLOW, NORMAL, FAST} WiperMode;

    void setWiperMode(WiperMode mode);
    WiperMode getWiperMode();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Driver

class Driver:
    DOWN, SLOW, NORMAL, FAST

    def setWiperMode(self, mode):
    def getWiperMode(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Driver;

public class Driver {
  public final static int DOWN, SLOW, NORMAL, FAST;

  public void setWiperMode(int mode);
  public int getWiperMode();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/set_wiper_mode` | `service` | `webots_ros::set_int` | |
| `/automobile/get_wiper_mode` | `service` | `webots_ros::get_int` | |

%tab-end

%end

##### Description

*Set and get the wipers' mode*

The `wbu_driver_set_wiper_mode` function allows the user to set (using the `WbuDriverWiperMode` enum) various speeds for the wipers from slow to fast.
Whilst the slow and normal mode share the same speed, the slow mode activates the wipers once every few seconds.
The `wbu_driver_get_wiper_mode` function allows the user to get the wipers' mode.

%figure "WbuDriverWiperMode enumeration"

| ENUM     | Value |
| -------- | ----- |
| `DOWN`   | 0     |
| `SLOW`   | 1     |
| `NORMAL` | 2     |
| `FAST`   | 3     |

%end

### Engine Models

When the control in torque of the car is enabled, at each step the output torque of the engine is recomputed.
First the rotational speed of the engine is estimated from the rotational speed of the wheels, then the output torque of the engine is computed (the formula depends on the engine type) using the rotational speed of the engine, then this output torque is multiplied by the state of the throttle and the gearbox coefficient, finally the torque is distributed (respecting the differential constraint) on the actuated wheels (depending on the car type).

#### Combustion Engine

If `a`, `b` and `c` are the values of the `engineFunctionCoefficients` parameter of the `Car` PROTO, the output torque is:

```
output_torque = c * rpm² + b * rpm + a
```

> **Note**: if the rpm is below the `engineMinRPM` parameter of the [Car](https://webots.cloud/run?url={{ url.github_tree }}/projects/vehicles/protos/abstract/Car.proto) PROTO, `engineMinRPM` is used instead of the real rpm, but if the rpm is above the `engineMaxRPM` parameter, then the output torque is 0.

#### Electric Engine

If `maxP` and `maxT` are respectively the `engineMaxPower` and `engineMaxTorque` parameters of the `Car` PROTO, the ouput torque is:

```
output_torque = min(maxT; maxP * 60 / (2 * pi * rpm))
```

#### Parallel Hybrid Engine

In that case, the output torque is simply the sum of the two previous models.
But if the real rpm is below the `engineMinRPM` parameter of the `Car` PROTO, the combustion engine is switched off.

#### Serial Hybrid Engine

Since this case is very similar to the electric engine model (from a simulation point of view), you should use the electric model instead.

#### Power-Split Hybrid Engine

If `ratio` and `splitRpm` are respectively the `hybridPowerSplitRatio` and `hybridPowerSplitRPM` parameters of the `Car` PROTO, the output torque is computed as follow:

```
output_torque_c = c * splitRpm² + b * splitRpm + a
output_torque_e = min(maxT; maxP * 60 / 2 * pi * rpm)
output_torque_total = output_torque_e + (1 - ratio) * output_torque_c
```

Here again, if the real rpm is below the `engineMinRPM` parameter of the `Car` PROTO the combustion engine is switched off.
