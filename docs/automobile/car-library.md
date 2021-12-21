## Car Library

The [car](#car-library) library is supposed to be used together with the [driver](driver-library.md) library.
It provides additional information and functions to which a normal human driver of a car does not have access (e.g., changing the blinking period of the indicator or getting the value of the wheels encoders).
All the functions included in this library are explained below.

### Car Library Functions

#### `Constructor`
#### `wbu_car_init`
#### `wbu_car_cleanup`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

void wbu_car_init();
void wbu_car_cleanup();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    Car();
    virtual ~Car();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    def __init__(self):
    def __del__(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public Car();
  protected void finalize();
  // ...
}
```

%tab-end

%tab "ROS"

> In ROS, car library initialization and cleanup are implicit.

%tab-end

%end

> **Note** [ROS]: To enable synchronous simulation you will have to call the `/robot/time_step` service with a positive `step` argument.
Then each time this service is called a car step will be executed (set the `step` argument to 0 to disable synchronization).

##### Description

*Initialise and clean*

These two functions are respectively used to initialize and properly close the [car](#car-library) library (the first one should be called at the very beginning of the controller program and the second one at the very end).
If you use the [driver](driver-library.md) library it is not needed to call these functions since they are already called from the corresponding functions of the [driver](driver-library.md) library.

---

#### `wbu_car_get_type`
#### `wbu_car_get_engine_type`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

typedef enum {
  WBU_CAR_TRACTION,
  WBU_CAR_PROPULSION,
  WBU_CAR_FOUR_BY_FOUR
} WbuCarType;

typedef enum {
  WBU_CAR_COMBUSTION_ENGINE,
  WBU_CAR_ELECTRIC_ENGINE,
  WBU_CAR_PARALLEL_HYBRID_ENGINE,
  WBU_CAR_POWER_SPLIT_HYBRID_ENGINE
} WbuCarEngineType;

WbuCarType wbu_car_get_type();
WbuCarEngineType wbu_car_get_engine_type();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    typedef enum {TRACTION, PROPULSION, FOUR_BY_FOUR} Type;
    typedef enum {COMBUSTION_ENGINE, ELECTRIC_ENGINE, PARALLEL_HYBRID_ENGINE, POWER_SPLIT_HYBRID_ENGINE} EngineType;

    Type getType();
    EngineType getEngineType();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    TRACTION, PROPULSION, FOUR_BY_FOUR
    COMBUSTION_ENGINE, ELECTRIC_ENGINE, PARALLEL_HYBRID_ENGINE, POWER_SPLIT_HYBRID_ENGINE

    def getType(self):
    def getEngineType(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public final static int TRACTION, PROPULSION, FOUR_BY_FOUR;
  public final static int COMBUSTION_ENGINE, ELECTRIC_ENGINE, PARALLEL_HYBRID_ENGINE, POWER_SPLIT_HYBRID_ENGINE;

  public int getType();
  public int getEngineType();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| [`/automobile/get_type`](car-library.md#wbu_car_get_type) | `service` | `webots_ros::get_int` | |
| [`/automobile/get_engine_type`](car-library.md#wbu_car_get_type) | `service` | `webots_ros::get_int` | |

%tab-end

%end

##### Description

*Get the car and engine type*

These two functions return respectively the type of transmission and of engine of the car.

%figure "WbuCarType enumeration"

| ENUM                     | Value |
| ------------------------ | ----- |
| `WBU_CAR_TRACTION`       | 0     |
| `WBU_CAR_PROPULSION`     | 1     |
| `WBU_CAR_FOUR_BY_FOUR`   | 2     |

%end

%figure "WbuCarEngineType enumeration"

| ENUM                                   | Value |
| -------------------------------------- | ----- |
| `WBU_CAR_COMBUSTION_ENGINE`            | 0     |
| `WBU_CAR_ELECTRIC_ENGINE`              | 1     |
| `WBU_CAR_PARALLEL_HYBRID_ENGINE`       | 2     |
| `WBU_CAR_POWER_SPLIT_HYBRID_ENGINE`    | 3     |

%end

---

#### `wbu_car_set_indicator_period`
#### `wbu_car_get_indicator_period`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

void wbu_car_set_indicator_period(double period);
double wbu_car_get_indicator_period();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    void setIndicatorPeriod(double period);
    double getIndicatorPeriod();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    def setIndicatorPeriod(self, period):
    def getIndicatorPeriod(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public void setIndicatorPeriod(double period);
  public double getIndicatorPeriod();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/set_indicator_period` | `service` | `webots_ros::set_float` | |
| `/automobile/get_indicator_period` | `service` | `webots_ros::get_float` | |

%tab-end

%end

##### Description

*Set and get the indicator period*

The `wbu_car_set_indicator_period` function is used to change the blinking period of the indicators.
The argument should be specified in seconds.

The `wbu_car_get_indicator_period` function returns the current blinking period of the indicators.

---

#### `wbu_car_get_backwards_lights`
#### `wbu_car_get_brake_lights`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

bool wbu_car_get_backwards_lights();
bool wbu_car_get_brake_lights();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    bool getBackwardsLights();
    bool getBrakeLights();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    def getBackwardsLights(self):
    def getBrakeLights(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public boolean getBackwardsLights();
  public boolean getBrakeLights();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/get_backwards_light` | `service` | `webots_ros::get_bool` | |
| `/automobile/get_brake_light` | `service` | `webots_ros::get_bool` | |

%tab-end

%end

##### Description

*Get the state of the backwards/brake lights*

These two functions return respectively the state of the backwards and brake lights (these two lights are switched on automatically by the library when appropriated).

---

#### `wbu_car_get_track_front`
#### `wbu_car_get_track_rear`
#### `wbu_car_get_wheelbase`
#### `wbu_car_get_front_wheel_radius`
#### `wbu_car_get_rear_wheel_radius`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

double wbu_car_get_track_front();
double wbu_car_get_track_rear();
double wbu_car_get_wheelbase();
double wbu_car_get_front_wheel_radius();
double wbu_car_get_rear_wheel_radius();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    double getTrackFront();
    double getTrackRear();
    double getWheelbase();
    double getFrontWheelRadius();
    double getRearWheelRadius();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    def getTrackFront(self):
    def getTrackRear(self):
    def getWheelbase(self):
    def getFrontWheelRadius(self):
    def getRearWheelRadius(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public double getTrackFront();
  public double getTrackRear();
  public double getWheelbase();
  public double getFrontWheelRadius();
  public double getRearWheelRadius();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/get_dimensions` | `service` | `webots_ros::automobile_get_dimensions` | `uint8 ask`<br/>---<br/>`float64 trackFront`<br/>`float64 trackRear`<br/>`float64 wheelBase`<br/>`float64 frontWheelRadius`<br/>`float64 rearWheelRadius` |

%tab-end

%end

##### Description

*Get car caracteristics*

All these functions provide important physical characteristics from the car.

---

#### `wbu_car_get_wheel_encoder`
#### `wbu_car_get_wheel_speed`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

typedef enum {
  WBU_CAR_WHEEL_FRONT_RIGHT,
  WBU_CAR_WHEEL_FRONT_LEFT,
  WBU_CAR_WHEEL_REAR_RIGHT,
  WBU_CAR_WHEEL_REAR_LEFT,
  WBU_CAR_WHEEL_NB
} WbuCarWheelIndex;

double wbu_car_get_wheel_encoder(WbuCarWheelIndex wheel_index);
double wbu_car_get_wheel_speed(WbuCarWheelIndex wheel_index);
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    typedef enum {WHEEL_FRONT_RIGHT, WHEEL_FRONT_LEFT, WHEEL_REAR_RIGHT, WHEEL_REAR_LEFT, WHEEL_NB} WheelIndex;

    double getWheelEncoder(WheelIndex wheel_index);
    double getWheelSpeed(WheelIndex wheel_index);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    WHEEL_FRONT_RIGHT, WHEEL_FRONT_LEFT, WHEEL_REAR_RIGHT, WHEEL_REAR_LEFT, WHEEL_NB

    def getWheelEncoder(self, wheel_index):
    def getWheelSpeed(self, wheel_index):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public final static int WHEEL_FRONT_RIGHT, WHEEL_FRONT_LEFT, WHEEL_REAR_RIGHT, WHEEL_REAR_LEFT, WHEEL_NB;

  public double getWheelEncoder(int wheel);
  public double getWheelSpeed(int wheel);
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/front_right_wheel_encoder`<br/>`/automobile/front_left_wheel_encoder`<br/>`/automobile/rear_right_wheel_encoder`<br/>`/automobile/rear_left_wheel_encoder`<br/> | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |
| `/automobile/front_right_wheel_speed`<br/>`/automobile/front_left_wheel_speed`<br/>`/automobile/rear_right_wheel_speed`<br/>`/automobile/rear_left_wheel_speed`<br/> | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |

%tab-end

%end

##### Description

*Get the wheels speed/encoder*

These two functions return respectively the state of the wheel encoder (in radians) and the instantaneous wheel rotational speed (in radians per second).
The `wheel_index` argument should match a value of the `WbuCarWheelIndex` enum.

%figure "WbuCarWheelIndex enumeration"

| ENUM                          | Value |
| ----------------------------- | ----- |
| `WBU_CAR_WHEEL_FRONT_RIGHT`   | 0     |
| `WBU_CAR_WHEEL_FRONT_LEFT`    | 1     |
| `WBU_CAR_WHEEL_REAR_RIGHT`    | 2     |
| `WBU_CAR_WHEEL_REAR_LEFT`     | 3     |
| `WBU_CAR_WHEEL_NB`            | 4     |

%end

---

#### `wbu_car_set_right_steering_angle`
#### `wbu_car_get_right_steering_angle`
#### `wbu_car_set_left_steering_angle`
#### `wbu_car_get_left_steering_angle`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

void wbu_car_set_right_steering_angle(double angle);
double wbu_car_get_right_steering_angle();
void wbu_car_set_left_steering_angle(double angle);
double wbu_car_get_left_steering_angle();
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    void setRightSteeringAngle(double angle);
    double getRightSteeringAngle();
    void setLeftSteeringAngle(double angle);
    double getLeftSteeringAngle();
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    def setRightSteeringAngle(self, angle):
    def getRightSteeringAngle(self):
    def setLeftSteeringAngle(self, angle):
    def getLeftSteeringAngle(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public void setRightSteeringAngle(double angle);
  public double getRightSteeringAngle();
  public void setLeftSteeringAngle(double angle);
  public double getLeftSteeringAngle();
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/right_steering_angle` | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |
| `/automobile/left_steering_angle` | `topic` | `webots_ros::Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |
| `/automobile/set_right_steering_angle` | `service` | `webots_ros::set_float` | |
| `/automobile/set_left_steering_angle` | `service` | `webots_ros::set_float` | |

%tab-end

%end

##### Description

*Set/get the right/left steering angle*

Functions `wbu_car_set_right_steering_angle` and `wbu_car_set_left_steering_angle` allow for direct setting of the steering angle for respectively the right and left wheel.
The difference between these setter functions and the usage of [`wbu_driver_set_steering_angle`](driver-library.md#wbu_driver_set_steering_angle) is that the latter computes and imposes a left and right steering angle based on the Ackermann steering geometry (which can yield different angles for left and right wheel), whereas these setters allow to specify the angle directly.
Functions `wbu_car_get_right_steering_angle` and `wbu_car_get_left_steering_angle` return the corresponding right and left steering angles, irrespective if these have been set directly or indirectly.

**Note**: Direct setting of the steering angles is useful especially for vehicles controlled in torque, for velocity control it is responsibility of the user to update the left and right speeds accordingly when changing the wheel angles as no automatic adaptation is made (contrary to the usage of [`wbu_driver_set_steering_angle`](driver-library.md#wbu_driver_set_steering_angle) where it does occur automatically).

---

#### `wbu_car_enable_limited_slip_differential`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

void wbu_car_enable_limited_slip_differential(bool enable);
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    void enableLimitedSlipDifferential(bool enable);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    def enableLimitedSlipDifferential(self, enable):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public void enableLimitedSlipDifferential(boolean enable);
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/enable_limited_slip_differential` | `service` | `webots_ros::set_bool` | |

%tab-end

%end

##### Description

*Enable/disable the limited slip differential mechanism*

This function allows the user to enable or disable the limited differential slip (it is enabled by default).
When the limited differential slip is enabled, at each time step, the torque (when control in torque is enabled) is redistributed amongst all the actuated wheels so that they rotate at the same speed (except the difference due to the geometric differential constraint).
If the limited differential slip is disabled, when a wheel starts to slip, it will rotate faster than the others.

---

#### `wbu_car_enable_indicator_auto_disabling`

%tab-component "language"

%tab "C"

```c
#include <webots/vehicle/car.h>

void wbu_car_enable_indicator_auto_disabling(bool enable);
```

%tab-end

%tab "C++"

```cpp
#include <webots/vehicle/Car.hpp>

namespace webots {
  class Car : public Driver {
    void enableIndicatorAutoDisabling(bool enable);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from vehicle import Car

class Car (Driver):
    def enableIndicatorAutoDisabling(self, enable):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.vehicle.Car;

public class Car extends Driver {
  public void enableIndicatorAutoDisabling(boolean enable);
  // ...
}
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/automobile/enable_indicator_auto_disabling` | `service` | `webots_ros::set_bool` | |

%tab-end

%end

##### Description

*Enable/disable the auto-disabling mechanism of the indicator*

This function allows the user to enable or disable the indicator auto-disabling mechanism (it is enabled by default).
When indicator auto-disabling mechanism is enabled, the indicator is automatically switched off when the car starts steering in the inverse direction of the one indicated by the indicator.
