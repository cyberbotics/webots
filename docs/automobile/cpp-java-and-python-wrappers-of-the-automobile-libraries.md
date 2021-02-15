## C++, Java and Python Wrappers of the Automobile Libraries

The [driver](driver-library.md) and [car](car-library.md) libraries are also available as oriented-object wrappers for the C++, the Java and the Python languages.

The [Driver](driver-library.md) and [Car](car-library.md) classes are containing all the methods described in the C API.
Camel case is used to define the method names.
The `init` and `cleanup` functions are called automatically from the constructor/destructor of the [Driver](driver-library.md) and [Car](car-library.md) classes.

> **Note** [Java]: The following program shows how to set the cruising speed and the steering angle in Java:

> ```java
> import com.cyberbotics.webots.controller.Robot;
> import com.cyberbotics.webots.controller.vehicle.Driver;
>
> public class VehicleDriver {
>   public static void main(String[] args) {
>     Driver driver = new Driver();
>     driver.setCruisingSpeed(20.0);
>
>     while (driver.step() != -1) {
>       double angle = 0.3 * Math.cos(driver.getTime());
>       driver.setSteeringAngle(angle);
>     };
>   }
> }
> ```
>
> You have to define the `CLASSPATH` environment variable and point it to `${WEBOTS_HOME}/lib/controller/java/vehicle.jar`.
> There is an example on how to add the `CLASSPATH` environment variable to `Makefile` and `runtime.ini` in [WEBOTS\_HOME/projects/vehicles/controllers/VehicleDriver]({{ url.github_tree }}/projects/vehicles/controllers/VehicleDriver).

> **Note** [Python]: The following program shows how to set the cruising speed and the steering angle in Python:

> ```python
> import math
>
> from vehicle import Driver
>
> driver = Driver()
> driver.setSteeringAngle(0.2)
> driver.setCruisingSpeed(20)
>
> while driver.step() != -1:
>   angle = 0.3 * math.cos(driver.getTime())
>   driver.setSteeringAngle(angle)
> ```
