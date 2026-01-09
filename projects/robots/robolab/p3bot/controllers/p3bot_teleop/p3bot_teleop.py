from controller import Robot, Keyboard

# --- Robot initialization ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Wheels motors ---
wheel_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
wheels = [robot.getDevice(name) for name in wheel_names]
for wheel in wheels:
    wheel.setPosition(float('inf')) 

# --- Keyboard initialization ---
keyboard = Keyboard()
keyboard.enable(timestep)

# --- Robot parameters ---
WHEEL_RADIUS = 0.08
LX = 0.125
LY = 0.24
ROTATION_INCREMENT_COEFFICIENT = 1.0
SumLxLyOverRadius = (LX + LY) / WHEEL_RADIUS * ROTATION_INCREMENT_COEFFICIENT

# --- Velicities ---
V_LINEAR = 0.5   # m/s
V_ANGULAR = 1.0  # rad/s

# --- Main loop ---
while robot.step(timestep) != -1:
    # Desired velocities
    vx = 0.0     # forward/backward
    vy = 0.0     # lateral
    omega = 0.0  # rotation

    key = keyboard.getKey()
    while key != -1:
        if key == Keyboard.UP:
            vx = V_LINEAR
        elif key == Keyboard.DOWN:
            vx = -V_LINEAR
        elif key == Keyboard.RIGHT:
            vy = V_LINEAR
        elif key == Keyboard.LEFT:
            vy = -V_LINEAR
        elif key == ord('A'):
            omega = V_ANGULAR
        elif key == ord('D'):
            omega = -V_ANGULAR
        key = keyboard.getKey()

    # Mecanum cinematics
    w1 =  ( vx / WHEEL_RADIUS) - ( vy / WHEEL_RADIUS) + ( omega * SumLxLyOverRadius)
    w2 = -( vx / WHEEL_RADIUS) - ( vy / WHEEL_RADIUS) + ( omega * SumLxLyOverRadius)
    w3 =  ( vx / WHEEL_RADIUS) + ( vy / WHEEL_RADIUS) + ( omega * SumLxLyOverRadius)
    w4 = -( vx / WHEEL_RADIUS) + ( vy / WHEEL_RADIUS) + ( omega * SumLxLyOverRadius)

    # Apply velocities to wheels
    wheels[0].setVelocity(w1)
    wheels[1].setVelocity(w2)
    wheels[2].setVelocity(w3)
    wheels[3].setVelocity(w4)
