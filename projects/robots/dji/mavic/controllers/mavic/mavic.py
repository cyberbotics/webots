from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

flLed = robot.getLED('front left led')
frLed = robot.getLED('front right led')

flp = robot.getMotor('front left propeller')
frp = robot.getMotor('front right propeller')
rlp = robot.getMotor('rear left propeller')
rrp = robot.getMotor('rear right propeller')

imu = robot.getInertialUnit('inertial unit')
imu.enable(timestep)
compass = robot.getCompass('compass')
compass.enable(timestep)
gps = robot.getGPS('gps')
gps.enable(timestep)

flp.setPosition(float('inf'))
flp.setVelocity(1.0)
frp.setPosition(float('inf'))
frp.setVelocity(-1.0)
rlp.setPosition(float('inf'))
rlp.setVelocity(-1.0)
rrp.setPosition(float('inf'))
rrp.setVelocity(1.0)


def clamp(x, lowerlimit, upperlimit):
    return max(lowerlimit, min(upperlimit, x))


def smoothstep(x, edge0=0.0, edge1=1.0):
    x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return x * x * (3 - 2 * x)


while robot.step(timestep) != -1:
    if robot.getTime() > 1:
        break

k = 28.0
kv = 3.0
kr = 5.0
kp = 5.0
ky = 0.1

tA = 0.5

while robot.step(timestep) != -1:
    ledState = int(robot.getTime()) % 2
    flLed.set(ledState)
    frLed.set(1 if ledState == 0 else 0)

    [roll, pitch, yaw] = imu.getRollPitchYaw()
    [x, y, z] = gps.getValues()

    roll += 1.5708
    r = kr * clamp(roll, -0.5, 0.5)
    p = kp * clamp(pitch, -0.5, 0.5)
    y = ky * clamp(-yaw, -0.5, 0.5)

    v = k + kv * clamp(tA - y, -0.3, 0.3)

    flp.setVelocity(v - r - p + y)
    frp.setVelocity(-(v + r - p - y))
    rlp.setVelocity(-(v - r + p - y))
    rrp.setVelocity(v + r + p + y)
