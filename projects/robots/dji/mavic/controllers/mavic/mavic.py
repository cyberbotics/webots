from controller import Robot, Keyboard
import math

robot = Robot()

timestep = int(robot.getBasicTimeStep())

camera = robot.getCamera('camera')
camera.enable(timestep)

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


def sign(x):
    return math.copysign(1, x)


keyboard = robot.getKeyboard()
keyboard.enable(timestep)

while robot.step(timestep) != -1:
    if robot.getTime() > 1:
        break

k = 28.7
kv = 2.5
kv2 = 0.0002
kr = 5.0
kp = 5.0
ky = 0.2

tA = 10.0

sum = 0.0

while robot.step(timestep) != -1:
    ledState = int(robot.getTime()) % 2
    flLed.set(ledState)
    frLed.set(1 if ledState == 0 else 0)

    [roll, pitch, yaw] = imu.getRollPitchYaw()
    [x, y, z] = gps.getValues()

    key = keyboard.getKey()
    a = 0.0
    b = 0.0
    c = 0.0
    while key > 0:
        if key == Keyboard.UP:
            a = 0.3
        elif key == Keyboard.DOWN:
            a = -0.3
        elif key == Keyboard.RIGHT:
            b = 0.5
        elif key == Keyboard.LEFT:
            b = -0.5
        elif key == Keyboard.SHIFT + Keyboard.RIGHT:
            c = -0.3
        elif key == Keyboard.SHIFT + Keyboard.LEFT:
            c = 0.3
        elif key == Keyboard.SHIFT + Keyboard.UP:
            tA += 0.1
        elif key == Keyboard.SHIFT + Keyboard.DOWN:
            tA = -0.1
            tA = 0.0 if tA < 0.0 else tA
        key = keyboard.getKey()

    dA = tA - y
    if sign(dA) != sign(sum):
        sum = 0.0
    sum += dA

    roll += 1.5708
    r = kr * clamp(roll, -0.5, 0.5) + c
    p = kp * clamp(pitch, -0.5, 0.5) + a
    # yF = ky * clamp(-yaw, -0.5, 0.5) + b
    yF = b

    v = k + kv * clamp(dA, -0.05, 0.05) + kv2 * sum

    flp.setVelocity(v - r - p + yF)
    frp.setVelocity(-(v + r - p - yF))
    rlp.setVelocity(-(v - r + p - yF))
    rrp.setVelocity(v + r + p + yF)
