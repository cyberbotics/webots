"""telemax_keyboard_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Keyboard, Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

keyboard =  robot.getKeyboard()
keyboard.enable(timestep)

frontMotor = robot.getMotor('front motor')
rearMotor = robot.getMotor('rear motor')
frontLeftTrack = robot.getMotor('left front track')
frontRightTrack = robot.getMotor('right front track')
rearLeftTrack = robot.getMotor('left rear track')
rearRightTrack = robot.getMotor('right rear track')


frontLeftTrack.setPosition(float('inf'))
frontRightTrack.setPosition(float('inf'))
rearLeftTrack.setPosition(float('inf'))
rearRightTrack.setPosition(float('inf'))
frontLeftTrack.setVelocity(0)
frontRightTrack.setVelocity(0)
rearLeftTrack.setVelocity(0)
rearRightTrack.setVelocity(0)

centralLed = robot.getLED('central led')
leftLed = robot.getLED('left led')
rightLed = robot.getLED('right led')

position = [0.0, 0.0]
speed = 0.0
angle = 0.0
previousKeys = []

while robot.step(timestep) != -1:
    key = keyboard.getKey()
    currentKeys = []
    while key != -1:
        currentKeys.append(key)
        if key == ord('Q'):
            position[0] += 0.01
        elif key == ord('A'):
            position[0] -= 0.01
        elif key == ord('W'):
            position[1] += 0.01
        elif key == ord('S'):
            position[1] -= 0.01
        elif key == Keyboard.UP:
            speed += 0.005
        elif key == Keyboard.DOWN:
            speed -= 0.005
        elif key == Keyboard.RIGHT:
            angle += 0.002
        elif key == Keyboard.LEFT:
            angle -= 0.002
        elif key == ord('I') and key not in previousKeys:
            leftLed.set(not leftLed.get())
        elif key == ord('O') and key not in previousKeys:
            centralLed.set(not centralLed.get())
        elif key == ord('P') and key not in previousKeys:
            rightLed.set(not rightLed.get())
        key = keyboard.getKey()

    previousKeys = currentKeys
    frontMotor.setPosition(position[0])
    rearMotor.setPosition(position[1])
    frontLeftTrack.setVelocity(speed + angle)
    frontRightTrack.setVelocity(speed - angle)
    rearLeftTrack.setVelocity(speed - angle)
    rearRightTrack.setVelocity(speed + angle)
