"""nao controller."""

from controller import Robot, Node

robot = Robot()

timestep = int(robot.getBasicTimeStep())

print ('LEDS:')
for d in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(d)
    if device.getNodeType() == Node.LED:
        print (device.getName())
        device.set(1)

while robot.step(timestep) != -1:
    pass
