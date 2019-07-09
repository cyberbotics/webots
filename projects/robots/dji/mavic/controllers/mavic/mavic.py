from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

flLed = robot.getLED('front left led')
frLed = robot.getLED('front right led')

flp = robot.getMotor('front left propeller')
frp = robot.getMotor('front right propeller')
rlp = robot.getMotor('rear left propeller')
rrp = robot.getMotor('rear right propeller')

flp.setPosition(float('inf'))
flp.setVelocity(1.0)
frp.setPosition(float('inf'))
frp.setVelocity(-1.0)
rlp.setPosition(float('inf'))
rlp.setVelocity(-1.0)
rrp.setPosition(float('inf'))
rrp.setVelocity(1.0)

while robot.step(timestep) != -1:
    if robot.getTime() > 1:
         break

flp.setVelocity(100.0)
frp.setVelocity(-100.0)
rlp.setVelocity(-100.0)
rrp.setVelocity(100.0)

while robot.step(timestep) != -1:
    ledState = int(robot.getTime()) % 2
    flLed.set(ledState)
    frLed.set(1 if ledState == 0 else 0)
