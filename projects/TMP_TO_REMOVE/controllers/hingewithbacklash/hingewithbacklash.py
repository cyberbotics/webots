from controller import Robot
import math 

TIME_STEP = 8
robot = Robot()

SPEED = 0.2

# classic hinge
cJointMot = robot.getDevice('cJointMot')
cJointMot.setPosition(float('inf'))
cJointMot.setVelocity(SPEED)
# hinge with backlash
hJointMot = robot.getDevice('hJointMot')
hJointMot.setPosition(float('inf'))
hJointMot.setVelocity(SPEED)

hJointPos = robot.getDevice('hJointPos')
hJointPos.enable(TIME_STEP)

hJointPos2 = robot.getDevice('hJointPos2')
hJointPos2.enable(TIME_STEP)

cJointPos = robot.getDevice('cJointPos')
cJointPos.enable(TIME_STEP)


while robot.step(TIME_STEP) != -1:
    cPos = cJointPos.getValue();
    hPos = hJointPos.getValue();
    hPos2 = hJointPos2.getValue();
    print("%.10f | %.10f %.10f" %(cPos, hPos, hPos2))
    
    if cPos > 0.50: #30 * math.pi / 180:
        hJointMot.setVelocity(-SPEED);
        cJointMot.setVelocity(-SPEED);
    if cPos < -0.50: #-30 * math.pi / 180:
        hJointMot.setVelocity(SPEED);
        cJointMot.setVelocity(SPEED);
