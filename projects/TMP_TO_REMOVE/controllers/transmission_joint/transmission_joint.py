from controller import Robot

TIME_STEP = 8
robot = Robot()

hJointMot = robot.getDevice('hJointMot')
hJointMot.setPosition(float('inf'))
hJointMot.setVelocity(5.0)

tJointMot = robot.getDevice('tJointMot')
tJointMot.setPosition(float('inf'))
tJointMot.setVelocity(5.0)

hJointPos = robot.getDevice('hJointPos')
hJointPos.enable(TIME_STEP)

tJointPos = robot.getDevice('tJointPos')
tJointPos.enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    
    hPos = hJointPos.getValue()
    tPos = tJointPos.getValue()
    print('%f %f' %(hPos, tPos))
    """
    if(hPos > 1.5708):
        hJointMot.setVelocity(-1.0)
    if(hPos < -1.5708):
        hJointMot.setVelocity(1.0)
    
    if(tPos > 1.5708):
        tJointMot.setVelocity(-1.0)
    if(tPos < -1.5708):
        tJointMot.setVelocity(1.0)
    """