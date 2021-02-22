from controller import Robot

TIME_STEP = 16
robot = Robot()
"""
hingeJointMot = robot.getDevice('motorHinge')
hingeJointMot.setPosition(float('inf'))
hingeJointMot.setVelocity(1.0)
"""
transmissionJointMot = robot.getDevice('motorTransmission')
transmissionJointMot.setPosition(float('inf'))
transmissionJointMot.setVelocity(1.0)
"""
hingeJointPos = robot.getDevice('motorHingePosition')
hingeJointPos.enable(TIME_STEP)
"""
transmissionJointPos = robot.getDevice('motorTransmissionPosition')
transmissionJointPos.enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    """
    pos = hingeJointPos.getValue()
    if(pos > 0.5236):
        hingeJointMot.setVelocity(-1.0)
    if(pos < -0.5236):
        hingeJointMot.setVelocity(1.0)
    """
    pos = transmissionJointPos.getValue()
    if(pos > 0.5236):
        transmissionJointMot.setVelocity(-1.0)
    if(pos < -0.5236):
        transmissionJointMot.setVelocity(1.0)