"""jetbot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

class JetBot(Robot):
    def init(self):
        self.left_motor = self.getDevice('left_wheel_hinge')
        self.right_motor = self.getDevice('right_wheel_hinge')
        self.camera = self.getDevice('camera')
        self.display = self.getDevice('display')
        self.right_motor.setPosition(float('+inf'))
        self.left_motor.setPosition(float('+inf'))
        
    def left(self, speed):
        self.right_motor.setVelocity(speed * 10)
        self.left_motor.setVelocity(-speed * 10)
        
    def right(self, speed):
        self.right_motor.setVelocity(-speed * 10)
        self.left_motor.setVelocity(speed * 10)
        
    def forward(self, speed):
        self.right_motor.setVelocity(speed * 10)
        self.left_motor.setVelocity(speed * 10)
        
    def backward(self, speed):
        self.right_motor.setVelocity(-speed * 10)
        self.left_motor.setVelocity(-speed * 10)
        
    def set_motors(self, speed_left, speed_right):
        self.right_motor.setVelocity(speed_right)
        self.left_motor.setVelocity(speed_left)
        
    def stop(self):
        self.right_motor.setVelocity(0)
        self.left_motor.setVelocity(0)
        

def step_forward(robot):
    robot.forward(0.4)
    robot.step(512)
    robot.stop()
    
def step_backward(robot):
    robot.backward(0.4)
    robot.step(512)
    robot.stop()
    
def step_left(robot):
    robot.left(0.3)
    robot.step(512)
    robot.stop()
    
def step_right(robot):
    robot.right(0.3)
    robot.step(512)
    robot.stop()


# create the Robot instance.
robot = JetBot()
robot.init()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


while robot.step(timestep) != -1:
    step_forward(robot)
    robot.step(480)
    step_backward(robot)
    robot.step(480)
    step_left(robot)
    robot.step(480)
    step_right(robot)
    robot.step(480)
    pass
