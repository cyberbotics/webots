"""Sample Webots controller for the humanoid sprint benchmark."""

from controller import Robot, Motion


class Sprinter(Robot):
    """Make the NAO robot run as fast as possible."""

    def initialize(self):
        """Get device pointers, enable sensors and set robot initial pose."""
        # This is the time step (ms) used in the motion file.
        self.timeStep = 16
        # Get pointers to the shoulder motors.
        self.RShoulderPitch = self.getMotor('RShoulderPitch')
        self.LShoulderPitch = self.getMotor('LShoulderPitch')
        # Move the arms down.
        self.RShoulderPitch.setPosition(1.1)
        self.LShoulderPitch.setPosition(1.1)
        
        # Get pointers to the sonar sensors and enable them
        self.RSonar = self.getDistanceSensor('Sonar/Right')
        self.LSonar = self.getDistanceSensor('Sonar/Left')
        self.RSonar.enable(16)
        self.LSonar.enable(16)
        
        # Get a pointer to the laser and enable it
        self.laser = self.getLidar('laser')
        self.laser.enable(32)
        self.laser.enablePointCloud()

        # # Get pointers to the 12 motors of the legs (not used).
        # self.RHipYawPitch = self.getMotor('RHipYawPitch')  # not used in forward.motion
        # self.LHipYawPitch = self.getMotor('LHipYawPitch')  # not used in forward.motion
        # self.RHipRoll = self.getMotor('RHipRoll')
        # self.LHipRoll = self.getMotor('LHipRoll')
        # self.RHipPitch = self.getMotor('RHipPitch')
        # self.LHipPitch = self.getMotor('LHipPitch')
        # self.RKneePitc = self.getMotor('RKneePitch')
        # self.LKneePitch = self.getMotor('LKneePitch')
        # self.RAnklePitch = self.getMotor('RAnklePitch')
        # self.LAnklePitch = self.getMotor('LAnklePitch')
        # self.RAnkleRoll = self.getMotor('RAnkleRoll')
        # self.LAnkleRoll = self.getMotor('LAnkleRoll')
        # getting pointer to the 2 shoulder motors

        # Get pointers to the onboard cameras (not used).
        self.CameraTop = self.getCamera('CameraTop')
        self.CameraBottom = self.getCamera('CameraBottom')
        # Enable the cameras.
        self.CameraTop.enable(2 * self.timeStep)
        self.CameraBottom.enable(2 * self.timeStep)

    def run(self):
        """Play the forward motion and loop on the walking cycle."""
        walk = Motion('forward.motion')
        walk.setLoop(True)
        walk.play()
        while True:
            # # Get the camera image
            # image = self.CameraTop.getImageArray()
            # Get the laser depth image
            rangeImage = self.laser.getRangeImage()
            # # Get the laser point-cloud
            # pointCloud = self.laser.getPointCloud()
            # # This motor is not controlled by forward.motion.
            # self.RHipYawPitch.setPosition(-1)
            # print walk.getTime()  # display the current time of the forward.motion
            if walk.getTime() == 1360:  # we reached the end of forward.motion
                walk.setTime(360)  # loop back to the beginning of the walking sequence
            # Stop if any obstacle is detected
            if self.RSonar.getValue() < 1.2 or self.LSonar.getValue() < 1.2:
                walk.stop()
            # Perform a simulation step, quit if the simulation is over.
            if self.step(self.timeStep) == -1:
                break


controller = Sprinter()
controller.initialize()
controller.run()