"""Sample Webots controller for the humanoid sprint benchmark."""

from controller import Robot, Motion


class Sprinter(Robot):
    """Make the NAO robot run as fast as possible."""

    def initialize(self):
        """Get device pointers, enable sensors and set robot initial pose."""
        # This is the time step (ms) used in the motion file.
        self.timeStep = 40
        # Get pointers to the shoulder motors.
        self.RShoulderPitch = self.getDevice('RShoulderPitch')
        self.LShoulderPitch = self.getDevice('LShoulderPitch')
        # Move the arms down.
        self.RShoulderPitch.setPosition(1.1)
        self.LShoulderPitch.setPosition(1.1)

        # # Get pointers to the 12 motors of the legs (not used).
        # self.RHipYawPitch = self.getDevice('RHipYawPitch')  # not used in forward.motion
        # self.LHipYawPitch = self.getDevice('LHipYawPitch')  # not used in forward.motion
        # self.RHipRoll = self.getDevice('RHipRoll')
        # self.LHipRoll = self.getDevice('LHipRoll')
        # self.RHipPitch = self.getDevice('RHipPitch')
        # self.LHipPitch = self.getDevice('LHipPitch')
        # self.RKneePitc = self.getDevice('RKneePitch')
        # self.LKneePitch = self.getDevice('LKneePitch')
        # self.RAnklePitch = self.getDevice('RAnklePitch')
        # self.LAnklePitch = self.getDevice('LAnklePitch')
        # self.RAnkleRoll = self.getDevice('RAnkleRoll')
        # self.LAnkleRoll = self.getDevice('LAnkleRoll')
        # getting pointer to the 2 shoulder motors

        # # Get pointers to the onboard cameras (not used).
        # self.CameraTop = self.getDevice('CameraTop')
        # self.CameraBottom = self.getDevice('CameraBottom')
        # # Enable the cameras.
        # self.CameraTop.enable(self.timeStep)
        # self.CameraBottom.enable(self.timeStep)

    def run(self):
        """Play the forward motion and loop on the walking cycle."""
        walk = Motion('forward.motion')
        walk.setLoop(True)
        walk.play()
        while True:
            # # This motor is not controlled by forward.motion.
            # self.RHipYawPitch.setPosition(-1)
            # print walk.getTime()  # display the current time of the forward.motion
            if walk.getTime() == 1360:  # we reached the end of forward.motion
                walk.setTime(360)  # loop back to the beginning of the walking sequence
            # Perform a simulation step, quit if the simulation is over.
            if self.step(self.timeStep) == -1:
                break


controller = Sprinter()
controller.initialize()
controller.run()
