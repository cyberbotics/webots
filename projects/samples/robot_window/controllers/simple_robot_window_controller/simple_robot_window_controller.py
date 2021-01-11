from controller import Robot


robot = Robot()
timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
    # Receive a message from the robot window
    message = robot.wwiReceiveText()

    if message:
        # Print the message if not None
        print(message)

        # Send a message back to the robot window
        robot.wwiSendText(message)
