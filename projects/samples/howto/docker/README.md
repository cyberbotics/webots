# Running Webots and Extern Controller in Separate Docker Containers

This folder contains a demo of running Webots inside a docker container and the extern controller inside another docker container.

## Use Case

Running Webots inside a docker and robot controllers inside other docker containers is useful in case of using Webots for a robot programming competition.
The competition organizers can run the competition scenario from their own world file and supervisor controllers.
The world file should contain a number robots with `<extern>` controllers which are meant to be used by competitors.
The simulation can be started from Python script that will:

1. Start Webots inside a docker container.
2. Start each robot controller inside its own docker controller.

This way, robot controllers are isolated from the Webots simulation and thus cannot cheat during the competition run.
The can only access a temporary folder which contains a local socket connection to Webots and memory mapped files (for camera images).
Thus, they cannot disrupt the simulation as they can only send and receive data through the local socket and memory mapped files.
In particular, they cannot change the behavior of the other controllers of the simulation (e.g., competing controllers, referee supervisor, etc.).

## Set-Up
