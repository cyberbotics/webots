# Running Webots and Extern Controller in Separate Docker Containers

This folder contains a demo of running Webots inside a docker container and an extern controller inside another docker container.
The demo works only on Linux.

## Use Case

Running Webots inside a docker and robot controllers inside other docker containers is useful in case of using Webots on a simulation server for a robot programming competition.
The competition organizers should provide the competition scenario with a world file and a referee supervisor controller.
The world file should contain a number robots with `<extern>` controllers which are meant to be used by competitors.
The simulation can be started from Python script that will:

1. Start Webots inside a docker container.
2. Start each robot controller inside its own docker container.

This way, the robot controllers are isolated from the Webots simulation and thus cannot cheat during the competition run.
They can only access a temporary folder which contains a local socket connection to Webots and memory mapped files (for camera images).
Thus, they cannot disrupt the simulation as they can only send and receive data through the local socket and memory mapped files.
Also, they cannot affect the behavior of the other controllers of the simulation (e.g., competing controllers, referee supervisor controller, etc.) thanks to the docker container isolation.

## Set-Up

Two launcher examples are provided here: `launcher_without_docker.py` and `launcher.py`.
They both launch the [camera.wbt](simulation/worlds/camera.wbt) world file and start the controller of the "MyBot" robot as an extern controller.
It is recommended to run and understand the first script before stepping into the second one, which is more complicated.
In both cases, you should see a Webots windows popping-up and running the camera controller as an extern controller.

### Without Docker

The `launcher_without_docker.py` script launches Webots with the `--extern-urls` command line option.
This option allows Webots to print to `stdout` the controller URLs that need to be started.
The script will parse the output of Webots and start the extern controller as requested by Webots.

It takes less than 3 seconds on a fairly powerful server to get the robot moving after starting the launcher.

### With Docker

You will have to [install Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and follow the [post-installation instructions](https://docs.docker.com/engine/install/linux-postinstall/): `sudo usermod -aG docker $USER` and `newgrp docker`.
You will also have to install the NVIDIA docker drivers as documented [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
Finally, you will have to install the [Compose plugin](https://docs.docker.com/compose/install/linux/) for docker: `sudo apt-get install docker-compose-plugin`

The docker base image used in this script is `cyberbotics/webots:R2023b` which you may want to change to target a different base image.
For example you may want to create your own docker base image from the current development branch.
This can be achieved from the https://github.com/cyberbotics/webots-docker repository by adapting the `Dockerfile` to use a local `webots` folder which can be created from a tarball distribution snapshot of the current development branch.

The `launcher.py` script goes through the following steps:

1. It builds a first docker image based on a recent Webots docker image and adds the world file to it.
2. It builds a second docker image based on the same Webots docker image as previously, adds the camera controller to it and compiles it.
3. It runs Webots inside a container from the first docker image for which it shares the /tmp/webots-1234/ipc/MyBot folder.
4. It parses the output of this container to get the controller URL printed by Webots for the extern controller.
5. It runs the camera controller specifying the URL inside a container from the second docker image for which it shares the same folder as previously.

It takes less than 4 seconds on a fairly powerful server to get the robot moving after starting the launcher.
That means the docker overhead is about one second.
