## Setup a Webots Project Repository

### Requirements

In order to run a Webots simulation from a source code repository, the repository should provide a number of dependencies information allowing to run the simulation:

- Operating system
- Version of Webots
- Controller dependencies (python version, python modules, ROS version, ROS packages, deep learning libraries, etc.)

### Docker Solution

All these dependencies could be bundled into a Docker image constructed from a root Docker image such as [cyberbotics/webots-cloud:R2022b](https://hub.docker.com/layers/cyberbotics/webots.cloud/R2022b/images/sha256-695841935fc212cfc21f59cf2a467cc4e6087309b6437a9081beb72f326ae407?context=explore) to which additional dependencies could be added.

#### Running a Simulation

When running a simulation, a single docker container will be used based on the `Dockerfile` located at the root of the project directory.
If no `Dockerfile` is provided, the simulation server will use [Dockerfile.default](https://github.com/cyberbotics/webots-server/blob/main/config/simulation/docker/Dockerfile.default).
As the default Dockerfile, you can use the following environment variables in your Dockerfile:
- `$MAKE`: 1 if a Makefile exists in the project directory, otherwise 0.
- `$PROJECT_PATH`: local docker project directory path
- `$WEBOTS_DEFAULT_IMAGE`: default image of Webots according to the version of your world. This image is provided at [dockerhub](https://hub.docker.com/r/cyberbotics/webots.cloud). Only the released versions are provided.

Webots will run inside this container to protect the host machine from malicious code that may be included in a robot controller or in a physics plug-in.

The `Dockerfile` can be used to build binaries "on-the-fly", this could be for controllers, physics plugin or robot windows plugins as shown in the example below.
However, it is also possible to directly provide the built binaries in the corresponding folders and use the default Dockerfile without any "on-the-fly" compilation.

A typical `Dockerfile` would look something like this:
```Dockerfile
FROM cyberbotics/webots.cloud:R2022b
ARG PROJECT_PATH
RUN mkdir -p $PROJECT_PATH
COPY . $PROJECT_PATH
RUN cd $PROJECT_PATH/plugins/physics_plugins/physics_plugin_name && make clean && make
RUN cd $PROJECT_PATH/controllers/controller_name && make clean && make
```

A second docker container can be used to run a browser IDE to edit and build specified controllers.
To enable this IDE, a `webots.yml` file has to be added at the root of the project directory.
A single line is necessary inside the file:
* `dockerCompose:theia:webots-project/controllers/`: Where `webots-project` is the `$PROJECT_PATH`. This line can be edited by adding a controller name if you want to specify only one controller.
All controllers contained in the project directory will be run during the simulation, however, only the controllers specified in the `webots.yml` can be viewed and modified with the IDE.

Please note that, for now, no compilation is possible in the IDE, thus **only the python controllers** can be modified by the users and interact with the simulations.


#### Going Further

If you need to set your own containers or use customized Theia IDE, the server's owner can add or modify docker-compose.yml files.
These files are used to specify the images to build and run, the accessible volumes, ports and more (see [docker-compose documentation](https://docs.docker.com/compose/)).
The simulation server provides by default [docker-compose-default.yml](https://github.com/cyberbotics/webots-server/blob/main/config/simulation/docker/docker-compose-default.yml) and for theia [docker-compose-theia.yml](https://github.com/cyberbotics/webots-server/blob/main/config/simulation/docker/docker-compose-theia.yml).
Furthermore, the simulation server provides a `docker-compose.yml` file with the following environment variables:

- `$IMAGE`, `$PROJECT_PATH`, `$MAKE`: refer to [Running a simulation](setup-a-webots-project-repository.md#running-a-simulation)
- `$PORT`: port used to connect to the simulation.
- `$COMPOSE_PROJECT_NAME`: id of the Webots instance of the client
- `$WEBOTS`: Webots commands

In addition for Theia:

- `$THEIA_V`: local docker volume Theia can access
- `$THEIA_PORT` = $PORT + 500 ; used to connect to Theia IDE browser.
