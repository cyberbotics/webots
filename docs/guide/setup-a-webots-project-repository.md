## Setup a Webots Project Repository

### Requirements

In order to run a Webots simulation from a source code repository, the repository should provide a number of dependencies information allowing to run the simulation:

- Operating system
- Version of Webots
- Controller dependencies (python version, python modules, ROS version, ROS packages, deep learning libraries, etc.)

### Docker Solution

All these dependencies could be bundled into a Docker image constructed from a root Docker image such as [cyberbotics/webots:R2022a-ubuntu20.04](https://hub.docker.com/layers/cyberbotics/webots/R2022a-ubuntu20.04/images/sha256-6ef88bc8cc95091efe928c664ff84ed46660d07f60fbbb2474f9b8dfb541ce47?context=explore) to which additional dependencies could be added.

#### Running a Demo

When running a demo, a single docker container will be used based on the `Dockerfile` located at the root of the project directory. If no `Dockerfile` is provided, the simulation server will use [Dockerfile.default](https://github.com/cyberbotics/webots/blob/develop/resources/web/server/config/simulation/docker/Dockerfile.default). As the default Dockerfile, you can use the following environment variables in your Dockerfile:
- `$MAKE`: 1 if a Makefile exists in the project directory, otherwise 0.
- `$PROJECT_PATH`: local docker project directory path
- `$WEBOTS_DEFAULT_IMAGE`: default image of Webots according to the version of your world. This image is provided on [dockerhub](https://hub.docker.com/r/cyberbotics/webots). Only the released versions are provided.

Webots will run inside this container to protect the host machine from malicious code that may be included in a robot controller or in a physics plug-in.

#### Running a Benchmark

In the case of a benchmark, two docker containers are used to run different components:
1. Webots running the benchmark scenario, including some supervisor controller and possibly some physics plug-in. As the demo, the organizer can specify its own Dockerfile at the root of the project.
2. a browser IDE to edit and build specified controllers. To enable this IDE, the organizer has to create a file `webots.yml` and add the line `theia:webots-project/controllers` where `webots-project` is the `$PROJECT_PATH`. For instance, you can specify only one controller by editing this line.

#### Going Further

If you need to set your own containers or use customized Theia IDE, the server's owner can add or modify docker-compose.yml files. These files are used to specify the images to build and run, the accessible volumes, ports and more (see [docker-compose documentation](https://docs.docker.com/compose/)). The simulation server provides by default [docker-compose-default.yml](https://github.com/cyberbotics/webots/blob/develop/resources/web/server/config/simulation/docker/docker-compose-default.yml) and for theia [docker-compose-theia.yml](https://github.com/cyberbotics/webots/blob/develop/resources/web/server/config/simulation/docker/docker-compose-theia.yml). Furthermore, the simulation server provides a `docker-compose.yml` file with the following environment variables:

- `$IMAGE`, `$PROJECT_PATH`, `$MAKE`: refer to [Running a demo](setup-a-webots-project-repository.md#running-a-demo)
- `$PORT`: port used to connect to the simulation.
- `$COMPOSE_PROJECT_NAME`: id of the Webots instance of the client
- `$WEBOTS`: Webots commands

In addition for Theia:

- `$THEIA_V`: local docker volume Theia can access
- `$THEIA_PORT` = $PORT + 500 ; used to connect to Theia IDE browser.
