## Webots.cloud

[webots.cloud](https://webots.cloud) is an open-source webservice to share simulations online.
You can share 3D scenes and animations recorded from a simulation, but also fully interactive simulations.
This is also the place where you can see the complete library of protos available in Webots and add your own.

### Publish 3D Scenes and Animations

Webots features a "share" button that allows you to upload a 3D [Web Scene](web-scene.md) or a [Web Animation](web-animation.md) on webots.cloud.
This is useful to show models of robots, sensors, actuators, objects, environments or simulation play-back to colleagues or to the wide public.
Once your scene or animation is uploaded, you get a link (web URL) pointing to your upload, which you can share.
Anyone with this link can view your 3D scene or animation with a simple mouse click.
Scenes and animations can also be uploaded on webots.cloud by clicking the "Add a new scene/animation" button, and uploading locally saved `X3D`, `JSON`, thumbnail and texture files.

### Share and use your Proto

The [protos section](https://webots.cloud/proto) of webots.cloud is the place where you can see the protos available in Webots, read about them and play with their parameters.
Anyone can add its own proto to the library. To do that, your proto files (`.proto` files, textures, meshes,...) shoud be hosted on a GitHub repository.
The proto folder must also have a `webots.yaml` file which should contain: `publish: true`.
To classify the proto correctly, webots.cloud needs to know what are the `keywords` of the proto. Please refer to [this section](../reference/proto-design-guidelines.md#keywords) to learn more about `keywords` and how to set them.
You can then add your proto by clicking the `Add a new proto` button at the bottom of the [proto page](https://webots.cloud/proto).

To add a proto from the library to a world, copy the github url of the proto and convert it to `raw.githubusercontent.com`. To to that, change `github.com` into `raw.githubusercontent.com` and remove the `/blob`. Then paste it at the top of the world file like this:
```
#VRML_SIM {{ webots.version.major }} utf8

IMPORTABLE EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/{{ webots.version.major }}/projects/appearances/protos/RoughOak.proto"

WorldInfo {
```


### Publish Cloud-Based Simulations

It is also possible to run a Webots simulation in webots.cloud interactively.
In order to do this, your simulation files (worlds, textures, models, protos, controllers, plugins, etc.) and setup files (Dockerfile, webots.yml) should be hosted on a GitHub repository.
You can register this repository on [webots.cloud/simulation](https://webots.cloud/simulation) by clicking the "Add a new simulation" button and entering the URL to your github repository.
You will then get a link to your simulation running online.
Anyone with this link will be able to run your simulation with a simple mouse click.
Behind the scenes, webots.cloud will create a Docker container in a GPU instance, checkout your GitHub repo and run your simulation in there.
The 3D view will be displayed online, possibly with robot windows interactively displaying curves or sliders for changing some parameters while the simulation is running.
This tool could be used to publish open-science results where both reviewers and readers can reproduce your experimental setup very easily.

#### Types of Simulations
Currently there are two types of simulations available on webots.cloud:
* **Demo**: A demo is a complete Webots simulation project including a world file and one or several robot controllers.
It may demonstrate some research achievement: a robot solving a problem or demonstrating some interesting capabilities.
It may include some robot window displaying sensor data or other data representing the internal state of the robot.
The robot window may also include the possibility for the user to interact with the simulation while it is running.
For example there could be some buttons to ask the robot to perform some speficic actions, or a slider to apply a force to the robot, or a checkbox to open or close a door, etc.
To setup a Webots repository that contains a demo, you should create it from the [template repository](https://github.com/cyberbotics/webots-cloud-simulation-template).
Then, you should commit your specific files: worlds, controllers, protos, robot windows, etc.
Finally, you should add a new simulation from the [simulation](https://webots.cloud/simulation) page and indicate the GitHub URL of your Webots world file, including the tag (or branch) name, e.g., `https://github.com/cyberbotics/webots-cloud-simulation-examples/blob/main/1_simple_simulation/worlds/moose_demo.wbt`.
* **Benchmark**: A benchmark is a simulation scenario which proposes a challenge involving a single participant.
A robot has to address a problem and its behavior is evaluated against a performance metrics.
This performance metrics is a scalar value which allows to compare the performance of different participant against the same challenge.
Several examples of benchmarks are provided on the [robotbenchmark](https://robotbenchmark.net) website.

Please note that for now, there are no fundamental differences between **benchmarks** and **demos** but we are working on implementing metrics into benchmarks as soon as possible.
Other types of simulations, like competitions will also be added in the future.

#### Dockerfile

The version information specified in the Dockerfile at the root of the repository indicates which Webots release will be used to run the simulation.
More information on what the Dockerfile should contain can be found in the [Docker Solution](setup-a-webots-project-repository.md#docker-solution) documentation.

For example:
```Dockerfile
FROM cyberbotics/webots.cloud:{{ webots.version.major }}-ubuntu22.04
ARG PROJECT_PATH
RUN mkdir -p $PROJECT_PATH
COPY . $PROJECT_PATH
```

#### YAML File

A file named `webots.yml` must be included at the root level of a project to determine publishing permissions, the type of project, as well as if an IDE should be present.
A typical example of a `webots.yml` file is the following:
```yaml
publish: true
type: benchmark
dockerCompose:theia:webots-project/controllers/controller-name/
```

The following variables can be set:
* `publish`: By default, `publish` is set to `true`. If this is the case, all worlds found in the same `worlds` directory as the specified world will be published and listed in the interactive run sessions on webots.cloud. If `publish` is set to `false`, the simulation will not be listed on webots.cloud. Please note that simulations are linked to a GitHub account, not a webots.cloud account. Therefore modifying the `publish` variable in the project's GitHub repository, followed by a resynchronization (refresh icon in the simulations list), is the only way to remove a simulation from webots.cloud.
* `type`: Currently, 2 different types of repositories are supported: `demo` and `benchmark`.

To include an IDE in a webots.cloud project, a line of the following type should be added:
* `dockerCompose:theia:webots-project/controllers/`: Where `webots-project` is the `$PROJECT_PATH`. This line can be edited by adding a controller name if you want to specify only one controller. For more information on the IDE docker container and its limitations, refer to the [Docker Solution](setup-a-webots-project-repository.md#docker-solution) page.

#### Template

A typical example of a simulation repository can be found in the [webots.cloud simulation template](https://github.com/cyberbotics/webots-cloud-simulation-template).
This template can be used as a base to create your own simulations on webots.cloud, and includes a functional `Dockerfile` and `webots.yml` file.

#### Examples

Several examples of simulation repositories can be found in the [webots.cloud simulation examples](https://github.com/cyberbotics/webots-cloud-simulation-examples) repository.
These examples illustrate how to create a basic simulation repository but also showcase more advanced use cases like adding additional dependencies to the `Dockerfile`, compiling controllers and running an online IDE.

### Server Information

The server tab on webots.cloud contains information on the [Simulation Servers](simulation-server.md). To setup your own server, follow the guide on how to setup a [Web Server](web-server.md).

### Demo

**Scene**:
- [Complete Apartment](https://webots.cloud/ScBs2O7)
- [Robotis OP2](https://webots.cloud/ScdAPg1)

**Animation**:
- [PR2](https://webots.cloud/AcpeTj6)
- [Nao room](https://webots.cloud/AcTNYs0)

**Proto**
- [NAO](https://webots.cloud/run?url={{ url.github_tree }}/projects/robots/softbank/nao/protos/Nao.proto)
- [WoodenPalletStack](https://webots.cloud/run?url={{ url.github_tree }}/projects/objects/factory/pallet/protos/WoodenPalletStack.proto)

**Simulation**:

- [OroBOT Simulation](https://webots.cloud/run?version=R2022b&url=https://github.com/cyberbotics/orobot/blob/main/worlds/OroBOT_uneven.wbt&type=demo)
- [Spot Simulation](https://webots.cloud/run?version=R2022b&url=https://github.com/cyberbotics/webots-cloud-simulation-examples/blob/main/2_compile_controller/worlds/spot.wbt&type=demo)
