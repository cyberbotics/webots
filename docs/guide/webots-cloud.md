## Webots.cloud

[webots.cloud](https://webots.cloud) is an open-source webservice to share simulations online.
You can share 3D scenes and animations recorded from a simulation, but also fully interactive simulations.

### Publish 3D Scenes and Animations

Webots features a "share" button that allows you to upload a 3D [Web Scene](web-scene.md) or a [Web Animation](web-animation.md) on webots.cloud.
This is useful to show models of robots, sensors, actuators, objects, environments or simulation play-back to colleagues or to the wide public.
Once your scene or animation is uploaded, you get a link (web URL) pointing to your upload, which you can share.
Anyone with this link can view your 3D scene or animation with a simple mouse click.
Scenes and animations can also be uploaded on webots.cloud by clicking the "Add a new scene/animation" button, and uploading locally saved `X3D`, `JSON`, thumbnail and texture files.

### Publish Cloud-Based Simulations

It is also possible to run a Webots simulation in webots.cloud interactively.
In order to do this, your simulation files (worlds, textures, models, protos, controllers, plugins, etc.) and setup files (Dockerfile, webots.yaml) should be hosted on a GitHub repository.
You can register this repository on [webots.cloud/simulation](https://webots.cloud/simulation) by clicking the "Add a new simulation" button and uploading the URL to your github repository.
You will then get a link to your simulation running online.
Anyone with this link will be able to run your simulation with a simple mouse click.
Behind the scenes, webots.cloud will create a Docker container in a GPU instance, checkout your GitHub repo and run your simulation in there.
The 3D view will be displayed online, possibly with robot windows interactively displaying curves or sliders for changing some parameters while the simulation is running.
This tool could be used to publish open-science results where both reviewers and readers can reproduce your experimental setup very easily.

#### Types of Simulations
Currently there only two types of simulations available on webots.cloud:
* **Demo**: A demo is a complete Webots simulation project including a world file and one or several robot controllers.
It may demonstrate some research achievement: a robot solving a problem or demonstrating some interesting capabilities.
It may include some robot window displaying sensor data or other data representing the internal state of the robot.
The robot window may also include the possibility for the user to interact with the simulation while it is running.
For example there could be some buttons to ask the robot to perform some speficic actions, or a slider to apply a force to the robot, or a checkbox to open or close a door, etc.
To setup a Webots repository that contains a demo, you should create it from the [template repository](https://github.com/cyberbotics/webots-demo-template).
Then, you should commit your specific files: worlds, controllers, protos, robot windows, etc.
Finally, you should add a new simulation from the [simulation](https://webots.cloud/simulation) page and indicate the GitHub URL of your Webots world file, including the tag (or branch) name, e.g., https://github.com/cyberbotics/webots/blob/R2022b/projects/languages/python/worlds/example.wbt.
* **Benchmark**: A benchmark is a simulation scenario which proposes a challenge involving a single participant.
A robot has to address a problem and its behavior is evaluated against a performance metrics.
This performance metrics is a scalar value which allows to compare the performance of different participant against the same challenge.
Several examples of benchmarks are provided on the [robotbenchmark](https://robotbenchmark.net) website.

Other types of simulations, like contests will be added later.

#### Dockerfile

The version information specified in the Dockerfile at the root of the repository indicates which Webots release will be used to run the simulation. More information on what the Dockerfile should contain can be found in the [Docker Solution](setup-a-webots-project-repository.md#docker-solution) documentation.

For example:
```Dockerfile
FROM cyberbotics/webots:R2022b-ubuntu20.04
ARG PROJECT_PATH
RUN mkdir -p $PROJECT_PATH
COPY . $PROJECT_PATH
```

#### YAML File

A file named `webots.yaml` must be included at the root level of a repository to determine publishing permissions, the type of project, as well as if an IDE should be present. A typical example of a `webots.yaml` file is the following:
```yaml
publish: true
type: benchmark
dockerCompose:theia:webots-project/controllers/controller-name/
```

The following variables can be set:
* `publish`: By default, `publish` is set to `true`. All worlds found in the same directory as the specified world will be be used by webots.cloud and listed as interactive run sessions. When `publish` is set to `false` the simulation will not be uploaded and can be removed from webots.cloud on resynchronization.
* `type`: Currently, 2 different types of repositories are supported: `demo` and `benchmark`.

To include an IDE in a webots.cloud project, a line of the following type should be added:
* `dockerCompose:theia:webots-project/controllers/`: Where `webots-project` is the `$PROJECT_PATH`. This line can be edited by adding a controller name if you want to specify only one controller.


### Server Information

The server tab on webots.cloud contains information on the [Simulation Servers](simulation-server.md). To setup your own server, follow the guide on how to setup a [Web Server](web-server.md).

### Demo

**Scene**:
- [Apartment](https://webots.cloud/SchkH69)
- [Aldebaran's Nao](https://webots.cloud/Scvuzo1)

**Animation**:
- [PR2](https://webots.cloud/AcpeTj6)
- [Nao room](https://webots.cloud/AcTNYs0)

**Simulation**:

- [OroBOT Simulation](https://webots.cloud/run?version=R2022b&url=https://github.com/ThomasOliverKimble/orobot/blob/main/worlds/OroBOT.wbt)
- [E-Puck](https://webots.cloud/run?version=R2022b&url=https://github.com/ThomasOliverKimble/GuidedTour/blob/guided-tour/e-puck/worlds/e-puck_line_demo.wbt)
