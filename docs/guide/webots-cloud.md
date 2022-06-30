## Webots.cloud

### Scene

A scene is a simple 3D model corresponding to a Webots world file embedded in webots cloud as a [Web Scene](web-scene.md). Scenes are static, that is nothing is moving. However, the users can change the viewpoint and zoom in to observe details in a scene.

### Animation

An animation is an animated model embedded in webots cloud as a [Web Animation](web-animation.md). It is usually the recording of a simulation that can be played back by the users.

### Simulation

#### Types of Simulations
Currently there are three types of simulations available on webots.cloud:
* **Demo**: A demo is a complete Webots simulation project including a world file and one or several robot controllers. It may demonstrate some research achievement: a robot solving a problem or demonstrating some interesting capabilities. It may include some robot window displaying sensor data or other data representing the internal state of the robot. The robot window may also include the possibility for the user to interact with the simulation while it is running. For example there could be some buttons to ask the robot to perform some speficic actions, or a slider to apply a force to the robot, or a checkbox to open or close a door, etc. To setup a Webots repository that contains a demo, you should create it from the [template repository](https://github.com/cyberbotics/webots-demo-template). Then, you should commit your specific files: worlds, controllers, protos, robot windows, etc. Finally, you should add a new demo from the [simulation](https://webots.cloud/simulation) page and indicate the GitHub URL of your Webots world file, including the tag (or branch) name, e.g., https://github.com/cyberbotics/webots/blob/R2021b/projects/languages/python/worlds/example.wbt.
* **Benchmark**: A benchmark is a simulation scenario which proposes a challenge involving a single participant. A robot has to address a problem and its behavior is evaluated against a performance metrics. This performance metrics is a scalar value which allows to compare the performance of different participant against the same challenge. Several examples of benchmarks are provided on the [robotbenchmark](https://robotbenchmark.net) website.
* **Competition**: A competition is a simulation scenario involving several participants (generally two participants).
It could be a soccer match between two teams of robots like RoboCup, or a survival game like Rat's Life, or a wrestling competition like Roboka, etc.
The performance metrics is a ranking between the competitors.
It may be implemented as a [round robin](https://en.wikipedia.org/wiki/Round-robin_tournament) tournanent, or with a series of quater finals, semi finals and finals, or using a bubble sort ranking algorithm, or with any other ranking system ([ELO](https://en.wikipedia.org/wiki/Elo_rating_system), [ATP](https://en.wikipedia.org/wiki/ATP_Rankings), etc.).


#### Dockerfile

The version information specified in the Dockerfile at the root of the repository indicates which Webots release will be used to run the simulation. More information on what the Dockerfile should contain can be found in the [Docker Solution](setup-a-webots-project-repository.md#docker-solution) documentation.

For example:
```Dockerfile
FROM cyberbotics/webots:R2020b-rev1-ubuntu20.04
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

Or:
```yaml
publish: true
type: competitor
competition: https://github.com/username/competition
```

The following variables can be set:
* `publish`: By default, `publish` is set to `true`. All worlds found in the same directory as the specified world will be be used by webots.cloud and listed as interactive run sessions. When `publish` is set to `false` the simulation will not be uploaded and can be removed from webots.cloud on resynchronization.
* `type`: Currently, 4 different types of repositories are supported: `demo`, `benchmark`, `competition` and `competitor`.
* `competitition`: This variable should only be set if the repository is of `type: competitor`. Here, the repository should contain an entry to a competition or benchmark scenario which is the source code of one robot controller.

To include an IDE in a webots.cloud project, a line of the following type should be added:
* `dockerCompose:theia:webots-project/controllers/`: Where `webots-project` is the `$PROJECT_PATH`. This line can be edited by adding a controller name if you want to specify only one controller.


### Servers

This tab contains information on the [Simulation Servers](simulation-server.md).