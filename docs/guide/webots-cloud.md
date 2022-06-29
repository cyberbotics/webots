## Webots.cloud

### Scenes

### Animations

### Projects

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

A file named `webots.yaml` must be included at the root level of a repository to determine the type of project, if an IDE should be present, as well as publishing permissions.

##### Type

Currently, we support 4 different types of repositories:

- Demo
  This is a simple simulation that can be run interactively.
  The `webots.yaml` file should contain a reference to the demo type and a publish setting, for example:
  ```yaml
  type: demo
  publish: true
  ```

By default, `publish` is set to `true`. All worlds found in the same directory as the specified world will be be used by webots.cloud and listed as interactive run sessions. When `publish` is set to `false` the simulation will not be uploaded and can be removed from webots.cloud on resynchronization.

- Benchmark
  This type of repository should contain the scenario of a benchmark, including a supervisor process performing the evalution of the controller(s) and a publish setting.
  ```yaml
  type: benchmark
  publish: true
  ```

- Competition
  This type of repository should contain the scenario of a competition, including a supervisor process performing the evalution of the controller(s) and a a publish setting.
  ```yaml
  type: competition
  publish: true
  ```

- Competitor
  This type of repository should contain an entry to a competition or benchmark scenario.
  It should typically contain only the source code of one robot controller.
  Eventually, it may contain also more controllers, PROTO files and other files, specific to a competition/benchmark scenario.
  ```yaml
  type: competitor
  competition: https://github.com/username/competition
  ```
  ```yaml
  type: competitor
  benchmark: https://github.com/username/benchmark
  ```

##### Integrated Development Environment

To include an IDE in a webots.cloud project, a specific line has to be added to the `webots.yaml` file.

```yaml
dockerCompose:theia:webots-project/controllers/controller-name/
```

### Servers