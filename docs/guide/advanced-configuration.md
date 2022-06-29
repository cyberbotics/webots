## Advanced Configuration

### Docker

If the `docker` configuration option is set to `true`, Docker will be used to start Webots, otherwise Webots will be started directly on the metal of the server.
If the simulation project contains a `Dockerfile` file at the root level, this file will be used to start the Webots instance inside the corresponding Docker container.
Otherwise, a `Dockerfile` will be created based on the standard Docker image of Webots.
The version of Webots for the Docker image is automatically computed from the header line of the simulation world file.
For example if the world file starts with the following line:

```
#VRML_SIM R2022a utf8
```

The simulation server will create a `Dockerfile` starting with:
```
FROM docker image cyberbotics/webots:R2022a-ubuntu20.04
```

Running Webots inside a Docker container is a very little overhead, but guarantees that the simulation server remain secure, regardless of the running simulations.

If you don't want to use Docker, you should ensure that the list of `allowedRepositories` provided in the configuration file doesn't contain any malware, otherwise, you are putting your simulation server at risk.

### Simulation Files Checkout

When the simulation server receives the 'start' command on the `/client` request, it will checkout the simulation files from the provided `url` pointing to a Webots world file on a GitHub repository:
```
https://github.com/alice/sim/blob/my_own_version/app/worlds/my_world.wbt
                   └───┬───┘      └─────┬──────┘ └─────────┬───────────┘
                  repository       tag or branch     path to world file
```
Currently, this protocol only supports public GitHub repositories.
In the above sample URL, the simulation server will checkout the `my_own_version` tag or branch of the `/app` directory from the `sim` repository of the `alice` GitHub user and it will start Webots with the specified `my_world.wbt` world file.

This protocol is still experimental and the robot windows are not yet supported.

### Checking Out Only a Git Folder

Github provides svn interface allowing to checkout only one folder on a specific branch or tag.
[Other options](https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo) are possible.
Given the URL to a Webots project is for example:

https://github.com/cyberbotics/webots/tree/master/projects/languages/python

We need to check out only this folder, not the whole webots repository.
This can be achieve with svn on the master branch:
`svn checkout https://github.com/cyberbotics/webots/branches/master/projects/languages/python`
Or on the R2021 tag:
`svn checkout https://github.com/cyberbotics/webots/tags/R2021/projects/languages/python`

To check if a branch or a tag exists:
`svn ls https://github.com/cyberbotics/webots/branches/master`
`svn ls https://github.com/cyberbotics/webots/tags/R2021b`
`git ls-remote --quiet --heads https://github.com/cyberbotics/webots.git master`
`git ls-remote --quiet --tags https://github.com/cyberbotics/webots.git R2021b`

`git ls-remote --quiet https://github.com/cyberbotics/webots.git R2021b` will tell whether `R2021b` is a branch or a tag.
