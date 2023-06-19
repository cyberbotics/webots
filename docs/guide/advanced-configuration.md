## Advanced Configuration

### Docker

If the `docker` configuration option is set to `true`, Docker will be used to start Webots, otherwise Webots will be started directly on the metal of the server.
If the simulation project contains a `Dockerfile` file at the root level, this file will be used to start the Webots instance inside the corresponding Docker container.
Otherwise, a `Dockerfile` will be created based on the standard Docker image of Webots.
The version of Webots for the Docker image is automatically computed from the header line of the simulation world file.
For example if the world file starts with the following line:

```
#VRML_SIM {{ webots.version.major }} utf8
```
**Note**: Webots versions lower that R2022b are not supported.

The simulation server will create a `Dockerfile` starting with:
```
FROM docker image cyberbotics/webots.cloud:{{ webots.version.major }}-ubuntu22.04
```

Running Webots inside a Docker container is a very little overhead, but guarantees that the simulation server remains secure, regardless of the running simulations.

If you don't want to use Docker, you should ensure that the list of `allowedRepositories` provided in the configuration file doesn't contain any links to malicious repositories, otherwise, you are putting your simulation server at risk.

### Simulation Files Checkout

When the simulation server receives the 'start' command on the `/client` request, it will checkout the simulation files from the provided `url` pointing to a Webots world file on a GitHub repository:
```
https://github.com/alice/sim/blob/my_own_version/app/worlds/my_world.wbt
                   ^^^^^^^^^      ^^^^^^^^^^^^^^ ^^^^^^^^^^^^^^^^^^^^^^^
                   repository     tag or branch   path to world file
```
Currently, this protocol only supports public GitHub repositories.
In the above sample URL, the simulation server will checkout the `my_own_version` tag or branch of the `/app` directory from the `sim` repository of the `alice` GitHub user and it will start Webots with the specified `my_world.wbt` world file.

This protocol is still experimental and the robot windows are not yet supported.

### Checking Out Only a Git Folder

Github provides svn interface allowing to checkout only one folder on a specific branch or tag.
[Other options](https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo) are possible.
Given the URL to a Webots project is for example: [https://github.com/cyberbotics/webots/tree/master/projects/languages/python](https://github.com/cyberbotics/webots/tree/master/projects/languages/python).

We need to check out only this folder, not the whole webots repository.
This can be achieve with svn on the master branch:

`svn checkout https://github.com/cyberbotics/webots/branches/master/projects/languages/python`

Or on the {{ webots.version.major }} tag:

`svn checkout https://github.com/cyberbotics/webots/tags/{{ webots.version.major }}/projects/languages/python`

To check if a branch or a tag exists:

`svn ls https://github.com/cyberbotics/webots/branches/master`

`svn ls https://github.com/cyberbotics/webots/tags/{{ webots.version.major }}`

`git ls-remote --quiet --heads https://github.com/cyberbotics/webots.git master`

`git ls-remote --quiet --tags https://github.com/cyberbotics/webots.git {{ webots.version.major }}`

`git ls-remote --quiet https://github.com/cyberbotics/webots.git {{ webots.version.major }}` will tell whether `{{ webots.version.major }}` is a branch or a tag.

### Tips and Troubleshooting

#### Decrease the Graphical Quality of Webots
Even if Webots is launched in `no-rendering` mode, the graphical scene will be computed. It is necessary for the camera to be able to see the scene for example.

However, the graphical quality of the Webots instance that run in the docker will not impact the graphical quality of the simulation displayed in the browser, apart from two exceptions: if you have a camera or if you use local textures (if in your protos/worlds you have images referenced as `textures/my_images.jpg` and not `https://the_address_of_my_picture.jpg`).

So, it can be a good idea to decrease the graphical settings to increase the number of instances of Webots that could run in parallel.
By changing the graphical settings (that are present in the `Preferences/OpenGL` tab of Webots) from maximum to minimum, it is possible to multiply the number of parallel instance by 3 or 4 if your GPU is the bottleneck.

To do that, you have to modify the configuration file of Webots (in `/$Home/.config/Cyberbotics/Webots-R202??.conf`), then create a new docker image. You can use the following Dockerfile:
```
FROM my_name/my_webots_repo:previous_webots_tag
COPY /path_to_the_configuation_file/Webots-R202??.conf /root/.config/Cyberbotics/Webots-R202??.conf
```

**Note**: This is already implemented in the Docker images provided by Cyberbotics.
#### Put the Assets in the Docker
Webots (in the docker) will have to load the world first. To reduce this loading time, it is possible to put the assets directly in the docker such that Webots will not need to download them from the web.

You can achieve that with the following Dockerfile:
```
FROM my_name/my_webots_repo:previous_webots_tag
COPY path_to_your_assets_folder/assets /root/.cache/Cyberbotics/Webots/assets
```

To work, your assets folder must correspond to the version of Webots that is in the docker.

For more informations about assets, see [here](installation-procedure.md#asset-cache-download).

**Note**: This is already implemented in the Docker images provided by Cyberbotics.

#### Increase the Number of Parallel Simulations Supported by the Server

The server can become saturated because both its CPU and GPU RAM are full.
What happens normally is that the GPU and CPU will grow in parallel as new simulations are added.
In most cases the GPU RAM will be saturated first.
Once it happens, the CPU RAM will take the share of the GPU and therefore fill up faster.
Once the CPU RAM is full, the swap will take over.
Once the swap is full, the computer will crash if we try to open more simulations.

##### Enable the Zram

[Zram](https://en.wikipedia.org/wiki/Zram) is a Linux kernel module that will compress the least used parts of the RAM.
It has the same role as swap but is normally faster.

To enable it:
```
sudo apt-get install zram-config
sudo service zram-config start
```

To check the zram installation:
```
cat /proc/swaps
```

It should display something like:
```
Filename                                Type        Size    Used   Priority
/dev/sda3                               partition   9215996 0      -1
/dev/zram0                              partition   755740  8104    5
/dev/zram1                              partition   755740  8004    5
/dev/zram2                              partition   755740  8120    5
/dev/zram3                              partition   755740  8064    5
```

#### Increase the Size of the Swap

Increasing the size of the swap will allow the server to support more simulations in parallel.
However, when the swap is filling-up, you may observe a performance drop.

You can follow [this tutorial](https://linuxhandbook.com/increase-swap-ubuntu/) to increase the size of the swap.
