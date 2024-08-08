## Installation Procedure

Usually, you will need to have system administrator rights to install Webots.
Once installed, Webots can be used by a regular, unprivileged user.
To install Webots, please follow this procedure:

1. Uninstall completely any old version of Webots that may have been installed on your computer previously.
2. Install Webots for your operating system as explained below.

> **Note**: After installation, the most important Webots features will be available, but some third party tools (such as Java, Python or MATLAB) may be necessary to run or compile specific projects.
The [chapter](language-setup.md) covers the set up of these tools.

### Installation on Linux

Webots will run on most recent Linux distributions running glibc2.11.1 or earlier.
This includes fairly recent Ubuntu, Debian, Fedora, SuSE, RedHat, etc.
Webots comes in three different package types: `.deb` (Debian package), `.tar.bz2` (tarball package) and `.snap` (snap package).
The Debian package is aimed at the latest LTS Ubuntu Linux distribution whereas the tarball and snap packages includes many dependency libraries and are therefore best suited for installation on other Linux distributions.
All these packages can be installed from our [official GitHub repository](https://github.com/cyberbotics/webots/releases).

The packages also contain a precompiled ROS API built with the latest recommended ROS distribution.
For more details about the ROS version supported out of the box by each package please refer to [this section](tutorial-9-using-ros.md#check-compatibility-of-webots-ros-api).

> **Note**: Webots will run much faster if you install an accelerated OpenGL drivers.
If you have a NVIDIA or AMD graphics card, it is highly recommended that you install the Linux graphics drivers from these manufacturers to take the full advantage of the OpenGL hardware acceleration with Webots.
Please find instructions in [this section](verifying-your-graphics-driver-installation.md).

#### Installing the Debian Package with the Advanced Packaging Tool (APT)

The advantage of this installation is that Webots will be updated automatically with system updates.
The installation requires the `root` privileges.

First of all, Webots should be authenticated with the [Cyberbotics.asc](https://cyberbotics.com/Cyberbotics.asc) signature file.

> **Note**: You can check with `apt-key list` if this signature file was already installed using the deprecated `apt-key add` method.
If so, you should delete it with `apt-key del <keyid>` before proceeding with the re-installation.
Similarly, if the repository was already listed, you should remove it using `apt-add-repository -y --remove 'deb https://cyberbotics.com/debian/ binary-amd64/'`.

You can install the [Cyberbotics.asc](https://cyberbotics.com/Cyberbotics.asc) signature file using this command:

```bash
sudo mkdir -p /etc/apt/keyrings
cd /etc/apt/keyrings
sudo wget -q https://cyberbotics.com/Cyberbotics.asc
```

Then, you can configure your APT package manager by adding the Cyberbotics repository.
Simply execute the following lines:

```bash
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update
```

Then proceed to the installation of Webots using:

```bash
sudo apt install webots
```

> **Note**: Although only the command line procedure is documented here, it is also possible to use any APT front-end tool, such as the Synaptic Package Manager, to proceed with the APT installation of Webots.

#### Installing the Debian Package Directly

This procedure explains how to install Webots directly from the Debian package (having the `.deb` extension), without using the APT system.
Unlike with the APT system, you will have to repeat this operation manually each time you want to upgrade to a newer version of Webots.

On Ubuntu, double-click on the Debian package file to open it with the Ubuntu Software App and click on the `Install` button.
If a previous version of Webots is already installed, then the text on the button could be different, like `Upgrade` or `Reinstall`.

Alternatively, the Debian package can also be installed using `apt` or `gdebi` with the `root` privileges:

```bash
sudo apt install ./webots_{{ webots.version.debian_package }}_amd64.deb
```

Or:

```bash
sudo gdebi webots_{{ webots.version.debian_package }}_amd64.deb
```

#### Installing the "tarball" Package

This section explains how to install Webots from the tarball package (having the `.tar.bz2` extension).

The tarball package can be installed without the `root` privileges.
It can be extracted anywhere using the `tar` `xjf` command line.
Once extracted, it is recommended to set the WEBOTS\_HOME environment variable to point to the `webots` directory obtained from the extraction of the tarball:

```bash
tar xjf webots-{{ webots.version.package }}-x86-64.tar.bz2
```

And:

```bash
export WEBOTS_HOME=/home/username/webots
```

The export line should however be included in a configuration script like "/etc/profile", so that it is set properly for every session.

You will need to install *make* and *g++* to compile your own robot controllers.
Other particular libraries could also be required to recompile some of the distributed binary files.
The package names could slightly change on different releases and distributions.
In this case an error message will be printed in the Webots console mentioning the missing dependency.
Webots also needs the *ffmpeg* and *libavcodec-extra* packages to create MPEG-4 movies.
Additionally *ubuntu-restricted-extras* could be needed to play the MPEG-4 movies encoded with H.264 codec.
Execute the following commands to enable the video creation and playback on Debian / Ubuntu based distributions:
```bash
sudo apt-get update
sudo apt-get install ffmpeg libavcodec-extra
sudo apt-get install ubuntu-restricted-extras
```
Using Anaconda could cause errors when recording videos, as the default conda installation of *ffmpeg* does not have *x264* enabled.
Execute the following command to install *ffmpeg* with *x264* support:
```bash
conda install x264 ffmpeg -c conda-forge
```

#### Installing the Snap Package

Snap packaging is a modern alternative to older packaging systems.
It runs software in a sand-boxed environment to guarantee the security of the operating system.
The snap package of Webots combines the advantages of the Debian package installed with APT and the tarball package.
To install it, simply follow the instructions from the official [snap store](https://snapcraft.io/webots) or proceed through the software center of your Ubuntu distribution.
It is very simple to install, automatically updates, runs on a large variety of Linux distributions and has no dependency.
However, the sand-boxing constraints of snaps yield the following limitations:

##### Download Size

The download is significantly bigger as it includes all the dependencies of Webots (ffmpeg, Python, C++ and Java compilers, etc.).
For Webots R2023b, the download size of the snap is 651MB compared to 146MB of the Debian package.

##### Extern Controllers

It is not possible to change the built-in dependencies of the snap package (Python interpreter, C/C++/Java compilers, JRE, etc.), or install any extra dependencies (native libraries, Python modules, etc.), or run MATLAB controllers.
However, when developing robot controllers, it is often useful to use various components such as a different version of Python, some Python modules (pip), native shared libraries, or to run some MATLAB controllers.
If such components are needed, users can install them on their system or local environment to create, possibly compile and link their robot controllers.
However, because of the snap sand-boxing, Webots will be unable to launch these controller itself.
To work around this problem, such controllers should be launched as extern controllers from outside of Webots.
Before launching extern controllers, you should set the `WEBOTS_HOME` environment variable to point to `/snap/webots/current/usr/share/webots` and run the `$WEBOTS_HOME/webots-controller` launcher.
The chapter entitled [running extern robot controllers](running-extern-robot-controllers.md) details how to run extern controllers, including with the snap version of Webots.

#### Installing the Docker Image

[Docker](https://www.docker.com) images of Webots based on Ubuntu 20.04 are available on [dockerhub](https://hub.docker.com/r/cyberbotics/webots).

These images can be used to run Webots in your continuous integration (CI) workflow without requiring any graphical user interface or to get a clean and sandboxed environment with Webots pre-installed including GPU accelerated graphical user interface.

##### Install Docker

Follow the [Docker installation instructions](https://docs.docker.com/engine/install/#server) to install docker.

##### Run Webots in Docker in Headless Mode

The docker image comes with a X virtual framebuffer (Xvfb) already installed and configured so that you can run Webots in headless mode.

To pull the image and start a docker container with it use the following command:
```bash
docker run -it cyberbotics/webots:latest
```

> **Note**: If you need a specific version of Webots or Ubuntu and not the latest ones, replace `latest` with the version you need (e.g. `R2021b-ubuntu20.04`).

After starting the docker container you can start Webots headlessly using xvfb:
```bash
xvfb-run webots --stdout --stderr --batch --mode=realtime /path/to/your/world/file
```

> **Note**: Since Webots runs in headless mode, the `--stdout` and `--stderr` arguments are used to redirect these streams from the Webots console to the console in which Webots was started, the `--batch` argument disables any blocking pop-up window and the `--mode=realtime` makes sure that the simulation is not started in pause mode (you may replace `realtime` by `fast`), finally don't forget to specify which simulation you want to run.

##### Run Webots in Docker with GUI

###### Without GPU Acceleration

To run Webots with a graphical user interface in a docker container, you need to enable connections to the X server before starting the docker container:
```bash
xhost +local:root > /dev/null 2>&1
```

> **Note**: If you need to disable connections to the X server, you can do it with the following command: `xhost -local:root > /dev/null 2>&1`.

You can then start the container with the following command:
```bash
docker run -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest
```

Or if you want to directly launch Webots:
```bash
docker run -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest webots
```

###### With GPU Acceleration

To run GPU accelerated docker containers, the `nvidia-docker2` package needs to be installed.
Please follow the [official instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) to install it.

> **Note**: GPU accelerated docker containers will work only with recent NVIDIA drivers and Docker versions (see the complete list of requirements [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#pre-requisites)).

Once this package is installed, use the same procedure than without GPU acceleration, but add the `--gpus=all` when starting the docker container:
```bash
docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest
```

##### Upgrade to the Latest Webots Image

If the latest Docker image was upgraded, e.g., with a new version of Webots, you should run the following command to upgrade your local copy:
```bash
docker pull cyberbotics/webots:latest
```

##### Troubleshooting

On some Linux systems, such as Arch Linux, you may get errors related to `fontconfig` when starting Webots.
If that happens, you should clear the font cache and start Webots again:

```bash
sudo rm /var/cache/fontconfig/*
rm ~/.cache/fontconfig/*
```

#### Server Edition

Webots requires some graphical features that are usually not available by default on a Linux server edition, [additional packages]({{ url.github_tree }}/scripts/install/linux_runtime_dependencies.sh) needs to be manually installed to make it work.

Webots can be run without GUI using a virtual framebuffer such as [Xvfb](https://en.wikipedia.org/wiki/Xvfb):
```bash
xvfb-run --auto-servernum webots --mode=fast --no-rendering --stdout --stderr --minimize --batch /path/to/world/file
```

### Installation on Windows

1. Download the "webots-{{ webots.version.package }}\_setup.exe" installation file from our [website](https://cyberbotics.com).
2. Double click on this file.
3. Follow the installation instructions.
4. (Optional) Follow the [programming language setup](language-setup.md) instructions, if you plan on using specific languages such as Python or Java.

It is possible to install Webots silently from an administrator DOS console, by typing:

```bash
webots-{{ webots.version.package }}_setup.exe /SUPPRESSMSGBOXES /VERYSILENT /NOCANCEL /NORESTART /ALLUSERS
```

Additional setup options are available and documented from:

```bash
webots-{{ webots.version.package }}_setup.exe /?
```

Once installed, if you observe 3D rendering anomalies or if Webots crashes, it is strongly recommend to upgrade your graphics driver.

#### Windows SmartScreen

It may be possible that Windows Defender SmartScreen will display a warning when starting the Webots installer:

%figure "Windows SmartScreen warning"
![windows_smartscreen_1.png](images/windows_smartscreen_1.thumbnail.jpg)
%end

This is likely caused by the fact that the release of Webots is recent and was not yet approved by Microsoft.
If the Webots installer was downloaded from the [official Cyberbotics web site](https://cyberbotics.com) or [official GitHub repository](https://github.com/cyberbotics/webots/releases) using the secure HTTPS protocol, then it is safe to install it.
You can pass this warning and install Webots by clicking on the "More info" link and the "Run anyway" button depicted below:

%figure "Windows SmartScreen pass"
![windows_smartscreen_2.png](images/windows_smartscreen_2.thumbnail.jpg)
%end

### Installation on macOS

#### From the Installation File

It is better to download Webots using `curl` so that it doesn't get tagged as "downloaded from the Internet" and won't be blocked by macOS Gatekeeper.
To proceed, open the Terminal and type the following instructions to download and mount the Webots disk image:
```bash
curl -L -O https://github.com/cyberbotics/webots/releases/download/{{ webots.version.package }}/webots-{{ webots.version.package }}.dmg
open webots-{{ webots.version.package }}.dmg
```

To install Webots only for the current user, without administrator privileges, proceed with:
```bash
mkdir ~/Applications
cp -r /Volumes/Webots/Webots.app ~/Applications
```

To install Webots for any user, copy the Webots app to the system `/Applications` folder instead (administrator privileges required).

Finally, you can launch Webots typing any of these instructions:
```bash
open ~/Applications/Webots.app    # to launch Webots using the open command
~/Applications/Webots.app/webots  # to launch Webots directly
```

Alternatively, you can double-click on the Webots icon to launch it.

#### Troubleshooting for Apple Silicon Users

If you are getting errors like these:
```bash
...(mach-o file, but is an incompatible architecture (have (arm64), need (x86_64)))
```

This is likely caused by Rosetta loading Webots under the x86 architecture instead of the native ARM. Unless you really want to use x86 binaries with your Webots simulation, make sure to turn off Rosetta. Sometimes macOS may try to open the app using Rosetta by default, which may cause issues when it comes to run robot controllers with dependencies on ARM libraries.

To check if it's opened using Rosetta, right click on the Webots application in Finder and select Get Info.

Make sure that the "Open using Rosetta" setting is unchecked, like in the picture below:

%figure "Rosetta setting"

![rosetta_setting.png](images/rosetta_setting.png)

%end

#### From the Homebrew Package

A [Homebrew package](https://formulae.brew.sh/cask/webots) is available for Webots.

If brew is not already installed on your computer, install it with the following command in a terminal:
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/released/install.sh)"
```

Webots can then be installed with:
```bash
brew install --cask webots
```

#### Working around macOS Gatekeeper

If Webots was downloaded from a web browser (e.g., not from a Terminal with `curl` or `wget`) macOS Gatekeeper may refuse to run Webots because it is from an unidentified developer (see [this figure](#unidentified-developer-dialog)).
You will need administrator privileges to be able to install Webots.

%figure "Unidentified developer dialog"

![mac-unidentified-developer-dialog.png](images/mac-unidentified-developer-dialog.png)

%end

You should <kbd>ctrl</kbd> + click (or right-click) on the Webots icon, and select the `Open` menu item.
Then, macOS should propose to open the application anyway (see [this figure](#unidentified-developer-dialog)).

%figure "Open Webots anyway"

![mac-open-anyway.png](images/mac-open-anyway.thumbnail.jpg)

%end

More information about disabling macOS Gatekeeper is available [here](https://disable-gatekeeper.github.io/).
You may also change your macOS security settings to open Webots anyway (`System Preferences / Security & Privacy / General / Allow apps downloaded from:`).

### Asset Cache Download

From Webots 2021b, the necessary assets used in a world are downloaded on the fly as they are requested, and cached for subsequent usage.
This allows to progressively download the assets as they are needed instead of downloading them all up-front, hence reducing the size of the distributions.
From Webots 2022b a zip version of the entire cache is also available for download, meaning instead of letting Webots build it over time it can be used directly.
This is beneficial for an offline usage of Webots or to mount it as a volume in a docker setting.
1. Ensure that the size of the Webots disk cache is at least 1024 MB to be able to store all the assets: `Preferences -> Network -> Disk Cache`.
2. Download the archive corresponding to your Webots version from the [releases](https://github.com/cyberbotics/webots/releases) page on github.
If you have installed a nightly build of Webots, then you need to download the archive corresponding to that specific build.
3. Depending on your operating system, the default location of the Webots cache is shown below.
Please note that the assets need to be in a folder named `assets`, as such when decompressing the archive it might be necessary to rename the folder or to remove any intermediary directories being created.

%tab-component "os"

%tab "Windows"

Extract the archive to:

`C:/Users/<USER>/AppData/Local/Cyberbotics/Webots/cache`

***Note:*** a folder named `assets` needs to be present in this location and if one already exists, it should be overwritten.

%tab-end

%tab "Linux"

Extract the archive to:

`~/.cache/Cyberbotics/Webots`

***Note:*** a folder named `assets` needs to be present in this location and if one already exists, it should be overwritten.

%tab-end

%tab "macOS"

Extract the archive to:

`~/Library/Caches/Cyberbotics/Webots"` or `"/Library/Caches/Cyberbotics/Webots`

***Note:*** a folder named `assets` needs to be present in this location and if one already exists, it should be overwritten.

%tab-end

%end

&nbsp;

In summary, for instance on Linux the expected structure should look something like:

```
~/.cache/Cyberbotics/Webots/assets/
├── 0033b105637903b72be80210f36ad6d4efac8813
├── 05599e9eefd659b2a2c0e4393ef60d1024b977ef
├── 103be80357b69185ac460c11e0d8a9d39b76d804
├── 11b83067b8ca597dbf24593f3790b3df8fa6b87c
├── 123a565fefa525671b4af73b7667a89d2c05ddd6
├── 1c004aaa4706ef38c764f5df1e17344035fe74fe
...

```
