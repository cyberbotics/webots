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

> **Note**: Webots will run much faster if you install an accelerated OpenGL drivers.
If you have a NVIDIA or AMD graphics card, it is highly recommended that you install the Linux graphics drivers from these manufacturers to take the full advantage of the OpenGL hardware acceleration with Webots.
Please find instructions in [this section](verifying-your-graphics-driver-installation.md).

#### Installing the Debian Package with the Advanced Packaging Tool (APT)

The advantage of this installation is that Webots will be updated automatically with system updates.
The installation requires the `root` privileges.

First of all, Webots should be authenticated with the [Cyberbotics.asc](https://cyberbotics.com/Cyberbotics.asc) signature file which can be installed using this command:

```sh
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
```

Then, you can configure your APT package manager by adding the Cyberbotics repository.
Simply execute the following lines:

```sh
sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
sudo apt-get update
```

As an alternative, you can easily add the Cyberbotics repository from the `Software and Updates` application.
In the `Other Software` tab, click on the `Add...` button and copy the following line:

```text
deb https://cyberbotics.com/debian/ binary-amd64/
```

When you close the window, the APT packages list should be automatically updated.
Otherwise you can manually execute the following command:

```sh
sudo apt-get update
```

Then proceed to the installation of Webots using:

```sh
sudo apt-get install webots
```

> **Note**: Although only the command line procedure is documented here, it is also possible to use any APT front-end tool, such as the Synaptic Package Manager, to proceed with the APT installation of Webots.

#### Installing the Debian Package Directly

This procedure explains how to install Webots directly from the Debian package (having the `.deb` extension), without using the APT system.
Unlike with the APT system, you will have to repeat this operation manually each time you want to upgrade to a newer version of Webots.

On Ubuntu, double-click on the Debian package file to open it with the Ubuntu Software App and click on the `Install` button.
If a previous version of Webots is already installed, then the text on the button could be different, like `Upgrade` or `Reinstall`.
Note that GNOME Software App distributed in the first release of Ubuntu 16.04 contains a bug preventing the installation of third-party packages.

Alternatively, the Debian package can also be installed using `apt` or `gdebi` with the `root` privileges:

```sh
sudo apt install ./webots_{{ webots.version.debian_package }}_amd64.deb
```

Or:

```sh
sudo gdebi webots_{{ webots.version.debian_package }}_amd64.deb
```

#### Installing the "tarball" Package

This section explains how to install Webots from the tarball package (having the `.tar.bz2` extension).
This package can be installed without the `root` privileges.
It can be uncompressed anywhere using the `tar` `xjf` command line.
Once uncompressed, it is recommended to set the WEBOTS\_HOME environment variable to point to the `webots` directory obtained from the uncompression of the tarball:

```sh
tar xjf webots-{{ webots.version.package }}-x86-64.tar.bz2
```

And:

```sh
export WEBOTS_HOME=/home/username/webots
```

The export line should however be included in a configuration script like "/etc/profile", so that it is set properly for every session.

> **Note**: Webots needs the *ffmpeg* and *libfdk-aac1* (from *ubuntu-restricted-extras* for H.264 codec) packages to create MPEG-4 movies.
You will also need to install *make* and *g++* to compile your own robot controllers.
Other particular libraries could also be required to recompile some of the distributed binary files.
In this case an error message will be printed in the Webots console mentioning the missing dependency.
The package names could slightly change on different releases and distributions.


#### Installing the Snap Package

Snap packaging is a modern alternative to older packaging systems.
It runs software in a sand-boxed environment to guarantee the security of the operating system.
The snap package of Webots combines the advantages of the Debian package installed with APT and the tarball package.
To install it, simply follow the instructions from the official [snap store](https://snapcraft.io/webots) or proceed through the software center of your Ubuntu distribution.
It is very simple to install, automatically updates, runs on a large variety of Linux distributions and has no dependency.
However, the sand-boxing constraints of snaps yield the following limitations:

##### Download Size

The download is significantly bigger as it includes all the dependencies of Webots (ffmpeg, python, C++ and java compilers, etc.).
For Webots R2019b revision 1, the download size of the snap is 1.8GB compared to 1.3GB of the Debian and tarball packages.

##### Extern Controllers

It is not possible to change the built-in dependencies of the snap package (Python interpreter, C/C++/Java compilers, JRE, etc.), or install any extra dependencies (native libraries, Python modules, etc.), or run MATLAB controllers.
However, when developing robot controllers, it is often useful to use various components such as a different version of Python, some Python modules (pip), native shared libraries, or to run some MATLAB controllers.
If such components are needed, users can install them on their system or local environment to create, possibly compile and link their robot controllers.
However, because of the snap sand-boxing, Webots will be unable to launch these controller itself.
To work around this problem, such controllers should be launched as extern controllers from outside of Webots.
Before launching extern controllers, you should set the `WEBOTS_HOME` environment variable to point to `/snap/webots/current/usr/share/webots` and add `$WEBOTS_HOME/lib` to your `LD_LIBRARY_PATH` environment variable, so that your controllers will find the necessary shared libraries.
The chapter entitled [running extern robot controllers](running-extern-robot-controllers.md) details how to run extern controllers, including with the snap version of Webots.


#### Server Edition

Webots requires some graphical features that are usually not available by default on a Linux server edition and additional packages needs to be available to make it work:

- `xserver-xorg-core`
- `libpulse0`

These packages are automatically installed when using the Debian package, but in case of the tarball package the user has to manually install them.

Additionally, it is also necessary to install an OS GUI, for example the Unity desktop `ubuntu-desktop` package.

### Installation on Windows

1. Download the "webots-{{ webots.version.package }}\_setup.exe" installation file from our [website](https://cyberbotics.com).
2. Double click on this file.
3. Follow the installation instructions.

It is possible to install Webots silently from an administrator DOS console, by typing:

```bash
webots-{{ webots.version.package }}_setup.exe /SILENT
```

Or:

```bash
webots-{{ webots.version.package }}\_setup.exe /VERYSILENT
```

Once installed, if you observe 3D rendering anomalies or if Webots crashes, it is strongly recommend to upgrade your graphics driver.

### Windows SmartScreen

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

1. Download the `webots-{{ webots.version.package }}.dmg` installation file from our [website](https://cyberbotics.com).
2. Double click on this file.
This will mount on the desktop a volume named "Webots" containing the "Webots" folder.
3. Move this folder to your "/Applications" folder or wherever you would like to install Webots.

### macOS Security

During the first Webots launch, macOS may complain about opening Webots because it is from an unidentified developer (see [this figure](#unidentified-developer-dialog)).

%figure "Unidentified developer dialog"

![mac-unidentified-developer-dialog.png](images/mac-unidentified-developer-dialog.png)

%end

In this case, `Ctrl + click` (or right-click) on the Webots icon, and select the `Open` menu item.
`macOS` should propose to open the application anyway (see [this figure](#unidentified-developer-dialog)).

%figure "Open Webots anyway"

![mac-open-anyway.png](images/mac-open-anyway.thumbnail.jpg)

%end

In earlier versions of macOS, this last operation may not work.
In this case, refer to your macOS security settings to open Webots anyway (`System Preferences / Security & Privacy / General / Allow apps downloaded from:`).
