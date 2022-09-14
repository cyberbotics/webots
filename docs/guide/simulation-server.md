## Simulation Server

Once a [session server](session-server.md) is installed, you can proceed with the installation of one or several simulation servers.
The first simulation server may run on the same machine as the session server.
Others should run on different machines.
There should be only one simulation server per machine.
Simulation servers can launch several instances of Webots on their machine.

### Description

The simulation server creates and starts a Webots instance with the desired simulation for each client request and sends the WebSocket URL of Webots to the client so that it can communicate directly with Webots.

These are the configuration parameters for the simulation server:
```
# server:              fully qualilified domain name of simulation server
# ssl:                 for https/wss URL (true by default)
# port:                local port on which the server is listening
# portRewrite:         port rewritten in the URL by apache (true by default)
# docker:              launch webots inside a docker (false by default)
# allowedRepositories: list of allowed GitHub simulation repositories
# blockedRepositories: list of blocked GitHub simulation repositories
# shareIdleTime:       maximum load for running non-allowed repositories (50% by default)
# notify:              webservices to be notified about the server status (https://webots.cloud by default)
# projectsDir:         directory in which projects are located
# webotsHome:          directory in which Webots is installed (WEBOTS_HOME)
# maxConnections:      maximum number of simultaneous Webots instances
# logDir:              directory where the log files are written
# monitorLogEnabled:   store monitor data in a file (true by default)
# debug:               output debug information to stdout (false by default)
# timeout:             number of seconds after which a simulation is automatically closed (two hours by default, minimum six minutes)
```

HTTP request handlers:
* The `/load` request returns the current load of the machine computed as the maximum value between all the monitored data (main CPU and GPU load and memory usage, and network usage).

* The `/monitor` request opens a page showing some information about the current status of the simulation server machines, i.e the number of Webots instances running, the load of the CPU and GPU, the network usage.

WebSocket request handlers:
* The `/client` request on the simulation server URL will setup a new Webots instance and return the Webots WebSocket URL.
The payload should be a JSON object named `start` containing a `url` string and optionally a `mode` string which can be either `x3d` (default value) or `mjpeg`.
```json
{
  "start": {
    "url": "https://github.com/alice/sim/blob/my_own_version/app/worlds/my_world.wbt",
    "mode": "mjpeg"
  }
}
```

### Hardware Requirements

A simulation server should run on any machine with a NVIDIA graphics card.
A NVIDIA graphics card is strongly recommended as we found no solution to run 3D hardware accelerated graphics in a Docker with a AMD graphics card.
We recommend to use a machine with a powerful multi-core CPU and powerful graphics card.
This will allow you to run a large number of Webots simulations in parallel on the same machine.
For example, the following systems are a very good configurations:
- AMD Ryzen 9 5950X & GeForce RTX 3080
- AMD Ryzen 7 5800X & GeForce GTX 1660 SUPER

It is recommended to [configure the BIOS of the computer for automatic reboot after power outage](session-server.md#hardware-requirements).

### Network Requirements

The simulation machines don't need to have their own fully qualified domain name.
However, all these machines should be installed on the same local network as the [session server](session-server.md).
The machine running the session server should be able to access any port of the simulation machines, which is usually the case on local networks.

### Software Requirements

If you are installing the simulation server on the same machine as the session server, you can skip steps 1 to 4.

1. Install Ubuntu 20.04:
    - Choose the desktop version of Ubuntu 20.04. Simulation servers needs a X display and 3D OpenGL hardware acceleration which is easier to setup from a desktop version.
    - create a user account named `cyberbotics` (or anything else).
2. Makes that this user account has the auto-login feature enabled, so that when you boot the machine it get automatically logged in.
3. Configure the [unattended upgrades](https://www.linuxbabe.com/ubuntu/automatic-security-update-unattended-upgrades-ubuntu) to reboot after security updates.
4. For your convenience, it is recommended to install ssh: `apt install ssh`, so that you can login remotely, possibly with display export, e.g., `ssh -X`.
5. Install Python 3: `sudo apt-get install python3-pip python-is-python3`.
6. Install Python dependencies: `pip install pynvml requests psutil tornado distro`.
7. Install git and subversion: `sudo apt-get install git subversion`. They are used by the simulation server to checkout the code of the projects.
8. Choose one:
    - Install Docker if you want to run Webots and the controllers safely in a Docker (recommended): `sudo apt install docker.io` and follow the [post-installation instructions](https://docs.docker.com/engine/install/linux-postinstall/): `sudo usermod -aG docker $USER` and `newgrp docker`. You will also have to install the NVIDIA Docker drivers as documented [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
    - Or [install Webots](https://github.com/cyberbotics/webots/releases/latest) if you do not want to run the Webots instances in Docker (not recommended as it might compromise the security of your server and would make it more difficult to handle different versions of Webots). In this configuration, only the simulations from the repositories explicitly declared in `allowedRepositories` are allowed to run.
9. Install docker-compose if you want to run Webots simulation in dockers: `pip install docker-compose`
10. Clone the [webots-server](https://github.com/cyberbotics/webots-server) repository in `~/webots-server`
11. Optional: make the NVIDIA accelerated X server work also headless (with no screen connected):
    - `sudo nvidia-xconfig --allow-empty-initial-configuration`

### Setup

1. Install the docker images of the Webots versions that you want to support.

    ```
    docker pull cyberbotics/webots.cloud:R2022b
    docker pull cyberbotics/webots.cloud:R2022b-numpy
    ```
2. Configure the simulation server: create a file named `~/webots-server/config/simulation/simulation.json` with the following contents (to be adapted to your local setup):

    ```
    {
      "webotsHome": "/home/cyberbotics/webots",
      "port": 2000,
      "fullyQualifiedDomainName": "cyberbotics1.epfl.ch",
      "portRewrite": true,
      "logDir": "log/",
      "debug": true
    }
    ```
3. Configure Apache 2 on the session server machine to redirect traffic on the simulation machine:
    - If you are running the simulation server on the same machine as the session server, you can skip this step.
    - Otherwise edit /etc/apache2/site-available/000-default-le-ssl.conf and modify the rewrite rules to direct the traffic to the various machines (session server and simulation servers):

    ```
    RewriteRule ^/1999/(.*)$ "ws://localhost:$1/$2" [P,L]            # session server
    RewriteRule ^/2(\d{3})/(.*)$ "ws://<IP address 2>:$1/$2" [P,L]   # simulation server server with ports in the range 2000-2999
    RewriteRule ^/3(\d{3})/(.*)$ "ws://<IP address 3>:$1/$2" [P,L]   # other simulation server server with ports in the range 3000-3999
    ⋮

    RewriteRule ^/1999/(.*)$ "http://localhost:$1/$2" [P,L]          # session server
    RewriteRule ^/2(\d{3})/(.*)$ "http://<IP address 2>:$1/$2" [P,L] # simulation server with ports in the range 2000-2999
    RewriteRule ^/3(\d{3})/(.*)$ "http://<IP address 3>:$1/$2" [P,L] # other simulation server with ports in the range 3000-3999
    ⋮
    ```

4. Configure the session server to use this simulation server: edit `~/webots-server/config/session/session.json` to add the simulation server in the simulationServers section:

    ```
    ⋮
    "simulationServers": [
       "cyberbotics1.epfl.ch/2000"
    ]
    ⋮
    ```

5. Setup the automatic launch of the simulation server on reboot.

    ```
    cd ~/.config
    mkdir -p autostart
    cd autostart
    echo "[Desktop Entry]" > simulation_server.desktop
    echo "Name=simulation_server" >> simulation_server.desktop
    echo "Exec=python /home/cyberbotics/webots-server/simulation_server.py /home/cyberbotics/webots-server/config/simulation/simulation.json" >> simulation_server.desktop
    echo "Type=Application" >> simulation_server.desktop
    echo "X-GNOME-Autostart-enabled=true" >> simulation_server.desktop
    ```
