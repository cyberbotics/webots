## Web Simulation

### Description

This section describes how to setup a simulation web service similar to [robotbenchmark.net](https://robotbenchmark.net) to run Webots in the cloud.
Such a system may be distributed on several machines.
One machine runs a session server that communicates with several simulation servers.
Each machine runs one instance of simulation server that receives requests from the session server and instantiates for each connected client a new Webots instance that communicates directly with the client. Webots instances are executed in a secure environment using [Firejail Security Sandbox](https://firejail.wordpress.com/).

The Web Simulation system is still work in progress and could change in the next releases of Webots.

### Streaming Server

#### Prerequisites

The prerequisites for the server machine(s) are the following:

- Ubuntu last LTS or Windows (macOS X is also supported, but less tested)
- Webots
- Python 3
- Web service dependencies ([Windows instructions](https://github.com/omichel/webots/wiki/Windows-Optional-Dependencies#webots-web-service), [Linux instructions](https://github.com/omichel/webots/wiki/Linux-Optional-Dependencies#webots-web-service)):
  - tornado (http://www.tornadoweb.org/en/stable/, `pip install tornado`)
  - websocket-client (https://pypi.python.org/pypi/websocket-client/, `pip install websocket-client`)
  - nvidia-ml-py3 (https://pypi.python.org/pypi/nvidia-ml-py3/, `pip install nvidia-ml-py3`)
  - psutil (https://pypi.python.org/pypi/psutil/, `pip install psutil`)
  - requests (https://pypi.python.org/pypi/requests/, `pip install requests`)
  - optional: firejail (https://firejail.wordpress.com/, `apt install firejail`)

Note that the simulation server machines have to met the [Webots system requirements](system-requirements.md).
They may however be virtual machine, such as AWS instances.
GPU instances are strongly recommended for performance reasons, especially if the simulation involves sensors relying on OpenGL rendering (cameras, lidars, etc.).

#### Overview

In order to run Webots in the cloud, you need to run at least one session server and one or more simulation servers.
The simulation servers should run on different machines while the session server may run on a machine where a simulation server is running.
Both servers are Python scripts: `simulation_server.py` and `session_server.py` located in "[WEBOTS\_HOME/resources/web/server/](https://github.com/omichel/webots/tree/master/resources/web/server/)".

Note that Webots have to be installed on all the machines where the simulation server is running.


%figure "Web simulation server network infrastructure"

![context_menu.png](images/web_simulation_network_infrastructure.thumbnail.png)

%end

#### Protocol

When a web client needs to know whether it may start a simulation, it will open a WebSocket connection to the session server to monitor the availability of simulation servers.
The session server will answer '1' if some simulation server is available and '0' if none is available.
Whenever this situation changes, the session server will notify the web clients.
Note that the session server will never send twice the same value, it sends only changes in the availability of simulation servers.

When a web client wants to start a simulation, it will send an AJAX request to the session server.
The session server will then send the WebSocket URL of an available simulation server or an error if none are available. The web client will then contact directly the simulation server to startup the simulation.
The simulation server will start Webots with the simulation requested by the client.
The projects files requested by the client should be stored on the website host and requested by the simulation server through an AJAX request at `<host>/ajax/download_project.php`.
Then, the simulation server will send the WebSocket URL of Webots, so that the client can communicate directly with Webots.
If the WebSocket connection to the simulation server is closed or broken, the simulation server will close Webots.
If Webots quits, the simulation server will notify the web client and close the connection with it.

The session server regularly queries the simulation servers to monitor their load.
If a simulation server becomes overloaded, then, the session server will consider it is not available any more and will not offer it to web clients.

%figure "Web simulation protocol sequence diagram"
%chart
sequenceDiagram
  participant U as User
  participant C as Web Client
  participant W as Web Server
  participant SE as Session Server
  participant SS1 as Simulation Server 1
  participant SW1 as Webots
  participant SS2 as Simulation Server 2

  SE->>SS1: Check server availability
  activate SS1
    SS1-->>SE: Return machine load
  deactivate SS1
  SE->>SS2: Check server availability
  activate SS2
    SS2-->>SE: Return machine load
  deactivate SS2

  activate C
    C->>W: Load web page
    activate W
      W->>SE: Check server availability
      activate SE
        SE-->>W: 0 or 1
      deactivate SE
      W-->>C: Return web page content
    deactivate W
    U->>C: Start simulation
    Note left of C: webots.min.js
    C->>SE: Open simulation session
    activate SE
      SE-->>C: Return simulation server web socket URL
    deactivate SE
    C->>SS1: Start simulation
    activate SS1
      SS1->>W: Download simulation project
      activate W
        Note right of W: download_project.php
        W-->>SS1: Return simulation project archive
      deactivate W
      SS1->>SW1: Start Webots
      activate SW1
        SS1-->>C: Return Webots web socket URL
      deactivate SS1
      C->>SW1: Register client
      SW1->>C: Send simulation worlds status
      U->>C: Run simulation
      C->>SW1: Change simulation running mode
      loop Each simulation step
        SW1->>C: Send simulation world status
      end
      U->>C: Edit and save controller
      C->>W: Send new file
      activate W
        Note right of W: upload_file.php
        W-->>C: Return OK
      deactivate W
      C->>SW1: Send new file
      U->>C: Reset simulation
      C->>SW1: Reset simulation
      loop Each simulation step
        SW1->>C: Send simulation worlds status
      end
      C-->>SS1: {Client disconnected}
    deactivate C
    activate SS1
      SS1->>SW1: Kill Webots
    deactivate SW1
  deactivate SS1
%end
%end

#### Session Server

The session server is the entry point for requesting the start of a new simulation.
It manages the load of the simulation server machines and sends the URL of the available simulation server to the client.

These are the configuration parameters for the session server:
- `port`: local port on which the server is listening.
- `server`: host where this session script is running.
- `administrator`: email address of administrator that will receive notifications about the status of the simulation server machines.
- `mailServer`: mail server host from which the notifications are sent.
- `mailSender`: email address used to send the notifications.
- `simulationServers`: lists all the available simulation servers.
- `sslCertificate`: path to the certificate file for a SSL enabled server.
- `sslKey`: path to the private key file for a SSL enabled server.
- `logDir`: directory where the log file is written. Default value is "WEBOTS\_HOME/resources/web/server/log/session".

HTTP request handlers:
* The `/` request queries the availability of the simulation servers and returns 1 if some are available or 0 if no simulation server is available.

* The `/monitor` request opens a page showing an overview of the available simulation servers and their load.

WebSocket request handler:
* The `/session` request checks for available simulation servers and returns the URL of the minimally loaded server.

#### Simulation Server

The simulation server creates and starts a Webots instance with the desired simulation for each client request and sends the WebSocket URL of Webots to the client so that it can communicate directly with Webots.

These are the configuration parameters for the simulation server:
- `port`: local port on which the server is listening (launching Webots instances).
- `sslKey`: path to the private key file for a SSL enabled server.
- `sslCertificate`: path to the certificate file for a SSL enabled server.
- `projectsDir`: directory in which Webots projects are located.
- `keyDir`: directory where the website host keys needed for validation are stored. This folder should include a file named as the host (for example "robotbenchmark.net") containing a key identifying it.
- `logDir`: directory where the log files are written. Default value is "WEBOTS\_HOME/resources/web/server/log/simulation".
- `monitorLogEnabled`: specify if the monitor data have to be stored in a file.


HTTP request handlers:
* The `/load` request returns the current load of the machine computed as the maximum value between all the monitored data (main CPU and GPU load and memory usage, and network usage).

* The `/monitor` request opens a page showing some information about the current status of the simulation server machines, i.e the number of Webots instances running, the load of the CPU and GPU, the network usage.


WebSocket request handlers:
* The `/client` request on the simulation server URL will setup a new Webots instance and return the Webots WebSocket URL. The payload have to contain a `init` value with the format `[host, projectPart, worldFilename, user1Id, user1Name, user1Authentication, user2Id, user2Name, customData]`. This data is then used to retrieve the simulation data and setup the Webots project.

#### Network Settings

Please note that in order to be visible from the outside network, all the port used by the `session_server.py` and `simulation_server.py` scripts should be open (e.g. on simple networks, this can be done by modifying the NAT settings of the router).
The firewall of the local computer may complain about this operation, in this case, please modify its settings.

#### SSL Encryption

The simulation server works with or without SSL encryption.
SSL encryption is however strongly recommended.
The application requires the `fullchain.pem` and `privkey.pem` files and their path have to be specified in the `sslKey` and `sslCertificate` values of session and simulation configuration file.
Note that Webots will look for the file "WEBOTS\_HOME/resources/web/server/ssl/cert.pem", so you may have to rename `fullchain.pem` and copy it in the `ssl` folder or create a soft link.

#### Startup Procedure

The startup procedure is the following:

1. Start all the simulation servers (`simulation_server.py`).
2. Start the session server (`session_server.py`).

This procedure should be automated in a startup script, so that the servers are restarted after a reboot of the machine.

This folder also contains a `server.sh` utility script to automatically start and stop `session_server.py` and `simulation_server.py` with a given configuration file.

Please make sure that the `WEBOTS_HOME` variable is set before running the simulation and session server scripts.

### Website Host

#### Simulation Data Management

The host where the client website is running should have a `ajax` named folder at the root level containing these scripts:
* `download-project.php`: a script that returns a zipped archive containing the Webots simulation files to be run. It receives these parameters as POST data:
  * `user1Id`: id identifying the owner of the project, if an accounted application is used.
  * `user1Authentication`: password hash or authentication data for the user.
  * `key`: key identifying the host to authenticate the sender.
  * `project`: name of the Webots project to be downloaded.
* `upload-file.php`: a script that uploads the new controller version when the user modifies and saves it from the editor in the web interface. The POST parameters are:
  * `dirname`: name of the directory where the file has to be uploaded, mainly consisting on the project name.
  * `filename`: name of the file to be uploaded.
  * `content`: content of the file to be stored.

Sample PHP files are located in "[WEBOTS\_HOME/resources/web/server/](https://github.com/omichel/webots/tree/master/resources/web/templates/)".

#### How to Embed a Web Scene in Your Website

Similarly to [this section](web-streaming.md#how-to-embed-a-web-scene-in-your-website), to embed the simulation it is enough to instantiate a `webots.View` object from the [webots.min.js] package.
In this case the `webots.View.broadcast` parameters doesn't have to be set to true.
But some other parameters could be used:
* `webots.CustomData`: application specific data to be passed to the simulation server and then to the `download_project.php` (see [Simulation data management](web-simulation.md#simulation-data-management) section) script. These can be used to specify the setup of the simulation that will be copied in the Webots instance folder.
* `webots.showRevert`: defines whether the revert button should be displayed in the web interface toolbar.
* `webots.showQuit`:  defines whether the quit button should be displayed in the web interface toolbar.

If the application requires individual access and authentication, then these additional parameters are available:
* `webots.User1Id`: id of the main user (integer value > 0). If 0 or unset, the user is not logged in.
* `webots.User1Name`: user name of the main user.
* `webots.User1Authentication`: password hash or authentication data for the main user. Empty or unset if user not authenticated.
* `webots.User2Id`: id of the secondary user, used for example in case of a soccer match between two different users. 0 or unset if not used.
* `webots.User2Name`: user name of the secondary user.

This is the API of the `webots.View` class:
* `webots.View(view3D, mobile)`: constructor instantiating the simulation web interface and taking as argument:
  * `view3d`: DOM element that will contain the simulation web interface.
  * `mobile`: boolean variable specifying if the application is running on a mobile device.
* `webots.View.open(url)`: load the given simulation. Three different `url` formats are supported:
  * URL to a WBT file (i.e. "ws://localhost:80/simple/worlds/simple.wbt"): this is the format required to start a web simulation. The `url` value specifies both the session server host and the desired simulation name.
  * WebSocket URL (i.e. "ws://localhost:80"): this format is used for web broadcast streaming.
  * URL to a X3D file (i.e. "file.x3d"): this format is used for showing a [web scene](web-scene.md) or a [web animation](web-animation.md)).
* `webots.View.setTimeout(timeout)`: utility function to specify how long the simulation can run without user interaction. Setting a timeout value is useful to save resources on the server side and avoid having very long simulation running continuously on the machines and saturating the machines. The `timeout` value is expressed in seconds.
* `webots.View.setWebotsDocUrl(url)`: utility function to specify the URL of the Webots documentation shown in the Help window.

### Scene Refresh Rate

The scene refresh rate is defined by the `WorldInfo.FPS` field.
The same fields than for the [web animation](web-animation.md#limitations) are updated.

### Limitations

The streaming server has the same limitations as the [Web streaming](web-streaming.md#limitations).

### Technologies and Limitations

The data is sent to the clients using [WebSockets](https://www.websocket.org/).
In case of related issues, make sure that `WebSockets` are enabled in your Web browser settings.

Please refer to the limitations described in [this section](web-animation.md#remarks-on-the-used-technologies-and-their-limitations).
