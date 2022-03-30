## Web Simulation

### Description

This section describes how to setup a simulation web service similar to [robotbenchmark.net](https://robotbenchmark.net) to run Webots in the cloud.
Such a system may be distributed on several machines to provide a powerful cluster of simulation servers.
One machine runs a session server that communicates with several simulation servers.
Each machine runs one instance of a simulation server that receives requests from the session server and starts for each connected client a new instance of Webots that communicates directly with the client.

Webots instances can be executed in a secure environment using [Docker](https://www.docker.com).
This is needed if the simulations are coming from the outside world and may contain some malicious code that could compromise the simulation server.
That is the case with [robotbenchmark.net](https://robotbenchmark.net) where robot controllers are python programs written by external users and may potentially harm the simulation server.
Other use cases include simulations created by external users that include binary code for a physics plug-in or a robot window.
Running them in a Docker container ensures the integrity of the simulation server.
However, if the simulations executed on a simulation server can't contain any malicious code, then it is safe to run the Webots instances without Docker.
This is the case if the simulation servers run only simulations from a limited list of allowed GitHub repositories controlled by the owner of the simulation servers.

**Note:** The Web Simulation system is still work in progress and could change in the next releases of Webots.

### Streaming Server

#### Prerequisites

The prerequisites for the server machine(s) are the following:

- Ubuntu 20.04 LTS or newer
- Web service dependencies ([Linux instructions](https://github.com/omichel/webots/wiki/Linux-Optional-Dependencies#webots-web-service)):

The simulation server machines have to met the [Webots system requirements](system-requirements.md).
They may however be virtual machine, such as AWS instances.
GPU instances are strongly recommended for performance reasons, especially if the simulation involves sensors relying on OpenGL rendering (cameras, lidars, range-finders).

#### Overview

In order to run Webots in the cloud, you need to run at least one session server and one or more simulation servers.
The simulation servers should run on different machines while the session server may run on a machine where a simulation server is running.
Both servers are Python scripts named `simulation_server.py` and `session_server.py` and located in "[WEBOTS\_HOME/resources/web/server/](https://github.com/cyberbotics/webots/tree/released/resources/web/server/)".

Either Docker or Webots has to be installed on all the machines where a simulation server is running.

%figure "Web simulation server network infrastructure"

![context_menu.png](images/web_simulation_network_infrastructure.thumbnail.png)

%end

#### Quick Start
This section gives a simple step-by-step guide on how to start a streaming server with one session and one simulation server.
We assume you use Ubuntu 20.04 or newer.

First, you need to install dependencies:
```bash
sudo apt install subversion docker python3-tornado python3-pynvml
```

Then, start a session server and a simulation server:
```bash
cd $WEBOTS_HOME/resources/web/server
./server.sh start local
```
This will start the session server with the [config/session/local.json]({{ url.github_tree }}/resources/web/server/config/session/local.json) configuration file and the simulation server with the [config/simulation/local.json]({{ url.github_tree }}/resources/web/server/config/simulation/local.json) configuration file.

You should now be able to check the status of your session server at [http://localhost:1999/monitor](http://localhost:1999/monitor).

The session server should display a list of simulation servers.
In your case, only one simulation server should be listed.
If you click on the simulation server link named localhost:2000, you should see it's status page at [http://localhost:2000/monitor](http://localhost:2000/monitor).

The session server keeps a track of the available simulation servers and their respective compute load.
It assigns a connection to the simulation server with the lowest compute load (similar to a load balancer).

Then, the selected simulation server starts a Webots instance that communicates directly with the client.

To test the session and simulation servers, simply open the `$WEBOTS_HOME/resources/web/streaming_viewer/index.html` file in your browser.
In the user interface, under the `Connect to:` field, type for example:
```
http://localhost:1999/session?url=https://github.com/cyberbotics/webots/tree/develop/projects/languages/python/worlds/example.wbt
```
Click the `Connect` button to initiate the streaming.
Webots will clone the `example.wbt` simulation from GitHub and start it.

If you want to stop both the session and simulation servers, run the following:
```
cd $WEBOTS_HOME/resources/web/server
./server.sh stop
```

Alternatively, you could have started the session server and simulation server with the following commands:

```
python session_server.py
```

And, from another terminal:

```
python simulation_server.py
```

Further in this document, you will find more details on how to start multiple simulation servers, how to monitor servers with log files, how to get e-mail notifications, how to rewrite the ports to setup SSL connections, etc.

#### Protocol

When a web client needs to know whether it may start a simulation, it will open a WebSocket connection to the session server to monitor the availability of simulation servers.
The session server will answer '1' if some simulation server is available and '0' if none is available.
Whenever this situation changes, the session server will notify the web clients.
Note that the session server will never send twice the same value, it sends only changes in the availability of simulation servers.

When a web client wants to start a simulation, it will send an AJAX request to the session server.
The session server will then send the WebSocket URL of an available simulation server or an error if none are available.
The web client will then contact directly the simulation server to start-up the simulation.
The projects files requested by the client should be stored on some GitHub repository, which will be checked out on the simulation server machine.
The simulation server will start Webots with the simulation requested by the client.
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
  participant G as GitHub Repository
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
    Note left of C: webots-streaming web component
    C->>SE: Open simulation session
    activate SE
      SE-->>C: Return simulation server web socket URL
    deactivate SE
    C->>SS1: Start simulation
    activate SS1
      SS1->>G: Clone simulation project
      activate G
        Note right of G: git clone repository
        G-->>SS1: Simulation project
      deactivate G
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
```
# server:             fully qualilified domain name of the session server
# ssl:                for https/wss URL (true by default)
# port:               local port on which the server is listening
# portRewrite:        port rewritten in the URL by apache (true by default)
# simulationServers:  lists all the available simulation servers
# administrator:      email address that will receive notifications
# mailServer:         SMTP mail server host for sending notifications
# mailServerPort:     SMTP mail server port
# mailSender:         email address used to send the notifications
# mailSenderUser:     user name to authenticate on the SMTP server
# mailSenderPassword: password of mailSenderUser
# logDir:             directory where the log file is written
# debug:              output debug information to stdout (false by default)
```

HTTP request handlers:
* The `/` request queries the availability of the simulation servers and returns 1 if some are available or 0 if no simulation server is available.

* The `/monitor` request opens a page showing an overview of the available simulation servers and their load.

WebSocket request handler:
* The `/session` request checks for available simulation servers and returns the URL of the minimally loaded server.

#### Simulation Server

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
# shareIdleTime:       maximum load for running non-allowed repositories
# notify:              webservices to be notified about the server status
# projectsDir:         directory in which projects are located
# webotsHome:          directory in which Webots is installed (WEBOTS_HOME)
# maxConnections:      maximum number of simultaneous Webots instances
# logDir:              directory where the log files are written
# monitorLogEnabled:   store monitor data in a file (true by default)
# debug:               output debug information to stdout (false by default)
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

#### Docker

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

#### Simulation Files Checkout

When the simulation server receives the 'start' command on the `/client` request, it will checkout the simulation files from the provided `url` pointing to a Webots world file on a GitHub repository:
```
https://github.com/alice/sim/blob/my_own_version/app/worlds/my_world.wbt
                   └───┬───┘      └─────┬──────┘ └─────────┬───────────┘
                  repository       tag or branch     path to world file
```
Currently, this protocol only supports public GitHub repositories.
In the above sample URL, the simulation server will checkout the `my_own_version` tag or branch of the `/app` directory from the `sim` repository of the `alice` GitHub user and it will start Webots with the specified `my_world.wbt` world file.

This protocol is still experimental and the robot windows are not yet supported.

#### Port Rewrite

In order to make the web simulation work properly from the outside world over SSL, you have to configure your web server, `session_server.py` and `simulation_server.py` to perform port rewrite.
Requests arriving to the web server will be redirected to the local network, e.g., `wss://webserver.com/2000/client` &rarr; `ws://webserver.com:2000/client` and `https://webserver.com/2000/monitor` &rarr; `http://webserver.com:2000/monitor`.
You just need to have port 443 open in your local firewall for incoming connections.
Such a setup is compatible with any client firewall allowing outgoing connections on port 443 to your server (which is very standard).

Port rewrite can be tested on a local host by setting up [XAMPP to use SSL](https://gist.github.com/nguyenanhtu/33aa7ffb6c36fdc110ea8624eeb51e69).
In the `session_server.py` configuration, the `simulationServers` should be listed using the outside URL: `hostname/2000` instead of `hostname:2000`.
Your web server should be configured to redirect `http` traffic to `https` and to rewrite ports in URLs for both `https` and `wss`.
With the Apache web server, this can be achieved by adding the following rules in your `httpd.conf` file:

%tab-component "generic"

%tab "webserver.com"
```
LoadModule proxy_module modules/mod_proxy.so
LoadModule proxy_http_module modules/mod_proxy_http.so
LoadModule proxy_wstunnel_module modules/mod_proxy_wstunnel.so

<VirtualHost *:80>
  ServerName webserver.com
  ServerAlias www.webserver.com

  [ ... ]

  RewriteEngine on

  # this rule redirect HTTP requests to HTTPS, removing the 'www.' prefix if any
  RewriteCond %{SERVER_NAME} =%{SERVER_NAME} [OR]
  RewriteCond %{SERVER_NAME} =www.%{SERVER_NAME}
  RewriteRule ^ https://%{SERVER_NAME}%{REQUEST_URI} [END,NE,R=permanent]
</VirtualHost>

<VirtualHost *:443>
  ServerName webserver.com
  ServerAlias www.webserver.com

  [ ... ]

  RewriteEngine on

  # this rule removes the 'www.' prefix from the hostname in the URL if any
  RewriteCond %{SERVER_NAME} =www.webserver.com
  RewriteRule ^ https://webserver.com%{REQUEST_URI} [END,NE,R=permanent]

  # port redirection rules (for session_server.py, simulation_server.py and webots)
  # websockets (should come first)
  RewriteCond %{HTTP:Upgrade} websocket [NC]
  RewriteCond %{HTTP:Connection} upgrade [NC]
  RewriteRule ^/(\d*)/(.*)$ "ws://%{SERVER_NAME}:$1/$2" [P,L]
  # http traffic (should come after websocket)
  RewriteRule ^/(\d*)/(.*)$ "http://%{SERVER_NAME}:$1/$2" [P,L]

</VirtualHost>
```
%tab-end

%tab "localhost"
```
LoadModule alias_module modules/mod_alias.so
LoadModule proxy_module modules/mod_proxy.so
LoadModule proxy_http_module modules/mod_proxy_http.so
LoadModule proxy_wstunnel_module modules/mod_proxy_wstunnel.so

<VirtualHost *:80>
  ServerName localhost

  [ ... ]

  # this rule redirect HTTP requests to HTTPS
  Redirect "/" "https://localhost/"
</VirtualHost>

<VirtualHost *:443>
  ServerName localhost

  [ ... ]

  RewriteEngine on

  # port redirection rules (for session_server.py, simulation_server.py and webots)
  # websockets (should come first)
  RewriteCond %{HTTP:Upgrade} websocket [NC]
  RewriteCond %{HTTP:Connection} upgrade [NC]
  RewriteRule ^/(\d*)/(.*)$ "ws://%{SERVER_NAME}:$1/$2" [P,L]
  # http traffic (should come after websocket)
  RewriteRule ^/(\d*)/(.*)$ "http://%{SERVER_NAME}:$1/$2" [P,L]

</VirtualHost>
```
%tab-end

%end

**Note:** Port rewrite can be disabled for testing purposes in the `session_server.py` and `simulation_server.py` by simply setting the `portRewrite` configuration option to `false`.

#### Server Display

On Linux, a working Linux display (":0") should be available to run Webots remotely.
This can be achieved generally simply by executing 'xhost +' on the server computer.

**Note:** On some computers, in addition to the previous comment, the display entity is linked with the fact that a monitor is plugged.
In this case, you can open automatically a user session when the computer is switched on, run `session_server.py` and `simulation_server.py` automatically when the session starts up, and let the monitor switched on at Ubuntu startup.
If you have a headless system, i.e., a system without any physical monitors attached, then with the NVIDIA graphics card you could fake a monitor in the X session.
This solution basically consists in adding a screen configuration to the X server configuration file by copying the Extended Display Identification Data (EDID) of a temporary attached monitor.

#### Startup Procedure

The startup procedure is the following:

1. Start all the simulation servers (`simulation_server.py`).
2. Start the session server (`session_server.py`).

This procedure should be automated in a startup script, so that the servers are restarted after a reboot of the machine.

This folder also contains a `server.sh` utility script to automatically start and stop `session_server.py` and `simulation_server.py` with a given configuration file.

Please make sure that the `WEBOTS_HOME` variable is set before running the simulation and session server scripts.

#### How to Embed a Web Scene in Your Website

Similarly to [this section](web-streaming.md#how-to-embed-a-web-scene-in-your-website), to embed the simulation it is enough to instantiate a `webots-view` web component from the [WebotsView.js] package.

This is the API of the `webots-streaming` web component:
* `connect(servers, mode, broadcast, mobileDevice, callback, disconnectCallback) `: function instantiating the simulation web interface and taking as argument:
  * `server`: The URL of the server. Different URL formats are supported:
      * URL to a session server: "https://beta.webots.cloud/ajax/server/session.php?url=https://github.com/cyberbotics/webots/projects/languages/python/worlds/example.wbt"
      * WebSocket URL (i.e. "ws://localhost:80"): this format is used for web broadcast streaming.
      * URL to a X3D file (i.e. "file.x3d"): this format is used for showing a [web scene](web-scene.md) or a [web animation](web-animation.md).
  * `mode`: `x3d` or `mjpeg`.
  * `broadcast`: boolean variable enabling or not the broadcast.
  * `isMobileDevice`: boolean variable specifying if the application is running on a mobile device.
  * `callback`: function to be executed once the simulation is ready.
  * `disconnectCallback`: function to be executed once the web scene is closed.
* `close()`: close the simulation web scene. Note that if the `webots-view` element is removed from the HTML page or `loadScene`, `connect` or `loadAnimation` is called, `close` will be automatically called.
* `hasView()`: return true if a view exist, false otherwise.
* `hideToolbar()`: hide the toolbar. Must be called after connect.
* `ondisconnect()`: a function that can be overridden. It will be called when the simulation disconnects.
* `onready()`: a function that can be overridden. It will be called once the simulation is loaded.
* `resize()`: automatically resize the web-component.
* `showToolbar()`: show the toolbar. Must be called after connect. The toolbar is displayed by default.
* `sendMessage(message)`: send a message to the streaming server through the web socket. Examples of messages could be:
    * `real-time:-1`: to play the simulation.
    * `pause`: to pause the simulation.
    * `robot:{"name":"supervisor","message":"reset"}`: to send a message to the controller of a robot named "supervisor".
* `setWebotsMessageCallback(callback)`: define a function that will be called every time a message is sent by Webots.
    * `callback`: the function to be called when a message is received, the text of the message is passed to this function as the only argument.
* `setWebotsErrorMessageCallback(callback)`: define a function that will be called every time an error is send by Webots.
    * `callback`: the function to be called when an error is received, the text of the error is passed to this function as the only argument.

Moreover, the following attributes are available:
* `data-server`: URL of the server.
* `data-mode`: `x3d` or `mjpeg`.
* `data-broadcast`: boolean variable enabling or not the broadcast.
* `data-isMobileDevice`: boolean variable specifying if the application is running on a mobile device.
* `showIde`: specify if the IDE button must be displayed on the toolbar. Must be called before connect. The IDE button is hidden by default.
* `showPlay`: specify if the play button must be displayed on the toolbar. Must be called before connect. The quit button is displayed by default.
* `showReload `: specify if the reload button must be displayed on the toolbar. Must be called before connect. The reload button is hidden by default.
* `showReset`: specify if the reset button must be displayed on the toolbar. Must be called before connect. The reset button is displayed by default.
* `showRobotWindow`: specify if the robot window button must be displayed on the toolbar. Must be called before connect. The robot window button is displayed by default.
* `showWorldSelection`: specify if the world selection button must be displayed on the toolbar. Must be called before connect. The world selection is displayed by default.
* `showStep`: specify if the step button must be displayed on the toolbar. Must be called before connect. The step button is displayed by default.
* `showQuit`: specify if the quit button must be displayed on the toolbar. Must be called before connect. The quit button is displayed by default.

The attributes of `webots-view` are only evaluated once: when the page is loaded. If the `data-server` attribute is set, the `webots-view` web-component will automatically connect to the `server`.

Warning: note that if the `data-scene` attribute (see [web animation](web-animation.md)) and the `data-server` are both set, the `data-scene` will take precedence and try to load a scene.

An example of a file using this API is available [here](https://cyberbotics1.epfl.ch/open-roberta/setup_viewer.js) and is used to run [this sample](https://cyberbotics1.epfl.ch/open-roberta/).

### Scene Refresh Rate

The scene refresh rate is defined by the `WorldInfo.FPS` field.
The same fields as in the [web animation](web-animation.md#limitations) are updated.

### Technologies and Limitations

The streaming server has the same limitations as the [Web animation](web-animation.md#remarks-on-the-used-technologies-and-their-limitations) and the [Web streaming](web-streaming.md#limitations).
The data is sent to the clients using [WebSockets](https://www.websocket.org/).
The WebSockets should therefore be enabled in your Web browser (this is the default setting).
