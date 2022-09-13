## Protocol and Quick Start

### Protocol

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

### Quick Start
This section gives a simple step-by-step guide on how to start a streaming server with one session and one simulation server.
We assume you use Ubuntu 20.04 or newer and have the latest version of Webots installed.

First, you need to install the web service dependencies:
```bash
sudo apt-get install python3-pip python-is-python3
pip install pynvml requests psutil tornado distro
```

Then, clone the [webots-server](https://github.com/cyberbotics/webots-server) repository and enter it:
```bash
git clone git@github.com:cyberbotics/webots-server.git
cd webots-server
```

After that open "[config/simulation/local.json](https://github.com/cyberbotics/webots-server/blob/main/config/simulation/local.json)" and if needed, modify the following line with the path to your Webots installation:
```
"webotsHome": "/usr/local/webots"
```

Finally start a session server and a simulation server:
```bash
./server.sh start local
```
This will start the session server with the [config/session/local.json](https://github.com/cyberbotics/webots-server/blob/main/config/session/local.json) configuration file and the simulation server with the [config/simulation/local.json](https://github.com/cyberbotics/webots-server/blob/main/config/simulation/local.json) configuration file.

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
http://localhost:1999/session?url=https://github.com/cyberbotics/webots-cloud-simulation-examples/blob/main/6_binary/worlds/ned.wbt
```
Click the `Connect` button to initiate the streaming.
Webots will clone the `ned.wbt` simulation from GitHub and start it.

If you want to stop both the session and simulation servers, run the following:
```
cd ~/webots-server
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
