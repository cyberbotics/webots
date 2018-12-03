# Simulation and session servers

## Overview

In order to run Webots in the cloud, you need to run at least one session server and one or more simulation servers.
The simulation servers should run on different machines while the session server may run on a machine where a simulation server
is running. Both servers are Python scripts: `simulation_server.py` and `session_server.py` located in this folder.

The simulation servers should be already running when the session server is launched, so that the session server can query them.

## Protocol

When a web client needs to know whether it may start a simulation, it will open a Websocket connection to the session server to
monitor the availability of simulation servers. The session server will answer '1' if some simulation server is available and
'0' if none is available. Whenever this situation changes, the session server will notify the web clients. Note: the session
server will never send twice the same value, it sends only changes in the availability of simulation servers.

When a web client wants to start a simulation, it will send an AJAX request to the session server. The session server will then
send the Websocket URL of an available simulation server or an error if none are available. The web client will then contact
directly the simulation server to startup the simulation. The simulation server will start Webots with the world file requested
by the client. Then, it will send the Websocket URL of Webots, so that the client can communicate directly with Webots. If the
Websocket connection to the simulation server is closed or broken, the simulation server will close Webots. If Webots quits,
the simulation server will notify the web client and close the connection with it.

The session server regularly queries the simulation servers to monitor their load. If a simulation server becomes overloaded,
then, the session server will consider it is not available any more and will not offer it to web clients.

## Setup

The `config` folder contains two subfolders: `session` and `simulation`:

- the `session` subfolder contains configuration files for the session server. The `default.json` configuration file is used
by default and runs a session server on localhost which assumes there is a single simulation server on the localhost as well. It
is useful for testing locally the system (using XAMPP).
- the `simulation` subfolder contains configuration files for the simulation server. The `default.json` configuration file is
used by default and runs a simulation server on localhost. It is useful for testing locally the system (using XAMPP).

Similarly to the `config` folder, the `log` folder contains two subfolders: `session` and `simulation`, which contain log files
produced respectively by the `session_server.py` and `simulation_server.py` scripts.
Note: the session server log is not yet implemented (and therefore the corresponding subfolder is not yet present).

The `ssl` folder may contain the SSL key and SSL certificate needed for running the servers in SSL mode (HTTPS/WSS).
The same key and certificate can be used by both the `session_server.py` and the `simulation_server.py`.
They are used on the Cyberbotics servers.

## Startup procedure

The startup procedure is the following:

1. Start all the simulation servers (`simulation_server.py`).
2. Start the session server (`session_server.py`).

This procedure should be automated in a startup script, so that the servers are restarted after a reboot of the machine.
