# Simulation and session servers

## Overview

In order to run Webots in the cloud, you need to run at least one session server and one or more simulation servers.
The simulation servers should run on different machines while the session server may run on a machine where a simulation server
is running. Both servers are Python scripts: `simulation_server.py` and `session_server.py` located in this folder.

## Protocol

When a web client needs to know whether it may start a simulation, it will open a WebSocket connection to the session server to
monitor the availability of simulation servers. The session server will answer '1' if some simulation server is available and
'0' if none is available. Whenever this situation changes, the session server will notify the web clients. Note: the session
server will never send twice the same value, it sends only changes in the availability of simulation servers.

When a web client wants to start a simulation, it will send an AJAX request to the session server. The session server will then
send the WebSocket URL of an available simulation server or an error if none are available. The web client will then contact
directly the simulation server to startup the simulation. The simulation server will start Webots with the world file requested
by the client. Then, it will send the WebSocket URL of Webots, so that the client can communicate directly with Webots. If the
WebSocket connection to the simulation server is closed or broken, the simulation server will close Webots. If Webots quits,
the simulation server will notify the web client and close the connection with it.

The session server regularly queries the simulation servers to monitor their load. If a simulation server becomes overloaded,
then, the session server will consider it is not available any more and will not offer it to web clients.

## Prerequisites

- Ubuntu last LTS or Windows (Mac OS X also supported, but less tested)
- Python 3
- Dependencies ([Windows instructions](https://github.com/omichel/webots/wiki/Windows-Optional-Dependencies#webots-web-service), [Linux instructions](https://github.com/omichel/webots/wiki/Linux-Optional-Dependencies#webots-web-service)):
  - tornado (http://www.tornadoweb.org/en/stable/, `pip install tornado`)
  - websocket-client (https://pypi.python.org/pypi/websocket-client/, `pip install websocket-client`)
  - nvidia-ml-py3 (https://pypi.python.org/pypi/nvidia-ml-py3/, `pip install nvidia-ml-py3`)
  - psutil (https://pypi.python.org/pypi/psutil/, `pip install psutil`)
  - requests (https://pypi.python.org/pypi/requests/, `pip install requests`)
  - optional: firejail (https://firejail.wordpress.com/, `apt install firejail`)
- Webots R2019b is installed

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

This folder also contains a `server.sh` utility script to automatically start and stop `session_server.py` and `simulation_server.py` with a given configuration file.

## Ports

Some ports have to be open from outside:

- The port used by the `session_server.py` server (default: 1999)
- The port used by the `simulation_server.py` server (default: 2000)
- The ports used by Webots and Webots controllers (default: from 2001 and up)

## Display

On Linux, a working Linux display (":0") should be available to run Webots remotely.
This can be achieved generally simply by executing 'xhost +' on the server computer.

**Note :** On some computers, in addition to the previous comment, the display entity is linked with the fact that a monitor is plugged.
In this case, you can open automatically a user session when the computer is switched on,
run `session_server.py` and `simulation_server.py` automatically when the session starts up,
and let the monitor switched on at Ubuntu startup.
