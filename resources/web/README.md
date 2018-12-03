# Webots web interface

This folder contains the server, templates and javascript libraries used to provide a web interface to Webots.

## Prerequisites

- Ubuntu last LTS or Windows (Mac OS X also supported, but less tested)
- Python 2.7
- tornado (http://www.tornadoweb.org/en/stable/, `pip install tornado`)
- websocket-client (https://pypi.python.org/pypi/websocket-client/, `pip install websocket-client`)
- Webots R2019a is installed (one PRO license per instance or one downloaded PRO license)


## How to run

Go to the `server` folder and run `./simulation_server.py`. The server options are located in `setup.json`.

## Ports

Some ports have to be open from outside:

- The port used by the `simulation_server.py` server (default: 80)
- The ports used by Webots and Webots controllers (default: from 2000 and up)

## Configuration

Depending on the configuration defined in the `setup.json` file, e.g., the `web`, `session` and `simulation` boolean values, the `simulation_server.py` will provide different services:

1. a web server hosting the web content of the different projects as well as an index page for projects.
2. a session server (GET) indicating to the client an available simulation server.
3. a simulation server (WebSocket) creating instances of Webots on the same computer and connecting them to the client.

In a development environment, it is good to enable all three services and have everything run on the same server machine.
In a production environment, the web server service is not used, an instance of a session server is running on a machine and
several instances of simulation servers are running in parallel on different machines (one per machine).

When a simulation server gets a new connection, it copies the project directory into a temporary `instance` directory, and
starts a Webots instance. Then, it manages the high level communications between the web client and the Webots instance and
also it handles the `websocket disconnected` or `webots failure` cases.

## Display

On Linux, a working Linux display (":0") should be available to run Webots remotely.
This can be achieved generally simply by executing 'xhost +' on the server computer.

**Note :** On some computers, in addition to the previous comment,
the display entity is linked with the fact that a monitor is plugged.
In this case,  you can open automatically a user session when the computer is switched on,
run "simulation_server.py" automatically when the session starts up,
and let the monitor switched on at Ubuntu startup.


## Project folders

### projects/web/blockly

Contains the client web page for the blockly app (learning robot visual programming)

- `index.html` deals with the GUI layout, and the communication with the websocket to `simulation_server.py`
- `blockly.html` deals with the Google Blockly panel, and the websocket to `robot_server.py`
- `worlds` contains the simulations to run
- `worlds.json` contains the worlds meta information including:
  - `name`: Used to change the world
  - `instructions`: Used to display the specific instructions to a world
  - `max_blocks`: Used to bound the number of blockly blocks
  - `allowed_blocks`: Used to restrict the blockly blocks types
  - `solution`: Used to store a possible solution
 `controllers/robot_server` runs a websocket server (tornado, max 1 client), and move the robot according to the websocket orders.

### projects/web/nao_race

Contains the simpler nao_race python programming benchmark.
Only one of the three controllers is actually communicating with the web page: supervisor_server.py.
It is used to download and upload controller files and to reload the world.

### project/web/simple

Contains a simple web application with a simulation world without any controller.
