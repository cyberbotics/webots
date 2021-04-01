# Robocup Virtual Humanoid League 2021

This folder contains the simulation setup for the Robocup Virtual Humanoid League 2021.

## Requirements

In order to run this simulation, you will need to have a [fairly powerful](https://cyberbotics.com/doc/guide/system-requirements) Linux, Windows or macOS computer.
You will also need to get familiar with Webots by reading the [Webots User Guide](https://cyberbotics.com/doc/guide/) and following the [Tutorials](https://cyberbotics.com/doc/guide/tutorials).

## Installation

1. [Build Webots from the source](https://github.com/cyberbotics/webots/wiki) from the [feature-robocup-controllers](https://github.com/cyberbotics/webots/tree/feature-robocup-controllers) branch (recommended) or install the latest nightly build of [Webots R2021b](https://github.com/cyberbotics/webots/releases) and checkout the content of this folder locally.
2. Install the latest version of the [GameController](https://github.com/sheepsy90/GameController).
3. Define the `GAME_CONTROLLER_HOME` environment variable to point to the `GameController` folder: `export GAME_CONTROLLER_HOME=/my/folder/GameController`

## Run the Demo

1. Open the [robocup.wbt](worlds/robocup.wbt) world file in Webots and run it until you see the GameController window showing up.
2. You can manually move the robots and the ball using the mouse (<kbd>Shift</kbd>-right-click-and-drag).
3. Launch the sample robot controller [client.cpp](controllers/player/client.cpp) by typing `./client` in the [controllers/player](controllers/player) folder.
4. The sample client program will simply move the neck of one of the Darwin-OP robot.

## Modify the Game and Teams Configuration

1. Quit Webots.
2. Edit the [game.json](controllers/referee/game.json) file to change the game configuration.
3. Edit the [team_1.json](controllers/referee/team_1.json) and [team_2.json](controllers/referee/team_2.json) files to change the teams configuration.
4. Restart the simulation.

## Program your Own Robot Controllers

1. Update the [game.json](controllers/referee/game.json) configuration file and create your own team configuration files, taking inspiration from [team_1.json](controllers/referee/team_1.json) and [team_2.json](controllers/referee/team_2.json).
2. Create your own robot controllers, taking inspiration from the sample [client.cpp](controllers/player/client.cpp).

## Create your Own Robot Model

Create your robot model in the [protos](protos) folder taking inspiration from [RobocupRobot.proto](protos/RobocupRobot.proto) and adjust your team configuration file accordingly.
Your proto should have the following exposed parameters:

```js
PROTO MyOwnRobocupRobot [
  field SFVec3f                 translation    0 0 0   # Is `Transform.translation`.
  field SFRotation              rotation       0 0 1 0 # Is `Transform.rotation`.
  field SFString                name           ""      # Is `Solid.name`.
  field MFString                controllerArgs []      # Is `Robot.controllerArgs`.
]
{
  Robot {
    translation IS translation
    rotation IS rotation
    name IS name
    controllerArgs IS controllerArgs
    ⋮
  }
}
```

You should parse the `name` field and ensure your robot adapts to it to display its team color and player number:

```lua
  if fields.name.value ~= '' then
    -- name is supposed to be something like "red player 2" or "blue player 1"
    local words = {}
    for word in fields.name.value:gmatch("%w+") do table.insert(words, word) end
    local color = words[1]
    local number = words[3]
```

Then, the `color` and `number` variables should be used by your PROTO file to display the requested color and player number.
This can be achieved by forging a texture name from these variables or using them directly to assign material colors, create shapes, etc.
More information about Webots PROTO is available from the [Webots Reference Manual](https://cyberbotics.com/doc/reference/proto).
