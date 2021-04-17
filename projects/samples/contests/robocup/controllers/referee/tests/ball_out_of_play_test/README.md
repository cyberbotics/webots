On this test scenario, we have some robots initially placed outside of the field in valid positions. The ball is going to leave the field from different lines and sides and the autoRef should take appropriate actions, also one goal is going to be scored for each team and the game is going to be a draw:

The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`, then `SET`, and then `PLAY`.
3. - Simulation is paused.
   - The robot `Red 1` is manually moved to: `1 0 0.24`.
   - A force of 15 Newton is applied to the ball along X-axis, simulation is resumed.
   - The ball bounces on `Red 1`.
   - Simulation is paused, ball is moved to `1 3.08 0.08`.
   - A force of 5 Newton is applied on the ball along Y-Axis, simulation is resumed.
   - Ball leaves the field.
   - Ball is automatically placed back to 1 3 0.08. ([#14](https://github.com/RoboCup-Humanoid-TC/webots/issues/14))

The following information should be contained in logs (among others):

```
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_1 RobocupRobot on port 10001 at halfTimeStartingPose: translation -3.5 -3.06 0.24, rotation 0 0 1 1.57
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_1 RobocupRobot on port 10021 at halfTimeStartingPose: translation 3.5 -3.06 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_INITIAL
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
[SSSS.xxx|SSSS.xxx] Info: Ball touched by red player 1.
[SSSS.xxx|SSSS.xxx] Info: Ball left the field at 0 3.08 0.08 after being touched by red player 1.
[SSSS.xxx|SSSS.xxx] Info: Ball respawned at 1 3 0.08
```
