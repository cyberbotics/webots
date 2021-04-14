On this test scenario, we have some robots initially placed outside of the field in valid positions. The ball is going to leave the field from different lines and sides and the autoRef should take appropriate actions, also one goal is going to be scored for each team and the game is going to be a draw:

The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`.
3. - Simulation is paused,
   - All the robots are moved into their own field. Here is some exact locations as a test (rotations are not altered):
   - `Red 1`:  `-3.5 -2 0.24` (if the robot is on the left side)\
   `Red 1`:  `3.5 -2 0.24` (if the robot is on the right side)

   - `Blue 1`:  `-3.5 -2 0.24` (if the robot is on the left side)\
   `Blue 1`:  `3.5 -2 0.24` (if the robot is on the right side)
   - Then, the simulation is resumed.
4. Time elapses and game state changes to `SET`.
5. Time elapses and game state changes to `PLAY`.
6. The ball is manually touched by the `Red 1`, and then it entirely crosses the touchline, from `0 3.08 0.08`.
7. It is re-spawned on the field line.([#14](https://github.com/RoboCup-Humanoid-TC/webots/issues/14))
8. The ball again is is manually touched by the `Red 1`, and then it is about to leave the field, but it doesn't entirely leave the field. It is at `0 3.06 0.08`.
9. The game continues without any interference by the AutoRef. ([#16](https://github.com/RoboCup-Humanoid-TC/webots/issues/16))
10. The ball is manually touched by the `Red 1`, and then it entirely crosses the goal line on the side of team blue, from:\
   `4.58 -2 0.08` (if the robot `Red 1` is on the left side)\
   `-4.58 -2 0.08` (if the robot `Red 1` is on the right side)
11. the ball is replaced on the centerline at the intersection with the centerline on the side the ball left the field. ([#17](https://github.com/RoboCup-Humanoid-TC/webots/issues/17))
12. The ball is manually touched by the `Red 1`, and then it entirely crosses the goal line on the side of team red, from:\
   `-4.58 -2 0.08` (if the robot `Red 1` is on the left side)\
   `4.58 -2 0.08` (if the robot `Red 1` is on the right side)
13. The ball is replaced on the corner of the touch line and goal line on side the ball left the field. ([#19](https://github.com/RoboCup-Humanoid-TC/webots/issues/19))
14. One goal is manually scored for each team by the UI of the game controller.
15. Time elapses until the end of the game. AutoRef declares the game to be a draw after the end of the game and game state switches to `FINISHED`. ([#13](https://github.com/RoboCup-Humanoid-TC/webots/issues/13))

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
[SSSS.xxx|SSSS.xxx] Info: Ball respawned at 0 3 0.08
[SSSS.xxx|SSSS.xxx] Info: Ball touched again by same player.
[SSSS.xxx|SSSS.xxx] Info: Ball left the field at 4.58 -2 0.08 after being touched by red player 1.
[SSSS.xxx|SSSS.xxx] Info: Ball respawned at 0 -2 0.08
[SSSS.xxx|SSSS.xxx] Info: Ball touched again by same player.
[SSSS.xxx|SSSS.xxx] Info: Ball left the field at -4.58 -2 0.08 after being touched by red player 1.
[SSSS.xxx|SSSS.xxx] Info: Ball respawned at -4.5 -3 0.08
[SSSS.xxx|SSSS.xxx] Info: Sending STATE:FINISH to GameController
[SSSS.xxx|SSSS.xxx] Info: End of match
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_FINISHED
[SSSS.xxx|SSSS.xxx] Info: End of the game.
[SSSS.xxx|SSSS.xxx] Info: The score is 1-1.
[SSSS.xxx|SSSS.xxx] Info: This is a draw.
```
