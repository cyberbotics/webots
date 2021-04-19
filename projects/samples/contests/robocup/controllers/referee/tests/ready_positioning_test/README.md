On this test scenario, we have some robots initially placed outside of the field in valid positions and are going to be in illegal positions after finishing ready state:

The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`.
3. - Simulation is paused,
   - The robot number 2 from the team that does not have the kick-off is manually moved to the center circle in it's own half.\
   The exact position for test:\
   `-0.40 0.00 0.24` (if the robot is on the left side)\
   `0.40 0.00 0.24` (if the robot is on the right side)
   - The robot number 2 from the team having the kick-off is also manually moved to the center circle in it's own half.\
   The exact position for test:\
   `-0.40 0.00 0.24` (if the robot is on the left side)\
   `0.40 0.00 0.24` (if the robot is on the right side)
   - The robot `Red 1` is moved to opponent's field.\
   The exact position for test:\
   `2 0.00 0.24` (if the robot is on the left side)\
   `-2 0.00 0.24` (if the robot is on the right side)
   - The robot `Blue 1` is moved to a valid position in its own field.\
   The exact position for test:\
   `-0.15 -2 0.24` (if the robot is on the left side)\
   `0.15 -2 0.24` (if the robot is on the right side)
   - Simulation is resumed.
4. Time elapses and game state changes to `SET`.
5. The robot that is in the opponent's field is removed by the auto-referee. ([#25](https://github.com/RoboCup-Humanoid-TC/webots/issues/25))
6. The robot from the team that does not have the kick-off and is in the center circle in it's own half is also removed. ([#27](https://github.com/RoboCup-Humanoid-TC/webots/issues/27))
7. The robot from the team that has the kick-off and is in the center circle in it's own half is not removed.
8. Robots `Red 3`, `Red 4`, and `Blue 3` are removed by the auto-referee as they haven't moved before the end of `READY` phase. ([#31](https://github.com/RoboCup-Humanoid-TC/webots/issues/31))
9. Then the simulation is paused before the game state is changed to `PLAY`. The robot `Blue 1` is manually fallen into the opponent's
   half of the field, simulation is resumed.
10. Again before `PLAY` state, the robot `Blue 1` who is touching opponent's field is removed the auto-referee. ([#26](https://github.com/RoboCup-Humanoid-TC/webots/issues/26))

The following information should be contained in logs (among others):

```
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_1 RobocupRobot on port 10001 at halfTimeStartingPose: translation 3.5 -3.06 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_2 RobocupRobot on port 10002 at halfTimeStartingPose: translation 3.5 3.06 0.24, rotation 0 0 1 4.711592653589793
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_3 RobocupRobot on port 10003 at halfTimeStartingPose: translation 0.75 -3.06 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_4 RobocupRobot on port 10004 at halfTimeStartingPose: translation 0.75 3.06 0.24, rotation 0 0 1 4.711592653589793
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_1 RobocupRobot on port 10021 at halfTimeStartingPose: translation -3.5 -3.06 0.24, rotation 0 0 1 1.57
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_2 RobocupRobot on port 10022 at halfTimeStartingPose: translation -3.5 3.06 0.24, rotation 0 0 1 -1.57
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_3 RobocupRobot on port 10023 at halfTimeStartingPose: translation -0.75 -3.06 0.24, rotation 0 0 1 1.57
[SSSS.xxx|SSSS.xxx] Info: Sending STATE:READY to GameController
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for red player 1: outside team side at kickoff. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for `x` player 2: inside center circle during oppenent's kickoff. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for red player 3: did not move to kickoff position. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for red player 4: did not move to kickoff position. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for blue player 3: did not move to kickoff position. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for blue player 1: outside team side at kickoff. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
```
