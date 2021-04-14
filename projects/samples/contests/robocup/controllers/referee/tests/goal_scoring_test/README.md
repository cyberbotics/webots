On this test scenario, we have some robots initially placed outside of the field in valid positions and are going to be in an illegal popsition after finishing ready state:

The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`.
3. - Simulation is paused,
   - All the robots are moved into their own field. Here is some exact locations as a test (rotations are not altered):
   - `Red 1`:  `-3.5 -2 0.24`\
   `Red 2`:  `-3.5 2 0.24`\
   `Red 3`:  `-0.75 -2 0.24`\
   `Red 4`:  `-0.75 2 0.24`

   - `Blue 1`:  `-3.5 -2 0.24`\
   `Blue 2`:  `-3.5 2 0.24`\
   `Blue 3`:  `-0.75 -2 0.24`\
   (Of course the positions for the team on the right side should be flipped)
   - Then, the simulation is resumed.
4. Time elapses and game state changes to `SET`.
5. Time elapses and game state changes to `PLAY`.
6. After changing the state to `PLAY`, the robot `Red 3` is manually moved to near the opponent's goal, exactly to:\
   `4 0.00 0.24` (if the robot is on the left side)\
   `-4 0.00 0.24` (if the robot is on the right side)
7. The ball is manually touched by the `Red 3` and then is moved to the opponent's goal. Then, a goal is scored for team red. ([#7](https://github.com/RoboCup-Humanoid-TC/webots/issues/7))
8. The game state switches to `READY`.
9. - Simulation is paused,
   - Robot `Red 3` is moved into its own field. exactly to:(rotation is not altered)\
   `-2 0.00 0.24` (if the robot is on the left side)\
   `2 0.00 0.24` (if the robot is on the right side)
   - The simulation is resumed.
10. After 45 seconds, the game state switches to `SET`.
11. The ball is spawned at the center mark with team blue having kick-off.
12. Time elapses and game state changes to `PLAY`.
13. After changing the state to `PLAY`, the robot `Red 3` is manually moved to near its own goal, exactly to:\
   `-4 0.00 0.24` (if the robot is on the left side)\
   `4 0.00 0.24` (if the robot is on the right side)\
   and the ball is manually touched by the `Red 3` and then is moved to its own goal. Then, a goal is scored for team blue. ([#9](https://github.com/RoboCup-Humanoid-TC/webots/issues/9))
14. Steps 8, 9, 10, 11, and 12 are repeated.
15. Time elapses untill the end of the first half. The AutoRef calls the first half time to be over. Then the sides and kick-off is flipped while the state is `INITIAL`. Then, steps 2, 3, 4, 5, and 6 are repeated. ([#6](https://github.com/RoboCup-Humanoid-TC/webots/issues/6))
16. The ball is manually touched by the `Red 3` and then is moved partially the opponent's goal, exactly to:\
   `4.56 0.00 0.24` (if the robot `Red 3` is on the left side)\
   `-4.56 0.00 0.24` (if the robot `Red 3` is on the right side)
17. No goal is scored as the ball doesn't entirely passes the goal line. ([#8](https://github.com/RoboCup-Humanoid-TC/webots/issues/8))
18. Then, a goal is manually scored for team red by the UI of the game controller.
19. Time elapses until the end of the game. AutoRef declares team red winner of the match after the end of the game and game state switches to `FINISHED`. ([#12](https://github.com/RoboCup-Humanoid-TC/webots/issues/12))

The following information should be contained in logs (among others):

```
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_1 RobocupRobot on port 10001 at halfTimeStartingPose: translation 3.5 -3.06 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_2 RobocupRobot on port 10002 at halfTimeStartingPose: translation 3.5 3.06 0.24, rotation 0 0 1 4.711592653589793
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_3 RobocupRobot on port 10003 at halfTimeStartingPose: translation 0.75 -3.06 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_4 RobocupRobot on port 10004 at halfTimeStartingPose: translation 0.75 3.06 0.24, rotation 0 0 1 4.711592653589793
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_1 RobocupRobot on port 10021 at halfTimeStartingPose: translation -3.5 -3.06 0.24, rotation 0 0 1 1.57
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_2 RobocupRobot on port 10022 at halfTimeStartingPose: translation -3.5 3.06 0.24, rotation 0 0 1 -1.57
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_3 RobocupRobot on port 10023 at halfTimeStartingPose: translation -0.75 -3.06 0.24, rotation 0 0 1 1.57
[SSSS.xxx|0000.000] Info: Left side is `x`
[SSSS.xxx|0000.000] Info: Kickoff is `x`
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_INITIAL
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
[SSSS.xxx|SSSS.xxx] Info: Ball touched by red player 3.
[SSSS.xxx|SSSS.xxx] Info: Ball touched again by same player.
[SSSS.xxx|SSSS.xxx] Info: Ball left the field at -4.58969 0.0703013 0.0771557 after being touched by red player 3.
[SSSS.xxx|SSSS.xxx] Info: Sending SCORE:1 to GameController
[SSSS.xxx|SSSS.xxx] Info: Kickoff is blue
[SSSS.xxx|SSSS.xxx] Info: Score in blue goal by red player 3
[SSSS.xxx|SSSS.xxx] Info: Ball respawned at 0 0 0.08
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
[SSSS.xxx|SSSS.xxx] Info: Ball touched by red player 3.
[SSSS.xxx|SSSS.xxx] Info: Ball left the field at 4.59087 -0.15876 0.076968 after being touched by red player 3.
[SSSS.xxx|SSSS.xxx] Info: Sending SCORE:2 to GameController
[SSSS.xxx|SSSS.xxx] Info: Kickoff is red
[SSSS.xxx|SSSS.xxx] Info: Score in red goal by red player 3
[SSSS.xxx|SSSS.xxx] Info: Ball respawned at 0 0 0.08
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
[SSSS.xxx|SSSS.xxx] Info: Sending STATE:FINISH to GameController
[SSSS.xxx|SSSS.xxx] Info: End of first half
[SSSS.xxx|SSSS.xxx] Info: red player 1 reset to halfTimeStartingPose: translation -3.5 -3.06 0.24, rotation 0 0 1 1.57
[SSSS.xxx|SSSS.xxx] Info: red player 2 reset to halfTimeStartingPose: translation -3.5 3.06 0.24, rotation 0 0 1 -1.5700000000000003
[SSSS.xxx|SSSS.xxx] Info: red player 3 reset to halfTimeStartingPose: translation -0.75 -3.06 0.24, rotation 0 0 1 1.57
[SSSS.xxx|SSSS.xxx] Info: red player 4 reset to halfTimeStartingPose: translation -0.75 3.06 0.24, rotation 0 0 1 -1.5700000000000003
[SSSS.xxx|SSSS.xxx] Info: blue player 1 reset to halfTimeStartingPose: translation 3.5 -3.06 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|SSSS.xxx] Info: blue player 2 reset to halfTimeStartingPose: translation 3.5 3.06 0.24, rotation 0 0 1 4.711592653589793
[SSSS.xxx|SSSS.xxx] Info: blue player 3 reset to halfTimeStartingPose: translation 0.75 -3.06 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_FINISHED
[SSSS.xxx|SSSS.xxx] Info: Beginning of second half.
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_INITIAL
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
[SSSS.xxx|SSSS.xxx] Info: Ball touched by red player 3.
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
[SSSS.xxx|SSSS.xxx] Info: Sending STATE:FINISH to GameController
[SSSS.xxx|SSSS.xxx] Info: End of match
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_FINISHED
[SSSS.xxx|SSSS.xxx] Info: End of the game.
[SSSS.xxx|SSSS.xxx] Info: The score is 2-1.
[SSSS.xxx|SSSS.xxx] Info: The winner is the red team.
```
