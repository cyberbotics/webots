On this test scenario, the behavior of the AutoRef facing an own goal is going to be tested:

The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`, then `SET`, and then `PLAY`.
3. After changing the state to `PLAY`:
   - Simulation is paused. Then, robot `Red 1` is manually moved to near its own goal, exactly to:\
   `-4 0.00 0.24` (if the robot is on the left side)\
   `4 0.00 0.24` (if the robot is on the right side)
   - Then, the ball is manually moved to near red's own goal, in front of `Red 1`, exactly to:\
   `-4.25 0.00 0.08` (if team red is on the left side)\
   `4.25 0.00 0.08` (if team red is on the right side)
   - A force of 5 Newton is applied to the ball along positive or negative X-axis according to robot's side (from the ball towards the robot), simulation is resumed.
   - The ball bounces on `Red 1`.
4. - Simulation is paused.
   - Then, the ball is manually moved to near red's own goal, in front of `Red 1`, exactly to:\
   `-4.25 0.00 0.08` (if team red is on the left side)\
   `4.25 0.00 0.08` (if team red is on the right side)
   - A force of 5 Newton is applied to the ball along positive or negative X-axis according to robot's side (from the ball towards the goal), simulation is resumed.
   - Ball entirely crosses the goal line and enters the goal.
   - Then, a goal is scored for team blue. ([#9](https://github.com/RoboCup-Humanoid-TC/webots/issues/9))
5. The game state switches to `READY`.
6. After 45 seconds, the game state switches to `SET`.
7. The ball is spawned at the center mark with team blue having kick-off.
8. Time elapses and game state changes to `PLAY`.

The following information should be contained in logs (among others):

```
[SSSS.xxx|SSSS.xxx] Info: Ball touched by red player 1.
[SSSS.xxx|SSSS.xxx] Info: Ball left the field at (-4.5712084619910724 0.01775100835668539 0.07844848079007057) after being touched by red player 1.
[SSSS.xxx|SSSS.xxx] Info: Sending 11232:SCORE:2 to GameController.
[SSSS.xxx|SSSS.xxx] Info: Score in red goal by red player 1
[SSSS.xxx|SSSS.xxx] Info: Kickoff is red
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY.
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET.
[SSSS.xxx|SSSS.xxx] Info: Ball respawned at 0 0 0.08.
[SSSS.xxx|SSSS.xxx] Info: Sending 18344:STATE:PLAY to GameController.
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING.
```
