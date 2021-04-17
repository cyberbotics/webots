On this test scenario, the ball is going to leave the field from the opponent's goal line and the autoRef should take appropriate actions:

The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`, then `SET`, and then `PLAY`.
3. - Simulation is paused.
   - The robot `Red 1` is manually moved to: `1 0 0.24`.
   - A force of 15 Newton is applied to the ball along X-axis, simulation is resumed.
   - The ball bounces on `Red 1`.
   - Simulation is paused, ball is moved to near the goal line on the side of team blue, exactly to:\
   `4.45 -2 0.08` (if team red is on the left side)\
   `-4.45 -2 0.08` (if team red is on the right side)
   - A force of 5 Newton is applied on the ball along the positive or negative X-Axis relying on the side of team red (towards outside the field), simulation is resumed.
   - Ball leaves the field.
   - The ball is replaced on the touchline at the intersection with the centerline on the side the ball left the field. ([#17](https://github.com/RoboCup-Humanoid-TC/webots/issues/17))

The following information should be contained in logs (among others):

```
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
[SSSS.xxx|SSSS.xxx] Info: Ball touched by red player 1.
[SSSS.xxx|SSSS.xxx] Info: Ball left the field at (-4.58 -2 0.08) after being touched by red player 1.
[SSSS.xxx|SSSS.xxx] Info: Ball respawned at 0 -3 0.08.
```
