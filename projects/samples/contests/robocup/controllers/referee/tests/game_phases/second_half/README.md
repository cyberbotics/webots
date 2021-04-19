On this test scenario, taking appropriate actions between two half times is going to be tested:


The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`, then `SET`, and then `PLAY`.
3. 10 minutes of game time in the first half elapses. The AutoRef calls the first half time to be over.
4. The robots respawn on the starting positions at the side of the field.
5. The sides of play and the kick-off have flipped in the GameController.
6. 15 seconds (real-time) elapse.
7. The game state switches to READY.
8. After 45 seconds game state switches to SET.
9. The ball is placed on the center mark.
10. After 5 seconds, the game state switches to PLAY. ([#6](https://github.com/RoboCup-Humanoid-TC/webots/issues/6))


The following information should be contained in logs (among others):

```
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING.
[SSSS.xxx|SSSS.xxx] Info: Ball in play.
[SSSS.xxx|SSSS.xxx] Info: Sending 83360:STATE:FINISH to GameController.
[SSSS.xxx|SSSS.xxx] Info: End of first half.
[SSSS.xxx|SSSS.xxx] Info: red player 1 reset to halfTimeStartingPose: translation (-3.5 -3.06 0.24), rotation (0 0 1 1.57).
[SSSS.xxx|SSSS.xxx] Info: red player 2 reset to halfTimeStartingPose: translation (-3.5 3.06 0.24), rotation (0 0 1 -1.5700000000000003).
[SSSS.xxx|SSSS.xxx] Info: red player 3 reset to halfTimeStartingPose: translation (-0.75 -3.06 0.24), rotation (0 0 1 1.57).
[SSSS.xxx|SSSS.xxx] Info: red player 4 reset to halfTimeStartingPose: translation (-0.75 3.06 0.24), rotation (0 0 1 -1.5700000000000003).
[SSSS.xxx|SSSS.xxx] Info: blue player 1 reset to halfTimeStartingPose: translation (3.5 -3.06 0.24), rotation (0 0 1 1.571592653589793).
[SSSS.xxx|SSSS.xxx] Info: blue player 2 reset to halfTimeStartingPose: translation (3.5 3.06 0.24), rotation (0 0 1 4.711592653589793).
[SSSS.xxx|SSSS.xxx] Info: blue player 3 reset to halfTimeStartingPose: translation (0.75 -3.06 0.24), rotation (0 0 1 1.571592653589793).
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_FINISHED.
[SSSS.xxx|SSSS.xxx] Info: Sending 83383:STATE:SECOND-HALF to GameController.
[SSSS.xxx|SSSS.xxx] Info: Beginning of second half.
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_INITIAL.
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY.
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET.
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING.
```
