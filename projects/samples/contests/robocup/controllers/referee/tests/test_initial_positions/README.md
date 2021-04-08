On this test scenario, we have multiple valid and invalid positions for the
robots of both team:

- Team 1 (RED): 2 robots with invalid position
  - 1: entirely on the field
  - 2: partially on the border line
- Team 2 (BLUE): 2 robots have an invalid position
  - 2: partially on the back line
  - 4: on the wrong side

The following should happen:

1. All robots are spawned on their appropriate side, according to the team
   configuration files (except `Blue 4` who is on the wrong side).
2. During 2 minutes (real-time not simulated time), game state is `INITIAL` all
   robots stay static.
3. AutoRef sends a message to the GameController, status is changed to `READY`
4. Penalties are being called for illegaly positioned robots who are moved to
   appropriate locations.
5. Simulation is paused, `Red 1`, `Red 3` and `Blue 1` are manually moved inside
   the field, simulation is resumed.
6. Removal penalty is applied to `Red 1` (illegal entry).
7. Time elapses and game state changes to `SET`.
8. Robots `Red 4` and `Blue 3` are removed from the field by the auto-referee.
9. Time elapses and game state changes to `PLAY`.
10. Simulation is paused, `Red 1`, `Blue 2` and `Blue 3` are manually moved
    inside the field, simulation is resumed.
11. No robot is penalized.

The following information should be contained in logs (among others):

```
[00:00:00:000][HH:MM:SS:xxx] Red 1 spawn at [-4.0, -2.06, 0.24]
[00:00:00:000][HH:MM:SS:xxx] Red 2 spawn at [-4.0, 3.02, 0.24]
[00:00:00:000][HH:MM:SS:xxx] Red 3 spawn at [-0.75, -3.06, 0.24]
[00:00:00:000][HH:MM:SS:xxx] Red 4 spawn at [-0.75, 3.06, 0.24]
[00:00:00:000][HH:MM:SS:xxx] Blue 1 spawn at [-4.7, -2.06, 0.24]
[00:00:00:000][HH:MM:SS:xxx] Blue 2 spawn at [-4.55, 2.06, 0.24]
[00:00:00:000][HH:MM:SS:xxx] Blue 3 spawn at [-0.75, -3.06, 0.24]
[00:00:00:000][HH:MM:SS:xxx] Blue 4 spawn at [1.75, 3.06, 0.24]
[HH:MM:SS:xxx][HH:MM:SS:xxx] Setting state to READY
[HH:MM:SS:xxx][HH:MM:SS:xxx] Penalty PICKUP for Red 1: illegal position
[HH:MM:SS:xxx][HH:MM:SS:xxx] Red 1 moved to [x,x,x]
[HH:MM:SS:xxx][HH:MM:SS:xxx] Penalty PICKUP for Red 2: illegal position
[HH:MM:SS:xxx][HH:MM:SS:xxx] Red 2 moved to [x,x,x]
[HH:MM:SS:xxx][HH:MM:SS:xxx] Penalty PICKUP for Blue 2: illegal position
[HH:MM:SS:xxx][HH:MM:SS:xxx] Blue 2 moved to [x,x,x]
[HH:MM:SS:xxx][HH:MM:SS:xxx] Penalty PICKUP for Blue 4: illegal position
[HH:MM:SS:xxx][HH:MM:SS:xxx] Blue 4 moved to [x,x,x]
[HH:MM:SS:xxx][HH:MM:SS:xxx] Penalty PICKUP for Red 1: illegal entry
[HH:MM:SS:xxx][HH:MM:SS:xxx] Red 1 moved to [x,x,x]
[HH:MM:SS:xxx][HH:MM:SS:xxx] Setting state to SET
[HH:MM:SS:xxx][HH:MM:SS:xxx] Penalty PICKUP for Red 4: illegal position
[HH:MM:SS:xxx][HH:MM:SS:xxx] Red 4 moved to [x,x,x]
[HH:MM:SS:xxx][HH:MM:SS:xxx] Penalty PICKUP for Blue 3: illegal position
[HH:MM:SS:xxx][HH:MM:SS:xxx] Blue 3 moved to [x,x,x]
[HH:MM:SS:xxx][HH:MM:SS:xxx] Setting state to PLAY
```
