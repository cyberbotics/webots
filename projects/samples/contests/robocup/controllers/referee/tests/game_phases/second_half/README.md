# Second half

## What is tested

On this test scenario, taking appropriate actions between two half times is going to be tested

## Setup

- Team RED has kickoff

## Description

1. 10 minutes of game time in the first half elapses. The AutoRef calls the first half time to be over.
2. The robots respawn on the starting positions at the side of the field.
3. 15 seconds (real-time) elapse.
4. The game state switches to READY.
5. Red 2 and Blue 2 are placed in the opponent's field, so the AutoRef should penalize them as the READY state finishes.
6. After 45 seconds game state switches to SET.
7.  After 5 seconds, the game state switches to PLAY.