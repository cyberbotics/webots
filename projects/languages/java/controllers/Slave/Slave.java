// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  This controller gives to its robot the following behavior:
 *               According to the messages it receives, the robot change its
 *               behavior.
 */

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.AnsiCodes;

public class Slave extends Robot {

  // possible states of the robot
  private enum Mode {
    STOP, MOVE_FORWARD, AVOID_OBSTACLES, TURN
  }

  private final int timeStep = 32;
  private final double maxSpeed = 10.0;

  private Mode mode = Mode.AVOID_OBSTACLES;
  private Camera camera;
  private Receiver receiver;
  private DistanceSensor[] distanceSensors;
  private Motor[] motors;

  private double boundSpeed(double speed) {
    return Math.max(-maxSpeed, Math.min(maxSpeed, speed));
  }

  public Slave() {
    camera = getCamera("camera");
    camera.enable(4*timeStep);
    receiver = getReceiver("receiver");
    receiver.enable(timeStep);
    motors = new Motor[] {getMotor("left wheel motor"),getMotor("right wheel motor")};
    motors[0].setPosition(Double.POSITIVE_INFINITY);
    motors[1].setPosition(Double.POSITIVE_INFINITY);
    motors[0].setVelocity(0.0);
    motors[1].setVelocity(0.0);
    distanceSensors = new DistanceSensor[] {getDistanceSensor("ds0"),getDistanceSensor("ds1")};
    for (int i=0; i<2; i++) {
      distanceSensors[i].enable(timeStep);
    }
  }

  public void run() {

    // perform a simulation steps and leave the loop when the simulation is ended
    while (step(timeStep) != -1) {
      // Read sensors, particularly the order of the supervisor
      if (receiver.getQueueLength()>0){
        String message = new String(receiver.getData());
        receiver.nextPacket();
        System.out.println("I should "+AnsiCodes.RED_FOREGROUND+message+AnsiCodes.RESET+"!");
        if (message.equals("avoid obstacles"))
          mode = Mode.AVOID_OBSTACLES;
        else if (message.equals("move forward"))
          mode = Mode.MOVE_FORWARD;
        else if (message.equals("stop"))
          mode = Mode.STOP;
        else if (message.equals("turn"))
          mode = Mode.TURN;
      }
      double delta = distanceSensors[0].getValue()-distanceSensors[1].getValue();
      double[] speeds = {0.0, 0.0};

      // send actuators commands according to the mode
      switch (mode){
        case AVOID_OBSTACLES:
        speeds[0] = boundSpeed(maxSpeed / 2.0 + 0.1 * delta);
        speeds[1] = boundSpeed(maxSpeed / 2.0 - 0.1 * delta);
          break;
        case MOVE_FORWARD:
          speeds[0] = maxSpeed;
          speeds[1] = maxSpeed;
          break;
        case TURN:
          speeds[0] =  maxSpeed / 2.0;
          speeds[1] = -maxSpeed / 2.0;
          break;
        default:
          break;
      }
      motors[0].setVelocity(speeds[0]);
      motors[1].setVelocity(speeds[1]);
    }
  }

  public static void main(String[] args) {
    Slave controller = new Slave();
    controller.run();
  }
}
