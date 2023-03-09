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
 * Description:  The controller for the Lego Mindstorm robot which follows a
 *               line on the ground and avoids obstacles.
 */

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.TouchSensor;

public class Rover extends Robot {
  TouchSensor leftBumper, rightBumper;
  Motor leftMotor, rightMotor;
  DistanceSensor groundSensor;
  int avoidance_counter;

  public static void main(String args[]) {
    Rover r=new Rover();
    r.run();
  }

  public void run() {
    leftBumper   = getTouchSensor("S1");
    groundSensor = getDistanceSensor("S2");
    rightBumper  = getTouchSensor("S3");
    leftMotor    = getMotor("left wheel motor");
    rightMotor   = getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    avoidance_counter = 0;
    double leftSpeed, rightSpeed;

    leftBumper.enable(64);
    rightBumper.enable(64);
    groundSensor.enable(64);
    while(step(64)!=-1) {
      /*
       * If we are not avoiding an obstacle we check first the bumpers
       * to know if we have bumped into one and then the groundSensor to
       * know in which direction we should move.
       */
      if (avoidance_counter == 0) {
        if (leftBumper.getValue() > 0 ||
            rightBumper.getValue() > 0) {
          leftSpeed = -0.6;
          rightSpeed = -1;
          avoidance_counter = 1000;
        } else if (groundSensor.getValue() > 43) {
          leftSpeed = 1;
          rightSpeed = 0.2;
        } else {
          leftSpeed = 0.2;
          rightSpeed = 1;
        }
      } else {
        /*
         * If we are avoiding, the movement is seperated into three
         * different parts. First we move back from the obstacle,
         * slightly turning, then we move straight forward in order to
         * avoid the obstacle. Finall we keep moving forward wut
         * slightly turning in order to find the line.
         */
        if (avoidance_counter > 800) {
          leftSpeed = -0.6;
          rightSpeed = -1;
        } else if (avoidance_counter > 600) {
          leftSpeed = 1;
          rightSpeed = 1;
        } else if (avoidance_counter > 70)  {
          leftSpeed = 0.7;
          rightSpeed = 1;
          if (groundSensor.getValue() > 43)
            avoidance_counter = 1;
        } else {
          leftSpeed = 1;
          rightSpeed = 1;
          if (groundSensor.getValue() > 43)
            avoidance_counter = 1;
        }
        avoidance_counter--;
      }
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
      step(64);
    }
  }
}
