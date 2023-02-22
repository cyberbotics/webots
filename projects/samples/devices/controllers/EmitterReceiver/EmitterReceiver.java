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

//
// Description:   A controller which sends or receives data while avoiding the obstacles.
//

import com.cyberbotics.webots.controller.*;

public class EmitterReceiver extends Robot {

  protected final int TIME_STEP = 64;
  protected DistanceSensor ir0, ir1;
  protected Emitter emitter;
  protected Receiver receiver;
  protected Motor leftMotor, rightMotor;
  private String lastMessage = "None";

  // Avoid printing continuously the same message
  protected void printConsole(String message) {
    if (! message.equals(lastMessage)) {
      System.out.println(message);
      lastMessage = message;
    }
  }

  public EmitterReceiver() {

    // As we are using the same controller for the emitter and the
    // receiver, we need to distinguish them.
    if (getName().startsWith("MyBot emitter")) {
      emitter = getEmitter("emitter");
      // Change channel if needed
      int channel = emitter.getChannel();
      if (channel != 1)
        emitter.setChannel(1);
    } else if (getName().startsWith("MyBot receiver")) {
      receiver = getReceiver("receiver");
      receiver.enable(TIME_STEP);
    } else {
      System.err.println("Unrecognized robot name '" + getName() + "'. Exiting...");
      return;
    }

    // Get and enable distance sensors
    ir0 = getDistanceSensor("ds0");
    ir1 = getDistanceSensor("ds1");
    ir0.enable(TIME_STEP);
    ir1.enable(TIME_STEP);

    // get the motors and set target position to infinity (speed control)
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
  }

  public void run() {
    // Forever
    for (;;) {
      if (emitter != null) {
        // The emitter robot simply sends the string
        final String out_message = "Hello !";
        emitter.send(out_message.getBytes());
      }

      if (receiver != null) {
        // The receiver robot
        // Is there a packet in the receiver's queue ?
        if (receiver.getQueueLength() > 0) {

          // Read current packet's data
          String in_message = new String(receiver.getData());
          printConsole("Communicating: received \"" + in_message + "\"\n");

          // fetch next packet
          receiver.nextPacket();
        }
        else
          printConsole("Communication broken !");
      }

      // read distance sensors
      double ir0Value = ir0.getValue();
      double ir1Value = ir1.getValue();

      double leftSpeed, rightSpeed, speed = 60;
      if (ir1Value > 500) {

        // If both distance sensors are detecting something, this
        // means that we are facing a wall. In this case we need to
        // move backwards.
        if (ir0Value > 200) {
          if (emitter != null) {
            leftSpeed = -speed;
            rightSpeed = -speed / 2;
          }
          else {
            leftSpeed = -speed / 2;
            rightSpeed = -speed;
          }
        }
        else {
          // We turn proportionnaly to the sensors value because
          // the closer we are from the wall, the more we need to turn
          leftSpeed = -ir1Value / 10;
          rightSpeed = (ir0Value / 10) + 5;
        }
      }
      else if (ir0Value > 500) {
        leftSpeed = (ir1Value / 10) + 5;
        rightSpeed = -ir0Value / 10;
      }
      else {
        // If nothing was detected we can move forward at maximal speed.
        leftSpeed = speed;
        rightSpeed = speed;
      }

      // Set the motor speeds
      leftMotor.setVelocity(0.1 * leftSpeed);
      rightMotor.setVelocity(0.1 * rightSpeed);

      step(TIME_STEP);   // Run simulation for some milliseconds
    }
  }

  public static void main(String[] args) {
    EmitterReceiver robot = new EmitterReceiver();
    robot.run();
  }
}
