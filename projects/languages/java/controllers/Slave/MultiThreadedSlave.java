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
 * Description:  Multi-threaded example. Same behavior as MultiThreadedSlave.java
 */

import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;

import java.lang.Thread;
import java.util.concurrent.CyclicBarrier;

public class MultiThreadedSlave extends Robot {

  // Enable and listen the receiver and change the robot mode
  public static class ReceiverListenerRunnable implements Runnable {

    private MultiThreadedSlave robot;
    private Receiver receiver;
    private CyclicBarrier barrier;

    public ReceiverListenerRunnable(CyclicBarrier barrier, MultiThreadedSlave robot) {
      this.barrier = barrier;
      this.robot = robot;
      receiver = robot.getReceiver("receiver");
      receiver.enable(robot.timeStep);
    }

    public void run() {

      try {

        while (true) {
          if (receiver.getQueueLength() > 0) {
            String message = new String(receiver.getData());
            receiver.nextPacket();
            System.out.println("I should "+message+"!");
            if (message.equals("avoid obstacles"))
              robot.robotMode = RobotMode.AVOID_OBSTACLES; // change directly the robot's mode
            else if (message.equals("move forward"))
              robot.robotMode = RobotMode.MOVE_FORWARD;
            else if (message.equals("stop"))
              robot.robotMode = RobotMode.STOP;
            else if (message.equals("turn"))
              robot.robotMode = RobotMode.TURN;
          }

          barrier.await();
          // step is performed
          barrier.await();
        }
      } catch (Exception e) {
        // leave the loop
      }
    }
  }

  // Move the robot according to its mode
  public static class RobotControlRunnable implements Runnable {

    private MultiThreadedSlave robot;
    private CyclicBarrier barrier;
    private final double maxSpeed = 10.0;
    private DistanceSensor[] distanceSensors;
    private Motor leftMotor, rightMotor;

    public RobotControlRunnable(CyclicBarrier barrier, MultiThreadedSlave robot) {
      this.barrier = barrier;
      this.robot = robot;
      distanceSensors = new DistanceSensor[] {robot.getDistanceSensor("ds0"), robot.getDistanceSensor("ds1")};
      for (int i=0; i<2; i++) {
        distanceSensors[i].enable(robot.timeStep);
      }
      leftMotor = robot.getMotor("left wheel motor");
      rightMotor = robot.getMotor("right wheel motor");
      leftMotor.setPosition(Double.POSITIVE_INFINITY);
      rightMotor.setPosition(Double.POSITIVE_INFINITY);
      leftMotor.setVelocity(0.0);
      rightMotor.setVelocity(0.0);
    }

    private double boundSpeed(double speed) {
      return Math.max(-maxSpeed, Math.min(maxSpeed, speed));
    }

    public void run() {
      try {

        while (true) {

          double delta = distanceSensors[0].getValue()-distanceSensors[1].getValue();
          double[] speeds = {0.0, 0.0};

          // send actuators commands according to the robotMode
          switch (robot.robotMode){
            case AVOID_OBSTACLES:
              speeds[0] = boundSpeed(maxSpeed/2+delta);
              speeds[1] = boundSpeed(maxSpeed/2-delta);
              break;
            case MOVE_FORWARD:
              speeds[0] = maxSpeed;
              speeds[1] = maxSpeed;
              break;
            case TURN:
              speeds[0] =  maxSpeed/2;
              speeds[1] = -maxSpeed/2;
              break;
            default:
              break;
          }
          leftMotor.setVelocity(speeds[0]);
          rightMotor.setVelocity(speeds[1]);

          barrier.await();
          // step is performed
          barrier.await();
        }
      } catch (Exception e) {
        // leave the loop
      }
    }
  }

  // possible states of the robot
  private enum RobotMode {
    STOP, MOVE_FORWARD, AVOID_OBSTACLES, TURN
  }

  private RobotMode robotMode = RobotMode.AVOID_OBSTACLES;
  public final int timeStep = 32;

  public static void main(String[] args) {
    MultiThreadedSlave robot = new MultiThreadedSlave();

    try {
      CyclicBarrier barrier = new CyclicBarrier(3);

      Thread receiverListenerThread = new Thread(new ReceiverListenerRunnable(barrier, robot));
      Thread robotControlThread = new Thread(new RobotControlRunnable(barrier, robot));

      robot.step(robot.timeStep);

      receiverListenerThread.start();
      robotControlThread.start();

      while (true) {
        barrier.await();

        int ret = robot.step(robot.timeStep); // perform the physics step
        if (ret == -1) {
          break;
        }

        barrier.await();
      }

      receiverListenerThread.interrupt();
      robotControlThread.interrupt();

    } catch (Exception e) {
      System.err.println("Exception: " + e.getMessage());
    }
  }
}
