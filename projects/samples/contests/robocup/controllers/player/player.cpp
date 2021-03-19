#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include "messages.pb.h"

// ports numbers

// red 1: 10001
// red 2: 10002
// red 3: 10003
// red 4: 10004
// blue 1: 10021
// blue 2: 10022
// blue 3: 10023
// blue 4: 10024

int main(int argc, char *argv[]) {
  const std::string player_names[] = {
    "red player 1",  "red player 2",  "red player 3",  "red player 4",
    "blue player 1", "blue player 2", "blue player 3", "blue player 4",
  };
  const int ports[] = {10001, 10002, 10003, 10004, 10021, 100022, 100023, 100024};
  webots::Robot *robot = new webots::Robot();
  const double timeStep = robot->getBasicTimeStep();
  const std::string name = robot->getName();
  int player_id = -1;
  int port = -1;
  for (unsigned int i = 0; i < sizeof(player_names); i++)
    if (name.compare(player_names[i]) == 0) {
      player_id = i;
      port = ports[i];
      break;
    }
  std::cout << "player ID = " << player_id << " running on port " << port << std::endl;
  while (robot->step(timeStep) != -1) {
    // read sensors outputs
    // process behavior
    // write actuators inputs
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}
