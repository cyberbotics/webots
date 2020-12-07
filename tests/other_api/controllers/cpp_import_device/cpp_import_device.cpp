#include <webots/DistanceSensor.hpp>
#include <webots/Field.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#include "../../../lib/TestUtils.hpp"

using namespace webots;

int main(int argc, char **argv) {
  Supervisor *robot = new Supervisor();
  TestUtils ts(robot, argv[0]);

  int timeStep = (int)robot->getBasicTimeStep();
  Field *childrenField = robot->getSelf()->getField("children");

  // Import the first PositionSensor
  childrenField->importMFNodeFromString(0, "DistanceSensor { name \"ds1\" }");
  DistanceSensor *ds1 = robot->getDistanceSensor("ds1");
  ts.assertPointerNotNull(ds1, "DistanceSensor 2 not found after import.");
  ds1->enable(timeStep);

  // Import the second PositionSensor
  childrenField->importMFNodeFromString(1, "DistanceSensor { name \"ds2\" }");
  DistanceSensor *ds2 = robot->getDistanceSensor("ds2");
  ts.assertPointerNotNull(ds2, "DistanceSensor 2 not found after import.");
  ds2->enable(timeStep);

  ts.sendSuccess();
}
