#include <webots/Brake.hpp>
#include <webots/Robot.hpp>

#include "../../../lib/TestUtils.hpp"

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  TestUtils ts(robot, argv[0]);

  int deviceTag1 = robot->getDeviceTagFromIndex(0);
  int deviceTag2 = robot->getDeviceTagFromIndex(1);
  Brake *device1 = dynamic_cast<Brake *>(robot->getDeviceFromTag(deviceTag1));
  Brake *device2 = dynamic_cast<Brake *>(robot->getDeviceFromTag(deviceTag2));

  ts.assertPointerNotNull(device1, "Brake 1 not found.");
  ts.assertPointerNotNull(device2, "Brake 2 not found.");
  ts.assertIntNotEqual(device1->getTag(), device2->getTag(), "Brakes 1 and 2 are the same device.");

  ts.sendSuccess();
}
