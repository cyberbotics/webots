#include <webots/Accelerometer.hpp>
#include <webots/Robot.hpp>

#include "../../../lib/TestUtils.hpp"

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  TestUtils ts(robot, argv[0]);

  int deviceTag1 = robot->getDeviceTagFromIndex(0);
  int deviceTag2 = robot->getDeviceTagFromIndex(1);
  Accelerometer *device1 = dynamic_cast<Accelerometer *>(robot->getDeviceFromTag(deviceTag1));
  Accelerometer *device2 = dynamic_cast<Accelerometer *>(robot->getDeviceFromTag(deviceTag2));

  ts.assertPointerNotNull(device1, "Accelerometer 1 not found.");
  ts.assertPointerNotNull(device2, "Accelerometer 2 not found.");
  ts.assertIntNotEqual(device1->getTag(), device2->getTag(), "Accelerometer 1 and 2 are the same device.");

  ts.sendSuccess();
}
