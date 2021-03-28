#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <RobotisOp2VisionManager.hpp>

#define NMOTORS 20

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

webots::Motor *mMotors[NMOTORS];
webots::PositionSensor *mPositionSensors[NMOTORS];
webots::Accelerometer *mAccelerometer;
webots::Gyro *mGyro;

int main(int argc, char **argv) {
  webots::Robot *robot = new webots::Robot();
  int timeStep = (int)robot->getBasicTimeStep();
  
  mAccelerometer = robot->getAccelerometer("Accelerometer");
  mAccelerometer->enable(timeStep);
  mGyro = robot->getGyro("Gyro");
  mGyro->enable(timeStep);
  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = robot->getMotor(motorNames[i]);
    std::string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = robot->getPositionSensor(sensorName);
    mPositionSensors[i]->enable(timeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  managers::RobotisOp2MotionManager *mMotionManager = new managers::RobotisOp2MotionManager(robot);
  managers::RobotisOp2GaitManager *mGaitManager = new managers::RobotisOp2GaitManager(robot, "config.ini");

  robot->step(timeStep);

  mMotionManager->playPage(1);
  mGaitManager->start();
  mGaitManager->step(timeStep);
  while (robot->step(timeStep) != -1) {
    mGaitManager->setXAmplitude(0.5);
    // mGaitManager->setAAmplitude(neckPosition);
    mGaitManager->step(timeStep);
  };

  delete robot;
  return 0;
}
