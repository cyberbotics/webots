// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbKinematicDifferentialWheels.hpp"

#include "WbCylinder.hpp"
#include "WbDifferentialWheels.hpp"
#include "WbHingeJoint.hpp"
#include "WbMotor.hpp"
#include "WbRobot.hpp"

WbKinematicDifferentialWheels::WbKinematicDifferentialWheels(WbRobot *robot, double wheelsRadius, double axleLength,
                                                             WbHingeJoint *leftJoint, WbHingeJoint *rightJoint,
                                                             bool isWbDifferentialWheels) :
  mWheelsRadius(wheelsRadius),
  mAxleLength(axleLength),
  mRobot(robot),
  mIsWbDifferentialWheels(isWbDifferentialWheels) {
  mWheelJoints[0] = leftJoint;
  mWheelJoints[1] = rightJoint;
  mKinematicDisplacementNumber = 0;
}

void WbKinematicDifferentialWheels::applyKinematicMotion(double ms) {
  mKinematicDisplacementNumber = 0;
  mKinematicDisplacement.setXy(0, 0);
  mRobot->savePreviousTransform();

  if (mIsWbDifferentialWheels)  // WbDifferentialWheels defines it's own motion model (because of encoders and no actual joint)
    return;

  double leftVelocity = mWheelJoints[0]->motor()->currentVelocity();
  double rightVelocity = mWheelJoints[1]->motor()->currentVelocity();

  double t = ms * 0.001;
  // rotation angle
  const double delta_direction = (rightVelocity - leftVelocity) * t * mWheelsRadius / (mAxleLength * 2.0);
  const double a = mRobot->rotation().angle() + delta_direction;            // [rad/s]
  const double v0 = -0.5 * (leftVelocity + rightVelocity) * mWheelsRadius;  // [m/s]

  const double v = v0 * t;
  mRobot->setTranslation(mRobot->translation() + WbVector3(v * sin(a), 0.0, v * cos(a)));
  if (delta_direction != 0.0)
    mRobot->setRotationAngle(mRobot->rotation().angle() + 2.0 * delta_direction);

  mRobot->updateOdeGeomPosition();
  mRobot->printKinematicWarningIfNeeded();
}

void WbKinematicDifferentialWheels::applyKinematicDisplacement() {
  // update position
  WbVector3 position = mRobot->translation();
  mRobot->setTranslation(position[0] - mKinematicDisplacement[0] / mKinematicDisplacementNumber, position[1],
                         position[2] - mKinematicDisplacement[1] / mKinematicDisplacementNumber);
  mRobot->updateOdeGeomPosition();
}

WbCylinder *WbKinematicDifferentialWheels::getRecursivelyBigestCylinder(WbBaseNode *node) {
  WbCylinder *wheelCylinder = NULL;
  if (node) {
    WbCylinder *cylinder = dynamic_cast<WbCylinder *>(node);
    if (cylinder)
      return cylinder;
    WbGroup *group = dynamic_cast<WbGroup *>(node);
    if (group) {
      for (int i = 0; i < group->childCount(); ++i) {
        cylinder = getRecursivelyBigestCylinder(group->child(i));
        if (cylinder) {
          if (wheelCylinder && cylinder->radius() > wheelCylinder->radius())
            wheelCylinder = cylinder;
          else if (!wheelCylinder)
            wheelCylinder = cylinder;
        }
      }
    }
    WbShape *shape = dynamic_cast<WbShape *>(node);
    if (shape) {
      cylinder = dynamic_cast<WbCylinder *>(shape->geometry());
      if (cylinder)
        return cylinder;
    }
  }
  return wheelCylinder;
}

WbKinematicDifferentialWheels *WbKinematicDifferentialWheels::createKinematicDifferentialWheelsIfNeeded(WbRobot *robot) {
  if (robot->isDynamic())
    return NULL;
  // check if this is a WbDifferentialWheels
  WbDifferentialWheels *differentialWheels = dynamic_cast<WbDifferentialWheels *>(robot);
  if (differentialWheels)
    return new WbKinematicDifferentialWheels(robot, differentialWheels->wheelRadius(), differentialWheels->axleLength(), NULL,
                                             NULL, true);
  // check if the required joints and motors exist
  WbHingeJoint *leftJoint = NULL;
  WbHingeJoint *rightJoint = NULL;
  const QVector<WbBasicJoint *> joints = robot->jointChildren();
  QVector<WbHingeJoint *> motorizedJoints;
  for (int i = 0; i < joints.size(); ++i) {
    WbHingeJoint *joint = dynamic_cast<WbHingeJoint *>(joints.at(i));
    if (!joint || !joint->motor() || !joint->solidEndPoint())
      continue;
    motorizedJoints << joint;
  }
  if (motorizedJoints.size() < 2)
    return NULL;
  // check all the possible pairs of joints
  for (int i = 0; i < motorizedJoints.size(); ++i) {
    leftJoint = motorizedJoints.at(i);
    WbCylinder *leftWheelCylinder = getRecursivelyBigestCylinder(leftJoint->solidEndPoint()->boundingObject());
    // make sure this joint has a cylinder bounding object
    if (!leftWheelCylinder || leftWheelCylinder->radius() <= 0.0)
      continue;
    double leftWheelRadius = leftWheelCylinder->radius();
    double leftWheelDistance = (robot->position() - leftWheelCylinder->upperTransform()->position()).length();
    for (int j = i + 1; j < motorizedJoints.size(); ++j) {
      rightJoint = motorizedJoints.at(j);
      // make sure this joint has a cylinder bounding object
      WbCylinder *rightWheelCylinder = getRecursivelyBigestCylinder(rightJoint->solidEndPoint()->boundingObject());
      if (!rightWheelCylinder || rightWheelCylinder->radius() <= 0.0)
        continue;
      // make sure both cylinders have the same size
      double rightWheelRadius = rightWheelCylinder->radius();
      if (leftWheelRadius != rightWheelRadius)
        continue;
      double rightWheelDistance = (robot->position() - rightWheelCylinder->upperTransform()->position()).length();
      // make sure the wheels are equaly centered
      if (leftWheelDistance != rightWheelDistance)
        continue;
      // make sure the joint axes are parallel
      if (!leftJoint->axis().cross(rightJoint->axis()).isNull())
        continue;
      // make sure the axis between the 2 anchors is parallel to the joints axes
      WbVector3 anchorAxis = rightJoint->anchor() - leftJoint->anchor();
      if (!leftJoint->axis().cross(anchorAxis).isNull())
        continue;
      WbVector3 globalLeftAnchor =
        robot->rotationMatrix().transposed() * (robot->position() - leftWheelCylinder->upperTransform()->position());
      WbVector3 globalRightAnchor =
        robot->rotationMatrix().transposed() * (robot->position() - rightWheelCylinder->upperTransform()->position());
      if (globalLeftAnchor.x() > globalRightAnchor.x())  // make sure the joint are not inversed
        return new WbKinematicDifferentialWheels(robot, leftWheelRadius, leftWheelDistance + rightWheelDistance, leftJoint,
                                                 rightJoint);
      else
        return new WbKinematicDifferentialWheels(robot, leftWheelRadius, leftWheelDistance + rightWheelDistance, rightJoint,
                                                 leftJoint);
    }
  }
  return NULL;
}
