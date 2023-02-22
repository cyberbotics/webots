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

#include <webots/utils/Motion.hpp>

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace webots;
using namespace std;

vector<Motion *> Motion::cMotions;

// --- helper functions ---

// trim functions
static string &ltrim(string &s) {
  s.erase(s.begin(), find_if(s.begin(), s.end(), not1(ptr_fun<int, int>(isspace))));
  return s;
}

static string &rtrim(string &s) {
  s.erase(find_if(s.rbegin(), s.rend(), not1(ptr_fun<int, int>(isspace))).base(), s.end());
  return s;
}

static string &trim(string &s) {
  return ltrim(rtrim(s));
}

// split functions
static vector<string> &split(const string &s, char delim, vector<string> &elems) {
  stringstream ss(s);
  string item;
  while (getline(ss, item, delim))
    elems.push_back(item);
  return elems;
}

static vector<string> split(const string &s, char delim) {
  vector<string> elems;
  split(s, delim, elems);
  return elems;
}

// --- end of helper functions ---

struct MotorCommand {
  bool defined;
  double value;
};

struct Pose {
  string name;
  int time;  // milliseconds
  vector<MotorCommand *> commands;
};

Motion::Motion(const string &fileName) :
  mValid(false),
  mDuration(0),
  mReverse(false),
  mLoop(false),
  mPlaying(false),
  mElapsed(0),
  mPreviousTime(0) {
  cMotions.push_back(this);

  ifstream ifs;
  ifs.open(fileName.c_str(), ifstream::in);

  if (ifs) {
    string line;
    int lineCounter = 0;
    bool header = true;

    while (getline(ifs, line)) {
      lineCounter++;
      trim(line);
      vector<string> tokens = split(line, ',');
      int tokenCount = tokens.size();
      int tokenId = 0;
      if (tokenCount < 2) {
        cerr << fileName << ": unexpected token number at line " << lineCounter << endl;
        return;
      }
      if (header) {
        if (tokens[0].compare("#WEBOTS_MOTION") != 0) {
          cerr << fileName << ": invalid header (expected = \"#WEBOTS_MOTION\", received = \"" << tokens[0] << "\")" << endl;
          return;
        }
        if (tokens[1].compare("V1.0") != 0) {
          cerr << fileName << ": invalid header version (expected = \"V1.0\", received = \"" << tokens[1] << "\")" << endl;
          return;
        }
        for (tokenId = 2; tokenId < tokenCount; tokenId++) {
          string token = tokens[tokenId];
          mMotors.push_back(Robot::getInstance()->getMotor(token));
        }
        header = false;

        // except to be valid as soon as the header is correctly read
        mValid = true;
      } else {
        if (tokens.size() - 2 != mMotors.size()) {
          cerr << fileName << ": invlaid token number at line " << lineCounter << endl;
          continue;
        }
        Pose *pose = new Pose;
        pose->time = timeFromString(tokens[0]);
        pose->name = tokens[1];
        mDuration = pose->time;
        for (tokenId = 2; tokenId < tokenCount; tokenId++) {
          string token = tokens[tokenId];
          MotorCommand *command = new MotorCommand;
          if (token.compare("*") == 0)
            command->defined = false;
          else {
            command->defined = true;
            command->value = atof(token.c_str());
          }
          pose->commands.push_back(command);
        }
        mPoses.push_back(pose);
      }
    }
  }

  ifs.close();
}

Motion::~Motion() {
  clearInternalStructure();

  vector<Motion *>::iterator motionIt;
  for (motionIt = cMotions.begin(); motionIt != cMotions.end(); ++motionIt) {
    Motion *motion = *motionIt;
    if (motion == this) {
      cMotions.erase(motionIt);
      break;
    }
  }
}

bool Motion::isOver() const {
  if (mLoop)
    return false;
  else
    return (mReverse && mElapsed <= 0) || (!mReverse && mElapsed >= mDuration);
}

void Motion::setTime(int time) {
  if (time < 0)
    mElapsed = 0;
  else if (time > mDuration)
    mElapsed = mDuration;
  else
    mElapsed = time;
}

void Motion::playMotions() {
  vector<Motion *>::iterator motionIt;
  for (motionIt = cMotions.begin(); motionIt != cMotions.end(); ++motionIt) {
    Motion *motion = *motionIt;
    motion->playStep();
  }
}

void Motion::playStep() {
  // actuate
  for (unsigned int i = 0; i < mMotors.size(); i++) {
    Motor *motor = mMotors[i];

    double beforeValue = 0.0;
    int beforeTime = -1;
    double afterValue = 0.0;
    int afterTime = -1;

    // find beforeValue & beforeTime
    vector<Pose *>::iterator poseIt;
    for (poseIt = mPoses.begin(); poseIt != mPoses.end(); ++poseIt) {
      Pose *pose = *poseIt;
      if (pose->time <= mElapsed) {
        MotorCommand *command = pose->commands[i];
        if (command->defined) {
          beforeTime = pose->time;
          beforeValue = command->value;
        }
      } else
        break;
    }

    // find afterValue & afterTime
    vector<Pose *>::reverse_iterator poseRIt;
    for (poseRIt = mPoses.rbegin(); poseRIt != mPoses.rend(); ++poseRIt) {
      Pose *pose = *poseRIt;
      if (pose->time >= mElapsed) {
        MotorCommand *command = pose->commands[i];
        if (command->defined) {
          afterTime = pose->time;
          afterValue = command->value;
        }
      } else
        break;
    }

    // compute position
    bool setPos = false;
    double pos = 0.0;
    if (afterTime != -1 && mElapsed > mDuration) {
      setPos = true;
      pos = afterValue;
    } else if (beforeTime != -1 && afterTime != -1) {
      setPos = true;
      if (afterTime == beforeTime)
        pos = beforeValue;
      else
        pos = beforeValue + (mElapsed - beforeTime) * (afterValue - beforeValue) / (afterTime - beforeTime);
    } else if (beforeTime != -1) {
      setPos = true;
      pos = beforeValue;
    }

    // apply position
    if (setPos)
      motor->setPosition(pos);
  }

  // update internal variables
  Robot *robot = Robot::getInstance();
  int time = robot->getTime() * 1000.0;
  int delta = time - mPreviousTime;
  mPreviousTime = time;

  if (mReverse) {
    if (mElapsed <= 0) {
      if (mLoop)
        mElapsed = mDuration;
      else {
        mElapsed = 0;
        mPlaying = false;
      }
    } else
      mElapsed -= delta;
  } else {
    if (mElapsed >= mDuration) {
      if (mLoop)
        mElapsed = 0;
      else {
        mElapsed = mDuration;
        mPlaying = false;
      }
    } else
      mElapsed += delta;
  }
}

void Motion::clearInternalStructure() {
  vector<Pose *>::iterator poseIt;
  for (poseIt = mPoses.begin(); poseIt != mPoses.end(); ++poseIt) {
    Pose *pose = *poseIt;
    vector<MotorCommand *>::iterator commandIt;
    for (commandIt = pose->commands.begin(); commandIt != pose->commands.end(); ++commandIt) {
      MotorCommand *command = *commandIt;
      delete command;
    }
    pose->commands.clear();
    delete pose;
  }
  mPoses.clear();
}

void Motion::play() {
  Robot *robot = Robot::getInstance();
  mPreviousTime = robot->getTime() * 1000.0;

  mPlaying = true;

  // if we reached either end: restart from other end
  if (mReverse && mElapsed <= 0)
    mElapsed = mDuration;
  else if (!mReverse && mElapsed >= mDuration)
    mElapsed = 0;
}

int Motion::timeFromString(const string &time) {
  vector<string> tokens = split(time, ':');
  if (tokens.size() != 3) {
    cerr << "Syntax error in time definition: \"" << time << "\"" << endl;
    return 0;
  }

  int minutes = atoi(tokens[0].c_str());
  int seconds = atoi(tokens[1].c_str());
  int milliseconds = atoi(tokens[2].c_str());

  return minutes * 60000 + seconds * 1000 + milliseconds;
}
