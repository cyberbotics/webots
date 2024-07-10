#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

#include <string.h>
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif

#include <webots/Emitter.hpp>
#include <webots/Robot.hpp>

using namespace webots;

const std::string TEST_OUTPUT_FILENAME = "../../../output.txt";

class TestUtils {
public:
  TestUtils(Robot *robot, const std::string &testName) : mRobot(robot), mEmitter(NULL) {
    assert(robot);
    // remove the full path
    std::size_t index = testName.find_last_of("/\\");
    mTestName = testName.substr(index + 1);
    // keep the basename (remove the .exe extension)
    index = mTestName.find_last_of(".");
    mTestName = mTestName.substr(0, index);
    notifyControllerStatus(true);
  }

  void sendSuccess() {
    // write the result to the file
    std::ofstream file;
    file.open(TEST_OUTPUT_FILENAME, std::ofstream::out | std::ofstream::app);
    file << "OK: " << mTestName << std::endl;
    file.close();

    // write the result to the console
    std::cout << "OK: " << mTestName << std::endl;
    notifyControllerStatus(false);

    delete mRobot;
    exit(EXIT_SUCCESS);
  }

  void assertIntNotEqual(int value, int not_expected, const std::string &errorMessage) {
    if (value == not_expected)
      sendErrorAndExit(errorMessage);
  }

  void assertPointerNotNull(const void *ptr, const std::string &errorMessage) {
    if (ptr == NULL)
      sendErrorAndExit(errorMessage);
  }

  void assertStringEquals(std::string &value, std::string &expected, const std::string &errorMessage) {
    if (value != expected)
      sendErrorAndExit(errorMessage);
  }

private:
  std::string mTestName;
  Robot *mRobot;
  Emitter *mEmitter;

  void notifyControllerStatus(bool running) {
    if (!mEmitter)
      mEmitter = mRobot->getEmitter("ts_emitter");

    char msg[256];
#ifdef _WIN32
    sprintf(msg, "ts %d %ld", running, GetCurrentProcessId());
#else
    sprintf(msg, "ts %d %d", running, getpid());
#endif
    mEmitter->send(msg, strlen(msg));
  }

  void sendErrorAndExit(const std::string &errorMessage) {
    // write the result to the file
    std::ofstream file;
    file.open(TEST_OUTPUT_FILENAME, std::ofstream::out | std::ofstream::app);
    file << "FAILURE with " << mTestName << ": " << errorMessage << std::endl;
    file.close();

    // write the result to the console
    std::cout << "FAILURE with " << mTestName << ": " << errorMessage << std::endl;

    // cleanup
    notifyControllerStatus(false);
    delete mRobot;
    exit(EXIT_FAILURE);
  }
};

#endif
