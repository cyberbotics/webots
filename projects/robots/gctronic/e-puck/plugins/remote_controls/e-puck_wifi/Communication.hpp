#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <webots/types.h>
#include <string>

class EPuckCommandPacket;
class EPuckInputPacket;

class Communication {
public:
  Communication();
  virtual ~Communication();

  bool initialize(const std::string &port);
  void cleanup();
  bool isInitialized() const { return mFd > 0; }
  bool send(const char *, int size);
  int receive(char *, int size, bool block);

private:
  int mFd;
};

#endif
