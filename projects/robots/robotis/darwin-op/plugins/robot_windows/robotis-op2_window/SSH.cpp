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

#include "SSH.hpp"
#include "ZIP.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <cassert>
#include <iostream>

#include <webots/camera.h>
#include <webots/robot.h>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QIODevice>
#include <QtCore/QProcessEnvironment>
#include <QtCore/QTextStream>

using namespace std;

SSH::SSH(QObject *parent, RobotisModel robotisModel) :
  QObject(parent),
  mError(""),
  mSSHSession(NULL),
  mSSHChannel(NULL),
  mSFTPChannel(NULL),
  mSFTPFile(NULL),
  mTerminate(false),
  mActualModel(robotisModel) {
}

SSH::~SSH() {
  closeSFTPChannel();
  closeSSHChannel();
  closeSSHSession();
}

int SSH::openSSHSession(const QString &IPAddress, const QString &username, const QString &password) {
  mTerminate = false;
  // Open session and set options
  mSSHSession = ssh_new();
  if (mSSHSession == NULL) {
    mError = ssh_get_error(mSSHSession);
    return -1;
  }
  ssh_options_set(mSSHSession, SSH_OPTIONS_HOST, IPAddress.toUtf8());
  ssh_options_set(mSSHSession, SSH_OPTIONS_USER, username.toUtf8());
  // Connect to server
  if (ssh_connect(mSSHSession) != SSH_OK) {
    mError = ssh_get_error(mSSHSession);
    ssh_free(mSSHSession);
    mSSHSession = NULL;
    return -1;
  }
  // Verify the server's identity
  if (verifyKnownHost() < 0) {
    mError = "Connection error: server identity not verified";
    ssh_disconnect(mSSHSession);
    ssh_free(mSSHSession);
    mSSHSession = NULL;
    return -1;
  }
  // Authenticate ourselves
  if (ssh_userauth_password(mSSHSession, NULL, password.toUtf8()) != SSH_AUTH_SUCCESS) {
    mError = ssh_get_error(mSSHSession);
    ssh_disconnect(mSSHSession);
    ssh_free(mSSHSession);
    mSSHSession = NULL;
    return -1;
  }
  return 1;
}

#if LIBSSH_VERSION_INT >= SSH_VERSION_INT(0, 9, 0)
int SSH::verifyKnownHost() {
  int state;

  state = ssh_session_is_known_server(mSSHSession);

  switch (state) {
    case SSH_KNOWN_HOSTS_OK:
      break;  // ok
    case SSH_KNOWN_HOSTS_OTHER:
    case SSH_KNOWN_HOSTS_CHANGED:
      cerr << "SSH-RSA key has changed." << endl;
      cerr << "For security reasons, the connection will be stopped" << endl;
      cerr << "Please remove the old SSH-RSA key from the known_hosts file ("
#ifdef _WIN32
              "C:\\Users\\<username>\\.ssh\\known_hosts"
#elif defined(__APPLE__)
              "/Users/<username>/.ssh/known_hosts"
#else  // Linux
              "/home/<username>/.ssh/known_hosts"
#endif
              ")."
           << endl;
      return -1;
    case SSH_KNOWN_HOSTS_NOT_FOUND:
    case SSH_KNOWN_HOSTS_UNKNOWN:
      if (ssh_session_update_known_hosts(mSSHSession) != SSH_OK) {
        mError = strerror(errno);
        return -1;
      }
      break;
    case SSH_KNOWN_HOSTS_ERROR:
      mError = ssh_get_error(mSSHSession);
      return -1;
  }
  return 0;
}
#else  // SSH version < 0.9.0
int SSH::verifyKnownHost() {
  int state;

  state = ssh_is_server_known(mSSHSession);

  switch (state) {
    case SSH_SERVER_KNOWN_OK:
      break;  // ok
    case SSH_SERVER_FOUND_OTHER:
    case SSH_SERVER_KNOWN_CHANGED:
      cerr << "SSH-RSA key has changed." << endl;
      cerr << "For security reasons, the connection will be stopped" << endl;
      cerr << "Please remove the old SSH-RSA key from the known_hosts file ("
#ifdef _WIN32
              "C:\\Users\\<username>\\.ssh\\known_hosts"
#elif defined(__APPLE__)
              "/Users/<username>/.ssh/known_hosts"
#else  // Linux
              "/home/<username>/.ssh/known_hosts"
#endif
              ")."
           << endl;
      return -1;
    case SSH_SERVER_FILE_NOT_FOUND:  // fallback to SSH_SERVER_NOT_KNOWN
    case SSH_SERVER_NOT_KNOWN:
      if (ssh_write_knownhost(mSSHSession) < 0) {
        mError = strerror(errno);
        return -1;
      }
      break;
    case SSH_SERVER_ERROR:
      mError = ssh_get_error(mSSHSession);
      return -1;
  }
  return 0;
}
#endif

void SSH::closeSSHSession() {
  if (mSSHSession != NULL) {
    ssh_disconnect(mSSHSession);
    ssh_free(mSSHSession);
    mSSHSession = NULL;
  }
}

int SSH::openSSHChannel() {
  assert(mSSHSession);

  mSSHChannel = ssh_channel_new(mSSHSession);
  if (mSSHChannel == NULL)
    return -1;

  if (ssh_channel_open_session(mSSHChannel) != SSH_OK) {
    ssh_channel_free(mSSHChannel);
    mSSHChannel = NULL;
    return -1;
  }
  if (ssh_channel_request_shell(mSSHChannel) != SSH_OK) {
    cerr << "ssh_channel_request_shell() failed" << endl;
    closeSSHChannel();
    return -1;
  }
  return 1;
}

void SSH::closeSSHChannel() {
  if (mSSHChannel != NULL) {
    ssh_channel_send_eof(mSSHChannel);
    ssh_channel_close(mSSHChannel);
    ssh_channel_free(mSSHChannel);
    mSSHChannel = NULL;
  }
}

void SSH::readChannel(bool display, int err) {
  char c[32];
  int i;
  int max = sizeof(c) - 1;
  while (true) {
    const int n = ssh_channel_poll_timeout(mSSHChannel, 500, err);
    if (n == 0 || n == SSH_EOF || ssh_channel_is_eof(mSSHChannel) || mTerminate)
      break;
    while (true) {
      // we need to set a timeout here (1 second) to allow the mTerminate variable
      // to be checked from time to time to terminate the robot controller
      i = ssh_channel_read_timeout(mSSHChannel, c, n > max ? max : n, err, 1000);
      if (i == 0 || mTerminate)  // nothing to read
        break;
      c[i] = '\0';
      QString s = QString::fromUtf8(c);
      if (display)
        log(s, err == 1);
      if (err)
        mStderr += s;
      else
        mStdout += s;
    }
    if (mTerminate)
      break;
  }
}

int SSH::executeSSHCommand(const QString &command, bool display, bool wait) {
  assert(mSSHSession);
  openSSHChannel();
  assert(mSSHChannel);
  // cppcheck-suppress ignoredReturnValue
  log("robot$ " + command + '\n');
  QString cmd(command);
  ssh_channel_write(mSSHChannel, cmd.toUtf8(), cmd.size());
  if (ssh_channel_send_eof(mSSHChannel) != SSH_OK) {
    cerr << "ssh_channel_send_eof() failed" << endl;
    return -1;
  }
  mStdout = "";
  mStderr = "";
  if (wait == false)
    emit done();
  // read the return value on stderr and stdout
  while (!ssh_channel_is_eof(mSSHChannel)) {
    readChannel(display, 1);  // stderr
    if (mTerminate)
      break;
    readChannel(display, 0);  // stdout
    if (mTerminate)
      break;
  }
  closeSSHChannel();
  return 1;
}

int SSH::openSFTPChannel() {
  assert(mSSHSession);
  // Open SFTP channel
  mSFTPChannel = sftp_new(mSSHSession);
  if (mSFTPChannel == NULL) {
    mError = ssh_get_error(mSSHSession);
    return -1;
  }

  // Initilize SFTP channel
  if (sftp_init(mSFTPChannel) != SSH_OK) {
    mError = ssh_get_error(mSSHSession);
    return -1;
  }
  return 1;
}

void SSH::closeSFTPChannel() {
  if (mSFTPChannel != NULL) {
    sftp_free(mSFTPChannel);
    mSFTPChannel = NULL;
  }
}

int SSH::sendFile(const QString &source, const QString &target) {
  assert(mSSHSession && mSFTPChannel);
  info("Sending file '" + source + "' to '" + target + "'.");

  int i = 0;
  int access_type = O_WRONLY | O_CREAT | O_TRUNC, nwritten;

  // Open local file
  FILE *file;
  file = fopen(source.toUtf8(), "rb");

  // Obtain file size:
  fseek(file, 0, SEEK_END);
  int length = ftell(file);
  rewind(file);

  // Allocate memory to contain the whole file:
  char *buffer = (char *)malloc(length);

  // Copy the file into the buffer
  int size = fread(buffer, 1, length, file);

  // Open remote file
  mSFTPFile = sftp_open(mSFTPChannel, target.toUtf8(), access_type, S_IRWXU);
  if (mSFTPFile == NULL) {
    mError = ssh_get_error(mSSHSession);
    fclose(file);
    free(buffer);
    return -1;
  }

  int maxPaquetSize = 200000;
  int packetNumber = (int)(size / maxPaquetSize);
  // Send packetNumber packet of 200kb
  for (i = 0; i < packetNumber; i++) {
    nwritten = sftp_write(mSFTPFile, (buffer + (i * maxPaquetSize)), maxPaquetSize);
    if (nwritten != maxPaquetSize) {
      mError = ssh_get_error(mSSHSession);
      sftp_close(mSFTPFile);
      fclose(file);
      free(buffer);
      return -1;
    }
  }
  // Send last packet smaller than 200kb
  nwritten = sftp_write(mSFTPFile, (buffer + (i * maxPaquetSize)), size - (packetNumber * maxPaquetSize));
  if (nwritten != (size - (packetNumber * maxPaquetSize))) {
    mError = ssh_get_error(mSSHSession);
    sftp_close(mSFTPFile);
    fclose(file);
    free(buffer);
    return -1;
  }

  // Close remote file
  if (sftp_close(mSFTPFile) != SSH_OK) {
    mError = ssh_get_error(mSSHSession);
    fclose(file);
    free(buffer);
    return -1;
  }

  // Close local file
  fclose(file);
  free(buffer);
  return 1;
}

int SSH::readRemoteFile(const QString &fileName, char *buffer, int buffer_size) {
  assert(mSSHSession && mSFTPChannel);
  info("Reading remote file '" + fileName + "'.");

  sftp_file file;
  file = sftp_open(mSFTPChannel, fileName.toUtf8(), O_RDONLY, 0);
  if (file == NULL) {
    info("Remote file does not exist: '" + fileName + "'.");
    return -1;  // file not present
  }
  int size = sftp_read(file, buffer, buffer_size - 1);
  sftp_close(file);
  buffer[size] = '\0';
  return size;
}

const QString SSH::error() {
  if (mSSHSession == NULL || !mError.isEmpty())
    return mError;
  else
    return ssh_get_error(mSSHSession);
}

bool SSH::isFrameworkUpToDate() {
  int index1 = 0, index2 = 0;
  QString version, versionInstalled;
  QStringList versionList, versionInstalledList;
  QFile mFrameworkVersionFile(QProcessEnvironment::systemEnvironment().value("WEBOTS_HOME") +
                              "/projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis/version.txt");

  if (mFrameworkVersionFile.exists()) {
    if (mFrameworkVersionFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QTextStream versionStream(&mFrameworkVersionFile);

      if (!versionStream.atEnd()) {
        version = versionStream.readLine();
        versionList = version.split(".");
        index1 = versionList.at(0).toInt();
        index2 = versionList.at(1).toInt();
      } else
        return false;  // PROBLEM file empty
    } else
      return false;  // PROBLEM could not open file
    mFrameworkVersionFile.close();
  } else
    return false;  // PROBLEM file does not exist

  char buffer[32];
  if (readRemoteFile("/" + mainDirectoryName() + "/version.txt", buffer, sizeof(buffer)) < 0)  // FIXME: crashes here
    return false;

  versionInstalled = QString(buffer);
  versionInstalledList = versionInstalled.split(".");
  if (versionInstalledList.size() != 2)
    return false;

  if ((index1 > versionInstalledList.at(0).toInt()) || (index2 > versionInstalledList.at(1).toInt()))
    return false;  // new version of the Framework

  return true;
}

int SSH::updateFramework(const QString &username, const QString &password) {
  info("Updating framework.");
  emit status("Updating framework.");

  const QString webotsHomePath = QProcessEnvironment::systemEnvironment().value("WEBOTS_HOME");
  const QString robotisDir = webotsHomePath + "/projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis";
  const QString installArchive =
    QDir::tempPath() + "/webots_robotis_" + QString::number((int)QCoreApplication::applicationPid()) + "_update.zip";

  // Create archive
  ZIP::CompressFolder(installArchive, robotisDir + "/Data", true, "Data");
  ZIP::AddFolderToArchive(installArchive, robotisDir + "/Linux", true, "Linux");
  ZIP::AddFolderToArchive(installArchive, robotisDir + "/Framework", true, "Framework");
  ZIP::AddFileToArchive(installArchive, robotisDir + "/version.txt", "version.txt");

  // Send archive file
  if (sendFile(installArchive, "/" + mainDirectoryName() + "/update.zip") < 0) {
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }
  executeSSHCommand("chmod 755 /" + mainDirectoryName() + "/update.zip");

  // Delete local archive
  QFile deleteArchive(installArchive);
  if (deleteArchive.exists())
    deleteArchive.remove();

  // Decompress remote controller files
  executeSSHCommand("unzip /" + mainDirectoryName() + "/update.zip");

  // Move files and delete archive
  executeSSHCommand("cp /home/" + username + "/version.txt /" + mainDirectoryName() + "/version.txt");
  executeSSHCommand("cp -r /home/" + username + "/Framework /" + mainDirectoryName());
  executeSSHCommand("cp -r /home/" + username + "/Linux /" + mainDirectoryName());
  executeSSHCommand("cp -r /home/" + username + "/Data /" + mainDirectoryName());
  executeSSHCommand("rm /" + mainDirectoryName() + "/update.zip");
  executeSSHCommand("rm /home/" + username + "/version.txt");
  executeSSHCommand("rm -r /home/" + username + "/Framework");
  executeSSHCommand("rm -r /home/" + username + "/Linux");
  executeSSHCommand("rm -r /home/" + username + "/Data");

  // Compile Framework
  executeSSHCommand("make -C /" + mainDirectoryName() + "/Linux/build -f Makefile clean");
  executeSSHCommand("make -C /" + mainDirectoryName() + "/Linux/build -f Makefile");

  // Recompile demo program
  executeSSHCommand("make -C /" + mainDirectoryName() + "/Linux/project/demo -f Makefile clean");
  executeSSHCommand("make -C /" + mainDirectoryName() + "/Linux/project/demo -f Makefile");

  // Delete version file of the Wrapper in order to force the update/recompilation of it
  executeSSHCommand("rm /" + mainDirectoryName() + "/Linux/project/webots/config/version.txt");

  return 1;
}

int SSH::updateFrameworkIfNeeded(const QString &username, const QString &password) {
  info("Checking Framework version.");
  if (isFrameworkUpToDate()) {
    info("Framework is up to date.");
    return 1;
  }
  return updateFramework(username, password);
}

int SSH::updateWrapper(const QString &username, const QString &password) {
  info("Installing Webots API.");
  emit status("Installing Webots API");

  const QString webotsHomePath = QProcessEnvironment::systemEnvironment().value("WEBOTS_HOME");
  const QString managerDir = webotsHomePath + "/projects/robots/robotis/darwin-op/libraries/managers";
  const QString robotisDir = webotsHomePath + "/projects/robots/robotis/darwin-op";
  const QString installArchive =
    QDir::tempPath() + "/webots_robotis_" + QString::number((int)QCoreApplication::applicationPid()) + "_install.zip";

  // Create archive
  emit status("Installing Webots API: Zipping files.");
  ZIP::CompressFolder(installArchive, managerDir + "/include", true, "include");
  ZIP::AddFolderToArchive(installArchive, managerDir + "/src", true, "src");
  ZIP::AddFolderToArchive(installArchive, managerDir + "/lib", true, "lib");
  ZIP::AddFolderToArchive(installArchive, robotisDir + "/transfer", true, "transfer");
  ZIP::AddFolderToArchive(installArchive, robotisDir + "/config", true, "config");
  ZIP::AddFolderToArchive(installArchive, robotisDir + "/check_start_position", true, "check_start_position");
  ZIP::AddFolderToArchive(installArchive, robotisDir + "/remote_control", true, "remote_control");

  // Clean directory /robotis/Linux/project/webots
  // Remove directory webots but save controller installed
  emit status("Installing Webots API: Checking /" + mainDirectoryName() + "/Linux/project/webots/default.");
  executeSSHCommand("ls /" + mainDirectoryName() + "/Linux/project/webots/default >/dev/null 2>&1 && echo 1 || echo 0", false);
  if (mStdout[0] == '1')
    executeSSHCommand("mv /" + mainDirectoryName() + "/Linux/project/webots/default /home/" + username + "/default");
  emit status("Installing Webots API: Deleting previous installation if any.");
  executeSSHCommand("rm -rf /" + mainDirectoryName() + "/Linux/project/webots");

  // Create new directory webots
  emit status("Installing Webots API: Recreating webots directory.");
  if (sftp_mkdir(mSFTPChannel, ("/" + mainDirectoryName() + "/Linux/project/webots").toUtf8().data(), S_IRWXU) != 0) {
    cerr << "Problem while creating directory webots: " << ssh_get_error(mSSHSession) << endl;
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }
  // Create new directory controller
  if (sftp_mkdir(mSFTPChannel, ("/" + mainDirectoryName() + "/Linux/project/webots/controllers").toUtf8().data(), S_IRWXU) !=
      0) {
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }

  // Create new directory backup
  if (sftp_mkdir(mSFTPChannel, ("/" + mainDirectoryName() + "/Linux/project/webots/backup").toUtf8().data(), S_IRWXU) != 0) {
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }

  executeSSHCommand("ls /home/" + username + "/default >/dev/null 2>&1 && echo 1 || echo 0", false);
  if (mStdout[0] == '1')  // Reinstall previous controller installed
    executeSSHCommand("mv /home/" + username + "/default /" + mainDirectoryName() + "/Linux/project/webots/default");

  // Send archive file
  emit status("Installing Webots API: Uploading to the robot");
  if (sendFile(installArchive, "/" + mainDirectoryName() + "/Linux/project/webots/install.zip") < 0) {
    // Delete local archive
    QFile deleteArchive(installArchive);
    if (deleteArchive.exists())
      deleteArchive.remove();
    return -1;
  }
  executeSSHCommand("chmod 755 /" + mainDirectoryName() + "/Linux/project/webots/install.zip");

  // Delete local archive
  QFile deleteArchive(installArchive);
  if (deleteArchive.exists())
    deleteArchive.remove();

  // Decompress remote controller files
  emit status("Installing Webots API: Uncompressing ZIP file on the robot.");
  executeSSHCommand("unzip /" + mainDirectoryName() + "/Linux/project/webots/install.zip");

  // Move files and delete archive
  executeSSHCommand("mv /home/" + username + "/config/rc.local_original /" + mainDirectoryName() +
                    "/Linux/project/webots/backup/rc.local_original");
  executeSSHCommand("mv /home/" + username + "/include /" + mainDirectoryName() + "/Linux/project/webots/include");
  executeSSHCommand("mv /home/" + username + "/lib /" + mainDirectoryName() + "/Linux/project/webots/lib");
  executeSSHCommand("mv /home/" + username + "/src /" + mainDirectoryName() + "/Linux/project/webots/src");
  executeSSHCommand("mv /home/" + username + "/transfer /" + mainDirectoryName() + "/Linux/project/webots/transfer");
  executeSSHCommand("mv /home/" + username + "/check_start_position /" + mainDirectoryName() +
                    "/Linux/project/webots/check_start_position");
  executeSSHCommand("mv /home/" + username + "/config /" + mainDirectoryName() + "/Linux/project/webots/config");
  executeSSHCommand("mv /home/" + username + "/remote_control /" + mainDirectoryName() +
                    "/Linux/project/webots/remote_control");
  executeSSHCommand("rm /" + mainDirectoryName() + "/Linux/project/webots/install.zip");

  // Compile Wrapper
  emit status("Installing Webots API: Cleaning.");
  // touch source files with the time of the robot (in order to avoid Makefile problems if time of the robot and computer are
  // not synchronized)
  QString command = "find /" + mainDirectoryName() + "/Linux/project/webots -type f -exec touch {} +";
  executeSSHCommand(command);
  executeSSHCommand("make " + makeVars() + " -C /" + mainDirectoryName() +
                    "/Linux/project/webots/check_start_position -f Makefile.robotis-op2 clean");
  executeSSHCommand("make " + makeVars() + " -C /" + mainDirectoryName() +
                    "/Linux/project/webots/remote_control -f Makefile.robotis-op2 clean");
  executeSSHCommand("make " + makeVars() + " -C /" + mainDirectoryName() +
                    "/Linux/project/webots/transfer/lib -f Makefile clean");
  executeSSHCommand("make " + makeVars() + " -C /" + mainDirectoryName() +
                    "/Linux/project/webots/lib -f Makefile.robotis-op2 clean");
  emit status("Installing Webots API: Compiling \"check_start_position\"");
  executeSSHCommand("make " + makeVars() + " -C /" + mainDirectoryName() +
                    "/Linux/project/webots/check_start_position -f Makefile.robotis-op2");
  emit status("Installing Webots API: Compiling \"remote_control\"");
  executeSSHCommand("make " + makeVars() + " -C /" + mainDirectoryName() +
                    "/Linux/project/webots/remote_control -f Makefile.robotis-op2");
  emit status("Installing Webots API: Compiling \"transfer/lib\"");
  executeSSHCommand("make " + makeVars() + " -C /" + mainDirectoryName() + "/Linux/project/webots/transfer/lib -f Makefile");
  emit status("Installing Webots API: Compiling \"lib\"");
  executeSSHCommand("make " + makeVars() + " -C /" + mainDirectoryName() + "/Linux/project/webots/lib -f Makefile.robotis-op2");
  // Hook demo program
  command =
    "echo " + password + " | sudo -S cp /" + mainDirectoryName() + "/Linux/project/webots/config/rc.local_custom /etc/rc.local";
  executeSSHCommand(command);
  executeSSHCommand("ls /" + mainDirectoryName() + "/Linux/project/webots/default >/dev/null 2>&1 && echo 1 || echo 0", false);
  if (mStdout[0] == '0')
    executeSSHCommand("ln -s /" + mainDirectoryName() + "/Linux/project/demo/demo /" + mainDirectoryName() +
                      "/Linux/project/webots/default");
  emit status("Installing Webots API: Done.");

  return 1;
}

QString SSH::mainDirectoryName() {
  if (mActualModel == DARWIN_OP)
    return QString("darwin");
  if (mActualModel == ROBOTIS_OP2)
    return QString("robotis");
  assert(false);  // should not happen
  return QString();
}

QString SSH::makeVars() {
  return "ROBOTISOP2_ROOT=/" + mainDirectoryName();
}

void SSH::log(const QString &message, bool error) {
  emit print(message, error);
}

void SSH::info(const QString &message) {
  // cppcheck-suppress ignoredReturnValue
  log("[INFO] " + message + '\n');
}

bool SSH::isWrapperUpToDate() {
  int index1 = 0, index2 = 0, index3 = 0;
  QString version, versionInstalled;
  QStringList versionList, versionInstalledList;
  QFile mWrapperVersionFile(QProcessEnvironment::systemEnvironment().value("WEBOTS_HOME") +
                            "/projects/robots/robotis/darwin-op/config/version.txt");

  if (mWrapperVersionFile.exists()) {
    if (mWrapperVersionFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QTextStream versionStream(&mWrapperVersionFile);

      if (!versionStream.atEnd()) {
        version = versionStream.readLine();
        versionList = version.split(".");
        index1 = versionList.at(0).toInt();
        index2 = versionList.at(1).toInt();
        index3 = versionList.at(2).toInt();
      } else
        return false;  // file empty
    } else
      return false;  // could not open file
    mWrapperVersionFile.close();
  } else
    return false;  // file does not exist

  char buffer[32];
  if (readRemoteFile("/" + mainDirectoryName() + "/Linux/project/webots/config/version.txt", buffer, sizeof(buffer)) < 0)
    return false;

  versionInstalled = QString(buffer);
  versionInstalledList = versionInstalled.split(".");
  if ((index1 > versionInstalledList.at(0).toInt()) || (index2 > versionInstalledList.at(1).toInt()) ||
      (index3 > versionInstalledList.at(2).toInt()))
    return false;  // new version of the Wrapper

  return true;
}

int SSH::updateWrapperIfNeeded(const QString &username, const QString &password) {
  info("Checking Webots API version.");
  if (isWrapperUpToDate()) {
    info("Webots API is up to date.");
    return 1;
  }
  return updateWrapper(username, password);
}

int SSH::startRemoteCompilation(const QString &IPAddress, const QString &username, const QString &password,
                                bool makeDefaultController) {
  if (openSSHSession(IPAddress, username, password) == -1)
    return -1;
  openSFTPChannel();
  updateFrameworkIfNeeded(username, password);
  updateWrapperIfNeeded(username, password);

  info("Starting remote compilation.");
  emit status("Starting remote compilation: Zipping controller files.");
  const QString controller(wb_robot_get_controller_name());
  info("Controller: '" + controller + "'.");
  const QString controllerArchive(QDir::tempPath() + "/webots_robotis_" +
                                  QString::number((int)QCoreApplication::applicationPid()) + "_controller.zip");
  info("Controller archive: '" + controllerArchive + "'.");
  ZIP::CompressFolder(controllerArchive, QString(wb_robot_get_project_path()) + "/controllers/" + controller, true,
                      controller.toUtf8());

  emit status("Starting remote compilation: Cleaning up the robot.");
  executeSSHCommand("echo " + password +
                    " | sudo -S killall -q default demo controller remote_control");  // kill default demo process (if any)
  // Clean directory controllers
  executeSSHCommand("rm -rf /" + mainDirectoryName() + "/Linux/project/webots/controllers/*");

  // Send archive file
  emit status("Starting remote compilation: Uploading to the robot.");
  sendFile(controllerArchive, "/" + mainDirectoryName() + "/Linux/project/webots/controllers/controller.zip");

  executeSSHCommand("chmod 755 /" + mainDirectoryName() + "/Linux/project/webots/controllers/controller.zip");

  // Delete local archive
  QFile deleteArchive(controllerArchive);
  if (deleteArchive.exists())
    deleteArchive.remove();

  // Decompress remote controller files
  executeSSHCommand("unzip /" + mainDirectoryName() + "/Linux/project/webots/controllers/controller.zip -d /" +
                    mainDirectoryName() + "/Linux/project/webots/controllers");

  // Deleting archive
  executeSSHCommand("rm /" + mainDirectoryName() + "/Linux/project/webots/controllers/controller.zip");

  // Compile controller
  QString makeClean = "make " + makeVars() + " -C /" + mainDirectoryName() + "/Linux/project/webots/controllers/" + controller +
                      " -f Makefile.robotis-op2 clean";
  executeSSHCommand(makeClean);
  // touch source files with the time of the robot (in order to avoid Makefile problems if time of the robot and computer are
  // not synchronized)
  QString command =
    "find /" + mainDirectoryName() + "/Linux/project/webots/controllers/" + controller + "/* -exec touch {} \\;";
  executeSSHCommand(command);
  QString makeController = "make " + makeVars() + " -C /" + mainDirectoryName() + "/Linux/project/webots/controllers/" +
                           controller + " -f Makefile.robotis-op2";
  executeSSHCommand(makeController);
  if (!mStderr.isEmpty()) {  // return in case of compilation failure
    mError = "Controller compilation failed.";
    closeSFTPChannel();
    closeSSHSession();
    return 1;
  }
  info("Checking ready position...");
  executeSSHCommand("echo " + password + " | sudo -S /" + mainDirectoryName() +
                    "/Linux/project/webots/check_start_position/check_start_position");
  if (!mStdout.startsWith("Success")) {
    log("Robot is not in ready position.\n", true);
    emit status("Status: Robot is not in ready position.");
    // Remove compilation files
    QString removeController = "rm -r /" + mainDirectoryName() + "/Linux/project/webots/controllers/" + controller;
    executeSSHCommand(removeController);
  } else {
    // Verify that controller exists -> no compilation error
    QString controllerExist = "ls /" + mainDirectoryName() +
                              "/Linux/project/webots/controllers >/dev/null 2>&1 && echo 1 || echo 0" + controller + "/" +
                              controller + " | grep -c " + controller;
    executeSSHCommand(controllerExist, false);
    if (mStdout[0] == '1') {        // controller exists
      if (makeDefaultController) {  // setup as default controller
        executeSSHCommand("rm /" + mainDirectoryName() + "/Linux/project/webots/default");
        QString softLinkCommand = "cp /" + mainDirectoryName() + "/Linux/project/webots/controllers/" + controller + "/" +
                                  controller + " /" + mainDirectoryName() + "/Linux/project/webots/default";
        executeSSHCommand(softLinkCommand);
      }
      // Start controller
      executeSSHCommand("mv /" + mainDirectoryName() + "/Linux/project/webots/controllers/" + controller + "/" + controller +
                        " /" + mainDirectoryName() + "/Linux/project/webots/controllers/" + controller + "/controller");
      executeSSHCommand("echo -e \'#!/bin/bash\\nexport DISPLAY=:0\\n/" + mainDirectoryName() +
                        "/Linux/project/webots/controllers/" + controller + "/controller\\n\' > /" + mainDirectoryName() +
                        "/Linux/project/webots/controllers/" + controller + "/" + controller);
      executeSSHCommand("chmod a+x /" + mainDirectoryName() + "/Linux/project/webots/controllers/" + controller + "/" +
                        controller);
      executeSSHCommand("echo " + password + " | sudo -S /" + mainDirectoryName() + "/Linux/project/webots/controllers/" +
                          controller + "/" + controller,
                        true, false);  // wait until we terminate it
      executeSSHCommand("echo " + password + " | sudo -S killall -q controller " + controller);
      executeSSHCommand("rm -rf /" + mainDirectoryName() + "/Linux/project/webots/controllers/*");
    } else  // controller does not exist
      executeSSHCommand("rm -rf /" + mainDirectoryName() + "/Linux/project/webots/controllers/*");
  }
  closeSFTPChannel();
  closeSSHSession();
  return 1;
}

int SSH::startRemoteControl(const QString &IPAddress, const QString &username, const QString &password) {
  if (openSSHSession(IPAddress, username, password) == -1)
    return -1;
  openSFTPChannel();
  info("Starting remote control.");
  executeSSHCommand("echo " + password + " | sudo -S killall -q remote_control default controller");
  updateFrameworkIfNeeded(username, password);
  updateWrapperIfNeeded(username, password);
  WbDeviceTag camera = wb_robot_get_device("Camera");
  int cameraWidth = wb_camera_get_width(camera);
  int cameraHeight = wb_camera_get_height(camera);
  if ((cameraWidth != 320 && cameraWidth != 160 && cameraWidth != 80 && cameraWidth != 40) ||
      (cameraHeight != 240 && cameraHeight != 120 && cameraHeight != 60 && cameraHeight != 30)) {
    QString errorString = QObject::tr("Unsupported camera resolution. Aborting...\nOnly the following resolutions are "
                                      "available:\n\tWidth:  320/160/80/40\n\tHeight: 240/120/60/30\n");
    log(errorString, true);
    closeSFTPChannel();
    closeSSHSession();
    return -1;
  }
  info("Starting remote control server...");
  executeSSHCommand("echo " + password + " | sudo -S /" + mainDirectoryName() +
                      "/Linux/project/webots/remote_control/remote_control " + QString::number(320 / cameraWidth) + " " +
                      QString::number(240 / cameraHeight),
                    true, false);  // wait until we terminate it
  info("Remote control server ended.");
  closeSFTPChannel();
  closeSSHSession();
  return 1;
}

int SSH::uninstall(const QString &IPAddress, const QString &username, const QString &password) {
  info("Uninstalling Webots API...");
  if (openSSHSession(IPAddress, username, password) == -1)
    return -1;
  executeSSHCommand(
    "ls /" + mainDirectoryName() + "/Linux/project/webots/backup/rc.local_original >/dev/null 2>&1 && echo 1 || echo 0", false);
  if (mStdout[0] == '1')  // Restore rc.local
    executeSSHCommand("echo " + password + " | sudo -S cp /" + mainDirectoryName() +
                      "/Linux/project/webots/backup/rc.local_original /etc/rc.local");
  executeSSHCommand("rm -rf /" + mainDirectoryName() + "/Linux/project/webots");
  closeSSHSession();
  info("Uninstallation complete.");
  return 1;
}
