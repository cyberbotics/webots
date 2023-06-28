# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a system command."""

import os
import queue
import subprocess
import sys
import threading


class Command(object):
    """Launch a system command."""

    def __init__(self, argList, ansiEscape=False):
        """Constructor."""
        self.ansiEscape = ansiEscape
        self.cmd = argList
        self.resetAttributes()
        self.mainProcessMutex = threading.Lock()

    def resetAttributes(self):
        """Reset the internal attributes."""
        self.expectedStringFound = False
        self.isTimeout = False
        self.mainProcess = None
        self.mainThread = None
        self.returncode = 0
        self.output = ''

    def terminate(self, force):
        """Terminate the command."""
        self.isRunningFlag = False

        if self.mainProcess:
            self.mainProcess.terminate()

        if force and sys.platform == 'darwin' and self.mainProcess:
            self.mainProcess.kill()

    def isRunning(self):
        """Detect if the command is running."""
        return self.mainProcess is not None

    def stopMainProcess(self):
        """Stop the main process."""
        if self.mainProcess:
            self.mainProcess.terminate()
            if self.mainThread:
                self.mainThread.join()
        self.mainProcess = None
        self.mainThread = None

    def run(self, timeout=None, expectedString=None, silent=True,
            forceTermination=True, shell=False, redirectionFile=None):
        """Run the command and monitor STDERR and STDOUT pipe."""
        def mainTarget():
            if self.redirectionFile is None:
                self.mainProcess = subprocess.Popen(
                    self.cmd, shell=self.shell, bufsize=1, universal_newlines=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            else:
                outFile = open(self.redirectionFile, "w")
                self.mainProcess = subprocess.Popen(
                    self.cmd, shell=self.shell, bufsize=1, universal_newlines=True,
                    stdout=outFile, stderr=outFile)
            while self.mainProcess.poll() is None:
                self.mainProcess.wait()
            self.returncode = self.mainProcess.returncode
            with self.mainProcessMutex:
                self.mainProcess = None
            if self.redirectionFile is not None:
                outFile.close()

        def outputWriterTarget():
            while self.isRunningFlag:
                line = ''
                with self.mainProcessMutex:
                    if self.mainProcess:
                        line = self.mainProcess.stdout.readline()  # blocking
                if line:
                    self.output += line
                    if not self.silent:
                        if self.ansiEscape:
                            if line.startswith("OK: "):
                                line = '\033[92m' + line  # green
                            elif line.startswith("FAILURE"):
                                line = '\033[91m' + line  # red
                            else:
                                line = '\033[0m' + line
                        print(line[:-1])
                        if sys.platform == 'win32':
                            sys.stdout.flush()
                    if self.expectedString and self.expectedString in line:
                        self.expectedStringFound = True
                        self.terminate(force=True)
                        return

        self.resetAttributes()

        self.expectedString = expectedString
        self.silent = silent
        self.timeout = timeout
        self.shell = shell
        self.redirectionFile = redirectionFile

        self.isRunningFlag = True

        try:
            self.outputWriterThread = threading.Thread(
                target=outputWriterTarget)
            self.outputWriterThread.start()

            self.mainThread = threading.Thread(target=mainTarget)
            self.mainThread.start()

            self.mainThread.join(timeout)
            self.isRunningFlag = False

            if self.mainProcess and self.mainThread.is_alive():  # timeout case
                self.isTimeout = True
                if forceTermination:
                    self.stopMainProcess()

        except (KeyboardInterrupt, SystemExit):
            self.isRunningFlag = False
            if self.mainProcess and self.mainThread.is_alive():
                self.terminate(force=False)
            exit()

    def runTest(self, timeout=None, silent=True, forceTermination=True, shell=False):
        """Run the command and redirect the STDERR and STDOUT to files."""

        def enqueue_stream(stream, queue, type):
            for line in iter(stream.readline, b''):
                queue.put(str(type) + line.decode('utf-8'))
            stream.close()

        def enqueue_process(process, queue):
            process.wait()
            queue.put('x')

        def mainTarget():
            outFile = open(self.outFileName, "w")
            errFile = open(self.errFileName, "w")

            p = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            q = queue.Queue()
            to = threading.Thread(target=enqueue_stream, args=(p.stdout, q, 1))
            te = threading.Thread(target=enqueue_stream, args=(p.stderr, q, 2))
            tp = threading.Thread(target=enqueue_process, args=(p, q))
            te.start()
            to.start()
            tp.start()

            while True:
                line = q.get()
                if line[0] == 'x':
                    break
                if line[0] == '2':  # stderr
                    errFile.write(line[1:])
                    errFile.flush()
                else:  # stdout
                    outFile.write(line[1:])
                    outFile.flush()
            tp.join()
            to.join()
            te.join()
            outFile.close()
            errFile.close()
            self.returncode = p.returncode

        self.outFileName = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'webots_stdout.txt')
        self.errFileName = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'webots_stderr.txt')
        self.resetAttributes()

        self.silent = silent
        self.timeout = timeout
        self.isRunningFlag = True

        try:
            self.mainThread = threading.Thread(target=mainTarget)
            self.mainThread.start()

            self.mainThread.join(timeout)
            self.isRunningFlag = False

            if self.mainProcess and self.mainThread.is_alive():  # timeout case
                self.isTimeout = True
                if forceTermination:
                    self.stopMainProcess()

        except (KeyboardInterrupt, SystemExit):
            self.isRunningFlag = False
            if self.mainProcess and self.mainThread.is_alive():
                self.terminate(force=False)
            exit()
