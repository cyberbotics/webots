#!/usr/bin/env python

# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test suite."""

import sys
import os
import shutil
import platform
import datetime
import getpass
import glob
import subprocess
import threading
import time
import multiprocessing
import argparse

from command import Command
from cache.cache_environment import update_cache_urls

if sys.platform == 'linux':
    result = subprocess.run(['lsb_release', '-sr'], stdout=subprocess.PIPE)
    is_ubuntu_22_04 = result.stdout.decode().strip() == '22.04'
else:
    is_ubuntu_22_04 = False

# monitor failures
failures = 0
systemFailures = []
whitelist = ['ContextResult::kTransientFailure: Failed to send GpuChannelMsg_CreateCommandBuffer']
# parse arguments
filesArguments = []
parser = argparse.ArgumentParser(description='Test-suite command line options')
parser.add_argument('--nomake', dest='nomake', default=False, action='store_true', help='The controllers are not re-compiled.')
parser.add_argument('--no-ansi-escape', dest='ansi_escape', default=True, action='store_false', help='Disables ansi escape.')
parser.add_argument('--group', '-g', type=str, dest='group', default=[], help='Specifies which group of tests should be run.',
                    choices=['api', 'cache', 'other_api', 'physics', 'protos', 'parser', 'rendering', 'with_rendering'])
parser.add_argument('worlds', nargs='*', default=[])
args = parser.parse_args()

filesArguments = args.worlds

if args.group:
    testGroups = [str(args.group)]
else:
    testGroups = ['api', 'cache', 'other_api', 'physics', 'protos', 'parser', 'rendering', 'with_rendering']

if sys.platform == 'win32':
    testGroups.remove('parser')  # this one doesn't work on Windows

# global files
testsFolderPath = os.path.dirname(os.path.abspath(__file__)) + os.sep
outputFilename = testsFolderPath + 'output.txt'
defaultProjectPath = testsFolderPath + 'default' + os.sep
supervisorControllerName = 'test_suite_supervisor'
protoFileNames = ['TestSuiteSupervisor.proto', 'TestSuiteEmitter.proto']
tempWorldCounterFilename = testsFolderPath + 'world_counter.txt'
webotsStdOutFilename = testsFolderPath + 'webots_stdout.txt'
webotsStdErrFilename = testsFolderPath + 'webots_stderr.txt'

# Webots setup (cf. setupWebots() below)
webotsFullPath = ''
webotsVersion = ''


def setupWebots():
    """Find webots binary thanks to WEBOTS_HOME."""
    os.putenv('WEBOTS_TEST_SUITE', 'TRUE')
    os.putenv('WEBOTS_EMPTY_PROJECT_PATH', defaultProjectPath)

    global webotsFullPath
    global webotsVersion
    global webotsSysInfo

    if sys.platform == 'win32':
        webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + 'msys64' + \
            os.sep + 'mingw64' + os.sep + 'bin' + os.sep + 'webots.exe'
    else:
        webotsBinary = 'webots'
        if 'WEBOTS_HOME' in os.environ:
            webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + webotsBinary
        else:
            webotsFullPath = '..' + os.sep + '..' + os.sep + webotsBinary
        if not os.path.isfile(webotsFullPath):
            sys.exit('Error: ' + webotsBinary + ' binary not found')
        webotsFullPath = os.path.normpath(webotsFullPath)

    command = Command(webotsFullPath + ' --version')
    command.run()
    if command.returncode != 0:
        raise RuntimeError('Error when getting the Webots version: ' + command.output)
    try:
        webotsVersion = command.output.replace('\n', ' ').split(' ')[2].split('.')
    except IndexError:
        raise RuntimeError('Cannot parse Webots version: ' + command.output)

    command = Command(webotsFullPath + ' --sysinfo')
    command.run()
    if command.returncode != 0:
        raise RuntimeError('Error when getting the Webots information of the system')
    webotsSysInfo = command.output.split('\n')


def findFirstWorldFilename(worldsFilename):
    """Get the first world file name."""
    file = open(worldsFilename)
    worldFilename = file.readline().strip()
    file.close()
    return worldFilename


def resetIndexFile(indexFilename):
    """Create the index file."""
    file = open(indexFilename, 'w')
    file.write('0\n')
    file.close()


def formatString(s):
    """Add a predefined number of spaces after the ':' character."""
    try:
        index = s.index(': ')
        s0 = '{:<20}'.format(s[0:index])
        s0 += s[index:]
        return s0
    except ValueError:  # can be thrown by string.index()
        return s


def resetOutputFile():
    """Create the output file."""
    file = open(outputFilename, 'w')
    file.write(formatString('Webots binary: ' + webotsFullPath) + '\n')
    file.write(formatString('Webots version: ' + str(webotsVersion)) + '\n')
    file.write(formatString(
        'Operating System: ' + platform.platform() +
        ' [' + platform.machine() + '] ' + platform.processor() +
        ' (' + platform.node() + ')') + '\n'
    )
    file.write(formatString('Date: ' + datetime.datetime.now().ctime()) + '\n')
    file.write(formatString('Tester: ' + getpass.getuser()) + '\n')
    for line in webotsSysInfo:
        file.write(formatString(line) + '\n')
    file.close()


def appendToOutputFile(txt):
    """Append txt to output file."""
    file = open(outputFilename, 'a')
    file.write(txt)
    file.close()


def executeMake():
    """Execute 'make release' to ensure every controller/plugin is compiled."""
    curdir = os.getcwd()
    os.chdir(testsFolderPath)
    command = Command('make release -j%d' % multiprocessing.cpu_count())
    command.run(silent=False)
    os.chdir(curdir)
    if command.returncode != 0:
        raise RuntimeError('Error when executing the Make command')


def generateWorldsList(groupName, worldsFilename):
    """Generate the list of worlds to run."""
    f = open(worldsFilename, 'w')
    worldsCount = 0
    # generate the list from the arguments
    if filesArguments:
        for file in filesArguments:
            if file.startswith(groupName):
                f.write(file + '\n')
        worldsCount = len(filesArguments)

    # generate the list from 'ls worlds/*.wbt'
    else:
        filenames = glob.glob(testsFolderPath + groupName + os.sep + 'worlds' + os.sep + '*.wbt')

        # remove the generic name
        for filename in filenames:
            if filename.endswith('test_suite'):
                filenames.remove(filename)

        # alphabetical order
        filenames.sort()

        # to file
        for filename in filenames:
            # speaker test not working on github action because of missing sound drivers
            # robot window and movie recording test not working on BETA Ubuntu 22.04 GitHub Action environment
            if (not filename.endswith('_temp.wbt') and
                    not ('GITHUB_ACTIONS' in os.environ and (
                        filename.endswith('speaker.wbt') or
                        filename.endswith('local_proto_with_texture.wbt') or
                        (filename.endswith('robot_window_html.wbt') and is_ubuntu_22_04) or
                        (filename.endswith('supervisor_start_stop_movie.wbt') and is_ubuntu_22_04)
                        ))):
                f.write(filename + '\n')
                worldsCount += 1

    f.close()
    return worldsCount


def monitorOutputFile(finalMessage):
    """Display the output file on the console."""
    global monitorOutputCommand
    monitorOutputCommand = Command('tail -f ' + outputFilename, args.ansi_escape)
    monitorOutputCommand.run(expectedString=finalMessage, silent=False)


if not args.nomake:
    executeMake()
setupWebots()
resetOutputFile()

finalMessage = 'Test suite complete'
thread = threading.Thread(target=monitorOutputFile, args=[finalMessage])
thread.start()

webotsArguments = '--mode=fast --stdout --stderr --batch'
webotsArgumentsNoRendering = webotsArguments + ' --no-rendering --minimize'


for groupName in testGroups:
    if groupName == 'cache':
        update_cache_urls(False)  # setup new environment

    testFailed = False

    appendToOutputFile('\n### ' + groupName + ' test\n\n')

    # clear stdout and stderr files
    open(webotsStdErrFilename, 'w').close()
    open(webotsStdOutFilename, 'w').close()

    worldsFilename = testsFolderPath + groupName + os.sep + 'worlds.txt'
    indexFilename = testsFolderPath + groupName + os.sep + 'worlds_index.txt'

    # init temporary world counter file
    tempFile = open(tempWorldCounterFilename, 'w')
    tempFile.write('0')
    tempFile.close()

    supervisorTargetDirectory = testsFolderPath + groupName + os.sep + 'controllers' + os.sep + \
        supervisorControllerName
    if not os.path.exists(supervisorTargetDirectory):
        os.makedirs(supervisorTargetDirectory)
    shutil.copyfile(
        defaultProjectPath + 'controllers' + os.sep +
        supervisorControllerName + os.sep +
        supervisorControllerName + '.py',
        supervisorTargetDirectory + os.sep + supervisorControllerName + '.py'
    )
    # parser tests uses a slightly different Supervisor PROTO
    protosTargetDirectory = testsFolderPath + groupName + os.sep + 'protos'
    protosSourceDirectory = defaultProjectPath + 'protos' + os.sep
    if not os.path.exists(protosTargetDirectory):
        os.makedirs(protosTargetDirectory)
    for protoFileName in protoFileNames:
        shutil.copyfile(protosSourceDirectory + protoFileName,
                        protosTargetDirectory + os.sep + protoFileName)
    worldsCount = generateWorldsList(groupName, worldsFilename)
    firstSimulation = findFirstWorldFilename(worldsFilename)
    if not os.path.exists(firstSimulation):
        continue

    resetIndexFile(indexFilename)

    # Here is an example to run webots in gdb and display the stack
    # when it crashes.
    # this is particularly useful to debug on the jenkins server
    #  command = Command('gdb -ex run --args ' + webotsFullPath + '-bin ' +
    #                    firstSimulation + ' --mode=fast --no-rendering --minimize')
    #  command.run(silent = False)

    clearCache = ' --clear-cache' if groupName == 'cache' else ''
    if groupName == 'with_rendering':
        command = Command(webotsFullPath + ' ' + firstSimulation + ' ' + webotsArguments + clearCache)
    else:
        command = Command(webotsFullPath + ' ' + firstSimulation + ' ' + webotsArgumentsNoRendering + clearCache)

    # redirect stdout and stderr to files
    command.runTest(timeout=10 * 60)  # 10 minutes

    if command.isTimeout or command.returncode != 0:
        if command.isTimeout:
            failures += 1
            appendToOutputFile(
                'FAILURE: Webots has been terminated by the test suite script\n')
        else:
            failures += 1
            appendToOutputFile(
                'FAILURE: Webots exits abnormally with this error code: ' + str(command.returncode) + '\n')
        testFailed = True
    else:
        # check count of executed worlds
        tempFile = open(tempWorldCounterFilename)
        counterString = tempFile.read()
        tempFile.close()
        if int(counterString) < worldsCount:
            testFailed = True
            appendToOutputFile('FAILURE: Some tests have not been executed\n')
            appendToOutputFile('- expected number of worlds: %d\n' % (worldsCount))
            appendToOutputFile('- number of worlds actually tested: %s)\n' % (counterString))
        else:
            lines = open(webotsStdErrFilename, 'r').readlines()
            for line in lines:
                if 'Failure' in line:
                    # check if it should be ignored
                    if not any(item in line for item in whitelist):
                        failures += 1
                        systemFailures.append(line)

    if testFailed:
        appendToOutputFile('\nWebots complete STDOUT log:\n')
        with open(webotsStdOutFilename) as f:
            for line in f:
                appendToOutputFile(line)
        appendToOutputFile('\nWebots complete STDERR log:\n')
        with open(webotsStdErrFilename) as f:
            for line in f:
                appendToOutputFile(line)
                if '(core dumped)' in line:
                    seg_fault_line = line[0:line.find(' Segmentation fault')]
                    pid = int(seg_fault_line[seg_fault_line.rfind(' ') + 1:])
                    core_dump_file = '/tmp/core_webots-bin.' + str(pid)
                    if os.path.exists(core_dump_file):
                        appendToOutputFile(subprocess.check_output([
                            'gdb', '--batch', '--quiet', '-ex', 'bt', '-ex',
                            'quit', '../bin/webots-bin', core_dump_file
                        ]))
                        os.remove(core_dump_file)
                    else:
                        appendToOutputFile(
                            'Cannot get the core dump file: "%s" does not exist.' % core_dump_file
                        )

    # undo changes (to avoid generating useless diffs)
    if groupName == 'cache':
        update_cache_urls(True)

appendToOutputFile('\n' + finalMessage + '\n')

if len(systemFailures) > 0:
    appendToOutputFile('\nSystem Failures:\n')
    for message in systemFailures:
        appendToOutputFile(message)

time.sleep(1)
if monitorOutputCommand.isRunning():
    monitorOutputCommand.terminate(force=True)

with open(outputFilename, 'r') as file:
    content = file.read()
    failures += content.count('FAILURE ')

sys.exit(failures)
