#!/usr/bin/env python3

import psutil
import re
import sys
import time
import os

'''
Utility script for Linux server machines that kills all instances of streaming Webots processes
fulfilling the following conditions:
- created 3 days ago or earlier
- controller files not modified in the last 24 hours
Only the batch script is terminated but this script will also check and notify if Webots binary is still running.
'''


def checkIfRecentlyModified(path, timeout):
    for root, subdirs, files in os.walk(path):
        for filename in files:
            fileStatsObj = os.stat(os.path.join(root, filename))
            if fileStatsObj.st_mtime < timeout:
                return True
    return False


currentTime = time.time()
creationTimeLimit = currentTime - 86400 * 3  # 3 days in seconds
modificationTimeLimit = currentTime - 86400 * 1  # 1 day in seconds
processName = 'webots'
streamPattern = re.compile('^--stream.*')
listOfProcessObjects = []


# Iterate over the all the running process
for proc in psutil.process_iter():
    try:
        pinfo = proc.as_dict(attrs=['pid', 'name'])
        # Check if process name contains the given name string.
        if processName in pinfo['name'].lower() and (proc.create_time() < creationTimeLimit) and \
           any(streamPattern.match(argument) for argument in proc.cmdline()):
            for argument in proc.cmdline():
                if 'webots/instances/' in argument:
                    pinfo['path'] = argument.split('/worlds/')[0]
            pinfo['create_time'] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(proc.create_time()))

            if pinfo['name'] == 'webots':
                if checkIfRecentlyModified(pinfo['path'], modificationTimeLimit):
                    print("Kill '{}': pid {} create on {} ('{}')".format(pinfo['name'], pinfo['pid'], pinfo['create_time'],
                                                                         pinfo['path']))
                    proc.terminate()
                    listOfProcessObjects.append(pinfo)
                else:
                    print("Still running'{}': pid {} create on {} ('{}')".format(pinfo['name'], pinfo['pid'],
                                                                                 pinfo['create_time'], pinfo['id']))
            else:
                listOfProcessObjects.append(pinfo)
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        pass


# Check if all instances are killed
if len(listOfProcessObjects) > 0:
    print("Waiting...")
    for i in range(5):
        time.sleep(1)  # delay for 1 seconds waiting processes to stop
        done = True
        for elem in listOfProcessObjects:
            if psutil.pid_exists(elem['pid']):
                done = False
                if i == 4:
                    print("Process not terminated: '{}', pid {} created on {}".format(
                        elem['name'], elem['pid'], elem['create_time']))
            else:
                listOfProcessObjects.remove(elem)
        if done:
            print('Done.')
            sys.exit(0)
else:
    print('No running process found')
