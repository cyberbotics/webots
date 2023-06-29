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

"""Python Supervisor interface with SUMO using TraCI."""

from SumoSupervisor import SumoSupervisor

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile


def sumoImportError():
    sys.stderr.write("SUMO not found.\n")
    if sys.platform.startswith('linux'):
        if os.getenv('SNAP_NAME'):
            sys.stderr.write("Please set the 'SumoInterface.externController' field to TRUE and "
                             "launch the controller as extern controller.\n"
                             "When launching the controller as extern, you must specify the SUMO options like --no-netconvert "
                             "or --no-gui manually.\n"
                             "Install SUMO ")
        else:
            sys.stderr.write("Please install it ")
        sys.stderr.write("with `sudo apt install sumo sumo-tools` and reboot.\n")
    else:
        sys.stderr.write("Please install it following the instructions at https://sumo.dlr.de/docs/Installing/.\n")
    sys.exit("Or check that the SUMO_HOME environment variable points to the directory of your SUMO installation.")


# we need to import python modules from the $SUMO_HOME/tools directory
try:
    WEBOTS_HOME = os.path.normpath(os.environ.get('WEBOTS_HOME'))
    if 'SUMO_HOME' in os.environ:
        sumoPath = os.environ['SUMO_HOME']
        print('Using SUMO from %s' % sumoPath)
    else:
        sumoImportError()

    if sys.platform.startswith('darwin'):
        libraryVariablePath = 'DYLD_LIBRARY_PATH'
    elif sys.platform.startswith('linux'):
        libraryVariablePath = 'LD_LIBRARY_PATH'
    else:
        libraryVariablePath = 'PATH'
    path = os.environ.get(libraryVariablePath)
    addToPath = os.path.join(sumoPath, 'bin')
    if sys.platform.startswith('linux'):
        addToPath = os.path.join(WEBOTS_HOME, 'lib') + ';' + os.path.join(sumoPath, 'bin')
    if path is None:
        os.putenv(libraryVariablePath, addToPath)
    else:
        os.putenv(libraryVariablePath, addToPath + ';' + path)
    sys.path.append(os.path.join(sumoPath, 'tools'))
    import traci
    import sumolib
except ImportError:
    sumoImportError()


def get_options():
    """Parse the controler arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-gui", dest="noGUI", action="store_true", default=False,
                        help="runs the command line version of sumo")
    parser.add_argument("--verbose", dest="verbose", action="store_true", default=False,
                        help="prints sumo output in Webots console")
    parser.add_argument("--no-netconvert", dest="noNetconvert", action="store_true", default=False,
                        help="does not run netconvert before launching sumo")
    parser.add_argument("--disable-traffic-lights", dest="disableTrafficLights", action="store_true", default=False,
                        help="disables the update of the traffic lights state in Webots")
    parser.add_argument("--step", type=int, dest="step", default=200, help="specifies the time step of sumo [ms]")
    parser.add_argument("--max-vehicles", type=int, dest="maxVehicles", default=100,
                        help="specifies the maximum vehicles to add on Webots side")
    parser.add_argument("--rotate-wheels", dest="rotateWheels", action="store_true", default=False,
                        help="enables the wheels rotation.")
    parser.add_argument("--radius", type=int, dest="radius", default=-1,
                        help="specifies the visibility radius of the vehicles in meters (-1 means no limit)")
    parser.add_argument("--enable-height", dest="enableHeight", action="store_true", default=False,
                        help="specifies if height information should be extracted from the edge name")
    parser.add_argument("--directory", dest="directory", default="",
                        help="specifies the directory where are located the files defining the network")
    parser.add_argument("--port", type=int, dest="port", default=8873, help="specifies which port to use")
    parser.add_argument("--seed", type=int, dest="seed", default=1,
                        help="specifies the seed of the SUMO random number generator (0 for the '--random' option of SUMO)")
    parser.add_argument("--use-display", dest="useDisplay", action="store_true", default=False,
                        help="displays the gui view of SUMO in a Webots display (only working in gui mode)")
    parser.add_argument("--display-refresh-rate", type=int, dest="displayRefreshRate", default=1000,
                        help="specifies the refresh rate of the SUMO display in Webots")
    parser.add_argument("--display-zoom", type=float, dest="displayZoom", default=1.0,
                        help="specifies the initial zoom of the SUMO display in Webots (100 means no scaling)")
    parser.add_argument("--display-fit-size", dest="displayFitSize", action="store_true", default=False,
                        help="specifies if the image should be resized to fit the SUMO display size or not")
    parser.add_argument("--maximum-lateral-speed", type=float, dest="maximumLateralSpeed", default=2.5,
                        help="specifies the maximal lateral speed of any vehicle in meter per second.")
    parser.add_argument("--maximum-angular-speed", type=float, dest="maximumAngularSpeed", default=3,
                        help="specifies the maximal angular speed of any vehicle in radian per second.")
    parser.add_argument("--lane-change-delay", type=float, dest="laneChangeDelay", default=3,
                        help='specifies the time required to change lane (during this period position in Webots and SUMO may '
                        'not be perfectly synchronized anymore).')
    parser.add_argument("--sumo-arguments", dest="sumoArguments", default="", help="specifies additional SUMO arguments.")
    args = parser.parse_args()
    return args


# The main program starts from here

# Start sumo as a server and then connect and run
controller = SumoSupervisor()
options = get_options()

useDisplay = False
if options.noGUI:
    sumoBinary = os.path.join(sumoPath, 'bin', 'sumo')
else:
    if sys.platform.startswith('darwin') and not os.path.exists(os.path.join(os.sep, 'opt', 'X11')):
        sys.stderr.write("X11 is not installed and is required to launch the gui version of SUMO.\n")
        sys.stderr.write("You can easily install X11 following this link: https://support.apple.com/en-us/HT201341\n")
        sys.stderr.write("Starting command line version of SUMO instead.\n")
        sumoBinary = os.path.join(sumoPath, 'bin', 'sumo')
    else:
        sumoBinary = os.path.join(sumoPath, 'bin', 'sumo-gui')
        if options.useDisplay:
            useDisplay = True

# check if the target directory is in the WEBOTS_HOME path or not set, and adjust path if it is the case
directory = options.directory if options.directory == '' else os.path.normpath(options.directory)
if directory.startswith('WEBOTS_HOME'):
    directory = directory.replace('WEBOTS_HOME', WEBOTS_HOME)
elif directory == "":  # no directory set, use standard directory (same name of the world ending with '_net')
    directory = re.sub(r'.wbt$', '_net', controller.getWorldPath())
if not os.path.isdir(directory):
    sys.exit("You should specify in which directory are stored the network files associated to this world with the "
             "'--directory' argument or put them in the '%s' directory." % directory)

tmpDirectory = None
# generate the net file with the 'netconvert' utility
if not options.noNetconvert:
    # generate temporary directory and move network file in it
    tmpDirectory = tempfile.mkdtemp()
    for item in os.listdir(directory):
        s = os.path.join(directory, item)
        d = os.path.join(tmpDirectory, item)
        if os.path.isdir(s):
            shutil.copytree(s, d, True)
        else:
            shutil.copy2(s, d)
    directory = tmpDirectory
    print("Temporary network files generated in " + tmpDirectory + "\n")
    # check if default configuration file exist
    netConfigurationFile = os.path.join(directory, 'sumo.netccfg')
    if not os.path.isfile(netConfigurationFile):
        fileFound = ''
        # no default configuration file, try to find another one
        for file in os.listdir(directory):
            if file.endswith('.netccfg'):
                if fileFound == "":
                    netConfigurationFile = os.path.join(directory, file)
                    fileFound = file
                else:
                    print("More than one NETCONVERT configuration file found, using: " + fileFound + "\n")
                    break
    if not os.path.isfile(netConfigurationFile) and tmpDirectory is not None:
        shutil.rmtree(tmpDirectory)
        sys.exit("Could not find any NETCONVERT configuration file (*.netccfg).")
    if subprocess.call([os.path.join(sumoPath, 'bin', 'netconvert'), "-c", netConfigurationFile, "--xml-validation", "never"],
                       stdout=sys.stdout, stderr=sys.stderr) != 0:
        sys.exit("NETCONVERT failed to generate the network file.")

# this is the normal way of using traci. sumo is started as a
# subprocess and then the python script connects and runs
FNULL = open(os.devnull, 'w')
# check if default configuration file exist
configurationFile = os.path.join(directory, 'sumo.sumocfg')
if not os.path.isfile(configurationFile):
    fileFound = ""
    for file in os.listdir(directory):
        # no default configuration file, try to find another one
        if file.endswith('.sumocfg'):
            if fileFound == '':
                configurationFile = os.path.join(directory, file)
                fileFound = file
            else:
                print("More than one SUMO configuration file found, using: " + fileFound + "\n")
                break
if not os.path.isfile(configurationFile) and tmpDirectory is not None:
    shutil.rmtree(tmpDirectory)
    sys.exit("Could not find any SUMO configuration file (*.sumocfg).")

arguments = [sumoBinary, "-c", configurationFile, "--start",
             "--quit-on-end=true", "--step-length=" + str(options.step / 1000.0), "--remote-port", str(options.port)]

if options.seed == 0:
    arguments.append("--random")
else:
    arguments.append("--seed=" + str(options.seed))

if options.verbose:
    arguments.append("--verbose")

if os.path.isfile(os.path.join(directory, 'gui-settings.cfg')) and not options.noGUI:
    arguments.extend(["--gui-settings-file", os.path.join(directory, 'gui-settings.cfg')])

if options.sumoArguments != "":
    arguments.extend(options.sumoArguments.split())

sumoProcess = subprocess.Popen(arguments, stdout=FNULL, stderr=subprocess.STDOUT)
controller.run(options.port, options.disableTrafficLights, directory,
               options.step, options.rotateWheels, options.maxVehicles,
               options.radius, options.enableHeight, useDisplay,
               options.displayRefreshRate, options.displayZoom,
               options.displayFitSize, options.maximumLateralSpeed,
               options.maximumAngularSpeed, options.laneChangeDelay, traci, sumolib)
sumoProcess.terminate()

# remove temporary folder
if tmpDirectory is not None:
    print("Removing temporary network files in " + tmpDirectory + "\n")
    shutil.rmtree(tmpDirectory)
