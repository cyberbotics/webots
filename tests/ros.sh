#!/bin/bash

echo @@@ Compile ros webots_ros package
source /opt/ros/$ROS_DISTRO/setup.bash
[ -d $WEBOTS_HOME/webots_catkin_ws ] && rm -r $WEBOTS_HOME/webots_catkin_ws
mkdir -p $WEBOTS_HOME/webots_catkin_ws/src
cd $WEBOTS_HOME/webots_catkin_ws/src
catkin_init_workspace 2>&1 >> /dev/null
cp -r $WEBOTS_HOME/resources/webots_ros webots_ros
cp -r $WEBOTS_HOME/projects/robots/universal_robots/resources/ros_package/ur_e_webots ur_e_webots
cd $WEBOTS_HOME/webots_catkin_ws
echo @@@ Installing dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
echo @@@ Compiling package
catkin_make 2>&1 >> ros_compilation.log
export LD_LIBRARY_PATH=$TMP_LD_LIBRARY_PATH
if grep -q 'Error' ros_compilation.log; then
  echo @@@ Error: failed to compile ros webots_ros
  grep 'Error' ros_compilation.log
  exit -1
fi

source $WEBOTS_HOME/webots_catkin_ws/devel/setup.bash

# run the complete test
export ROSCONSOLE_FORMAT='${severity}: ${message}   Line: ${line}'
export ROSCONSOLE_CONFIG_FILE=$WEBOTS_HOME/tests/rosconsole.config
cd $WEBOTS_HOME/tests
echo @@@ Run ros complete test
roslaunch webots_ros complete_test.launch auto-close:=true no-gui:=true 2> stderr.log
if grep 'ERROR' stderr.log | grep -q -v 'ERROR: Cannot initialize the sound engine'; then
  echo @@@ Error: some tests of the ros complete test have failed
  cat stderr.log
  exit -1
fi

# checks that all the service messages specific to webots_ros are available
cd $WEBOTS_HOME/webots_catkin_ws
source devel/setup.bash
rossrv list >> available_services.log
echo @@@ Checking that all webots_ros services are available
# get all the service files
FILES=src/webots_ros/srv/*.srv
if [ ${#FILES[@]} -gt 1 ]; then
  echo @@@ Error: no service file found
  rm -rf $WEBOTS_HOME/webots_catkin_ws
  exit -1
fi
# check that each associated service is found
missing_service_file=false
for f in $FILES
do
  SERVICE_NAME=$(echo $f | cut -d '/' -f 4)
  SERVICE_NAME=$(echo $SERVICE_NAME | cut -d '.' -f 1)
  if ! grep -q webots_ros/$SERVICE_NAME available_services.log; then
    echo @@@ Error: service webots_ros/$SERVICE_NAME not found
  fi
done
if $missing_service_file ; then
  rm -rf $WEBOTS_HOME/webots_catkin_ws
  exit -1
else
  echo "OK: all service files found"
fi
