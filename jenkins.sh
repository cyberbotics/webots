#!/bin/bash

# use the display of the open session
export DISPLAY=:0

# environment variables
export WEBOTS_HOME=$WORKSPACE
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64/
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib
export WEBOTS_DEPENDENCY_PATH=/home/cyberbotics-jenkins/software
export DEPENDENCIES=/home/cyberbotics-jenkins/dependencies

# enable the creation of core dump files
ulimit -c unlimited

# clean the shared memory
./src/bin/clean_shm.sh

# removed untracked files
git clean -fdf  # the second 'f' is for not ignoring submodule folders

# deal with submodules
git submodule deinit -f .
git submodule init
git submodule update

qttargetversion=5.11.0
echo Qt target version: $qttargetversion

# remove and link again the ignored files (otherwise they may be dropped by Jenkins)

# Qt case
export QTDEPENDENCIES=$DEPENDENCIES/qt/$qttargetversion
rm -fr $WEBOTS_HOME/include/qt
rm -fr $WEBOTS_HOME/lib/qt
rm -fr $WEBOTS_HOME/lib/libicu*
rm -fr $WEBOTS_HOME/lib/libQt5*
rm -fr $WEBOTS_HOME/resources/web/local/qwebchannel.js
ln -fs $QTDEPENDENCIES/include/qt $WEBOTS_HOME/include/qt
ln -fs $QTDEPENDENCIES/lib/qt $WEBOTS_HOME/lib/qt
ln -fs $QTDEPENDENCIES/bin/qt/moc $WEBOTS_HOME/bin/qt/
ln -fs $QTDEPENDENCIES/bin/qt/lupdate $WEBOTS_HOME/bin/qt/
ln -fs $QTDEPENDENCIES/bin/qt/lrelease $WEBOTS_HOME/bin/qt/
ln -fs $QTDEPENDENCIES/lib/*.so* $WEBOTS_HOME/lib/
ln -fs $QTDEPENDENCIES/resources/web/local/qwebchannel.js $WEBOTS_HOME/resources/web/local/

# other cases
rm -fr $WEBOTS_HOME/include/libOIS $WEBOTS_HOME/lib/libOIS*
rm -fr $WEBOTS_HOME/include/libpico
rm -fr $WEBOTS_HOME/include/libssh
rm -fr $WEBOTS_HOME/include/libzip
rm -fr $WEBOTS_HOME/include/opencv2
rm -fr $WEBOTS_HOME/projects/default/controllers/ros/include/boost
rm -fr $WEBOTS_HOME/projects/default/controllers/ros/include/geometry_msgs
rm -fr $WEBOTS_HOME/projects/default/controllers/ros/include/log4cxx
rm -fr $WEBOTS_HOME/projects/default/controllers/ros/include/ros
rm -fr $WEBOTS_HOME/projects/default/controllers/ros/include/rosconsole
rm -fr $WEBOTS_HOME/projects/default/controllers/ros/include/rosgraph_msgs
rm -fr $WEBOTS_HOME/projects/default/controllers/ros/include/sensor_msgs
rm -fr $WEBOTS_HOME/projects/default/controllers/ros/include/std_msgs
rm -fr $WEBOTS_HOME/projects/robots/gctronic/e-puck/transfer/xc16
rm -fr $WEBOTS_HOME/resources/lua/modules/gd

ln -fs $DEPENDENCIES/ois/1.4/include/libOIS $WEBOTS_HOME/include/libOIS
ln -fs $DEPENDENCIES/ois/1.4/lib/libOIS.so $WEBOTS_HOME/lib/libOIS.so
ln -fs $DEPENDENCIES/ois/1.4/lib/libOIS-1.4.0.so $WEBOTS_HOME/lib/libOIS-1.4.0.so
ln -fs $DEPENDENCIES/pico/include/libpico $WEBOTS_HOME/include/libpico
ln -fs $DEPENDENCIES/pico/lib/libpico.so $WEBOTS_HOME/lib/libpico.so

ln -fs $DEPENDENCIES/pico/resources/pico $WEBOTS_HOME/resources/pico
ln -fs $DEPENDENCIES/lua-gd/gd $WEBOTS_HOME/resources/lua/modules/gd

ln -fs $WEBOTS_DEPENDENCY_PATH/openal/include $WEBOTS_HOME/include/openal
ln -fs $WEBOTS_DEPENDENCY_PATH/openal/build/libopenal.so.1.16.0 $WEBOTS_HOME/lib/libopenal.so.1
ln -fs $WEBOTS_DEPENDENCY_PATH/openal/build/libopenal.so.1.16.0 $WEBOTS_HOME/lib/libopenal.so
ln -fs $WEBOTS_DEPENDENCY_PATH/opencv-lin64/include/opencv2 $WEBOTS_HOME/include/opencv2
ln -fs $WEBOTS_DEPENDENCY_PATH/opencv-lin64/lib/libopencv_core.so.2.4.3 $WEBOTS_HOME/lib/libopencv_core.so.2.4
ln -fs $WEBOTS_DEPENDENCY_PATH/opencv-lin64/lib/libopencv_core.so.2.4.3 $WEBOTS_HOME/lib/libopencv_core.so.2
ln -fs $WEBOTS_DEPENDENCY_PATH/opencv-lin64/lib/libopencv_core.so.2.4.3 $WEBOTS_HOME/lib/libopencv_core.so
ln -fs $WEBOTS_DEPENDENCY_PATH/opencv-lin64/lib/libopencv_imgproc.so.2.4.3 $WEBOTS_HOME/lib/libopencv_imgproc.so.2.4
ln -fs $WEBOTS_DEPENDENCY_PATH/opencv-lin64/lib/libopencv_imgproc.so.2.4.3 $WEBOTS_HOME/lib/libopencv_imgproc.so.2
ln -fs $WEBOTS_DEPENDENCY_PATH/opencv-lin64/lib/libopencv_imgproc.so.2.4.3 $WEBOTS_HOME/lib/libopencv_imgproc.so

# ROS
ln -fs $DEPENDENCIES/ros/lib/*.so* $WEBOTS_HOME/projects/default/libraries/ros/
ln -fs $DEPENDENCIES/ros/include/Xml*.h $WEBOTS_HOME/projects/default/controllers/ros/include/
ln -fs $DEPENDENCIES/ros/include/boost $WEBOTS_HOME/projects/default/controllers/ros/include/
ln -fs $DEPENDENCIES/ros/include/geometry_msgs $WEBOTS_HOME/projects/default/controllers/ros/include/
ln -fs $DEPENDENCIES/ros/include/log4cxx $WEBOTS_HOME/projects/default/controllers/ros/include/
ln -fs $DEPENDENCIES/ros/include/ros $WEBOTS_HOME/projects/default/controllers/ros/include/
ln -fs $DEPENDENCIES/ros/include/rosconsole $WEBOTS_HOME/projects/default/controllers/ros/include/
ln -fs $DEPENDENCIES/ros/include/rosgraph_msgs $WEBOTS_HOME/projects/default/controllers/ros/include/
ln -fs $DEPENDENCIES/ros/include/sensor_msgs $WEBOTS_HOME/projects/default/controllers/ros/include/
ln -fs $DEPENDENCIES/ros/include/std_msgs $WEBOTS_HOME/projects/default/controllers/ros/include/

# xc16 should be writable (at least for the signature), so a symbolic link is not sufficient
cp -R $DEPENDENCIES/xc16 $WEBOTS_HOME/projects/robots/gctronic/e-puck/transfer/xc16

# sumo
ln -fs $DEPENDENCIES/sumo/*.tar.gz $WEBOTS_HOME/projects/default/resources/

cd $WEBOTS_HOME
# build the path given in argument. In case of failure try to clean and rebuild it
function build {
  path=$1
  tmp=$2[@]
  targets=("${!tmp}")

  echo @@@ Remove $path targets
  for target in "${targets[@]}" ; do
    echo "     - target: $target"
    rm -f $target
  done

  echo @@@ Build $path
  make --silent -j 7 -C $path debug 2>&1 | tee build.log

  are_some_targets_missing=false
  for target in "${targets[@]}" ; do
    if [ ! -f $target ]; then
      echo "Target '$target' not found."
      are_some_targets_missing=true
      break
    fi
  done

  if grep -q 'error:\|Error\|warning\|Stop' build.log || $are_some_targets_missing; then
    echo @@@ Error found when building $path. Try to make clean first.
    make --silent -C $path clean
    make --silent -j 7 -C $path debug 2>&1 | tee build.log
    if grep -q 'error:\|Error\|warning\|Stop' build.log; then
      echo @@@ Error found when building $path.
      grep 'error:\|Error\|warning' build.log
      exit -1
    fi
  fi
}

# the following directories to compile should match with the main Makefile
targets=(lib/libode.so lib/libode.so.3)
build src/ode            targets
targets=(src/glad/libglad.a)
build src/glad           targets
targets=(src/wren/libwren.a)
build src/wren           targets
targets=(webots bin/webots-bin)
build src/webots         targets
targets=(lib/libController.so)
build src/lib/Controller targets
targets=()
build resources          targets
targets=()
build projects           targets

# get doc and HTML robot windows dependencies
$WEBOTS_HOME/docs/local_exporter.py

# run the test suite
cd tests
echo @@@ Run test suite
make clean
python test_suite.py --no-ansi-escape
if [ $? -ne 0 ]; then
  echo "Test suite failed"
  exit -1
fi
if grep -q 'FAILURE' output.txt; then
  exit -1
fi

# remove ros log (if the ros log is big, the startup time for roscore will increase and it may cause time out errors in the test suite)
rm -rf ~/.ros/log

# test ros webots_ros package compilation
TMP_LD_LIBRARY_PATH=$LD_LIBRARY_PATH # get ride of message about no libssl version
unset LD_LIBRARY_PATH
echo @@@ Compile ros webots_ros package
rm -rf $WEBOTS_HOME/webots_catkin_ws
source /opt/ros/kinetic/setup.bash
mkdir -p $WEBOTS_HOME/webots_catkin_ws/src
cd $WEBOTS_HOME/webots_catkin_ws/src
catkin_init_workspace 2>&1 >> /dev/null
cp -r $WEBOTS_HOME/projects/languages/ros/webots_ros webots_ros
cp -r $WEBOTS_HOME/projects/default/controllers/ros/include/srv webots_ros/srv
cp -r $WEBOTS_HOME/projects/default/controllers/ros/include/msg webots_ros/msg
cd $WEBOTS_HOME/webots_catkin_ws
catkin_make 2>&1 >> ros_compilation.log
export LD_LIBRARY_PATH=$TMP_LD_LIBRARY_PATH
if grep -q 'Error' ros_compilation.log; then
  echo @@@ Error: failed to compile ros webots_ros
  grep 'Error' ros_compilation.log
  rm -rf $WEBOTS_HOME/webots_catkin_ws
  exit -1
fi

source $WEBOTS_HOME/webots_catkin_ws/devel/setup.bash

# run the complete test
export ROSCONSOLE_FORMAT='${severity}: ${message}   Line: ${line}'
export ROSCONSOLE_CONFIG_FILE=$WEBOTS_HOME/tests/rosconsole.config
cd $WEBOTS_HOME/tests
echo @@@ Run ros complete test
python ros_test_suite.py 2>&1 | tee ros_node.log
if grep -q 'ERROR' ros_node.log; then
  echo @@@ Error: some tests of the ros complete test have failed
  echo Node log:
  cat ros_node.log
  echo ROS log files:
  ls ~/.ros/log/latest/*.log
  echo ROS log files content:
  cat `ls ~/.ros/log/latest/*.log`
  echo Webots output:
  cat webots_ros.log
  rm -rf $WEBOTS_HOME/webots_catkin_ws
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

# clean ros workspace
rm -rf $WEBOTS_HOME/webots_catkin_ws

echo @@@ Jenkins script done
