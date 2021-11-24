#!/bin/bash -

# WEBOTS_HOME has to be specified
if [ -z "${WEBOTS_HOME}" ]; then
    WEBOTS_HOME=$(pwd)
fi

VERSION=$(cat ${WEBOTS_HOME}/scripts/packaging/webots_version.txt | sed 's/ /-/g')

# Dynamic libraries to be copied
DYNAMIC_LIBS="Controller CppController car CppCar driver CppDriver"

# Don't publish the libcontroller if it hasn't changed since yesterday
LAST_COMMIT_YESTERDAY=$(git log -1 --pretty=format:"%H" --before=yesterday)
LAST_COMMIT=$(git log -1 --pretty=format:"%H")
INCLUDE_DIFF_SINCE_YESTERDAY=$(git diff ${LAST_COMMIT_YESTERDAY}..${LAST_COMMIT} -- include/controller)
SOURCE_DIFF_SINCE_YESTERDAY=$(git diff ${LAST_COMMIT_YESTERDAY}..${LAST_COMMIT} -- src/controller)
VEHICLE_DIFF_SINCE_YESTERDAY=$(git diff ${LAST_COMMIT_YESTERDAY}..${LAST_COMMIT} -- projects/default/librairies/vehicle)
if [ -z "${INCLUDE_DIFF_SINCE_YESTERDAY}" ] && [ -z "${SOURCE_DIFF_SINCE_YESTERDAY}" ] && [ -z "${VEHICLE_DIFF_SINCE_YESTERDAY}" ]; then
    echo "There are no changes in 'include/controller', 'src/controller' and 'projects/default/librairies/vehicle' since yesterday"
    echo "Last commit yesterday: ${LAST_COMMIT_YESTERDAY}"
    echo "Last commit today: ${LAST_COMMIT}"
    exit 0
fi

# Get the repo
rm -rf /tmp/webots-controller || true
if [ ! -z "${GITHUB_ACTOR}" ]; then
    git config --global user.name ${GITHUB_ACTOR}
    git config --global user.email ${GITHUB_ACTOR}@github.com
fi
git clone git@github.com:cyberbotics/webots-libcontroller.git /tmp/webots-controller
if [ ! -d /tmp/webots-controller ]; then
    echo 'The repository is not properly cloned'
    exit 1
fi
cd /tmp/webots-controller

if [ ! -z $(git branch -a | egrep "/${VERSION}$") ]; then
    echo "Checkout to the existing branch ${VERSION}"
    git checkout ${VERSION}
else
    echo "Create a new branch ${VERSION}"
    git checkout -b ${VERSION}
fi

# Copy headers and C++ source code
if [ "${OSTYPE}" != "msys" ]; then
    # don't copy the include files on Windows as they are the same as on other platforms, and they generate a huge diff due to Windows line endings which differ from Linux/macOS.
    rm -rf include
    mkdir -p include
    cp -r ${WEBOTS_HOME}/include/controller/* include
    cp ${WEBOTS_HOME}/include/controller/c/webots/plugins/robot_window/{robot_window.h,robot_wwi.h} include
    
    rm -rf source/cpp
    mkdir -p source/cpp/vehicle
    cp ${WEBOTS_HOME}/src/controller/cpp/*.cpp source/cpp
    cp ${WEBOTS_HOME}/projects/default/libraries/vehicle/cpp/car/src/Car.cpp source/cpp/vehicle
    cp ${WEBOTS_HOME}/projects/default/libraries/vehicle/cpp/driver/src/Driver.cpp source/cpp/vehicle
fi

# Copy dynamic libs
rm -rf lib/${OSTYPE}
mkdir -p lib/${OSTYPE}
for filename in $DYNAMIC_LIBS; do
    echo $filename
    find ${WEBOTS_HOME}/lib/controller -maxdepth 1 -name "*${filename}*" | xargs -I{} cp {} lib/${OSTYPE}
done

# Copy Python libs
PYTHON_DIRECTORIES=$(find ${WEBOTS_HOME}/lib/controller/python3* -maxdepth 0 -type d)
for dirname in ${PYTHON_DIRECTORIES}; do
    echo $dirname
    touch ${dirname}/__init__.py
    cp -r ${dirname} lib/${OSTYPE}
done

# Push
git add -A
git commit -m "Automatic update"
git push origin ${VERSION}
