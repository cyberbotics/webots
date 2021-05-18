#!/bin/bash -

VERSION=$(cat ${WEBOTS_HOME}/scripts/packaging/webots_version.txt)
FILES="Controller CppController car CppCar driver CppDriver"
if [ -z "${WEBOTS_HOME}" ]; then
    echo 'The WEBOTS_HOME environment variable has to be defined'
    exit 1
fi


# Get the repo
rm -rf /tmp/webots-controller || true
git clone --depth=1 https://github.com/cyberbotics/webots-controller.git /tmp/webots-controller
cd /tmp/webots-controller
git checkout -b ${VERSION}

# Prepare the structure
mkdir -p lib/${OSTYPE}
mkdir -p include

# Copy files
cp -r ${WEBOTS_HOME}/include/controller/* include
for filename in $FILES
do
    echo $filename
    find ${WEBOTS_HOME}/lib/controller -maxdepth 1 -name "*${filename}*" | xargs -I{} cp {} lib/${OSTYPE}
done
