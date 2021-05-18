#!/bin/bash -

# WEBOTS_HOME has to be specified
if [ -z "${WEBOTS_HOME}" ]; then
    WEBOTS_HOME=$(pwd)
fi

VERSION=$(cat ${WEBOTS_HOME}/scripts/packaging/webots_version.txt)

# Dynamic libraries to be copied
DYNAMIC_LIBS="Controller CppController car CppCar driver CppDriver"

# GitHub authorization if not running locally
GITHUB_AUTH=''
if [ ! -z "${GITHUB_ACTOR}" ]; then
    GITHUB_AUTH="${GITHUB_ACTOR}:${GITHUB_TOKEN}@"
fi

# Get the repo
rm -rf /tmp/webots-controller || true
git clone --depth=1 https://${GITHUB_AUTH}github.com/cyberbotics/webots-controller.git /tmp/webots-controller
cd /tmp/webots-controller
git checkout -b ${VERSION}

# Prepare the structure
rm -rf lib/${OSTYPE}
mkdir -p lib/${OSTYPE}
rm -rf include
mkdir -p include

# Copy files
cp -r ${WEBOTS_HOME}/include/controller/* include
for filename in $DYNAMIC_LIBS
do
    echo $filename
    find ${WEBOTS_HOME}/lib/controller -maxdepth 1 -name "*${filename}*" | xargs -I{} cp {} lib/${OSTYPE}
done

# Push
git add -A
git commit -m "Automatic update"
git push origin ${VERSION}
