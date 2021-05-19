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
if [ ! -z "${GITHUB_ACTOR}" ]; then
    git config --global user.name ${GITHUB_ACTOR}
    git config --global user.email ${GITHUB_ACTOR}@github.com
fi
git clone --depth=1 git@github.com:cyberbotics/webots-controller.git /tmp/webots-controller
if [ ! -d /tmp/webots-controller ]; then
    echo 'The repository is not properly cloned'
    exit 1
fi
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
