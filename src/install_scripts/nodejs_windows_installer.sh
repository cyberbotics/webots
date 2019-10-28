#!/bin/bash

# install nodejs in webots/bin/node

NODEJS_VERSION=10.16.3

FOLDER=node-v${NODEJS_VERSION}-win-x64
FILE=${FOLDER}.zip
URL=https://nodejs.org/dist/v${NODEJS_VERSION}/${FILE}

cd ${WEBOTS_HOME}/src/install_scripts
echo ${FILE}
if [ ! -f ./${FILE} ]; then
  wget ${URL}
fi
unzip -q ${FILE} -d ../../bin
rm -rf ../../bin/node
mv ../../bin/${FOLDER} ../../bin/node
