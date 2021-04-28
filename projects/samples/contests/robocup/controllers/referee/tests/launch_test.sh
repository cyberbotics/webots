#!/bin/bash


if [ $# -lt 1 ]
then
    echo "Usage $0 <test_path> [--render]"
    exit -1
fi

WEBOTS_OPTIONS="--stdout --stderr --mode=fast"

if [ $# == 2 ]
then
    if [ $2 != "--render" ]
    then
        echo "Unknown option $2"
        exit -1
    fi
else
    WEBOTS_OPTIONS="${WEBOTS_OPTIONS} --no-rendering --minimize"
fi


export PYTHONPATH="${WEBOTS_HOME}/projects/samples/contests/robocup/controllers/referee"  # this should be removed once https://github.com/cyberbotics/webots/issues/3011 is fixed
if [ "$(expr substr $(uname -s) 1 10)" == "MSYS_NT-10" ]; then
  WEBOTS="webots"
else
  WEBOTS="${WEBOTS_HOME}/webots"
fi

TEST_FOLDER="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"

WEBOTS_ROBOCUP_TEST_SCENARIO="${TEST_FOLDER}/test_scenario.json" \
                            WEBOTS_ROBOCUP_GAME="${TEST_FOLDER}/game.json" \
                            $WEBOTS $WEBOTS_OPTIONS
