#!/bin/bash

if [ $# -ne 1 ]
then
    echo "Usage $0 <test_path>"
    exit -1
fi

WEBOTS="${WEBOTS_HOME}/webots"
TEST_FOLDER="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"

WEBOTS_ROBOCUP_TEST_SCENARIO="${TEST_FOLDER}/test_scenario.json" \
                            WEBOTS_ROBOCUP_GAME="${TEST_FOLDER}/game.json" \
                            $WEBOTS --stdout --stderr --mode=fast
