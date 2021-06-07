#!/bin/bash

RUN_TESTS=true
if [ $# -gt 0 ]
then
    if [ $1 == "--print-only" ]
    then
        RUN_TESTS=false
    else
        echo "Usage: $0 [--print-only]"
        exit -1
    fi
fi

readarray -d '' TEST_FILES < <(find . -name "test_scenario.json" -print0 | sort --zero-terminated --human-numeric-sort)

TOT_SUCCESS=0
TOT_TESTS=0
NTH_TEST=0

for test_file in "${TEST_FILES[@]}"
do
    ((NTH_TEST+=1))

    folder=$(dirname ${test_file})
    test_log="${folder}/test.log"
    referee_log="${folder}/referee.log"
    msg_prefix="[$NTH_TEST/${#TEST_FILES[@]}] $folder"

    if [ $RUN_TESTS = true ]
    then
        echo "$msg_prefix ..."
        ./launch_test.sh ${folder} &> ${test_log}
        cp ../log.txt ${referee_log}
    fi
    RESULT_LINE=$(awk '/TEST RESULTS/ { print $3 }' ${test_log})
    NB_SUCCESS=$(echo $RESULT_LINE | awk 'BEGIN { FS = "/" } ; { print $1 }')
    NB_TESTS=$(echo $RESULT_LINE | awk 'BEGIN { FS = "/" } ; { print $2 }')
    RESULT="PASS"
    if [ $NB_SUCCESS -lt $NB_TESTS ]
    then
        RESULT="FAIL"
    fi
    printf "$msg_prefix %s %2d/%2d\n" $RESULT $NB_SUCCESS $NB_TESTS

    ((TOT_SUCCESS+=NB_SUCCESS))
    ((TOT_TESTS+=NB_TESTS))
done

echo "=========================="
echo $(printf "# GLOBAL RESULTS: %3d/%3d #" $TOT_SUCCESS $TOT_TESTS)
echo "=========================="
