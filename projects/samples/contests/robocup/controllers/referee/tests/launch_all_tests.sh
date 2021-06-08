#!/bin/bash

PRINT_ONLY=false
if [ $# -gt 0 ]
then
    if [ $1 == "--print-only" ]
    then
        PRINT_ONLY=true
    else
        echo "Usage: $0 [--print-only]"
        exit -1
    fi
fi

script_dir=$(dirname "$(realpath -s "$0")")
source "$script_dir/common.sh"

assert_env_vars

should_run_tests() {
    if [ "$PRINT_ONLY" = true ]; then
        # don't run the test
        return 1
    fi

    # run the test, if no command line option is set
    return 0
}

readarray -d '' TEST_FILES < <(find . -name "test_scenario.json" -print0 | sort --zero-terminated --human-numeric-sort)

TOT_SUCCESS=0
TOT_TESTS=0
NTH_TEST=0

COLOR_RED='\e[0;31m'
COLOR_GREEN='\e[0;32m'
COLOR_RESET='\e[0m'

for test_file in "${TEST_FILES[@]}"
do
    ((NTH_TEST+=1))

    folder=$(dirname ${test_file})
    test_log="${folder}/test.log"
    referee_log="${folder}/referee.log"
    msg_prefix="[$NTH_TEST/${#TEST_FILES[@]}] $folder"

    if should_run_tests
    then
        echo "$msg_prefix ..."
        ./launch_test.sh ${folder} &> ${test_log}
        cp ../log.txt ${referee_log}
    fi
    RESULT_LINE=$(awk '/TEST RESULTS/ { print $3 }' ${test_log})
    NB_SUCCESS=$(echo $RESULT_LINE | awk 'BEGIN { FS = "/" } ; { print $1 }')
    NB_TESTS=$(echo $RESULT_LINE | awk 'BEGIN { FS = "/" } ; { print $2 }')
    if [ $NB_SUCCESS -lt $NB_TESTS ]
    then
        printf "$COLOR_RED$msg_prefix %s %2d/%2d$COLOR_RESET\n" FAIL $NB_SUCCESS $NB_TESTS
    else
        printf "$COLOR_GREEN$msg_prefix %s %2d/%2d$COLOR_RESET\n" PASS $NB_SUCCESS $NB_TESTS
    fi

    ((TOT_SUCCESS+=NB_SUCCESS))
    ((TOT_TESTS+=NB_TESTS))
done

echo "=========================="
echo $(printf "# GLOBAL RESULTS: %3d/%3d #" $TOT_SUCCESS $TOT_TESTS)
echo "=========================="
