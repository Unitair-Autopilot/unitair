#!/usr/bin/env bash
# Runs all the software-only tests sequentially.
# Requires the cfg path as an argument.
##################################################

# Make sure that the cfg path argument was given
if [ -z ${1} ]
    then echo "FATAL ($(basename ${0})): unitair cfg path argument was not provided" && exit 1
fi

# Run tests, return sum of error codes
let failures=0
rosrun unitair tools_test                            ; let failures=${failures}+${?}
rosrun unitair planner_test   "${1}/guidance.yaml"   ; let failures=${failures}+${?}
rosrun unitair regulator_test "${1}/control.yaml"    ; let failures=${failures}+${?}
rosrun unitair allocator_test "${1}/control.yaml"    ; let failures=${failures}+${?}
rosrun unitair estimator_test "${1}/navigation.yaml" ; let failures=${failures}+${?}
exit ${failures}
