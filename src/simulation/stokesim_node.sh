#!/usr/bin/env bash
# Bootstrap for starting stokesim via ROS Launch.
# Takes the stokesim config file-path as an argument, either in the form of an
# absolute path, or as the name of a UAV directory within 'unitair/cfg'.
# Note: stokesim-plugin ROS-nodes cannot remap topics because they don't have access to argv.
##################################################

# Make sure that the config argument was given
if [ -z ${1} ]
    then echo "FATAL ($(basename ${0})): stokesim config argument was not provided" && exit 1
fi

# To properly find plugins and such, stokesim should be run from unitair root
cd "$(dirname ${0})/../.."

# Check if the config argument is an absolute path (leading "/")
if [[ ${1} == /* ]]
    # If it is, use that exact config file-path to run stokesim
    then stokesim ${1}
    # Otherwise, use the matching unitair 'cfg' subdirectory
    else stokesim "cfg/${1}/simulation.yaml"
fi
