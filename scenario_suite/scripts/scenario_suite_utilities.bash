#!/bin/bash

# Check if sourced or executed
if [[ ${BASH_SOURCE[0]} = "${0}" ]]; then
    echo "ERROR: This script should be sourced not executed."
    exit
fi

# Checks if the given directory is a suite directory.
is_suite_dir() {
    # $1 - absolute directory path
    [ -d "$1" ] && \
    [ -f "$1/setup.bash" ] && \
    [ -d "$1/scenarios" ] && \
    [ -d "$1/geoscenarioserver" ]
    return $?
}

# Checks if the current working directory is a suite.
cwd_is_suite() {
    is_suite_dir ${PWD}
}

get_scenarios_from_cwd() {
    if cwd_is_suite; then
        get_scenarios_from_suite_dir "$PWD"
        return $?
    fi
    return 1
}

get_scenarios_from_suite_dir() {
    # $1 - absolute suite directory
    local scenarios_dir
    scenarios_dir="$1/scenarios"
    if [[ -d $scenarios_dir ]]; then
        ls -I scenario_template $scenarios_dir
        return 0
    fi
    return 1
}

get_parts_from_scenario_dir() {
    # $1 - absolute scenario directory
    local parts_dir
    parts_dir="$1/parts"
    if [[ -d $parts_dir ]]; then
        ls $parts_dir
        return 0
    fi
    return 1
}

get_maps_from_cwd() {
    get_maps_from_suite_dir "$PWD"
}

get_maps_from_suite_dir() {
    # $1 - absolute suite directory
    if ! is_suite_dir $1; then
        return 1
    fi
    local maps_dir
    maps_dir="$1/maps"
    if [[ ! -d "$maps_dir" ]]; then
        return 1
    else
        ls "$maps_dir"
    fi
}

is_valid_name() {
    # $1 - map or scenario name to validate

    # must start with a letter and can contain letters, numbers, underscores(_), and hyphens(-)
    [[ $1 =~ ^([a-zA-Z][a-zA-Z0-9_-]*)$ ]]
    return $?
}
