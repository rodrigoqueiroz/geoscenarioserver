#!/bin/bash
# Check if sourced or executed
if [[ ${BASH_SOURCE[0]} == "${0}" ]]; then
    echo "ERROR: This script should be sourced not executed."
    exit
fi
SUITE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCRIPT_DIR="${SUITE_DIR}/scripts"

if [[ -n $1 && $1 == --install ]]; then
    echo "source ${SUITE_DIR}/setup.bash" >> ${HOME}/.bashrc
fi

source ${SCRIPT_DIR}/scenario_suite_utilities.bash

_slaunch()
{
    local opts
    gss_opts="--no-dash --wait-for-input --wait-for-client --dash-pos --debug --file-log --write-trajectories --origin-from-vid --ros"
    opts=""
    case $COMP_CWORD in
        1)
            # autocomplete scenario names
            if cwd_is_suite; then
                opts="$opts $(get_scenarios_from_cwd) $gss_opts"
            fi
            ;;
        *)
            # autocomplete part names
            SCENARIO_NAME="${COMP_WORDS[1]}"
            SCENARIO_DIR="${SUITE_DIR}/scenarios/${SCENARIO_NAME}"
            # Scenario name must match the folder name
            if [ -d  "${SCENARIO_DIR}" ] && \
               [ -f  "${SCENARIO_DIR}/${SCENARIO_NAME}.osm" ]; then
                opts=$(get_parts_from_scenario_dir "${SCENARIO_DIR}")
                # remove parts that are already used from all parts
                for (( prev_comp=2; prev_comp<COMP_CWORD; prev_comp++ )); do
                    prev_part="${COMP_WORDS[prev_comp]}"
                    # if the previous part is a valid part file, remove it from the options
                    if [ -f "${SCENARIO_DIR}/parts/${prev_part}" ]; then
                        opts=${opts//${prev_part}}
                    fi
                done
                opts="$opts $gss_opts"
            fi
            ;;
    esac
    COMPREPLY=( $(compgen -W "${opts}" -- "${COMP_WORDS[COMP_CWORD]}") )
    return 0
}
alias slaunch="bash ${SUITE_DIR}/scripts/launch.bash"
complete -F _slaunch slaunch
