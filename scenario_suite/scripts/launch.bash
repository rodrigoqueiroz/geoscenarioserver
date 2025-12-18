#!/bin/bash
set -e

# get absolute directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BASE_SUITE_DIR="$( dirname $SCRIPT_DIR )"

source $SCRIPT_DIR/scenario_suite_utilities.bash

if cwd_is_suite; then
    suite_dir=$(pwd)
else
    suite_dir=${BASE_SUITE_DIR}
fi

if ! which -s pixi; then
    echo "Pixi not installed or not activated."
    echo "Follow the instructions for installing pixi at https://pixi.sh/latest/#installation"
    echo "or execute the following and re-open the terminal:"
    echo "   curl -fsSL https://pixi.sh/install.sh | bash"
    exit 1
fi

print_help() {
    echo ""
    echo "Usage: slaunch <scenario_name> [<part_name>.osm*] [--ros] [--mock-co-sim] [<gss_options>*]"
    echo ""
    echo "Launches the specified scenario with the GeoScenarioServer traffic simulator"
    if [[ -L ${suite_dir}/geoscenarioserver ]]; then
        echo "located via a symlink ${suite_dir}/geoscenarioserver, currently set to"
        echo ""
        readlink -e "${suite_dir}/geoscenarioserver"
    else
        echo "located at ${suite_dir}/geoscenarioserver."
    fi
    echo ""
    echo "Arguments:"
    echo "     <scenario_name>       name of the folder 'scenarios/<scenario_name>' (mandatory)"
    echo "     [<part_name>.osm*]    names of the files in the folder 'scenarios/<scenario_name>/parts' (optional list)"
    echo "     [--ros]               launch ROS2 node geoscenario_server (launch standalone by default)"
    echo "     [--mock-co-sim <real_time_factor>]"
    echo "                           launch mock_co_simulator with delta_time=0.025 and the given real_time_factor (only valid with --ros)"
    echo "                           real_time_factor: 0: max speed, (0..1): faster than real time, 1: real time, >1 = slower than real time"
    echo "     [<gss_options>*]      additional options for the GeoScenarioServer (optional list):"
    echo "                           --no-dash --wait-for-input --wait-for-client --dash-pos --debug --file-log --write-trajectories --origin-from-vid"
    echo ""
    echo "The following scenarios are available in the suite ${suite_dir}:"
    echo ""
    echo "$(get_scenarios_from_suite_dir ${suite_dir})"
    echo ""
    echo "The scenario with the given name <scenario_name> is located in" 
    echo "$suite_dir/scenarios/<scenario_name>/<scenario_name>.osm"
    echo ""
    echo "Scenario parts are located in"
    echo "$suite_dir/scenarios/<scenario_name>/parts/<part_name>.osm"
    echo ""
    echo "To enable the command 'slaunch' execute"
    echo ""
    echo "source <suite_dir>/setup.bash"
    echo ""
}

# if no arguments were provided, print the help (without showing the scenarios) and exit
if [[ $# = 0 ]]; then
    print_help
    exit 1
fi

# by default, assume the first argument is the scenario name
scenario_name=$1
shift

# the remaining arguments are the part names or gss options
args=("$@")
full_parts_list=""
gss_options=""
ros=
mock_co_sim=
ros_full_parts_list=()
ros_gss_options=()
number_is_for=
dash_pos_x=
dash_pos_y=
dash_pos_w=

for arg in ${args[@]}; do
    part_path="${suite_dir}/scenarios/${scenario_name}/parts/${arg}"
    if [ -f "$part_path" ]; then
        full_parts_list="${full_parts_list} ${part_path}"
        ros_full_parts_list+=("${part_path}")
    else
        case $arg in
            --ros)
                ros="true"
                ;;
            --mock-co-sim)
                number_is_for="mock-co-sim"
                ;;
            --no-dash)
                gss_options="${gss_options} ${arg}"
                ros_gss_options+=("no_dashboard: true")
                ;;
            --write-trajectories)
                gss_options="${gss_options} ${arg}"
                ros_gss_options+=("write_trajectories: true")
                ;;
            --origin-from-vid)
                gss_options="${gss_options} ${arg}"
                number_is_for="origin-from-vid"
                ;;
            --dash-pos)
                gss_options="${gss_options} ${arg}"
                number_is_for="dash-pos"
                ;;
            --wait-for-input|--wait-for-client|--debug|--file-log)
                gss_options="${gss_options} ${arg}"
                # not applicable for ROS2 server
                ;;
            [0-9]*)
                gss_options="${gss_options} ${arg}"
                # determine for which parameter this number is intended
                if [[ ${number_is_for} == "origin-from-vid" ]]; then
                    ros_gss_options+=("origin_from_vid: ${arg}")
                    number_is_for=
                elif [[ ${number_is_for} == "mock-co-sim" ]]; then
                    mock_co_sim=("-p real_time_factor:=${arg}")
                    number_is_for=
                elif [[ ${number_is_for} == "dash-pos" ]] && [[ -z ${dash_pos_x} ]]; then
                    dash_pos_x=${arg}
                elif [[ ${number_is_for} == "dash-pos" ]] && [[ -z ${dash_pos_y} ]]; then
                    dash_pos_y=${arg}
                elif [[ ${number_is_for} == "dash-pos" ]] && [[ -z ${dash_pos_w} ]]; then
                    dash_pos_w=${arg}
                elif [[ ${number_is_for} == "dash-pos" ]]; then
                    ros_gss_options+=("dashboard_position: [${dash_pos_x}, ${dash_pos_y}, ${dash_pos_w}, ${arg}]")
                    number_is_for=
                fi
                ;;
            *)
                echo "Argument '$arg' is not a valid part file, allowed GSS option, or integer value."
                exit 1
                ;;
        esac
    fi
    shift
done
scenarios_dir="$suite_dir/scenarios"

# check if the given scenario exists and is not the template
if [[ ! -d "$scenarios_dir/$scenario_name" || "$scenario_name" == "scenario_template" ]]; then
    echo ""
    echo "Invalid scenario name."
    print_help
    exit 1
fi

timestamp=$(date +"%Y-%m-%d-%H%M%S")
mkdir -p "$suite_dir/logs/$timestamp"


###############################
## Launch GeoScenario Server ##
###############################

# Example ROS2 server launch_command.log:

# pixi run ros_server --ros-args --params-file /<GSS_path>/scenario_suite/logs/2025-12-18-084927/launch_params.yaml

# Example launch_params.yaml:

# geoscenario_server:
#   ros__parameters:
#     map_path: /<GSS_path>/scenario_suite
#     scenario_files:
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/colliding_pedestrians.osm
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/parts/front-sp1.osm
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/parts/left-sp3.osm
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/parts/right-sp2.osm
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/parts/jaywalker-pp4-const.osm
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/parts/jaywalker-pp5-profile.osm
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/parts/occluding-pv10-pv20.osm
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/parts/occluding-pv30-pv40.osm
#       - /<GSS_path>/scenario_suite/scenarios/colliding_pedestrians/parts/vut_sdv1-reverse.osm
#     write_trajectories: true
#     origin_from_vid: 1
#     dashboard_position: [100, 200, 800, 900]

gss_scenario_dir="${scenarios_dir}/${scenario_name}"
gss_scenario_file="${gss_scenario_dir}/${scenario_name}.osm"
if [[ -f "${gss_scenario_file}" ]]; then
    echo "Starting GeoScenario server for ${gss_scenario_file}..."
    cd ${suite_dir}/geoscenarioserver/
    scenario="--scenario ${gss_scenario_file} ${full_parts_list}"
    map_path="--map-path ${suite_dir}"
    if [[ -d "${suite_dir}/btrees" ]]; then
        btree_locations="--btree-locations ${suite_dir}/btrees"
        ros_btree_locations="btree_locations: ${suite_dir}/btrees"
    else
        btree_locations=""
        ros_btree_locations=""
    fi
    gss_command_log="${suite_dir}/logs/${timestamp}/launch_command.log"
    mock_co_sim_command_log="${suite_dir}/logs/${timestamp}/mock_co_sim_command.log"
    gss_output_log="${suite_dir}/logs/${timestamp}/gss_output.log"
    mock_co_sim_output_log="${suite_dir}/logs/${timestamp}/mock_co_sim_output.log"
    export GSS_OUTPUTS="${suite_dir}/logs/${timestamp}"
    if [[ ${ros} == "true" ]]; then
        echo "Launching ROS2 GeoScenario server..."
        gss_launch_params="${suite_dir}/logs/${timestamp}/launch_params.yaml"
cat << END_YAML > ${gss_launch_params}
geoscenario_server:
  ros__parameters:
    map_path: ${suite_dir}
    scenario_files:
      - ${gss_scenario_file}
END_YAML
        for part in "${ros_full_parts_list[@]}"; do
            echo "      - ${part}" >> ${gss_launch_params}
        done
        if [[ -n ${ros_btree_locations} ]]; then
            echo "    ${ros_btree_locations}" >> ${gss_launch_params}
        fi
        if [[ -n ${ros_gss_options} ]]; then
            for option in "${ros_gss_options[@]}"; do
                echo "    ${option}" >> ${gss_launch_params}
            done
        fi
        ros_gss_command="pixi run ros_server --ros-args --params-file ${gss_launch_params}"
        if [[ -n ${mock_co_sim} ]]; then
            mock_co_sim_command="pixi run ros_mock_co_simulator --ros-args -p target_delta_time:=0.025 ${mock_co_sim}"
            echo "${mock_co_sim_command}" > "${mock_co_sim_command_log}"
            ${mock_co_sim_command} 2>&1 | tee "${mock_co_sim_output_log}" &
        fi
        # log the command for debugging
        echo "${ros_gss_command}" >> "${gss_command_log}"
        ${ros_gss_command} 2>&1 | tee "${gss_output_log}"
    else
        echo "Launching standalone GeoScenario server..."
        gss_command="pixi run gss ${scenario} ${map_path} ${btree_locations} ${gss_options}"
        # log the command for debugging
        echo "${gss_command}" > "${gss_command_log}"
        ${gss_command} 2>&1 | tee "${gss_output_log}"
    fi
elif  [[ -d ${gss_scenario_dir} ]]; then
    echo "No GeoScenario Server scenario file in ${gss_scenario_dir}."
fi

