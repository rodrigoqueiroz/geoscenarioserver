#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")

ARG_SINGLE="false"
ARG_VEHICLES="true"
ARG_PEDESTRIANS="true"
ARG_LONG="false"
ARG_NO_DASH=
ARG_INTERACTIVE="true"
SCENARIO_NAME=

print_help() {
    echo "Running all vehicle and pedestrian scenarios except long scenarios by default."
    echo "  --single            run only the given scenario"
    echo "  --op                run only pedestrian scenarios"
    echo "  --ov                run only vehicle scenarios"
    echo "  --long              include long vehicle scenarios (not included by default)"
    echo "  --no-dash           run without the dashboard."
    echo "  --non-interactive   do not prompt for <enter> (prompt by default)"
    echo ""
}

if [[ "$#" -eq 0 ]]; then
    print_help
else
    for arg in "$@"; do
        case $arg in
            "--op")
                ARG_VEHICLES="false"
                ;;
            "--ov")
                ARG_PEDESTRIANS="false"
                ;;
            "--long")
                ARG_LONG="true"
                ;;
            "--no-dash")
                ARG_NO_DASH="--no-dash"
                ;;
            "--non-interactive")
                ARG_INTERACTIVE="false"
                ;;
            "--single")
                ARG_SINGLE="true"
                ;;
            *)
                if [[ $ARG_SINGLE == "true" && -v SCENARIO_NAME ]]; then
                    SCENARIO_NAME=$arg
                else
                    echo "Invalid argument $arg"
                    echo ""
                    echo "Usage: "
                    echo ""
                    print_help
                    exit 1
                fi
                ;;
        esac
    done
fi

kill_python3()
{
    killall python3
}

run_scenario_save_regression()
{
    scenario=$1
    pixi run gss ${ARG_NO_DASH} --scenario ${scenario}
    # save and compare with regression
    scenario_relative=${scenario#$REPO_DIR/scenarios/}
    regression_folder=${REPO_DIR}/outputs/regressions/${scenario_relative}/
    save="no"
    if [[ -f "${regression_folder}violations.json" ]]; then
        echo ""
        echo "=== diff violations.json for \"$(basename ${scenario})\" ==="
        diff <(jq --sort-keys . ${REPO_DIR}/outputs/violations.json) <(jq --sort-keys . ${regression_folder}violations.json) 
        if [[ $? -gt 0 ]]; then
            ((regression_failures++))
            save="yes"
        fi
        echo ""
        echo "==="
    else
        echo ""
        echo "No regression file found for \"$(basename ${scenario})\""
        mkdir -p ${regression_folder}
        echo "=== violations.json for \"$(basename ${scenario})\" ==="
        cat ${REPO_DIR}/outputs/violations.json
        echo ""
        echo "==="
        save="yes"
    fi
    # save the output for regression testing
    # missing output indicates a problem with the scenario or GSS failure
    if [[ "${save}" == "yes" ]]; then
        mv -f ${REPO_DIR}/outputs/violations.json ${regression_folder}
        ((gss_failures+=$?))  # in case there was no violations.json at all
    fi
}

if [[ $ARG_SINGLE == "true" ]]; then
    run_scenario_save_regression $(realpath ${SCENARIO_NAME})
else
    # run all scenarios
    test_scenarios=""
    if [[ "$ARG_VEHICLES" == "true" ]]; then
        coretest_scenarios=$(find "${REPO_DIR}/scenarios/coretest_scenarios" -name "*.osm" | sort)
        test_scenarios=$(find "${REPO_DIR}/scenarios/test_scenarios" -name "*.osm" | sort)
    fi
    pedestrian_scenarios=""
    if [[ "$ARG_PEDESTRIANS" == "true" ]]; then
        pedestrian_scenarios=$(find "${REPO_DIR}/scenarios/pedestrian_scenarios" -name "*.osm" | sort)
    fi
    long_test_scenarios=
    if [[ "$ARG_LONG" == "true" ]]; then
        long_test_scenarios=$(find "${REPO_DIR}/scenarios/long_test_scenarios" -name "*.osm" | sort)
    fi
    all_scenarios="$coretest_scenarios $test_scenarios $pedestrian_scenarios $long_test_scenarios"
    cd ${REPO_DIR}
    regression_failures=0
    gss_failures=0
    for scenario in $all_scenarios; do
        echo "CTRL + C to exit the script."
        if [[ "$ARG_INTERACTIVE" == "true" ]]; then
            read -p "ENTER to run ${scenario#$REPO_DIR/}:"
        fi
        trap kill_python3 SIGINT
        echo "CTRL + C to quit the scenario."
        run_scenario_save_regression $scenario
        echo "Regression failures so far: ${regression_failures}"
        echo "GSS failures so far: ${gss_failures}"
        rm -f ${REPO_DIR}/outputs/*.png
        trap - SIGINT
    done

    exitcode=0
    if [[ ${regression_failures} -gt 0 ]]; then
        echo "Regression failures: ${regression_failures}"
        exitcode=1
    fi
    if [[ ${gss_failures} -gt 0 ]]; then
        echo "GSS failures: ${gss_failures}"
        exitcode=1
    fi
    exit ${exitcode}
fi
