# Regression testing

For a scenario `scenarios/<folder>/<file>.osm`, this folder contains the intended outputs of running the scenario in `outputs/regressions/<folder>/<file>.osm/`. Currently, each folder contains `violations.json`.

# Testing workflow

1. Run all test scenarios:
```
cd geoscenarioserver
bash scripts/run_scenarios.bash --no-dash --non-interactive
```

General usage:
```
bash scripts/run_scenarios.bash --help
Usage: 

Running all vehicle and pedestrian scenarios except long scenarios by default.
  --op                run only pedestrian scenarios
  --ov                run only vehicle scenarios
  --long              include long vehicle scenarios (not included by default)
  --no-dash           run without the dashboard.
  --non-interactive   do not prompt for <enter> (prompt by default)
```


2. Inspect the console output after each scenario.

a) A difference detected, for example
```
=== diff violations.json for "<scenario name>.osm" ===
2c2,4
<   "1": {},
---
>   "1": {
>     "some": "difference"
>   },

===
```

b) No difference
```
=== diff violations.json for "<scenario name>.osm" ===

===
```

3. The script moves the new `violations.json` to the scenario's regression folder only if a diffence was detected. 
Use `git status` and `git diff` to view the differences.

a) Discard the change if not desired using `git checkout`
b) Commit the changed file as a regression test
