echo 'GeoScenario Server test'
echo 'Running all scenarios inside scenarios/test_scenarios/*'
for file in scenarios/test_scenarios/*
do
    echo " - $file"
done
for file in scenarios/test_scenarios/*
do
    
    printf "########### SCENARIO: "
    echo "$file"
    python GSServer.py -s "$file"
done