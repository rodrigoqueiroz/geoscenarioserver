echo 'GeoScenario Server test'
echo 'Running all scenarios inside scenarios/nhtsa_scenarios/*'
for file in scenarios/nhtsa_scenarios/*
do
    echo " - $file"
done
for file in scenarios/nhtsa_scenarios/*
do
    
    printf "########### SCENARIO: "
    echo "$file"
    python GSServer.py -s "$file"
done