__author__ = "Rodrigo Queiroz"
__email__ = "rqueiroz@gsd.uwaterloo.ca"

import GSParser
import sys
import glob
import errno

if len(sys.argv) is not 2:
    print("Error: expected a single argument with a GeoScenario file.")
    exit()

path = '/users/rodrigo/WorkspaceLight/scenariosaaa/*/*.osm'
files = glob.glob(path)
#print (files)
for name in files:
	print(name)
	file = name
	GSParser = GSParser.GSParser()
	GSParser.load_and_validate_geoscenario(file)
	GSParser.report.print()
    #try:
    #    with open(name) as f:
    #        pass # do what you want
    #except IOError as exc:
    #    if exc.errno != errno.EISDIR:
    #        raise

#file = sys.argv[1]
#GSParser = GSParser.GSParser()
#GSParser.load_and_validate_geoscenario(file)
#GSParser.report.print()
