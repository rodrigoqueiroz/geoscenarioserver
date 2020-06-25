__author__ = "Rodrigo Queiroz"
__email__ = "rqueiroz@gsd.uwaterloo.ca"

import GSParser
import sys

if len(sys.argv) is not 2:
    print("Error: expected a single argument with a GeoScenario file.")
    exit()
file = sys.argv[1]
GSParser = GSParser.GSParser()
GSParser.load_and_validate_geoscenario(file)
GSParser.report.print()
