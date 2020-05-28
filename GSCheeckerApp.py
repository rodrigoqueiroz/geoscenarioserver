__author__ = "Rodrigo Queiroz"
__email__ = "rqueiroz@gsd.uwaterloo.ca"

import GSChecker
import sys

if len(sys.argv) is not 2:
    print("Error: expected a single argument with a GeoScenario file.")
    exit()
file = sys.argv[1]
gschecker = GSChecker.GSChecker()
gschecker.validate_geoscenario(file)
gschecker.report.print()