#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# VehicleState
# Class to hold configuration settings of the scenario
# --------------------------------------------

from dataclasses import dataclass, field
from typing import Dict

@dataclass
class SimConfig:
    lanelet_routes: Dict = field(default_factory=dict)
