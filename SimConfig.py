#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# VehicleState
# Class to hold configuration settings of the scenario
# --------------------------------------------

from dataclasses import dataclass, field
from typing import Dict, List

@dataclass
class SimConfig:
    lanelet_routes: Dict[int, List[int]] = field(default_factory=dict)
