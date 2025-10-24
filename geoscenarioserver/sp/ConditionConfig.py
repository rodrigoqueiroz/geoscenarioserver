#!/usr/bin/env python3
#slarter@uwaterloo.ca
# ----------------------------------------------------------------
# CONFIG Data Classes and Constants for Behaviour Tree Conditions
# ----------------------------------------------------------------
from __future__ import annotations  #Must be first Include. Will be standard in Python4
from dataclasses import dataclass
from geoscenarioserver.SimConfig import *
from geoscenarioserver.util.Utils import *
from enum import Enum, IntEnum
import random
import numpy as np
from typing import Dict

class Condition(Enum):
    C_CANCROSSBEFORERED = 1


@dataclass
class CCanCrossBeforeRedConfig():
    ckey:int = Condition.C_CANCROSSBEFORERED
    speed_increase_pct:float = 0.75
    dist_from_xwalk_exit:float = 3
