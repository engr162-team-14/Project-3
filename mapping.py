import brickpi3  
import grovepi 
import math
import time

import numpy as np
from enum import Enum, auto

class State(Enum):
    WALL = auto()
    EXPL = auto()
    UNEXPL = auto()

class Map:
    def __init__(self,):
        self.

class GridSq:
    def __init__(self, up = State.UNEXPL, down = State.EXPL, left = State.WALL, right = State.WALL):
        self.up = up
        self.down = down
        self.left = left
        self.right = right
    def get_up(self):
        return self.up
    def get_down(self):
        return self.down
    def get_left(self):
        return self.left
    def get_right(self):
        return self.right

