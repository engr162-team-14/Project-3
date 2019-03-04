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
    def __init__(self,x_len = 50,y_len = 50):
        self.grid = np.zeros((x_len,y_len))
    
    @property
    def grid(self):
        return self.__grid

    

class GridSq:
    def __init__(self, up = State.UNEXPL, down = State.EXPL, left = State.WALL, right = State.WALL, \
        next_up = None, next_down = None, next_left = None, next_right = None):
        self.up = up
        self.down = down
        self.left = left
        self.right = right
        self.next_up = next_up
        self.next_down = next_down
        self.next_left = next_left
        self.next_right = next_right
    def get_up(self):
        return self.up
    def get_down(self):
        return self.down
    def get_left(self):
        return self.left
    def get_right(self):
        return self.right

