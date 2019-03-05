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

    @grid.setter
    def grid(self, grid):
        # if x < 0:
        #     self.__ = 0
        # elif x > 1000:
        #     self.__x = 1000
        self.__grid = grid


    

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
    @property
    def up(self):
        return self.__up
    @up.setter
    def up(self,up):
        self.__up = up
    @property
    def down(self):
        return self.__down
    @down.setter
    def down(self,down):
        self.__down = down
    @property
    def left(self):
        return self.__left
    @left.setter
    def left(self,left):
        self.__left = left
    @property
    def right(self):
        return self.__right
    @right.setter
    def right(self,right):
        self.__right = right
    @property
    def next_up(self):
        return self.__next_up
    @next_up.setter
    def next_up(self,next_up):
        self.__next_up = next_up
    @property
    def next_down(self):
        return self.__next_down
    @next_down.setter
    def next_down(self,next_down):
        self.__next_down = next_down
    @property
    def next_left(self):
        return self.__next_left
    @next_left.setter
    def next_left(self,next_left):
        self.__next_left = next_left
    @property
    def next_right(self):
        return self.__next_right
    @next_right.setter
    def next_right(self,next_right):
        self.__next_right = next_right

