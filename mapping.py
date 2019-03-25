import brickpi3  
import grovepi 
import math
import time

import numpy as np
from enum import Enum

class Dir(Enum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3

class State(Enum):
    UNEXPL = -1
    UNKWN = 0
    EXPL = 1
    ORIG = 5
    HEAT = 2
    MAG = 3
    EXIT = 4
    CUR = 1234

class Unit(Enum):
    CM = 0
    IN = 1

class Map:
    def __init__(self, origin = [0,0], map_num = 0, team = 14, unit = Unit.CM, direc = Dir.UP):
        self.origin = origin
        self.grid = np.full((origin[1] + 1,origin[0] + 1),State.UNKWN)
        self.grid[-origin[1] - 1][origin[0]] = State.ORIG
        self.map_num = map_num
        self.team = team
        self.unit = unit
        self.hazard_info = [["Hazard Type","Parameter of Interest","Parameter Value","Hazard X Coordinate","Hazard Y Coordinate"]]
        self.cur_loc = origin
        self.cur_direc = direc
    @property
    def origin(self):
        return self.__origin
    @origin.setter
    def origin(self, origin):
        self.__origin = origin
    @property
    def grid(self):
        return self.__grid
    @grid.setter
    def grid(self, grid):
        self.__grid = grid
    @property
    def map_num(self):
        return self.__map_num
    @map_num.setter
    def map_num(self, map_num):
        self.__map_num = map_num
    @property
    def team(self):
        return self.__team
    @team.setter
    def team(self, team):
        self.__team = team
    @property
    def unit(self):
        return self.__unit
    @unit.setter
    def unit(self, unit):
        self.__unit = unit
    @property
    def hazard_info(self):
        return self.__hazard_info
    @hazard_info.setter
    def hazard_info(self, hazard_info):
        self.__hazard_info = hazard_info
    @property
    def cur_loc(self):
        return self.__cur_loc
    @cur_loc.setter
    def cur_loc(self, cur_loc):
        self.__cur_loc = cur_loc
    @property
    def cur_direc(self):
        return self.__cur_direc
    @cur_direc.setter
    def cur_direc(self, cur_direc):
        self.__cur_direc = cur_direc
    def appendRow(self):
        np.insert(self.grid, 0, np.full(len(self.grid[0]),State.UNKWN),axis=0)
    def appendCol(self):
        for r in self.grid:
            np.append(r,State.UNKWN)
    def addPoint(self,pt,point_type = State.EXPL):
        if pt[0] > len(self.grid[0]):
            self.appendCol()
        elif pt[1] > len(self.grid):
            self.appendRow()
        self.grid[-pt[1] - 1][pt[0]] = point_type
    def updateLocation(self):
        pass
    def findNearestUnexp(self,):
        pass
    def _convertMap(self):
        return None
    def addHazard(self,haz_type,value,loc):
        hazard_dict = {
            State.HEAT: [" "," "],
            State.MAG: [" "," "]
        }
        new_hazard = [hazard_dict[haz_type][0],hazard_dict[haz_type][1],value,loc[0],loc[1]]
        self.hazard_info.append(new_hazard)
    def pushInfo(self,):
        info = {"map": self._convertMap(),
                "hazard table": self.hazard_info 
        }
        return info