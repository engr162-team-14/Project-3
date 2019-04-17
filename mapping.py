import brickpi3  
import grovepi 
import math
import time
import csv

import numpy as np
from enum import Enum

class Dir(Enum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

class State(Enum):
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
        self.grid = np.full((origin[1] + 1,origin[0] + 1),State.UNKWN)
        self.grid[len(self.grid) - origin[1] - 1][origin[0]] = State.ORIG 
        self.map_num = map_num
        self.team = team
        self.unit = unit
        self.hazard_info = [["Hazard Type","Parameter of Interest","Parameter Value","Hazard X Coordinate","Hazard Y Coordinate"]]
        self.cur_loc = origin
        self.cur_direc = direc

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
    def cur_x(self):
        return self.cur_loc[0]

    @property
    def cur_y(self):
        return self.cur_loc[1]

    @property
    def cur_direc(self):
        return self.__cur_direc

    @cur_direc.setter
    def cur_direc(self, cur_direc):
        self.__cur_direc = cur_direc

    def _appendRow(self):
        self.grid = np.insert(self.grid, 0, np.full(len(self.grid[0]),State.UNKWN),axis=0)

    def _appendCol(self):
        for x in range(len(self.grid)):
            self.grid[x] = np.append(self.grid[x],State.UNKWN)

    def _setPoint(self,pt,point_type = State.EXPL):
        if pt[0] < 0 or pt[1] < 0:
            print("Error: Unable to add point with negative coordinates")
        if pt[0] > len(self.grid[0]) - 1:
            self._appendCol()
        if pt[1] > len(self.grid) - 1:
            self._appendRow()
        self.grid[len(self.grid) - pt[1] - 1][pt[0]] = point_type

    def _setPointRelative(self,rel_direc = Dir.UP, point_type = State.CUR):
        pt = []
        if self.cur_direc == Dir.UP:
            if rel_direc == Dir.UP:
                pt = [self.cur_x, self.cur_y + 1]
            elif rel_direc == Dir.DOWN:
                pt = [self.cur_x, self.cur_y - 1]
            elif rel_direc == Dir.LEFT:
                pt = [self.cur_x - 1, self.cur_y]
            elif rel_direc == Dir.RIGHT:
                pt = [self.cur_x + 1, self.cur_y]
        elif self.cur_direc == Dir.DOWN:
            if rel_direc == Dir.UP:
                pt = [self.cur_x, self.cur_y - 1]
            elif rel_direc == Dir.DOWN:
                pt = [self.cur_x, self.cur_y + 1]
            elif rel_direc == Dir.LEFT:
                pt = [self.cur_x + 1, self.cur_y]
            elif rel_direc == Dir.RIGHT:
                pt = [self.cur_x - 1, self.cur_y]
        elif self.cur_direc == Dir.LEFT:
            if rel_direc == Dir.UP:
                pt = [self.cur_x - 1, self.cur_y]
            elif rel_direc == Dir.DOWN:
                pt = [self.cur_x + 1, self.cur_y]
            elif rel_direc == Dir.LEFT:
                pt = [self.cur_x, self.cur_y - 1]
            elif rel_direc == Dir.RIGHT:
                pt = [self.cur_x, self.cur_y + 1]
        elif self.cur_direc == Dir.RIGHT:
            if rel_direc == Dir.UP:
                pt = [self.cur_x + 1, self.cur_y]
            elif rel_direc == Dir.DOWN:
                pt = [self.cur_x - 1, self.cur_y]
            elif rel_direc == Dir.LEFT:
                pt = [self.cur_x, self.cur_y + 1]
            elif rel_direc == Dir.RIGHT:
                pt = [self.cur_x, self.cur_y - 1]
        else:
            print("Error: Direction uninitialized or non-real value")

        self._setPoint(pt,point_type)
        return pt

    def _getPoint(self,pt):
        return self.grid[len(self.grid) - pt[1] - 1][pt[0]]

    def updateLocation(self):
        if self._getPoint(self.cur_loc) != State.ORIG:
            self._setPoint([self.cur_x,self.cur_y],State.EXPL)

        new_cur = self._setPointRelative(Dir.UP, State.CUR)
        self.cur_loc = new_cur

    def _convertMap(self):
        for r in self.grid:
            for c in self.grid:
                self.grid[r][c] = (self.grid[r][c]).value

    def addHazard(self,hazard_type,value):
        hazard_dict = {
            State.HEAT: ["Fire","Temperature (C)"],
            State.MAG: ["Damaged Power Station","Field Strenth (T)"]
        }
        hazard_loc = self._setPointRelative(Dir.UP, hazard_type)

        new_hazard = [hazard_dict[hazard_type][0],hazard_dict[hazard_type][1],value,hazard_loc[0], hazard_loc[1]]
        self.hazard_info.append(new_hazard)

    def pushInfo(self):
        self._setPoint(self.cur_loc,State.EXIT)
        self._convertMap()
        info = {"map": self.grid,
                "hazard table": self.hazard_info 
        }
        with open('map.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(info["map"])

        with open('hazards.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(info["hazard table"])    

        return info