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
        self.junc_instr = []
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
    @property
    def junc_instr(self):
        return self.__junc_instr
    @junc_instr.setter
    def junc_instr(self, junc_instr):
        self.__junc_instr = junc_instr
    def _appendRow(self):
        self.grid = np.insert(self.grid, 0, np.full(len(self.grid[0]),State.UNKWN),axis=0)
    def _appendCol(self):
        for x in range(len(self.grid)):
            self.grid[x] = np.append(self.grid[x],State.UNKWN)
    def _addPoint(self,pt,point_type = State.EXPL):
        if pt[0] < 0 or pt[1] < 0:
            print("Error: Unable to add point with negative coordinates")
        if pt[0] > len(self.grid[0]) - 1:
            self._appendCol()
        if pt[1] > len(self.grid) - 1:
            self._appendRow()
        self.grid[len(self.grid) - pt[1] - 1][pt[0]] = point_type
    def updateLocation(self):
        self._addPoint([self.cur_x,self.cur_y],State.EXPL)
        if self.cur_direc == Dir.UP:
            self._addPoint([self.cur_x,self.cur_y + 1],State.CUR)
            self.cur_loc = [self.cur_x,self.cur_y + 1]
        elif self.cur_direc == Dir.DOWN:
            self._addPoint([self.cur_x,self.cur_y - 1],State.CUR)
            self.cur_loc = [self.cur_x,self.cur_y - 1] 
        elif self.cur_direc == Dir.LEFT:
            self._addPoint([self.cur_x - 1,self.cur_y],State.CUR)
            self.cur_loc = [self.cur_x - 1,self.cur_y]
        elif self.cur_direc == Dir.RIGHT:
            self._addPoint([self.cur_x + 1,self.cur_y],State.CUR)
            self.cur_loc = [self.cur_x + 1,self.cur_y]
        else:
            print("Error: Direction uninitialized or non-real value")
    def evalJunction(self):
        if len(self.junc_instr) > 0:
            return self.junc_instr.pop(0)
        else:
            return None
    def findNearestUnexp(self):
        return None
    def _convertMap(self):
        for r in self.grid:
            for c in self.grid:
                self.grid[r][c] = (self.grid[r][c]).value
    def addHazard(self,haz_type,value,loc):
        hazard_dict = {
            State.HEAT: ["Fire","Temperature (C)"],
            State.MAG: ["Damaged Power Station","Field Strenth (T)"]
        }
        new_hazard = [hazard_dict[haz_type][0],hazard_dict[haz_type][1],value,loc[0],loc[1]]
        self.hazard_info.append(new_hazard)
        self._addPoint(loc,haz_type)
    def pushInfo(self):
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