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
    def __init__(self, origin = [0,0], map_num = 0, direc = Dir.UP, team = 14, unit_length = 40, unit = Unit.CM, notes = "n/a"):
        self.grid = np.full((origin[1] + 1,origin[0] + 1),State.UNKWN)
        self.grid[len(self.grid) - origin[1] - 1][origin[0]] = State.ORIG 
        self.origin = origin
        self.map_num = map_num
        self.team = team
        self.unit_length = unit_length
        self.unit = unit
        self.notes = notes
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
    def origin(self):
        return self.__origin

    @origin.setter
    def origin(self, origin):
        self.__origin = origin

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
    def unit_length(self):
        return self.__unit_length

    @unit_length.setter
    def unit_length(self, unit_length):
        self.__unit_length = unit_length

    @property
    def unit(self):
        return self.__unit

    @unit.setter
    def unit(self, unit):
        self.__unit = unit

    @property
    def notes(self):
        return self.__notes

    @notes.setter
    def notes(self, notes):
        self.__notes = notes
    
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
        try:
            self.grid = np.insert(self.grid, 0, np.full(len(self.grid[0]),State.UNKWN),axis=0)
        except Exception as error: 
            print("_appendRow:",error)

    def _appendCol(self):
        try:
            insert = np.full((len(self.grid),1), State.UNKWN)
            self.grid = np.append(self.grid, insert, axis=1)
        except Exception as error: 
            print("_appendCol:",error)

    def _setPoint(self,pt,point_type = State.EXPL):
        try:
            if pt[0] < 0 or pt[1] < 0:
                print("Error: Unable to add point with negative coordinates")
            if pt[0] > len(self.grid[0]) - 1:
                self._appendCol()
            if pt[1] > len(self.grid) - 1:
                self._appendRow()
            self.grid[len(self.grid) - pt[1] - 1][pt[0]] = point_type
        except Exception as error: 
            print("_setPoint:",error)

    def _setPointRelative(self,rel_direc = Dir.UP, point_type = State.CUR):
        try:
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
        except Exception as error: 
            print("_setPointRelative:",error)

    def _getPoint(self,pt):
        try:
            return self.grid[len(self.grid) - pt[1] - 1][pt[0]]
        except Exception as error: 
            print("_getPoint:",error)

    def updateLocation(self):
        try:
            if self._getPoint(self.cur_loc) != State.ORIG:
                self._setPoint([self.cur_x,self.cur_y],State.EXPL)

            new_cur = self._setPointRelative(Dir.UP, State.CUR)
            self.cur_loc = new_cur
        except Exception as error: 
            print("updateLocation:",error)

    def _convertMap(self):
        try:
            for r in range(len(self.grid)):
                for c in range(len(self.grid[r])):
                    self.grid[r][c] = (self.grid[r][c]).value
        except Exception as error: 
            print("_convertMap:",error)

    def addHazard(self,hazard_type,value):
        try:
            hazard_loc = self.cur_loc
            self._setPoint(self.cur_loc, hazard_type)

            hazard_dict = {
                State.HEAT: ["Fire","Temperature (C)"],
                State.MAG: ["Damaged Power Station","Field Strenth (T)"]
            }
            new_hazard = [hazard_dict[hazard_type][0],hazard_dict[hazard_type][1],value,hazard_loc[0], hazard_loc[1]]
            self.hazard_info.append(new_hazard)
        except Exception as error: 
            print("addHazard:",error)

    ##TODO: Test additional fields in csv files
    def pushInfo(self):
        try:
            self._setPoint(self.cur_loc,State.EXIT)
            self._convertMap()
            info = {"map": self.grid,
                    "hazard table": self.hazard_info 
            }
            with open('map_' + str(self.map_num) + '.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow("Team: %d" % (self.team))
                writer.writerow("Map: %d" % (self.map_num))
                writer.writerow("Unit length: %d" % (self.unit_length))
                writer.writerow("Unit length: %d" % (self.unit.name))
                writer.writerow("Origin: (%d,%d)" % (self.origin[0], self.origin[1]))
                writer.writerow("Notes: %s" % (self.notes))
                writer.writerow("")

                writer.writerows(info["map"])

            with open('hazards.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow("Team: %d" % (self.team))
                writer.writerow("Map: %d" % (self.map_num))
                writer.writerow("Notes: %s" % (self.notes))
                writer.writerow("")

                writer.writerows(info["hazard table"])    

            return info
        except Exception as error: 
            print("pushInfo:",error)