import math
from random import randint
from re import I
import numpy as np
import matplotlib.pyplot as plt
import constant
import utility
from Task import Task
from ChargingPoint import ChargingPoint
from Uav import Uav
from Position import Position
from Environement import Environement
from typing import List
import scipy.io
import statistics

class Positions:
    points = []
    def __init__(self) -> None:
        locations = scipy.io.loadmat('distancesCostMatrix/locations.mat')['points']

        for p in range(len(locations[0])):
            self.points.append(Position(locations[2, p], locations[1, p]))

    def getPoint(self, n: int) -> Position:
        return self.points[n]
