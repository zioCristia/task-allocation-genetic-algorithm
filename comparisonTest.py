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
from GeneticAlgo import GeneticAlgo

uav0 = Uav(0.6*10**6, 1, 3)
task0 = Task(Position(0, 1000), Position(0, 2300), 600, 1)

uav0.reset()
uav0.setCurrentTask(task0)
uav0.taskEnergy(task0)