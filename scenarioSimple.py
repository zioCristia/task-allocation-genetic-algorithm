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
import scipy.io
"""
SIMPLE SCENARIO
As writte in the thesis work, with 
"""

distancesUav0 = scipy.io.loadmat('distancesCostMatrix/Drone_Marco_1_dist_cost.mat')['Res']
distancesUav1 = scipy.io.loadmat('distancesCostMatrix/Drone_Marco_2_dist_cost.mat')['Res']
distancesUav2 = scipy.io.loadmat('distancesCostMatrix/Drone_Marco_3_dist_cost.mat')['Res']
distancesUav3 = scipy.io.loadmat('distancesCostMatrix/Drone_Marco_4_dist_cost.mat')['Res']

uav0 = Uav(1.2*10**6, 4, 3)
uav1 = Uav(0.9*10**6, 2, 1.5)
uav2 = Uav(0.9*10**6, 2, 1.5)
uav3 = Uav(0.9*10**6, 2, 1.5)

task0 = Task(Position(1000, 1000), Position(1000, 3000), 1500, 1)
task1 = Task(Position(3000, 7000), Position(2000, 3000), 4000, 2)
task2 = Task(Position(8000, 2000), Position(8000, 4000), 1800, 1)
task3 = Task(Position(6000, 4000), Position(4000, 6000), 2500, 3)
task4 = Task(Position(1000, 4000), Position(7000, 3000), 3000, 1)
cp0 = ChargingPoint(Position(2000,5000))
cp1 = ChargingPoint(Position(4000,4000))

# uavs = np.array((uav0,uav1))
# tasks = np.array([task0, task1, task2, task3, task4])
# cps = np.array([cp0, cp1])
uavs = [uav0,uav1]
tasks = [task0, task1, task2, task3, task4]
cps = [cp0, cp1]

ga = GeneticAlgo(uavs, tasks, cps)
ga.run()
# run.printSolution()
# run.graphEvaluations()
# run.graphSolution()