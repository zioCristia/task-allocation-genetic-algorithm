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
"""
First try assumptions:
* for the initial population we could have multiple identical individuals
* infinite energy per drone (no recharging tasks)
* all the tasks in the same day
* minimize the total delivery time and each drone delivery time
* all the drones start from the (0,0) position
* the drone will end in the same spot where started

* deadline delle task
* batteria
"""

uav0 = Uav(1.2*10**6, 4, 3, Position(0, 0))
uav1 = Uav(0.9*10**6, 2, 1.5, Position(0, 0))
task0 = Task(Position(1000, 1000), Position(1000, 3000), 50000, 1)
task1 = Task(Position(3000, 7000), Position(2000, 3000), 25000, 2)
task2 = Task(Position(8000, 2000), Position(8000, 4000), 15000, 1)
task3 = Task(Position(6000, 4000), Position(4000, 6000), 30000, 3)
task4 = Task(Position(1000, 4000), Position(7000, 3000), 10000, 1)
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