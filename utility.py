import math
import numpy as np
import scipy as sp
from Position import Position
# from Task import Task
from typing import List

def distance(position1: Position, position2: Position):
    return math.sqrt((position1.x - position2.x)**2 + (position1.y - position2.y)**2)

def taskDistance(position: Position, task):
    return distance(position, task.getStartPosition())

def tasksDistances(position: Position, tasks: List):
    distances = np.empty(len(tasks))
    
    for t in range(len(tasks)):
        distances[t] = taskDistance(position, tasks[t])
    
    return distances

# def printDronesTaskes(chromosomes cutPosition):
#     for d in range(constant.NU):
#         print("Drone " + d + ": ")
#         for t in range(cutPosition[d-1],cutPosition[d]):
#             print(chromosomes[t])
    
