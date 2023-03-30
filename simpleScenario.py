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
from Environement import Environement
from typing import List
import scipy.io
import statistics

def numberOfChargingTask(taskOrder: List[int]) -> int:
    output = 0
    for t in taskOrder:
        if t > 29 and t < 34:
            output += 1
    
    return output

"""
COMPLEX SCENARIO A
As writte in the thesis work, with 
"""
# TODO: adapt the tasks and chargingPoint to have id instaed of Position
# TODO: adapt to read distancesMatrix instead of calculate distance
distancesUav0 = scipy.io.loadmat('distancesCostMatrix/Drone_Marco_1_dist_cost.mat')['Res']
distancesUav1 = scipy.io.loadmat('distancesCostMatrix/Drone_Marco_2_dist_cost.mat')['Res']
distancesUav2 = scipy.io.loadmat('distancesCostMatrix/Drone_Marco_3_dist_cost.mat')['Res']
distancesUav3 = scipy.io.loadmat('distancesCostMatrix/Drone_Marco_4_dist_cost.mat')['Res']

# craetion of uavs
uav0 = Uav(0.68*10**6, 1, 1, costMatrix=distancesUav0, Ar=0.2, cd=0.3, Ad=0.4, maxVelocity=16)
uav1 = Uav(0.9*10**6, 2, 2, costMatrix=distancesUav1, Ar=0.28, cd=0.35, Ad=0.6, maxVelocity=19)
uav2 = Uav(1.17*10**6, 3, 3, costMatrix=distancesUav2, Ar=0.36, cd=0.40, Ad=0.8, maxVelocity=20)
uav3 = Uav(1.43*10**6, 4, 4, costMatrix=distancesUav3, Ar=0.44, cd=0.45, Ad=1, maxVelocity=22)
uavs = [uav1,uav3]

startId = 24
for u in uavs:
    u.setStartPositionId(startId)
    startId += 1

# cration of charging points
cps = [ChargingPoint(31, 33)]
# for i in range(30, 34):
#     cps.append(ChargingPoint(i))

# creation of tasks
payloadMasses = [0.5, 1, 2 ,3]

deadlines = [0.3358, 0.6417, 1.0638, 1.1702, 1.6346]

tasks = []
for i in range(40):
    payloadMass = payloadMasses[int(i/10)]
    task = Task(i, 40 - 1 - i, deadlines[i], payloadMass)
    tasks.append(task)

tasks = np.random.sample(tasks, len(deadlines))

# setting different deadline for each task
for i in range(len(deadlines)):
    tasks[i].setMaxDeliveryWindow(deadlines[i]*10**4)

energies = []
chargeExecuted = []

envSimple = Environement(uavs, tasks, cps)
gaSimple = GeneticAlgo(envSimple)

gaSimple.run()