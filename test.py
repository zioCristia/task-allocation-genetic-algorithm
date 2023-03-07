import numpy as np
import time
# import scipy.optimize as sp
# import math
from Task import Task
from Position import Position
from ChargingPoint import ChargingPoint
from Uav import Uav
from GeneticAlgo import GeneticAlgo

# array1 = np.array([1, 2, 3])
# array2 = np.array([4, 5, 6, 7])
# array3 = np.array([7, 8, 9])
# arrayOfArrays = np.array([array1, array2, array3], dtype=object)
# print(arrayOfArrays)

# arr = list(range(9))
# np.random.shuffle(arr)
# print(arr)

# print(np.sort(np.random.randint(5, size=(2))))
# print(list(range(0,5)))
# print(np.random.choice(list(range(3,5))))

# a = np.where(np.array([0,0,1,1,0,1]) == 1)

# for i in np.where(np.array([0,0,1,1,0,1]) == 1)[0]:
#     print(i)

# a = np.array([4, 5, 6, 7])
# b = np.array([2, 4, 5])
# c = np.append(a, b)
# print(np.argmin(c))

# uav1 = Uav(1.3*10**6, 4, 3, Position(0, 0))
# uav2 = Uav(0.9*10**6, 2, 1.5, Position(0, 0))
# task1 = Task(Position(1000, 1000), Position(1000, 3000), 5, 1)
# task2 = Task(Position(3000, 7000), Position(2000, 3000), 3, 2)
# task3 = Task(Position(8000, 2000), Position(8000, 4000), 2, 1)
# task4 = Task(Position(6000, 4000), Position(4000, 6000), 3, 3)
# task5 = Task(Position(1000, 4000), Position(7000, 3000), 1, 1)
# cp1 = ChargingPoint(Position(2000,3000))
# cp2 = ChargingPoint(Position(4000,4000))

# uavs = np.array((uav1,uav2))
# tasks = np.array([task1, task2, task3, task4, task5])
# cps = np.array([cp1, cp2])

# ga = GeneticAlgo(uavs, tasks, cps)

# print(ga.addChargingTasks(np.array([0, 3, 2, 1]), 0))

uav0 = Uav(1.2*10**6, 4, 3, Position(0, 0))
uav1 = Uav(0.9*10**6, 2, 1.5, Position(0, 0))
task0 = Task(Position(1000, 1000), Position(1000, 3000), 5, 1)
task1 = Task(Position(3000, 7000), Position(2000, 3000), 3, 2)
task2 = Task(Position(8000, 2000), Position(8000, 4000), 2, 1)
task3 = Task(Position(6000, 4000), Position(4000, 6000), 3, 3)
task4 = Task(Position(1000, 4000), Position(7000, 3000), 1, 1)
cp0 = ChargingPoint(Position(2000,5000))
cp1 = ChargingPoint(Position(4000,4000))

uav0.recharge()
uav1.recharge()
# Energy consumption test
# 31 420
print("uav 0 can take tasks 31? " + str(uav0.canTakeTasks([task3, task1]))) # false
print("uav 1 can take tasks 042? " + str(uav1.canTakeTasks([task0, task4, task2]))) # true
print("uav 1 can take tasks 420? " + str(uav1.canTakeTasks([task4, task2, task0]))) # false

print("uav 0 take 361")
uav0.reset()
print(uav0.getTotalEnergyUsed())
uav0.takeTask(task3)
print(uav0.getTotalEnergyUsed())
uav0.takeTask(cp1)
print(uav0.getTotalEnergyUsed())
uav0.takeTask(task1)
print(uav0.getTotalEnergyUsed())
print("uav 1 take 042")
uav1.reset()
print(uav1.getTotalEnergyUsed())
uav1.takeTask(task0)
print(uav1.getTotalEnergyUsed())
uav1.takeTask(task4)
print(uav1.getTotalEnergyUsed())
uav1.takeTask(task2)
print(uav1.getTotalEnergyUsed())
uav1.takeTask(cp1)
print(uav1.getTotalEnergyUsed())

uavs = [uav0,uav1]
tasks = [task0, task1, task2, task3, task4]
cps = [cp0, cp1]

run = GeneticAlgo(uavs, tasks, cps)
# print("Time for add charging tasks")
# # start = time.process_time()
# print(run.addChargingTasksPerDrone([3, 1], 0))
# # print("time: " + str(time.process_time() - start))
# print(run.addChargingTasksPerDrone([4, 2, 0], 1))
# print("uav 1 can take tasks 042? " + str(uav1.canTakeTasks([task0, task4, task2])))
print(run.addChargingTasksPerDrone([0, 4, 2], 1))

print("Uav 1 take task 26 from 6 position")
uav1.reset()
uav1.position = Position(4,4)
print(uav1.getTotalEnergyUsed())
uav1.takeTask(task2)
print(uav1.getTotalEnergyUsed())
uav1.takeTask(cp1)
print(uav1.getTotalEnergyUsed())

print("uav 1 can take tasks 132? " + str(uav1.canTakeTasks([task1, task3, task2]))) # false
print("uav 1 can take tasks 15362? " + str(uav1.canTakeTasks([task1, cp0, task3, cp1, task2]))) # true
uav1.reset()
uav1.position = Position(4,4)
print("uav 1 can take tasks 26 from position(4,4)? " + str(uav1.canTakeTasks([task2, cp1]))) # true
print(run.addChargingTasksPerDrone([1, 3, 2], 0))

uav1.reset()
print("uav 1 can take tasks 402)? " + str(uav1.canTakeTasks([task4, task0, task2]))) # false
print(run.addChargingTasksPerDrone([4, 0, 2], 1))
uav1.reset()
print("uav 1 can take tasks 4602)? " + str(uav1.canTakeTasks([task4, cp1, task0, task2]))) # false

print("uav 1 can take tasks 204)? " + str(uav1.canTakeTasks([task2, task0, task4]))) # false
print(run.addChargingTasksPerDrone([2, 0, 4], 1))
uav1.reset()
print("uav 1 can take tasks 024)? " + str(uav1.canTakeTasks([task0, task2, task4]))) # false
print(run.addChargingTasksPerDrone([0, 2, 4], 1))

# print("Time for taskEnergy")
# start = time.process_time()
# uav0.taskEnergyOptimizer(task1)
# print(time.process_time() - start)
# start = time.process_time()
# uav0.taskEnergyOptimizer(task1)
# print(time.process_time() - start)