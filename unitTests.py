from GeneticAlgo import GeneticAlgo
from Task import Task
from Position import Position
from ChargingPoint import ChargingPoint
from Uav import Uav
from GeneticAlgo import GeneticAlgo

uav0 = Uav(1.2*10**6, 4, 3, Position(0, 0))
uav1 = Uav(0.9*10**6, 2, 1.5, Position(0, 0))
task0 = Task(Position(1000, 1000), Position(1000, 3000), 5, 1)
task1 = Task(Position(3000, 7000), Position(2000, 3000), 3, 2)
task2 = Task(Position(8000, 2000), Position(8000, 4000), 2, 1)
task3 = Task(Position(6000, 4000), Position(4000, 6000), 3, 3)
task4 = Task(Position(1000, 4000), Position(7000, 3000), 1, 1)
cp0 = ChargingPoint(Position(2000,5000))
cp1 = ChargingPoint(Position(4000,4000))

uavs = [uav0,uav1]
tasks = [task0, task1, task2, task3, task4]
cps = [cp0, cp1]

ga = GeneticAlgo(uavs, tasks, cps)

# Random cut position
# print(ga.randomCutPosition())
# print(ga.randomCutPosition())