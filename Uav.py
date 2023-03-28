import scipy.optimize as sp
import numpy as np
from Position import Position
from Task import Task
from Traject import Traject
from typing import List
import utility as ut
import math
import matplotlib.pyplot as plt
from AlgoConstants import AlgoConstants as const

class Uav:
    eta = 0.8
    rho = 1.23
    g = 9.81
    Fm = 0.9  # UAV figure of merit

    positionId = 0
    currentBatteryCapacity = 0
    currentTask = 0
    distanceToTask = 0
    timeSpentPerTask = []
    energySpentPerTask = []
    trajectsEnergy = []
    currentTrajectTime = 0
    currentTrajectVelocity = 0

    def __init__(self, batteryCapacity: float, mass: float, maxPayloadMass: float, costMatrix: List = [], *, 
                 startPositionId: int = 0, 
                 Ar: float = 0.2,
                 cd: float = 0.3,
                 Ad: float = 0.2,
                 maxVelocity: float = 20,
                 positionId: int = 0, 
                 currentBatteryCapacity = 0, 
                 currentTask: Task = None, 
                 timeSpentPerTask: List = [], 
                 trajectsEnergy: List = []) -> None:
        
        self.batteryCapacity = batteryCapacity
        self.mass = mass
        self.maxPayloadMass = maxPayloadMass
        self.costMatrix = costMatrix

        self.startPositionId = startPositionId
        self.Ad = Ad
        self.cd = cd
        self.Ar = Ar   # total rotor disk area
        self.maxVelocity = maxVelocity
        self.positionId = positionId
        self.currentBatteryCapacity = currentBatteryCapacity
        if isinstance(currentTask, Task):
            self.currentTask = self.setCurrentTask(currentTask)
        self.timeSpentPerTask = timeSpentPerTask
        self.trajectsEnergy = trajectsEnergy

        self.totalEnergyUsed = 0
    
    def __str__(self) -> str:
        details = ''
        details += f'batteryCapacity : {self.batteryCapacity}\n'
        details += f'mass : {self.mass}\n'
        details += f'maxPayloadMass : {self.maxPayloadMass}\n'
        details += f'startPositionId : {self.startPositionId}\n'
        details += f'positionId : {self.positionId}\n'
        details += f'currentBatteryCapacity : {self.currentBatteryCapacity}\n'
        details += f'currentTask : {self.currentTask}\n'
        return details
    
    @classmethod
    def fromUav(cls, uav):
        return cls(batteryCapacity = uav.batteryCapacity, 
                   mass = uav.mass, 
                   maxPayloadMass = uav.maxPayloadMass,
                   costMatrix = uav.costMatrix, 
                   startPositionId = uav.startPositionId,
                   Ad = uav.Ad,
                   cd = uav.cd,
                   Ar = uav.Ar,
                   maxVelocity = uav.maxVelocity,
                   positionId = uav.positionId, 
                   currentBatteryCapacity = uav.currentBatteryCapacity, 
                   currentTask = uav.currentTask, 
                   timeSpentPerTask = uav.timeSpentPerTask, 
                   trajectsEnergy = uav.trajectsEnergy)

    def getMaxPayloadMass(self) -> float:
        return self.maxPayloadMass

    def getPositionId(self) -> int:
        return self.positionId
    
    def setStartPositionId(self, id: int):
        self.startPositionId = id
        
    def getMass(self):
        return self.mass
    
    def getTotalEnergyUsed(self):
        return self.totalEnergyUsed
    
    def setCurrentTask(self, task: Task):
        self.currentTask = task
        self.distanceToTask = self.costMatrix[self.positionId, task.startPositionId]

    def evaluateTasksEnergies(self, tasks: List[Task]):
        for t in tasks:
            self.setCurrentTask(t)
            taskEnergy = self.taskEnergy(t)
            self.positionId = t.getEndPositionId()
            self.timeSpentPerTask.append(self.currentTrajectTime)
            self.energySpentPerTask.append(taskEnergy)
            self.totalEnergyUsed += taskEnergy

    def takeTask(self, task: Task):
        self.setCurrentTask(task)
        taskEnergy = self.taskEnergy(task)
        if self.currentBatteryCapacity - taskEnergy < -0.001 or taskEnergy == -1:
            raise Exception("energy not available in drone")

        self.removeBatteryEnergy(taskEnergy)
        self.positionId = task.getEndPositionId()
        self.timeSpentPerTask.append(self.currentTrajectTime)
        self.energySpentPerTask.append(taskEnergy)
        
        if task.isChargingPoint():
            self.recharge()
    
    def taskEnergy(self, task: Task):
        currentTraject = Traject(self.positionId, self.currentTask, self.currentBatteryCapacity, self.getTotalTimeSpentTillNow())
        if currentTraject in self.trajectsEnergy:
            previousTraject = self.trajectsEnergy[np.where(np.array(self.trajectsEnergy) == currentTraject)[0][0]]
            self.currentTrajectEnergy = previousTraject.getEnergy()
            self.currentTrajectTime = previousTraject.getTime()
            # if self.currentTrajectEnergy != -1:
            return previousTraject.getEnergy()
            # else:
            #     currentTrajectEnergy = self.taskEnergyOptimizer(task)
            #     previousTraject.setEnergy(currentTrajectEnergy)
            #     previousTraject.setTime(self.currentTrajectTime)
            #     previousTraject.setVelocity(self.currentTrajectVelocity)
            #     return currentTrajectEnergy
        
        currentTrajectEnergy = self.taskEnergyOptimizer(task)
        currentTraject.setEnergy(currentTrajectEnergy)
        currentTraject.setTime(self.currentTrajectTime)
        currentTraject.setVelocity(self.currentTrajectVelocity)
        self.trajectsEnergy.append(currentTraject)
        
        return currentTrajectEnergy
    
    def taskEnergyOptimizer(self, task: Task):
        energy = 0
        velocity = 0
        d = self.distanceToTask
        C1 = sp.Bounds(1, self.maxVelocity)
        C2 = sp.NonlinearConstraint(self.energyConsumption, lb=0, ub=self.currentBatteryCapacity)

        if not task.isChargingPoint():
            C3 = sp.NonlinearConstraint(self.timeSpent, lb=0, ub=self.currentTask.getMaxDeliveryWindow())
            sol = sp.minimize(self.energyConsumption, (5,), bounds=(C1), constraints=(C2, C3))
            energy = sol.fun
            velocity = sol.x[0]
            if not sol.success and energy > self.currentBatteryCapacity:
                if const.DEBUG:
                    print ("Solution not found")
                return -1
        else:
            sol = sp.minimize(lambda v : d/v, (5,), bounds=(C1), constraints=(C2,))
            velocity = sol.x[0]
            energy = self.energyConsumption(velocity)
            if not sol.success and energy > self.currentBatteryCapacity:
                if const.DEBUG:
                    print ("Solution not found")
                return -1

        self.currentTrajectVelocity = velocity
        self.currentTrajectTime = (self.distanceToTask + self.getTrajectDistance(self.currentTask)) / velocity
        
        # print("Energy: " + str(energy) + " , current battery capacity: " + str(self.currentBatteryCapacity))
        return energy

    def printEnergyConsuptionGraph(self):
        x = np.linspace(1, self.maxVelocity)
        plt.axhline(self.currentBatteryCapacity, color = 'r', linestyle = '-')
        plt.plot(x, self.energyConsumption(x))
        plt.show()

    def removeBatteryEnergy(self, energy):
        if self.currentBatteryCapacity - energy < -0.001:
            raise Exception("energy not available in drone")

        self.currentBatteryCapacity -= energy
        self.totalEnergyUsed += energy
    
    def recharge(self):
        self.currentBatteryCapacity = self.batteryCapacity
    
    def isFull(self):
        return self.currentBatteryCapacity == self.batteryCapacity
    
    def canTakeTaskWeigth(self, task: Task) -> bool:
        return task.getPackageMass() <= self.maxPayloadMass

    def canTakeTask(self, task: Task) -> bool:
        if not self.canTakeTaskWeigth(task):
            return False

        storageCurrentTask = self.currentTask
        self.setCurrentTask(task)
        taskEnergy = self.taskEnergy(task)
        if taskEnergy == -1:
            return False
        self.setCurrentTask(storageCurrentTask)
        return self.currentBatteryCapacity - taskEnergy > -0.1

    def canTakeTasks(self, tasks: List[Task]) -> bool:
        tempUav = Uav.fromUav(self)
        tasksEnergy = 0

        for t in tasks:
            if not self.canTakeTaskWeigth(t):
                raise Exception("Task cannot be taken. Package mass exceed uav max payload.")
            
            tempUav.setCurrentTask(t)
            currentTaskEnergy = tempUav.taskEnergy(t)
            if currentTaskEnergy == -1:
                return False
            try:
                tempUav.removeBatteryEnergy(currentTaskEnergy)
            except:
                return False
            tempUav.positionId = t.getEndPositionId()

            if t.isChargingPoint():
                tempUav.recharge()
            
            tasksEnergy += currentTaskEnergy

        # return self.currentBatteryCapacity - tasksEnergy > 0
        return True
    
    def getCurrentBatteryCapacity(self):
        return self.currentBatteryCapacity

    def getTotalTimeSpentTillNow(self):
        return sum(self.timeSpentPerTask)
    
    def reset(self):
        self.recharge()
        self.totalEnergyUsed = 0
        self.positionId = self.startPositionId

        self.currentTask = 0
        self.distanceToTask = 0
        self.timeSpentPerTask = []
        self.energySpentPerTask = []
        self.currentTrajectTime = 0
        self.currentTrajectVelocity = 0

    def energyConsumption(self, v):
        l1 = self.distanceToTask
        l2 = self.getTrajectDistance(self.currentTask)

        mp = self.currentTask.getPackageMass()

        A = self.cd * self.rho * self.Ad * (l1 + l2) * v**2
        B = (l2 * math.sqrt(((self.mass + mp) * self.g)**3))/(v * self.Fm * math.sqrt(2 * self.rho * self.Ar))
        C = (l1 * math.sqrt((self.mass * self.g)**3))/(v * self.Fm * math.sqrt(2 * self.rho * self.Ar))

        return 1/self.eta * (A + B + C)
    
    def timeSpent(self, v):
        l1 = self.distanceToTask
        l2 = self.getTrajectDistance(self.currentTask)

        return self.getTotalTimeSpentTillNow() + (l1 + l2)/ v
    
    def printTrajectEnergies(self):
        for tr in self.trajectsEnergy:
            print(tr)

    def getTrajectDistance(self, task: Task) -> float:
        return self.costMatrix[task.getStartPositionId(), task.getEndPositionId()]
    
    def getDistanceToTaskFromPositionId(self, task: Task, fromPositionId: int) -> float:
        return self.costMatrix[fromPositionId, task.getStartPositionId()]