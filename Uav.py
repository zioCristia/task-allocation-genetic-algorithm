import scipy.optimize as sp
import numpy as np
from Position import Position
from Task import Task
from Traject import Traject
from typing import List
import utility as ut
import math
import matplotlib.pyplot as plt

class Uav:
    eta = 0.8
    cd = 0.3
    rho = 1.23
    Ad = 0.6
    Ar = 0.3   # total rotor disk area
    g = 9.81
    Fm = 0.9  # UAV figure of merit
    BLt = 0.3
    maxVelocity = 20    # TODO add to the constructor

    postion = Position(0,0)
    currentBatteryCapacity = 0
    currentTask = 0
    distanceToTask = 0
    timeSpentPerTask = []
    trajectsEnergy = []
    currentTrajectTime = 0
    currentTrajectVelocity = 0

    def __init__(self, batteryCapacity: float, mass: float, maxPayloadMass: float, startPosition: Position, *, position: Position = Position(0,0), currentBatteryCapacity = 0, currentTask: Task = None, timeSpentPerTask: List = [], trajectsEnergy: List = []) -> None:
        self.batteryCapacity = batteryCapacity
        self.mass = mass
        self.maxPayloadMass = maxPayloadMass
        self.startPosition = startPosition

        self.position = position
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
        details += f'startPosition : {self.startPosition}\n'
        details += f'position : {self.position}\n'
        details += f'currentBatteryCapacity : {self.currentBatteryCapacity}\n'
        details += f'currentTask : {self.currentTask}\n'
        return details
    
    @classmethod
    def fromUav(cls, uav):
        return cls(batteryCapacity = uav.batteryCapacity, mass = uav.mass, maxPayloadMass = uav.maxPayloadMass, startPosition = uav.startPosition, position = uav.position, currentBatteryCapacity = uav.currentBatteryCapacity, currentTask = uav.currentTask, timeSpentPerTask = uav.timeSpentPerTask, trajectsEnergy = uav.trajectsEnergy)

    def getMaxPayloadMass(self) -> float:
        return self.maxPayloadMass

    def getPosition(self) -> Position:
        return self.position
    
    def getMass(self):
        return self.mass
    
    def getTotalEnergyUsed(self):
        return self.totalEnergyUsed
    
    def setCurrentTask(self, task: Task):
        self.currentTask = task
        self.distanceToTask = ut.taskDistance(self.position, task)

    def takeTask(self, task: Task):
        self.setCurrentTask(task)
        taskEnergy = self.taskEnergy(task)
        if self.currentBatteryCapacity - taskEnergy < -0.001 or taskEnergy == 0:
            raise Exception("energy not available in drone")

        self.removeBatteryEnergy(taskEnergy)
        self.position = task.getEndPosition()
        self.timeSpentPerTask.append(self.currentTrajectTime)
        
        if task.isChargingPoint():
            self.recharge()
    
    def taskEnergy(self, task: Task):
        currentTraject = Traject(self.position, self.currentTask, self.currentBatteryCapacity)
        if currentTraject in self.trajectsEnergy:
            previousTraject = self.trajectsEnergy[np.where(np.array(self.trajectsEnergy) == currentTraject)[0][0]]
            return previousTraject.getEnergy()
        
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
            # C3 = sp.NonlinearConstraint(self.timeSpent, lb=0, ub=self.currentTask.getMaxDeliveryWindow())
            sol = sp.minimize(self.energyConsumption, (5,), bounds=(C1), constraints=(C2,))
            if not sol.success:
                # print ("Solution not found")
                return 0
            energy = sol.fun[0]
            velocity = sol.x[0]
        else:
            sol = sp.minimize(lambda v : d/v, (5,), bounds=(C1), constraints=(C2,))
            if not sol.success:
                # print ("Solution not found")
                return 0
            velocity = sol.x[0]
            energy = self.energyConsumption(velocity)

        self.currentTrajectVelocity = velocity
        self.currentTrajectTime = (self.distanceToTask + self.currentTask.getTrajectDistance()) / velocity / 60
        
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
    
    def canTakeTask(self, task: Task) -> bool:
        storageCurrentTask = self.currentTask
        self.setCurrentTask(task)
        taskEnergy = self.taskEnergy(task)
        if taskEnergy == 0:
            return False
        self.setCurrentTask(storageCurrentTask)
        return self.currentBatteryCapacity - taskEnergy > -0.1

    def canTakeTasks(self, tasks: List[Task]) -> bool:
        tempUav = Uav.fromUav(self)
        tasksEnergy = 0

        for t in tasks:
            tempUav.setCurrentTask(t)
            currentTaskEnergy = tempUav.taskEnergy(t)
            if currentTaskEnergy == 0:
                return False
            try:
                tempUav.removeBatteryEnergy(currentTaskEnergy)
            except:
                return False
            tempUav.position = t.getEndPosition()

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
        self.position = self.startPosition
        self.timeSpentPerTask = []

    def energyConsumption(self, v):
        l1 = self.distanceToTask
        l2 = self.currentTask.getTrajectDistance()

        mp = self.currentTask.getPackageMass()

        A = self.cd * self.rho * self.Ad * (l1 + l2) * v**2
        B = (l2 * math.sqrt(((self.mass + mp) * self.g)**3))/(v * self.Fm * math.sqrt(2 * self.rho * self.Ar))
        C = (l1 * math.sqrt((self.mass * self.g)**3))/(v * self.Fm * math.sqrt(2 * self.rho * self.Ar))

        return 1/self.eta * (A + B + C)
    
    def timeSpent(self, v):
        l1 = self.distanceToTask
        l2 = self.currentTask.getTrajectDistance()

        return self.getTotalTimeSpentTillNow() + (l1 + l2)/ v
    
    def printTrajectEnergies(self):
        for tr in self.trajectsEnergy:
            print(tr)
