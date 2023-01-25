import scipy.optimize as sp
import numpy as np
from Position import Position
from Task import Task
from typing import List
import utility as ut
import math

class Uav:
    eta = 1
    cd = 0.3
    rho = 1.0125
    Ad = 0.1
    Ar = 0.1   # total rotor disk area
    g = 9.81
    Fm = 1  # UAV figure of merit
    BLt = 0.3

    currentTask = 0
    distanceToTask = 0

    def __init__(self, batteryCapacity: float, mass: float, maxPayloadMass: float, startPosition: Position) -> None:
        self.batteryCapacity = batteryCapacity
        self.mass = mass
        self.maxPayloadMass = maxPayloadMass
        self.position = startPosition
        self.maxVelocity = 20
        self.currentBatteryCapacity = batteryCapacity
        self.totalEnergyUsed = 0
    
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
        taskEnergy = self.taskEnergy(task)
        if self.currentBatteryCapacity - taskEnergy < 0:
            raise Exception("energy not available in drone")

        self.removeBatteryEnergy(taskEnergy)
        self.position = task.getEndPosition()
        
        if task.isChargingPoint():
            self.recharge()

    def taskEnergy(self, task: Task):
        self.setCurrentTask(task)
        energy = 0
        if task.isChargingPoint():
            C1 = sp.NonlinearConstraint(self.energyConsumption, lb=0, ub=self.currentBatteryCapacity)
            energy = sp.minimize(self.energyConsumption, 1, constraints=(C1,)).fun[0]
        else:
            d = self.distanceToTask
            C1 = sp.Bounds(0, self.maxVelocity)
            C2 = sp.NonlinearConstraint(self.energyConsumption, lb=self.batteryCapacity*self.BLt, ub=self.currentBatteryCapacity)
            t = sp.minimize(lambda x : d/x, (1,), bounds=(C1))
            energy = self.energyConsumption(t.x[0])

        return energy

    def removeBatteryEnergy(self, energy):
        if self.currentBatteryCapacity - energy < 0:
            raise Exception("energy not available in drone")

        self.currentBatteryCapacity -= energy
        self.totalEnergyUsed += energy
    
    def recharge(self):
        self.currentBatteryCapacity = self.batteryCapacity
    
    def canTakeTask(self, task: Task) -> bool:
        return self.currentBatteryCapacity - self.taskEnergy(task) > 0

    def canTakeTasks(self, tasks: List[Task]) -> bool:
        taskEnergy = 0
        for t in tasks:
            taskEnergy += self.taskEnergy(t)

        return self.currentBatteryCapacity - taskEnergy > 0
    
    def getCurrentBatteryCapacity(self, currentBatteryCapacity):
        return self.currentBatteryCapacity

    def reset(self):
        self.recharge()
        self.totalEnergyUsed = 0
    
    def energyConsumption(self, v):
        # TODO add the distance from the current drone position and the task start position
        l1 = self.distanceToTask
        l2 = self.currentTask.getTrajectDistance()

        mp = self.currentTask.getPackageMass()

        A = self.cd * self.rho * self.Ad * (l1 + l2) * v**2
        B = (l2 * math.sqrt(((self.mass + mp) * self.g)**3))/(v * self.Fm * math.sqrt(2 * self.rho * self.Ar))
        C = (l1 * math.sqrt((self.mass * self.g)**3))/(v * self.Fm * math.sqrt(2 * self.rho * self.Ar))

        return 1/self.eta * (A + B + C)
