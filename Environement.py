from Task import Task
from Uav import Uav
from ChargingPoint import ChargingPoint
from AlgoConstants import AlgoConstants as const
from typing import List

class Environement:
    def __init__(self, uavs: List[Uav], tasks: List[Task], chargingPoints: List[ChargingPoint]):
        self.uavs = uavs
        self.tasks = tasks
        self.chargingPoints = chargingPoints
        self.NU = len(uavs)
        const.NU = self.NU
        self.NT = len(tasks)
        const.NT = self.NT
        self.NCP = len(chargingPoints)
        const.NCP = self.NCP

    def getUavs(self) -> List[Uav]:
        return self.uavs

    def getUavs(self, index: int) -> Uav:
        return self.uavs[index]

    def getTasks(self) -> List[Task]:
        return self.tasks

    def getUavs(self, index: int) -> Task:
        return self.tasks[index]

    def getChargingPoints(self) -> List[ChargingPoint]:
        return self.chargingPoints

    def getUavs(self, index: int) -> ChargingPoint:
        return self.chargingPoints[index]