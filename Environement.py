from Task import Task
from Uav import Uav
from ChargingPoint import ChargingPoint
from AlgoConstants import AlgoConstants as const
from typing import List

class Environement:
    def __init__(self, uavs: List[Uav], tasks: List[Task], chargingPoints: List[ChargingPoint]):
        self.__uavs = uavs
        self.__tasks = tasks
        self.__chargingPoints = chargingPoints
        self.__allTasks = [tasks, chargingPoints]

        self.__NU = len(uavs)
        const.NU = self.__NU

        self.__NT = len(tasks)
        const.NT = self.__NT

        self.__NCP = len(chargingPoints)
        const.NCP = self.__NCP

    def getUavs(self) -> List[Uav]:
        return self.__uavs

    def getUav(self, index: int) -> Uav:
        return self.__uavs[index]

    def getTasks(self) -> List[Task]:
        return self.__tasks

    def getTask(self, index: int) -> Task:
        return self.__tasks[index]

    def getChargingPoints(self) -> List[ChargingPoint]:
        return self.__chargingPoints

    def getChargingPoint(self, index: int) -> ChargingPoint:
        return self.__chargingPoints[index]

    def getAllTasks(self) -> List[ChargingPoint]:
        return self.__allTasks

    def getAllTask(self, index: int) -> ChargingPoint:
        return self.__allTasks[index]