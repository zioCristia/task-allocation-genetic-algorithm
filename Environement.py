from Task import Task
from Uav import Uav
from ChargingPoint import ChargingPoint
from typing import List

class Environement:
    def __init__(self, uavs: List[Uav], tasks: List[Task], chargingPoints: List[ChargingPoint]):
        self.uavs = uavs
        self.tasks = tasks
        self.chargingPoints = chargingPoints