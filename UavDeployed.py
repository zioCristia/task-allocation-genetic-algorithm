from Uav import Uav
from Task import Task
from typing import List

class UavDeployed(Uav):
    def __init__(self, tasks: List[Task], **kwargs) -> None:
        super().__init__(**kwargs)
        self.tasks = tasks

    @classmethod
    def fromUav(cls, uav, tasks):
        return cls(batteryCapacity=uav.batteryCapacity, mass=uav.mass, maxPayloadMass = uav.maxPayloadMass, startPosition = uav.startPosition,  tasks=tasks)