from Uav import Uav
from Task import Task
from typing import List
from Position import Position

class UavDeployed(Uav):
    def __init__(self, *, position: Position = Position(0,0), currentBatteryCapacity = 0, currentTask: Task = None, timeSpentPerTask: List = [], trajectsEnergy: List = [], **kwargs) -> None:
        # TODO: check if the constructor works properly
        super().__init__(**kwargs)

        self.position = position
        self.currentBatteryCapacity = currentBatteryCapacity
        if isinstance(currentTask, Task):
            self.currentTask = self.setCurrentTask(currentTask)
        self.timeSpentPerTask = timeSpentPerTask
        self.trajectsEnergy = trajectsEnergy

    @classmethod
    def fromUav(cls, uav, tasks):
        return cls(batteryCapacity=uav.batteryCapacity, mass=uav.mass, maxPayloadMass = uav.maxPayloadMass, startPosition = uav.startPosition,  tasks=tasks)