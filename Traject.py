from Task import Task
from ChargingPoint import ChargingPoint
# from Uav import Uav
from Position import Position

class Traject:
    def __init__(self, startPosition: Position, task: Task, uavBatteryEnergy: float, uavTotalTimeSpent: float, *, energy: float = 0, velocity = 0, time = 0) -> None:
        self.startPosition = startPosition
        self.task = task
        self.uavBatteryEnergy = uavBatteryEnergy
        self.uavTotalTimeSpent = uavTotalTimeSpent

        self.energy = energy
        self.velocity = velocity
        self.time = time

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Traject):
            return self.startPosition == other.startPosition and self.task == other.task and self.uavBatteryEnergy == other.uavBatteryEnergy and self.uavTotalTimeSpent == other.uavTotalTimeSpent
        return False
    
    def __str__(self) -> str:
        details = ''
        details += f'Start position : {self.startPosition}\n'
        details += f'Task start position : {self.task.startPosition}\n'
        details += f'Energy : {self.energy}\n'
        details += f'velocity : {self.velocity}\n'
        details += f'time : {self.time}\n'
        return details
    
    def setUav(self, uav):
        self.uavTakingTask = uav

    def getEnergy(self):
        return self.energy

    def setEnergy(self, energy):
        self.energy = energy
    
    def getTime(self):
        return self.time

    def setTime(self, time):
        self.time = time

    def setVelocity(self, velocity):
        self.velocity = velocity