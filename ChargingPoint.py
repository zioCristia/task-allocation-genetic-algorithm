from Position import Position
from Task import Task

class ChargingPoint(Task):
    def __init__(self, position: Position) -> None:
        Task.__init__(self, position, position, 365, 0, True)