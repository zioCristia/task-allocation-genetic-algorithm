import utility
from Position import Position

class Task:
    def __init__(self, startPosition: Position, endPosition: Position, maxDeliveryWindow: int, packageMass: float, chargingPoint: bool = False) -> None:
        self.startPosition = startPosition
        self.endPosition = endPosition
        self.trajectDistance = utility.distance(startPosition, endPosition)
        self.packageMass = packageMass
        self.maxDeliveryWindow = maxDeliveryWindow
        self.chargingPoint = chargingPoint

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Task):
            return self.startPosition == other.startPosition and self.endPosition == other.endPosition and self.packageMass == other.packageMass
        return False
    
    def __str__(self) -> str:
        details = ''
        details += f'Start position : {self.startPosition}\n'
        details += f'End position : {self.endPosition}\n'
        details += f'Package mass : {self.packageMass}\n'
        details += f'Max delivery window : {self.maxDeliveryWindow}\n'
        return details

    def getStartPosition(self) -> Position:
        return self.startPosition

    def getEndPosition(self) -> Position:
        return self.endPosition

    def getTrajectDistance(self) -> float:
        return self.trajectDistance
    
    def getPackageMass(self) -> float:
        return self.packageMass
    
    def getMaxDeliveryWindow(self) -> int:
        return self.maxDeliveryWindow

    def isChargingPoint(self):
        return self.chargingPoint

    def __repr__(self):
        return f"{self.__class__.__name__}(startPosition={self.startPosition})"
    