import utility
from Position import Position

class Task:
    def __init__(self, 
                 startPositionId: int, 
                 endPositionId: int, 
                 maxDeliveryWindow: int, 
                 packageMass: float, 
                 chargingPoint: bool = False) -> None:
        
        self.startPositionId = startPositionId
        self.endPositionId = endPositionId
        self.packageMass = packageMass
        self.maxDeliveryWindow = maxDeliveryWindow
        self.chargingPoint = chargingPoint

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Task):
            return self.startPositionId == other.startPositionId and self.endPositionId == other.endPositionId and self.packageMass == other.packageMass
        return False
    
    def __str__(self) -> str:
        details = ''
        details += f'Start position id : {self.startPositionId}\n'
        details += f'End position id : {self.endPositionId}\n'
        details += f'Package mass : {self.packageMass}\n'
        details += f'Max delivery window : {self.maxDeliveryWindow}\n'
        return details

    def getStartPositionId(self) -> Position:
        return self.startPositionId

    def getEndPositionId(self) -> Position:
        return self.endPositionId
    
    def getPackageMass(self) -> float:
        return self.packageMass
    
    def getMaxDeliveryWindow(self) -> int:
        return self.maxDeliveryWindow

    def setMaxDeliveryWindow(self, maxDeliveryWindow: int):
        self.maxDeliveryWindow = maxDeliveryWindow

    def isChargingPoint(self):
        return self.chargingPoint

    def __repr__(self):
        return f"{self.__class__.__name__}(startPositionId={self.startPositionId})"
    