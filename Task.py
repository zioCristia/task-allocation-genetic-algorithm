import utility
from Position import Position

class Task:
    def __init__(self, startPosition: Position, endPosition: Position, maxDeliveryWindow: int, packageMass: float = 1) -> None:
        self.__startPosition = startPosition
        self.__endPosition = endPosition
        self.__trajectDistance = utility.distance(startPosition, endPosition)
        self.__packageMass = packageMass
        self.__maxDeliveryWindow = maxDeliveryWindow

    def getStartPosition(self) -> Position:
        return self.__startPosition

    def getEndPosition(self) -> Position:
        return self.__endPosition

    def getTrajectDistance(self) -> float:
        return self.__trajectDistance
    
    def getPackageMass(self) -> float:
        return self.__packageMass
    
    def getMaxDeliveryWindow(self) -> int:
        return self.__maxDeliveryWindow
    