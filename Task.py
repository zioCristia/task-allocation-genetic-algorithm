import utility
from Position import Position

class Task:
    def __init__(self, startPosition: Position, endPosition: Position, maxDeliveryWindow: int, packageMass: float, isChargingPoint: bool = False) -> None:
        self.__startPosition = startPosition
        self.__endPosition = endPosition
        self.__trajectDistance = utility.distance(startPosition, endPosition)
        self.__packageMass = packageMass
        self.__maxDeliveryWindow = maxDeliveryWindow
        self.__isChargingPoint = isChargingPoint

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

    def isChargingPoint(self):
        return self.__isChargingPoint

    def energyRequired(self, uav) -> float:
        # TODO: calculate the energy required by the uav to accomplish the task
        return 0
    