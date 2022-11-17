from Position import Position

class Uav:
    def __init__(self, batteryCapacity: float, mass: float, maxPayloadMass: float, startPosition: Position) -> None:
        self.batteryCapacity = batteryCapacity
        self.mass = mass
        self.maxPayloadMass = maxPayloadMass
        self.startPosition = startPosition
        # not still used
        self.currentBatteryCapacity = batteryCapacity
        self.currentPayloadMass = 0
    
    def getMaxPayloadMass(self) -> float:
        return self.maxPayloadMass

    def getStartPosition(self) -> Position:
        return self.startPosition
    
    def getMass(self):
        return self.mass
    