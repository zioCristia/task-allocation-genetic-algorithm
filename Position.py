class Position:
    def __init__(self, xPosition, yPosition):
        self.x = xPosition
        self.y = yPosition

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y
        return False
    
    def __str__(self) -> str:
        return "(x: " + str(self.x) + ", y:" + str(self.y) + ")"
    
    def getX(self):
        return self.x
        
    def getY(self):
        return self.y
