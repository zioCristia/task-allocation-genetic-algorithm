import math

def distance(position1, position2):
    return math.sqrt((position1.x - position2.x)**2 + (position1.y - position2.y)**2)

# def printDronesTaskes(chromosomes cutPosition):
#     for d in range(constant.NU):
#         print("Drone " + d + ": ")
#         for t in range(cutPosition[d-1],cutPosition[d]):
#             print(chromosomes[t])
    
