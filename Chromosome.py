import numpy as np
from typing import List

class Chromosome:
    def __init__(self, tasksOrder: List[int], cutPositions: List[int]):
        self.__tasksOrder = tasksOrder
        self.__cutPositions = cutPositions

    # def __init__(self, tasksNumbers: int, uavNumbers: int):
    #     self.__tasksOrder = np.empty(tasksNumbers)
    #     self.__cutPositions = np.empty(uavNumbers)

    def getTasksOrder(self):
        return self.__tasksOrder
    
    def getCutPositions(self):
        return self.__cutPositions
    
    def setTasksOrder(self, tasksOrder):
        self.__tasksOrder = tasksOrder

    def setCutPositions(self, cutPositions):
        self.__cutPositions = cutPositions
