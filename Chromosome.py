import numpy as np
import random
from typing import List
from AlgoConstants import AlgoConstants as const

class Chromosome:
    def __init__(self, tasksOrder: List[int], cutPositions: List[int], *, timePerTaskPerUav = [], energyPerTaskPerUav = []):
        self.__tasksOrder = tasksOrder
        self.__tasksOrderWithRecharginTasks = tasksOrder
        self.__cutPositions = cutPositions
        self.setTasksPerUav()
        self.__timePerTaskPerUav = timePerTaskPerUav
        self.__energyPerTaskPerUav = energyPerTaskPerUav
    
    def __str__(self) -> str:
        details = ''
        for t in self.__tasksPerUav:
            details += f'{t} '
        return details

    @classmethod
    def fromTasksPerUav(cls, tasksPerUav: List):
        cutPositions = np.empty(len(tasksPerUav))
        tasksOrder = np.empty(0)

        offset = 0
        for u in range(len(tasksPerUav)):
            tasksOrder = np.append(tasksOrder, tasksPerUav[u])
            cutPositions[u] = len(tasksPerUav[u]) + offset
            offset += len(tasksPerUav[u])

        # (tasksOrder, cutPositions) = toChromosomes(tasksPerUav)
        return cls(tasksOrder = tasksOrder, cutPositions = cutPositions)

    def getTasksOrder(self):
        return self.__tasksOrder
    
    def getCutPositions(self):
        return self.__cutPositions
    
    def getTasksPerUav(self) -> List:
        return self.__tasksPerUav

    def setTasksOrder(self, tasksOrder):
        self.__tasksOrder = list(map(int, tasksOrder))
        self.__tasksPerUav = self.setTasksPerUav()

    def setTasksWithRT(self, tasksOrder):
        self.__tasksOrderWithRecharginTasks

    def setCutPositions(self, cutPositions):
        self.__cutPositions = list(map(int, cutPositions))
        self.__tasksPerUav = self.setTasksPerUav()
    
    def respectDeliveryWidow(self) -> bool:
        for uavTasks in self.__tasksPerUav:
            timeSpent = 0

            for taskIndex in uavTasks:
                currentTask = self.allTasks[int(self.__tasksOrder[taskIndex])]
                timeSpent += self.__timePerTaskPerUav[uavTasks][taskIndex]

                if currentTask.getMaxDeliveryWindow < timeSpent:
                    return False
            
        return True
    
    def respectDeliveryWindowOld(self) -> bool:
        offset = 0
        lastOffset = 0
        currentDrone = 0
        for t in range(len(self.__tasksOrder)):
            if (self.__cutPositions[currentDrone] == t):
                lastOffset = offset
                currentDrone += 1
            
            offset += 1
            currentTask = self.allTasks[int(self.__tasksOrder[t])]
            if not currentTask.isChargingPoint() and currentTask.getMaxDeliveryWindow() < t - lastOffset:
                return False
        
        return True

    def setTasksPerUav(self):
        tasksPerUav = []

        for u in range(len(self.__cutPositions)):
            droneTask = np.empty(0)

            if u == 0 and self.getCutPositions()[0] != 0:
                droneTask = self.getTasksOrder()[0:int(self.getCutPositions()[0])]
            else:
                droneTask = self.getTasksOrder()[int(self.getCutPositions()[u-1]):int(self.getCutPositions()[u])]

            tasksPerUav.append(np.array(droneTask))

        self.__tasksPerUav = tasksPerUav

    def setTimePerTaskPerUav(self, timePerTaskPerUav: List[float]):
        self.__timePerTaskPerUav = timePerTaskPerUav
        
    def setEnergyPerTaskPerUav(self, energyPerTaskPerUav: List[float]):
        self.__energyPerTaskPerUav = energyPerTaskPerUav

    def getTimePerTaskPerUav(self) -> List[float]:
        return self.__timePerTaskPerUav
        
    def getEnergyPerTaskPerUav(self) -> List[float]:
        return self.__energyPerTaskPerUav

    @classmethod
    def toChromosomes(cls, tasksPerUav: List):
        # not used
        cutPositions = np.empty(len(tasksPerUav))
        tasksOrder = np.empty(0)

        offset = 0
        for u in range(len(tasksPerUav)):
            tasksOrder = np.append(tasksOrder, tasksPerUav[u])
            cutPositions[u] = len(tasksPerUav[u] + offset)
            offset += len(tasksPerUav[u])

    def crossWith(self, tasksOrderForCross: List[int]):
        """Perform partially-mapped crossover (PMX) with the input chromosome.

        Args:
            tasksOrderForCross (List[int]): The task order for the second parent chromosome.

        Returns:
            Chromosome: The offspring chromosome generated from the input parents and the same cut positions.
        """
        # select two random cut points
        cut_points = sorted(np.random.choice(range(1, len(self.__tasksOrder)), size=2, replace=False))

        offspring = self.__tasksOrder.copy()
        
        # determine mapping sections and mapping pairs
        section1 = self.__tasksOrder[cut_points[0]:cut_points[1] + 1]
        section2 = tasksOrderForCross[cut_points[0]:cut_points[1] + 1]
        mapping_pairs = list(zip(section1, section2))
        
        # create mapping dictionaries for each parent
        mapping_dict1 = {pair[0]: pair[1] for pair in mapping_pairs}
        mapping_dict2 = {pair[1]: pair[0] for pair in mapping_pairs}
        
        # fill in remaining genes using mappings
        for i in range(len(offspring)):
            if i < cut_points[0] or i > cut_points[1]:
                while offspring[i] in mapping_dict2:
                    offspring[i] = mapping_dict2[offspring[i]]
        
        # fill in mapping sections of offspring with values from other parent
        offspring[cut_points[0]:cut_points[1] + 1] = section2
        
        return Chromosome(offspring, self.__cutPositions)

    