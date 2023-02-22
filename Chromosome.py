import numpy as np
from typing import List
from AlgoConstants import AlgoConstants as const

class Chromosome:
    def __init__(self, tasksOrder: List[int], cutPositions: List[int]):
        self.__tasksOrder = tasksOrder
        self.__tasksOrderWithRecharginTasks = tasksOrder
        self.__cutPositions = cutPositions
        self.setTasksPerUav()
    
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

    def respectDeliveryWindow(self) -> bool:
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
        leftCutIndex, rightCutIndex = self.randomCrossoverCuttingIndex()
        storedTaskOrder = np.array(self.__tasksOrder)

        fromFirstParent = storedTaskOrder[leftCutIndex:rightCutIndex]
        fromSecondParent = tasksOrderForCross[leftCutIndex:rightCutIndex]

        for p in range(leftCutIndex,rightCutIndex):
            i = np.where(self.getTasksOrder() == tasksOrderForCross[p])[0]
            if len(i) > 0:
                i = i[0]

                if i < leftCutIndex and i > rightCutIndex:
                    ni = self.newMappedTaskIndex(p, tasksOrderForCross[leftCutIndex:rightCutIndex])
                    self.__tasksOrder[i] = self.__tasksOrder[ni]

        self.__tasksOrder[leftCutIndex:rightCutIndex] = tasksOrderForCross[leftCutIndex:rightCutIndex]
        if sum(self.__tasksOrder) != 10:
            raise Exception("Crossing operation error")
        self.setTasksPerUav()

    def newMappedTaskIndex(self, startIndex, cuttedTaskOrder) -> int:
        newMappedTask = self.__tasksOrder[startIndex]
        newIndex = np.where(cuttedTaskOrder == newMappedTask)[0]

        if len(newIndex) > 0 :#and newMappedTask != cuttedTaskOrder[newIndex]:
            newIndex = newIndex[0]
            # self.__tasksOrder[i] = self.__tasksOrder[t]
            return self.newMappedTaskIndex(newIndex, cuttedTaskOrder)
        # else:
        #     self.__tasksOrder[i] = self.__tasksOrder[p]

        return startIndex
    
    def randomCrossoverCuttingIndex(self):
        leftCutIndex = np.random.randint(const.NT)
        rightCutIndex = np.random.randint(const.NT)
        while (leftCutIndex >= rightCutIndex):
            leftCutIndex = np.random.randint(const.NT)
            rightCutIndex = np.random.randint(const.NT)
        
        return leftCutIndex, rightCutIndex