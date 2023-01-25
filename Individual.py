from Chromosome import Chromosome
from typing import List
import numpy as np

class Individual:
    def __init__(self, chromosome: Chromosome, evaluation: float) -> None:
        self.chromosome = chromosome
        self.evaluation = evaluation
    
    def getChromosome(self):
        return self.chromosome

    def getEvaluation(self):
        return self.evaluation
    
    def setChromosome(self, chromosome: Chromosome):
        self.chromosome = chromosome

    def setEvaluation(self, evaluation: float):
        self.evaluation = evaluation
    
    def individualEvaluation(self, chromosome: Chromosome) -> float:
        return self.objectiveFunction(self.uavsDistances(chromosome), self.uavsMasses(chromosome))

    def objectiveFunction(self, distancesUav: List[float], massesUav: List[float]):
        return self.ALPHA * max(distancesUav*massesUav) + self.BETA * (distancesUav*massesUav).sum()
    
    def uavsDistances(self, chromosome: Chromosome):
        uavsDistances = np.empty((len(chromosome.getCutPositions())))

        lastCuttingPosition = 0
        for uavNumber in range(len(chromosome.getCutPositions())):
            uavsDistances[uavNumber] = self.totalTasksDistance(uavNumber, chromosome.getTasksOrder()[lastCuttingPosition : int(chromosome.getCutPositions()[uavNumber])])
            lastCuttingPosition = uavNumber
        
        return uavsDistances
    
    def uavsMasses(self, chromosome: Chromosome):
        uavsMasses = np.empty((len(chromosome.getCutPositions())))

        for uavNumber in range(len(chromosome.getCutPositions())):
            uavsMasses[uavNumber] = self.uavs[uavNumber].getMass()
        
        return uavsMasses