from Chromosome import Chromosome
from typing import List

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