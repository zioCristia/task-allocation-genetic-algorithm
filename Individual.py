from Chromosome import Chromosome
from typing import List
import numpy as np

class Individual:
    def __init__(self, chromosome: Chromosome, evaluation: float = 0) -> None:
        self.chromosome = chromosome
        self.evaluation = evaluation
    
    def __str__(self) -> str:
        details = ''
        details += f'Chromosome: {self.chromosome}\t'
        details += f'Evaluation: {self.evaluation}'
        return details
    
    @classmethod
    def fromIndividual(cls, individual):
        return cls(chromosome = individual.getChromosome(), evaluation = individual.getEvaluation())

    def getChromosome(self):
        return self.chromosome

    def getEvaluation(self):
        return self.evaluation
    
    def setChromosome(self, chromosome: Chromosome):
        self.chromosome = chromosome

    def setEvaluation(self, evaluation: float):
        self.evaluation = evaluation
    