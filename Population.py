from Individual import Individual
from Chromosome import Chromosome
from typing import List
import numpy as np
from AlgoConstants import AlgoConstants as aconst

class Population:
    def __init__(self, individuals: List[Individual]) -> None:
        self.population = individuals
        self.np = len(individuals)

    def crossover(self):
        for i in range(self.np):
            if np.random.rand() < aconst.PC:
                self.individualCrossover(i)

    def individualCrossover(self, individualIndex: int):
        # TOFIX check if it works
        chromosomeForCross = self.randomChromosomeForCrossDifferentFrom(individualIndex)
        chromosome = self.population[individualIndex].getChromosome()
        
        chromosome.crossWith(self.population[chromosomeForCross].getChromosome().getTasksOrder())
        self.population[individualIndex] = Individual(chromosome)
    
    def randomChromosomeForCrossDifferentFrom(self, n: int) -> int:
        forCross = np.random.randint(self.np)
        while forCross == n:
                    forCross = np.random.randint(self.NT)

        return forCross