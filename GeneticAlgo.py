import math
from random import randint
from re import I
import numpy as np
import matplotlib.pyplot as plt
import utility
from Chromosome import Chromosome
from Individual import Individual
from Task import Task
from Uav import Uav
from Position import Position
from typing import List
#from numpy.typing import NDArray

class GeneticAlgo:
    # minimum time weight factor
    ALPHA = 0.5
    # minimum consumption weight factor (alpha + beta = 1)
    BETA = 0.5
    # number of population individuals (paire number at least > 6*BEST_TAKEN)
    NP = 60
    # number of the fittest chromosomes taken without the roulette wheel selection
    BEST_TAKEN = 5
    # crossover probability
    PC = 0.3
    # mutation probability
    PM = 0.3
    # max iterations for genetic algo
    MAX_ITER = 300
    # number of the offspring individuals for mutation porpousos
    GROUP_MUTATION_NUMBER = 8

    population = np.empty((NP), dtype=Individual)
    oppositePopulation = np.empty((NP), dtype=Individual)
    solution = Individual(Chromosome(0,0), 0)

    def __init__(self, uavs: List[Uav], tasks: List[Task]): #np.ndarray for typing np array
        self.uavs = uavs
        self.tasks = tasks
        self.NU = len(uavs)
        self.NT = len(tasks)
        self.run()

    def getSolution(self):
        return self.solution

    def individualCreation(self) -> Individual:
        tasks = self.randomTaskAssignation()
        cutPosition = self.randomCutPosition()

        chromosome = Chromosome(tasks, cutPosition)

        return Individual(chromosome, self.individualEvaluation(chromosome))
    
    def randomTaskAssignation(self) -> List[int]:
        chromosome = list(range((self.NT)))
        np.random.shuffle(chromosome)

        return chromosome

    def randomCutPosition(self) -> List[int]:
        cutPosition = np.empty(self.NU)
        previousPosition = 0
        for i in range(self.NU-1):
            cutPosition[i] = np.random.choice(list(range(int(previousPosition), self.NT)))
            previousPosition = cutPosition[i]
        
        cutPosition[self.NU - 1] = self.NT
        
        return cutPosition

    def oppositeIndividualCreation(self, chromosome: Chromosome) -> Individual:
        oppositeTasks = np.empty((self.NT))
        for t in range(len(chromosome.getTasksOrder())): # or len()
            oppositeTasks[t] = self.NT - chromosome.getTasksOrder()[t] - 1
        
        chromosome = Chromosome(oppositeTasks, chromosome.getCutPositions())

        return Individual(chromosome, self.individualEvaluation(chromosome))

    def individualEvaluation(self, chromosome: Chromosome) -> float:
        return self.objectiveFunction(self.uavsDistances(chromosome))
    
    def uavsDistances(self, chromosome: Chromosome):
        uavsDistances = np.empty((len(chromosome.getCutPositions())))

        lastCuttingPosition = 0
        for uavNumber in range(len(chromosome.getCutPositions())):
            uavsDistances[uavNumber] = self.totalTasksDistance(uavNumber, chromosome.getTasksOrder()[lastCuttingPosition : int(chromosome.getCutPositions()[uavNumber])])
            lastCuttingPosition = uavNumber
        
        return uavsDistances

    def totalTasksDistance(self, uavIndex: int, tasksIndexes: List[int]) -> float:
        distance = 0
        lastUavPosition = self.uavs[uavIndex].getStartPosition()

        for t in tasksIndexes:
            t = int(t)
            distance += (self.tasks[t].getTrajectDistance() + utility.distance(lastUavPosition, self.tasks[t].getStartPosition()))
            lastUavPosition = self.tasks[t].getEndPosition()
        
        distance += utility.distance(lastUavPosition, uavs[uavIndex].startPosition)
        return distance

    def objectiveFunction(self, distancesUav: List[float]):
        return self.ALPHA * max(distancesUav) + self.BETA * distancesUav.sum()

    def initialPopulationCreation(self):
        for i in range(self.NP):
            self.population[i] = self.individualCreation()

    def oppositePopulationCreation(self):
        for i in range(self.NP):
            self.oppositePopulation[i] = self.oppositeIndividualCreation(self.population[i].getChromosome())

    def newPopulationSelection(self):
        # we take BEST_TAKEN chromosomes from population and opposite population and put them directly
        # in the output population. The rest is taken by a roulette wheel selection
        self.population = np.concatenate((self.takeBestN(self.population, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.population),
                                self.takeBestN(self.oppositePopulation, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.oppositePopulation)))

    def takeBestN(self, population: List[Individual], n: int) -> List[Individual]:
        bestIndividuals = np.empty((n), dtype=Individual)

        for i in range(n):
            best = np.argmin(population[i].getEvaluation())
            bestIndividuals[i] = population[best]
            population = np.delete(population, best, 0)

        return bestIndividuals

    def rouletteWheelSelection(self, population):
        individualsChosen = np.empty((int(self.NP/2) - self.BEST_TAKEN), dtype=Individual)

        for i in range(int(self.NP/2) - self.BEST_TAKEN):
            choice = np.random.choice(self.NP-i, p=self.probabilitiesSelection(self.getAllEvaluations(population)))
            individualsChosen[i] = population[choice]
            population = np.delete(population, choice, 0)

        return individualsChosen

    def probabilitiesSelection(self, evaluations):
        # create the proportional probabilities which are used for the roulette wheel selection
        probabilities = np.empty((evaluations.shape[0]))

        revEvaluations = self.reverseEvaluations(evaluations)
        sumEvaluations = revEvaluations.sum()

        for i in range(evaluations.shape[0]):
            probabilities[i] = revEvaluations[i] / sumEvaluations
        
        return probabilities

    def reverseEvaluations(self, evaluations):
        reverse = np.empty((evaluations.shape[0]))
        for e in range(evaluations.shape[0]):
            reverse[e] = 1 / evaluations[e]
        
        return reverse

    def getAllEvaluations(self, population):
        evaluations = np.empty((len(population)))
        for i in range(len(population)):
            evaluations[i] = population[i].getEvaluation()

        return evaluations

    def populationCrossover(self):
        # per ogni chormosoma in chromosomes, se la probabilità è minore della probabilità di crossover
        # esegui il crossover tra lui e un altro individuo a caso e poi lo aggiungi nella populazione di output,
        # sennò lo metti direttamente nell'output
        for c in range(self.NP):
            if np.random.rand() < self.PC:
                chromosomeForCross = np.random.randint(self.NT)
                while chromosomeForCross == c:
                    chromosomeForCross = np.random.randint(self.NT)
                
                newTaskOrder = self.individualCrossover(np.array(self.population[c].getChromosome().getTasksOrder()),
                                    self.population[chromosomeForCross].getChromosome().getTasksOrder())
                self.population[c].getChromosome().setTasksOrder(newTaskOrder)
                self.population[c].setEvaluation(self.individualEvaluation(self.population[c].getChromosome()))

    def individualCrossover(self, chromosomeToCross: List[int], chromosomeForCross):
        l = np.random.randint(self.NT)
        r = np.random.randint(self.NT)
        while (l >= r):
            l = np.random.randint(self.NT)
            r = np.random.randint(self.NT)

        for p in range(l,r):
            i = np.where(chromosomeToCross == chromosomeForCross[p])
            # i = chromosomeToCross.index(chromosomeForCross[p])
            chromosomeToCross[i] = chromosomeToCross[p]

        chromosomeToCross[l:r] = chromosomeForCross[l:r]

        return chromosomeToCross

    def offspringCreation(self):
        for i in range(0, self.NP, self.GROUP_MUTATION_NUMBER):
            if np.random.rand() < self.PM and i+self.GROUP_MUTATION_NUMBER < self.NP:
                self.population[i:i+self.GROUP_MUTATION_NUMBER] = self.groupMutation(self.population[i:i+self.GROUP_MUTATION_NUMBER])

    def groupMutation(self, individuals: List[Individual]) -> List[Individual]:
        bestChromosome = self.takeBestN(individuals, 1)[0].getChromosome()

        bestTasksOrder = bestChromosome.getTasksOrder()
        flipTasksOrder = self.flipMutation(bestTasksOrder)
        swapTasksOrder = self.swapMutation(bestTasksOrder)
        slideTasksOrder = self.slideMutation(bestTasksOrder)

        bestCutPositions = bestChromosome.getCutPositions()
        mutatedCutPosition = self.randomCutPosition()

        individuals[0] = Individual(bestTasksOrder, bestCutPositions)
        individuals[1] = Individual(flipTasksOrder, bestCutPositions)
        individuals[2] = Individual(swapTasksOrder, bestCutPositions)
        individuals[3] = Individual(slideTasksOrder, bestCutPositions)

        individuals[4] = Individual(bestTasksOrder, mutatedCutPosition)
        individuals[5] = Individual(flipTasksOrder, mutatedCutPosition)
        individuals[6] = Individual(swapTasksOrder, mutatedCutPosition)
        individuals[7] = Individual(slideTasksOrder, mutatedCutPosition)

        return individuals

    def flipMutation(self, tasksOrder: List[int]) -> List[int]:
        # TODO implement as in the paper
        position1 = np.random.randint(self.NT)
        position2 = np.random.choice(list(range(int(position1), self.NT)))
        return 0

    def swapMutation(self, individual: List[int]) -> List[int]:
        # TODO implement as in the paper
        return 0

    def slideMutation(self, individual: List[int]) -> List[int]:
        # TODO implement as in the paper
        return 0

    def run(self):
        self.initialPopulationCreation()

        print("\nBest starting evaluation")
        print(self.takeBestN(self.population, 1)[0].getEvaluation())

        for i in range(self.MAX_ITER):
            self.oppositePopulationCreation()
            self.newPopulationSelection()
            self.populationCrossover()
            # TODO: add mutation
            # offspringCreation(initialPopul)

        self.solution = self.takeBestN(self.population, 1)[0]
        
        print("\nBest ending evaluation")
        print(self.solution.getEvaluation())
        print("Best chromosomes")
        print(self.solution.getChromosome().getTasksOrder())
        print(self.solution.getChromosome().getCutPositions())
    
    def getSolution(self):
        return self.solution

    def graphSolution(self):
        solutionChromosome = self.solution.getChromosome()
        plt.plot([0], [0])
        for t in tasks:
            plt.plot([t.getStartPosition().getX(), t.getEndPosition().getX()], [t.getStartPosition().getY(), t.getEndPosition().getY()])
            plt.plot([t.getStartPosition().getX()], [t.getStartPosition().getY()], 'o')
            plt.plot([t.getEndPosition().getX()], [t.getEndPosition().getY()], '^')

        for i in range(self.NU):
            droneTask = np.empty(0)

            if i == 0 and solutionChromosome.getCutPositions()[0] != 0:
                droneTask = solutionChromosome.getTasksOrder()[0:int(solutionChromosome.getCutPositions()[0])]
            else:
                droneTask = solutionChromosome.getTasksOrder()[int(solutionChromosome.getCutPositions()[i-1]):int(solutionChromosome.getCutPositions()[i])]
            
            prevPosition = Position(0,0)
            for t in droneTask:
                currentTask = tasks[int(t)]
                currentPos = currentTask.getStartPosition()
                plt.plot([prevPosition.getX(), currentPos.getX()], [prevPosition.getY(), currentPos.getY()], '--')
                prevPosition = currentTask.getEndPosition()

        plt.show()


uav1 = Uav(5000, 2, 1.5, Position(0, 0))
uav2 = Uav(5000, 2, 1.5, Position(0, 0))
task1 = Task(Position(1, 1), Position(1, 3))
task2 = Task(Position(3, 7), Position(2, 3))
task3 = Task(Position(8, 2), Position(8, 4))
task4 = Task(Position(6, 4), Position(4, 6))
task5 = Task(Position(1, 4), Position(7, 3))

uavs = np.array((uav1,uav2))
tasks = np.array([task1, task2, task3, task4, task5])

run = GeneticAlgo(uavs, tasks)
print(run.getSolution())
run.graphSolution()