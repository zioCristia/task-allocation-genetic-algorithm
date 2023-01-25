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
from ChargingPoint import ChargingPoint

"""
VINCOLI:
* max payload per drone
* delivery window
* preferire drone che consuma meno
* preferire ripartizione che consuma meno (fitting function t*m)

TO ADD:
- iteration stop if last best evaluation similar current evaluation
- add use of respectDeliveryWindow()

- dopo deliveryWindowEvaluation, facciamo il controllo per ogni drone sulla batteria consumata e aggiungo task
    di ricarica se necessarie
- prossima task è fatta se basta la batteria, sennò prima faccio la ricarica
- velocità scelta per minimizzare l'eq1, in base a quella troviamo il tempo da dare alla fitting function
- aggiungo nel task order la task di ricarica stando attenti ad aggiornare le cut positions
- le task di ricarica sono per un primo momento nel punto 0 ma poi ne aggiungiamo diverse e mettiamo una ricerca alla migliore
- alla fine faccio di nuovo un controllo sul delivery window
"""

class GeneticAlgo:
    # minimum maximum time weight factor
    ALPHA = 0.5
    # minimum consumption weight factor (alpha + beta = 1)
    BETA = 0.5
    # number of population individuals (paire number at least > 6*BEST_TAKEN)
    NP = 60
    # number of the fittest chromosomes taken without the roulette wheel selection
    BEST_TAKEN = 3
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

    def __init__(self, uavs: List[Uav], tasks: List[Task], chargingPoints: List[ChargingPoint]): #np.ndarray for typing np array
        self.uavs = uavs
        self.tasks = tasks
        self.chargingPoints = chargingPoints
        self.allTasks = np.append(tasks, chargingPoints)
        self.NU = len(uavs)
        self.NT = len(tasks)
        self.NCP = len(chargingPoints)
        self.uavsTasksEnergy = np.empty(self.NU)
        self.run()

    def getSolution(self):
        return self.solution

    def getUavs(self):
        return self.uavs
    
    def individualCreation(self) -> Individual:
        chromosome = self.chromosomeCreation()
        while not self.respectDeliveryWindow(chromosome):
            chromosome = self.chromosomeCreation()
        
        return Individual(chromosome, self.individualEvaluation(chromosome))

    def chromosomeCreation(self) -> Chromosome:
        tasksOrder = np.empty(0)
        cutPositions = np.empty(self.NU)
        uavTaskSelection = self.taskUavSelection()
        
        offset = 0
        for u in range(self.NU):
            uavTask = np.where(uavTaskSelection == u)[0]
            np.random.shuffle(uavTask)
            uavTask = self.addChargingTasks(uavTask, u)
            tasksOrder = np.concatenate((tasksOrder, uavTask))
            cutPositions[u] = len(uavTask) + offset
            offset = len(tasksOrder)
        
        return Chromosome(tasksOrder, cutPositions)

    def addChargingTasks(self, taskOrder, uavNumber):
        uavs[uavNumber].reset()

        if len(taskOrder) == 0: 
            return taskOrder

        uav = uavs[uavNumber]

        uav.takeTask(tasks[taskOrder[0]])
        for t in range(1, len(taskOrder) - 1):
            tempUav = uav
            cp = self.optimumChargingPoint(tasks[t+1])

            if not tempUav.canTakeTasks(np.array([tasks[t], self.chargingPoints[cp]])):
                # the drone can't complete the task t and charge after, we go to a charging point first
                uav.takeTask(self.chargingPoints[cp])
                taskOrder = np.insert(taskOrder, t, cp + self.NT)   # bc we add the charging task at the end in all tasks array
                t+=1

            else:
                tempUav.takeTask(self.tasks[t])
                uav = tempUav

        uav.takeTask(tasks[taskOrder[len(taskOrder)-1]])
        uavs[uavNumber] = uav

        return taskOrder
    
    def optimumChargingPoint(self, task: Task, nextTask: Task = 0):
        distances = utility.tasksDistances(task.getEndPosition(), self.chargingPoints)

        if nextTask != 0:
            # TODO: check only the first elements of distances
            for cp in range(self.NCP):
                distances[cp] = utility.taskDistance(task.getEndPosition(), self.chargingPoints[cp]) + utility.taskDistance(self.chargingPoints[cp], nextTask.getEndPosition())
        
        return np.argmin(distances)

    def taskUavSelection(self):
        uavSelection = np.empty(self.NT)

        probabilitiesMatrix = self.taskRestrictionProbabilities()
        for t in range(self.NT):
            uavSelection[t] = np.random.choice(self.NU, p=probabilitiesMatrix[t])
        
        return uavSelection

    def taskRestrictionProbabilities(self):
        return self.mostEfficientUavProbabilities(self.maxPayloadMatrix())

    def mostEfficientUavProbabilities(self, tasksUavsMatrix):
        for t in range(self.NT):
            uavsMassSum = 0
            for u in np.where(tasksUavsMatrix[t] == 1)[0]:
                uavsMassSum += 1/uavs[u].getMass()
            
            for u in range(self.NU):
                tasksUavsMatrix[t][u] *= (1/uavs[u].getMass())/uavsMassSum
        
        return tasksUavsMatrix

    def maxPayloadMatrix(self):
        # TODO: add function to control all the task can be performed (min(sum(righe))>0)
        maxPayload = np.zeros((self.NT, self.NU))

        for t in range(self.NT):
            for u in range(self.NU):
                if self.tasks[t].getPackageMass() < self.uavs[u].getMaxPayloadMass():
                    maxPayload[t][u] = 1
        
        return maxPayload

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
        for u in range(self.NU):
            self.uavsTasksEnergy[u] = uavs[u].getTotalEnergyUsed()

        #return self.distanceMassObjectiveFunction(self.getUavsDistances(chromosome), self.getUavsMasses(chromosome))
        # energy optimization
        return self.energyObjectiveFunction()
    
    def respectConstraints(self, chromosome: Chromosome) -> bool:
        # TODO: add here the other constraints (payload)
        return self.respectDeliveryWindow(chromosome)

    def respectDeliveryWindow(self, chromosome: Chromosome) -> bool:
        offset = 0
        lastOffset = 0
        currentDrone = 0
        for t in range(len(chromosome.getTasksOrder())):
            if (chromosome.getCutPositions()[currentDrone] == t):
                lastOffset = offset
                currentDrone += 1
            
            offset += 1
            if (tasks[int(chromosome.getTasksOrder()[t])].getMaxDeliveryWindow() < t - lastOffset):
                return False
        
        return True

    def getUavsDistances(self, chromosome: Chromosome) -> List[float]:
        uavsDistances = np.empty((len(chromosome.getCutPositions())))

        lastCuttingPosition = 0
        for uavNumber in range(len(chromosome.getCutPositions())):
            uavsDistances[uavNumber] = self.totalTasksDistance(uavNumber, chromosome.getTasksOrder()[lastCuttingPosition : int(chromosome.getCutPositions()[uavNumber])])
            lastCuttingPosition = uavNumber
        
        return uavsDistances
    
    def getUavsMasses(self, chromosome: Chromosome) -> List[float]:
        uavsMasses = np.empty((len(chromosome.getCutPositions())))

        for uavNumber in range(len(chromosome.getCutPositions())):
            uavsMasses[uavNumber] = self.uavs[uavNumber].getMass()
        
        return uavsMasses

    def totalTasksDistance(self, uavIndex: int, tasksIndexes: List[int]) -> float:
        distance = 0
        lastUavPosition = self.uavs[uavIndex].getPosition()

        for t in tasksIndexes:
            t = int(t)
            distance += (self.tasks[t].getTrajectDistance() + utility.distance(lastUavPosition, self.tasks[t].getStartPosition()))
            lastUavPosition = self.tasks[t].getEndPosition()
        
        distance += utility.distance(lastUavPosition, uavs[uavIndex].getPosition())
        return distance

    def distanceMassObjectiveFunction(self, distancesUav: List[float], massesUav: List[float]):
        return self.ALPHA * max(distancesUav*massesUav) + self.BETA * (distancesUav*massesUav).sum()

    def energyObjectiveFunction(self):
        return self.ALPHA * max(self.uavsTasksEnergy) + self.BETA * self.uavsTasksEnergy.sum()

    def initialPopulationCreation(self):
        for i in range(self.NP):
            self.population[i] = self.individualCreation()

    def oppositePopulationCreation(self):
        self.oppositePopulation = np.empty(self.NP, dtype=Individual)
        for i in range(self.NP):
            self.oppositePopulation[i] = self.oppositeIndividualCreation(self.population[i].getChromosome())

    def deliveryWindowPopulationSelection(self):
        selectedPopulation = np.empty(0)
        selectedOppositePopulation = np.empty(0)

        for i in range(self.NP):
            if self.respectDeliveryWindow(self.population[i].getChromosome()):
                selectedPopulation = np.append(selectedPopulation, self.population[i])

            if self.respectDeliveryWindow(self.oppositePopulation[i].getChromosome()):
                selectedOppositePopulation = np.append(selectedOppositePopulation, self.oppositePopulation[i])
        
        self.population = selectedPopulation
        self.oppositePopulation = selectedOppositePopulation

    def newPopulationSelection(self):
        # we take BEST_TAKEN chromosomes from population and opposite population and put them directly
        # in the output population. The rest is taken by a roulette wheel selection
        (populationIndividualsNumber, oppositePopulationIndividualsNumber) = self.populationsNewSize()

        # self.population = np.concatenate((self.takeBestN(self.population, populationIndividualsNumber),
        #                                 self.takeBestN(self.oppositePopulation, oppositePopulationIndividualsNumber)))

        self.population = np.concatenate((self.takeBestN(self.population, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.population, populationIndividualsNumber - self.BEST_TAKEN),
                                self.takeBestN(self.oppositePopulation, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.oppositePopulation, oppositePopulationIndividualsNumber - self.BEST_TAKEN)))

    def populationsNewSize(self):
        totalIndividuals = self.population.size + self.oppositePopulation.size
        populationIndividualsNumber = int(self.NP * self.population.size/totalIndividuals)
        oppositePopulationIndividualsNumber = int(self.NP * self.oppositePopulation.size/totalIndividuals)

        if populationIndividualsNumber + oppositePopulationIndividualsNumber < self.NP:
            populationIndividualsNumber += self.NP - (populationIndividualsNumber + oppositePopulationIndividualsNumber)
        
        return (populationIndividualsNumber, oppositePopulationIndividualsNumber)

    def takeBestN(self, population: List[Individual], n: int) -> List[Individual]:
        bestIndividuals = np.empty((n), dtype=Individual)

        for i in range(n):
            best = np.argmin(self.getAllEvaluations(population))
            bestIndividuals[i] = population[best]
            population = np.delete(population, best, 0)

        return bestIndividuals

    def rouletteWheelSelection(self, population: List[Individual], n: int):
        individualsChosen = np.empty((n), dtype=Individual)

        for i in range(n):
            choice = np.random.choice(population.size, p=self.probabilitiesSelection(self.getAllEvaluations(population)))
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
            self.deliveryWindowPopulationSelection()
            self.newPopulationSelection()
            self.populationCrossover()
            # TODO: add mutation
            # offspringCreation(initialPopul)

        self.deliveryWindowPopulationSelection()
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
            plt.plot([t.getStartPosition().getX()], [t.getStartPosition().getY()], 'o', label='Task start position')
            plt.plot([t.getEndPosition().getX()], [t.getEndPosition().getY()], '^', label='Task end position')

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


uav1 = Uav(600, 2, 2, Position(0, 0))
uav2 = Uav(400, 7, 6, Position(0, 0))
task1 = Task(Position(1, 1), Position(1, 3), 5, 1)
task2 = Task(Position(3, 7), Position(2, 3), 3, 1)
task3 = Task(Position(8, 2), Position(8, 4), 2, 1)
task4 = Task(Position(6, 4), Position(4, 6), 3, 3.5)
task5 = Task(Position(1, 4), Position(7, 3), 1, 1)
cp1 = ChargingPoint(Position(2,3))
cp2 = ChargingPoint(Position(4,4))

uavs = np.array((uav1,uav2))
tasks = np.array([task1, task2, task3, task4, task5])
cps = np.array([cp1, cp2])

run = GeneticAlgo(uavs, tasks, cps)
print(run.getSolution())
run.graphSolution()