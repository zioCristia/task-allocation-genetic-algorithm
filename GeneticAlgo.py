import math
import time
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
from AlgoConstants import AlgoConstants as const

"""
VINCOLI:
* max payload per drone
* delivery window
* scelta degli individui fatta sul consumo energetico e non più sul tempo di consegna (o t*m)
* preferire drone che consuma meno
* preferire ripartizione che consuma meno (fitting function t*m)

TO ADD:
- iteration stop if last best evaluation similar current evaluation
    - miglioramento nelle ultime 10 iterazioni inferiore di epsilon esci
- cambiare logica minimizzazione energia
- verificare vincoli soddisfatti
- arriviamo un minimo globale? 50 test dove plotti i costi per il numero di iterazioni
- confronto con le aste
- verificare che 
- add use of respectDeliveryWindow() (or better respectConstraint) in the moment of a new individual creation or mutation,
    in this way we optimize the algo and avoid two loops. If newChromosome not respectConstraint we eliminate it.

- come faccio se una task non è possibile eseguirla neanche con la batteria piena?
- verificare quando utilizzare allTask e quando mantenere task
- far visualizzare le task di ricarica sulla mappa con un simbolo
- dopo l'ultima task non aggiungo una ricarica
- vado nella task di ricarica più vicina ma non sempre è quella più conveniente con quella successiva
"""

class GeneticAlgo:
    # minimum maximum time weight factor
    ALPHA = const.ALPHA
    # minimum consumption weight factor (alpha + beta = 1)
    BETA = const.BETA
    # number of population individuals (paire number at least > 6*BEST_TAKEN)
    NP = const.NP
    # number of the fittest chromosomes taken without the roulette wheel selection
    BEST_TAKEN = const.BEST_TAKEN
    # crossover probability
    PC = const.PC
    # mutation probability
    PM = const.PM
    # max iterations for genetic algo
    MAX_ITER = const.MAX_ITER
    # number of the offspring individuals for mutation porpousos
    GROUP_MUTATION_NUMBER = const.GROUP_MUTATION_NUMBER

    population = np.empty((NP), dtype=Individual)
    oppositePopulation = np.empty((NP), dtype=Individual)
    solution = Individual(Chromosome([0],[0]), 0)
    bestPopulationEvaluations = []
    solutionEvaluations = []
    solutionEvaluationsDifferences = []
    iterationNumber = 0

    def __init__(self, uavs: List[Uav], tasks: List[Task], chargingPoints: List[ChargingPoint]): #np.ndarray for typing np array
        self.uavs = uavs
        self.tasks = tasks
        self.chargingPoints = chargingPoints
        self.allTasks = np.append(tasks, chargingPoints)
        self.NU = len(uavs)
        const.NU = self.NU
        self.NT = len(tasks)
        const.NT = self.NT
        self.NCP = len(chargingPoints)
        const.NCP = self.NCP
        self.uavsTasksEnergy = np.empty(self.NU)
        # self.run()

    def getSolution(self):
        return self.solution

    def getUavs(self):
        return self.uavs
    
    def individualCreation(self) -> Individual:
        chromosome = self.chromosomeCreation()
        while not self.respectConstraints(chromosome):  # TODO change to Chromosome.respectConstraints
            chromosome = self.chromosomeCreation()
        
        return Individual(chromosome)

    def chromosomeCreation(self) -> Chromosome:
        tasksOrder = np.empty(0)
        cutPositions = np.empty(self.NU)
        uavTaskSelection = self.taskUavSelection()
        
        offset = 0
        for u in range(self.NU):
            uavTask = np.where(uavTaskSelection == u)[0]
            np.random.shuffle(uavTask)
            # uavTask = self.addChargingTasksPerDrone(uavTask, u)
            tasksOrder = np.concatenate((tasksOrder, uavTask))
            cutPositions[u] = len(uavTask) + offset
            offset = len(tasksOrder)
        
        return Chromosome(tasksOrder, cutPositions)

    def addChargingTasksAndEvaluationPopulations(self):
        individualsIndexToDelate = []
        for i in range(len(self.population)):
            chromosomeWithCT = self.addChargingTasks(self.population[i].getChromosome())
            if chromosomeWithCT == 0:
                individualsIndexToDelate.append(i)
            self.population[i] = Individual(chromosomeWithCT, self.individualEvaluation(chromosomeWithCT))
            self.population = np.delete(self.population, individualsIndexToDelate)

        individualsIndexToDelate = []
        for i in range(len(self.oppositePopulation)):
            chromosomeWithCT = self.addChargingTasks(self.oppositePopulation[i].getChromosome())
            if chromosomeWithCT == 0:
                individualsIndexToDelate.append(i)
            self.oppositePopulation[i] = Individual(chromosomeWithCT, self.individualEvaluation(chromosomeWithCT))
            self.oppositePopulation = np.delete(self.oppositePopulation, individualsIndexToDelate)
    
    def addChargingTasksPerDrone(self, taskOrder: List[int], uavNumber: int) -> List[int]:
        self.uavs[uavNumber].reset()
        uav = self.uavs[uavNumber]

        if len(taskOrder) == 0: 
            return taskOrder

        newTaskOrder = []
        taskIndex = 0
        while taskIndex < len(taskOrder):
            if not uav.canTakeTasks(self.taskAndRechargeTask(taskOrder[taskIndex])):
                # we take the charging task
                if taskIndex == 0 or uav.isFull():
                    print("task " + str(taskOrder[taskIndex]) + " cannot be taken by uav " + str(uavNumber) + " from position: " + str(self.uavs[uavNumber].getPosition()) + ". Energy not available!")
                    uav.canTakeTasks(self.taskAndRechargeTask(taskOrder[taskIndex]))
                    # uav.taskEnergy(self.tasks[int(taskOrder[0])])
                    return []
                currentCp = self.optimumChargingPoint(uav.getPosition())
                try:
                    if not uav.canTakeTask(self.chargingPoints[currentCp]):
                        uav.canTakeTask(self.chargingPoints[currentCp])
                        print("")
                    uav.takeTask(self.chargingPoints[currentCp])
                    newTaskOrder.append(currentCp + self.NT)
                except:
                    print("error with task order: " + str(taskOrder) + " at task: " + str(taskIndex) + ", taking cp task: " + str(currentCp))
                    return []
            else:
                uav.takeTask(self.tasks[int(taskOrder[taskIndex])])
                newTaskOrder.append(int(taskOrder[taskIndex]))
                taskIndex += 1

        self.uavs[uavNumber] = uav
        return newTaskOrder

    def addChargingTasksPerDroneOld(self, taskOrder, uavNumber) -> List[int]:
        # deprecated
        # TODO optimize this method to calculate only one time the opt energy for the dron, then check if was the same uav
        # and use same previous task and just take the previous energy calculation
        #print(taskOrder)
        self.uavs[uavNumber].reset()

        if len(taskOrder) == 0: 
            return taskOrder

        uav = self.uavs[uavNumber]

        if not uav.canTakeTasks(self.taskAndRechargeTask(0)):
            print("task " + str(taskOrder[0]) + " cannot be taken by uav " + str(uavNumber) + ". Energy not available!")
            return np.empty(0)

        uav.takeTask(self.tasks[int(taskOrder[0])])

        newTaskOrder = taskOrder
        offset = 0
        for t in range(1, len(taskOrder) - 1):
            tempUav = uav

            if not tempUav.canTakeTasks(self.taskAndRechargeTask(t)):
                # the drone can't complete the task t and charge after, we go to a charging point first
                currentCp = self.optimumChargingPoint(self.tasks[t].getEndPosition())
                uav.takeTask(self.chargingPoints[currentCp])
                if not uav.canTakeTask(self.tasks[t]):
                    print("task " + str(taskOrder[0]) + " cannot be taken by uav " + str(uavNumber) + ". Energy not available!")
                    return np.empty(0)

                uav.takeTask(self.tasks[t])
                newTaskOrder = np.insert(newTaskOrder, t + offset, currentCp + self.NT)   # bc we add the charging task at the end in all tasks array
                offset += 1
            else:
                tempUav.takeTask(self.tasks[t])
                uav = tempUav

        if len(taskOrder) > 1:
            if not uav.canTakeTask(self.tasks[int(taskOrder[len(taskOrder)-1])]):
                currentCp = self.optimumChargingPoint(uav.getPosition())
                uav.takeTask(self.chargingPoints[currentCp])
                newTaskOrder = np.insert(newTaskOrder, len(newTaskOrder)-1, currentCp + self.NT)
            
            uav.takeTask(self.tasks[int(taskOrder[len(taskOrder)-1])])
            self.uavs[uavNumber] = uav

        return newTaskOrder
    
    def addChargingTasks(self, chromosome: Chromosome):
        tasksOrder = []

        for u in range(self.NU):
            droneTask = chromosome.getTasksPerUav()[u]
            droneTasksWithCP = self.addChargingTasksPerDrone(droneTask, u)
            if droneTasksWithCP == []:
                return 0
            tasksOrder.append(droneTasksWithCP)

        return Chromosome.fromTasksPerUav(tasksOrder)

    def optimumChargingPoint(self, position: Position, nextTask: Task = 0):
        # TOFIX verify it works
        distances = utility.tasksDistances(position, self.chargingPoints)

        if nextTask != 0:
            # TODO: check only the first elements of distances
            for cp in range(self.NCP):
                distances[cp] = utility.taskDistance(position, self.chargingPoints[cp]) + utility.taskDistance(self.chargingPoints[cp].getStartPosition(), nextTask.getEndPosition())
        
        return np.argmin(distances)

    def taskAndRechargeTask(self, taskNumber: int):
        return [self.tasks[int(taskNumber)], self.chargingPoints[self.optimumChargingPoint(self.tasks[int(taskNumber)].getEndPosition())]]

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
                uavsMassSum += 1/self.uavs[u].getMass()
            
            for u in range(self.NU):
                tasksUavsMatrix[t][u] *= (1/self.uavs[u].getMass())/uavsMassSum
        
        return tasksUavsMatrix

    def maxPayloadMatrix(self):
        # TODO: add function to control that all the task can be performed (min(sum(righe))>0)
        maxPayload = np.zeros((self.NT, self.NU))

        for t in range(self.NT):
            for u in range(self.NU):
                if self.tasks[t].getPackageMass() <= self.uavs[u].getMaxPayloadMass():
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
        tasksOrder = chromosome.getTasksOrder()
        oppositeTasksOrder = np.empty((self.NT))

        index = 0
        for t in tasksOrder:
            if t < self.NT:
                oppositeTasksOrder[index] = self.NT - t - 1
                index += 1
        
        chromosome = Chromosome(oppositeTasksOrder, chromosome.getCutPositions())
        return Individual(chromosome)

    def individualEvaluation(self, chromosome: Chromosome) -> float:
        for u in range(self.NU):
            self.uavsTasksEnergy[u] = self.uavs[u].getTotalEnergyUsed()

        #return self.distanceMassObjectiveFunction(self.getUavsDistances(chromosome), self.getUavsMasses(chromosome))
        # energy optimization
        return self.energyObjectiveFunction()
    
    def respectConstraints(self, chromosome: Chromosome) -> bool:
        # TODO: add here the other constraints (payload), maybe better use directly methods on Chromosome
        return self.respectDeliveryWindow(chromosome) and self.respectMaxPayload(chromosome)

    def respectDeliveryWindow(self, chromosome: Chromosome) -> bool:
        # TODO use the Chromosome.respectDeliveryWindow, not this method
        offset = 0
        lastOffset = 0
        currentDrone = 0
        for t in range(len(chromosome.getTasksOrder())):
            if (chromosome.getCutPositions()[currentDrone] == t):
                lastOffset = offset
                currentDrone += 1
            
            offset += 1
            currentTask = self.allTasks[int(chromosome.getTasksOrder()[t])]
            if not currentTask.isChargingPoint() and currentTask.getMaxDeliveryWindow() < t - lastOffset:
                return False
        
        return True

    def respectMaxPayload(self, chromosome: Chromosome) -> bool:
        for u in range(self.NU):
            for t in chromosome.getTasksPerUav()[u]:
                if self.allTasks[int(t)].getPackageMass() > self.uavs[u].getMaxPayloadMass():
                    return False
        return True

    def getUavsDistances(self, chromosome: Chromosome) -> List[float]:
        # DEPRECATED: check before
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
        # DEPRECATED: check before
        distance = 0
        lastUavPosition = self.uavs[uavIndex].getPosition()

        for t in tasksIndexes:
            t = int(t)
            distance += (self.tasks[t].getTrajectDistance() + utility.distance(lastUavPosition, self.tasks[t].getStartPosition()))
            lastUavPosition = self.tasks[t].getEndPosition()
        
        distance += utility.distance(lastUavPosition, self.uavs[uavIndex].getPosition())
        return distance

    def distanceMassObjectiveFunction(self, distancesUav: List[float], massesUav: List[float]):
        return self.ALPHA * max(distancesUav*massesUav) + self.BETA * (distancesUav*massesUav).sum()

    def energyObjectiveFunction(self):
        return self.ALPHA * max(self.uavsTasksEnergy) + self.BETA * self.uavsTasksEnergy.sum()

    def initialPopulationCreation(self):
        for i in range(self.NP):
            self.population[i] = self.individualCreation()

    def populationFillsWithRandomIndividuals(self):
        # add new random individuals to the population till we reach the max number of individuals
        for i in range(len(self.population), self.NP):
            self.population = np.append(self.population, self.individualCreation())

    def oppositePopulationCreation(self):
        self.oppositePopulation = []

        for individual in self.population:
            oppositeIndividual = self.oppositeIndividualCreation(individual.getChromosome())
            self.oppositePopulation.append(oppositeIndividual)

    def constraintsPopulationsSelection(self):
        selectedPopulation = np.empty(0)
        selectedOppositePopulation = np.empty(0)

        for i in range(len(self.population)):
            if self.respectConstraints(self.population[i].getChromosome()):
                selectedPopulation = np.append(selectedPopulation, self.population[i])

        for i in range(len(self.oppositePopulation)):
            if self.respectConstraints(self.oppositePopulation[i].getChromosome()):
                selectedOppositePopulation = np.append(selectedOppositePopulation, self.oppositePopulation[i])
        
        self.population = selectedPopulation
        self.oppositePopulation = selectedOppositePopulation

    def newPopulationSelection(self):
        (populationIndividualsNumber, oppositePopulationIndividualsNumber) = self.populationsNewSize()
        
        # self.population = np.concatenate((self.rouletteWheelSelection(self.population, populationIndividualsNumber), 
        #                                   self.rouletteWheelSelection(self.oppositePopulation, oppositePopulationIndividualsNumber)))
        
        if len(self.population) > self.BEST_TAKEN:
            populationLeft = self.numberPopulationLeft(populationIndividualsNumber)
            self.population = np.concatenate((self.takeBestN(self.population, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.population, populationLeft)))
        else:
            self.population = self.rouletteWheelSelection(self.population, populationIndividualsNumber)
        
        if len(self.oppositePopulation) > self.BEST_TAKEN:
            oppositePopulationLeft = self.numberPopulationLeft(oppositePopulationIndividualsNumber)
            self.population = np.concatenate((self.takeBestN(self.oppositePopulation, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.oppositePopulation, oppositePopulationLeft)))
        else:
            self.population = self.rouletteWheelSelection(self.oppositePopulation, oppositePopulationIndividualsNumber)
    
    def newPopulationSelectionOld(self):
        # deprecated: we don't use anymore Best_Taken
        # we take BEST_TAKEN chromosomes from population and opposite population and put them directly
        # in the output population. The rest is taken by a roulette wheel selection
        (populationIndividualsNumber, oppositePopulationIndividualsNumber) = self.populationsNewSize()

        populationLeft = self.numberPopulationLeft(populationIndividualsNumber)
        oppositePopulationLeft = self.numberPopulationLeft(oppositePopulationIndividualsNumber)

        self.population = np.concatenate((self.takeBestN(self.population, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.population, populationLeft),
                                self.takeBestN(self.oppositePopulation, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.oppositePopulation, oppositePopulationLeft)))

    def populationsNewSize(self):
        totalIndividuals = self.population.size + self.oppositePopulation.size
        
        if self.oppositePopulation.size < self.NP / 2:
            oppositePopulationIndividualsNumber = self.oppositePopulation.size
            populationIndividualsNumber = self.NP - oppositePopulationIndividualsNumber
        else:
            oppositePopulationIndividualsNumber = int(self.NP * self.oppositePopulation.size/totalIndividuals)
            populationIndividualsNumber = int(self.NP * self.population.size/totalIndividuals)

        if populationIndividualsNumber + oppositePopulationIndividualsNumber < self.NP:
            populationIndividualsNumber += self.NP - (populationIndividualsNumber + oppositePopulationIndividualsNumber)
        
        return (populationIndividualsNumber, oppositePopulationIndividualsNumber)

    def takeBestN(self, population: List[Individual], n: int) -> List[Individual]:
        if len(population) < n:
            return population

        bestIndividuals = np.empty((n), dtype=Individual)

        for i in range(n):
            best = np.argmin(self.getAllEvaluations(population))
            bestIndividuals[i] = population[best]
            population = np.delete(population, best, 0)

        return bestIndividuals

    def rouletteWheelSelection(self, population: List[Individual], n: int) -> List[Individual]:
        if n > len(population):
            n = len(population)

        individualsChosen = np.empty((n), dtype=Individual)

        for i in range(n):
            choice = np.random.choice(population.size, p=self.probabilitiesSelection(self.getAllEvaluations(population)))
            individualsChosen[i] = population[choice]
            population = np.delete(population, choice, 0)

        return individualsChosen

    def numberPopulationLeft(self, populationNumber: int) -> int:
        populationLeft = populationNumber - self.BEST_TAKEN

        if populationLeft < 0:
            return 0
        
        return populationLeft

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

    def populationsCrossover(self):
        # per ogni chormosoma in chromosomes, se la probabilità è minore della probabilità di crossover
        # esegui il crossover tra lui e un altro individuo a caso e poi lo aggiungi nella popolazione di output,
        # sennò lo metti direttamente nell'output
        for c in range(len(self.population)):
            if np.random.rand() < self.PC:
                self.individualCrossoverInPopulation(c)

        for c in range(len(self.oppositePopulation)):
            if np.random.rand() < self.PC:
                self.individualCrossoverInOppositePopulation(c)

    def randomChromosomeForCrossDifferentFrom(self, n: int, maxNumber: int) -> int:
        # TODO: change it when use class Population
        forCross = np.random.randint(maxNumber)
        while forCross == n:
                    forCross = np.random.randint(self.NT)

        return forCross

    def removeRechargingTaskPopulation(self):
        for i in self.population:
            rechargingTasksRemovedPerUav = self.removeRechargingTaskPerUav(i.getChromosome().getTasksPerUav())
            i.setChromosome(Chromosome.fromTasksPerUav(rechargingTasksRemovedPerUav))

    def removeRechargingTaskPerUav(self, tasksOrderPerUav: List):
        tasksOrderWithoutCTPerUav = []

        for tPerUav in tasksOrderPerUav:
            tasksOrder = []
            for t in tPerUav:
                if t < self.NT:
                    tasksOrder.append(t)
            tasksOrderWithoutCTPerUav.append(tasksOrder)
        
        return tasksOrderWithoutCTPerUav

    def individualCrossoverInPopulation(self, individualIndex: int):
        # TODO change this method to be a Population class method
        # TOFIX check if it works
        chromosomeForCross = self.randomChromosomeForCrossDifferentFrom(individualIndex, len(self.population))
        chromosome = self.population[individualIndex].getChromosome()
        
        chromosome.crossWith(self.population[chromosomeForCross].getChromosome().getTasksOrder())
        #self.population[individualIndex] = Individual(chromosome)

    def individualCrossoverInOppositePopulation(self, individualIndex: int):
        # TODO change this method to be a Population class method
        # TOFIX check if it works
        chromosomeForCross = self.randomChromosomeForCrossDifferentFrom(individualIndex, len(self.oppositePopulation))
        chromosome = self.oppositePopulation[individualIndex].getChromosome()
        
        chromosome.crossWith(self.oppositePopulation[chromosomeForCross].getChromosome().getTasksOrder())
        crossedChromosome = self.addChargingTasks(chromosome)
        if crossedChromosome != 0:
            self.oppositePopulation[individualIndex] = Individual(crossedChromosome, self.individualEvaluation(crossedChromosome))

    def offspringCreation(self):
        for i in range(0, len(self.population), self.GROUP_MUTATION_NUMBER):
            if np.random.rand() < self.PM and i + self.GROUP_MUTATION_NUMBER < self.NP:
                if len(self.population) < self.NP:
                    self.population = np.append(self.population, self.groupMutation(self.population[i:i+self.GROUP_MUTATION_NUMBER]))
                else:
                    self.population[i:i+self.GROUP_MUTATION_NUMBER] = self.groupMutation(self.population[i:i+self.GROUP_MUTATION_NUMBER])

        for i in range(0, len(self.oppositePopulation), self.GROUP_MUTATION_NUMBER):
            if np.random.rand() < self.PM and i + self.GROUP_MUTATION_NUMBER < self.NP:
                if len(self.oppositePopulation) < self.NP:
                    self.oppositePopulation = np.append(self.oppositePopulation, self.groupMutation(self.oppositePopulation[i:i+self.GROUP_MUTATION_NUMBER]))
                else:
                    self.oppositePopulation[i:i+self.GROUP_MUTATION_NUMBER] = self.groupMutation(self.oppositePopulation[i:i+self.GROUP_MUTATION_NUMBER])

    def groupMutation(self, individuals: List[Individual]) -> List[Individual]:
        bestChromosome = self.takeBestN(individuals, 1)[0].getChromosome()

        bestTasksOrder = bestChromosome.getTasksOrder()
        flipTasksOrder = self.flipMutation(bestTasksOrder)
        swapTasksOrder = self.swapMutation(bestTasksOrder)
        slideTasksOrder = self.slideMutation(bestTasksOrder)

        bestCutPositions = bestChromosome.getCutPositions()
        mutatedCutPosition = self.randomCutPosition()

        newIndividuals = []
        newIndividuals.append(Individual(Chromosome(bestTasksOrder, bestCutPositions)))
        newIndividuals.append(Individual(Chromosome(flipTasksOrder, bestCutPositions)))
        newIndividuals.append(Individual(Chromosome(swapTasksOrder, bestCutPositions)))
        newIndividuals.append(Individual(Chromosome(slideTasksOrder, bestCutPositions)))

        newIndividuals.append(Individual(Chromosome(bestTasksOrder, mutatedCutPosition)))
        newIndividuals.append(Individual(Chromosome(flipTasksOrder, mutatedCutPosition)))
        newIndividuals.append(Individual(Chromosome(swapTasksOrder, mutatedCutPosition)))
        newIndividuals.append(Individual(Chromosome(slideTasksOrder, mutatedCutPosition)))

        return individuals

    def flipMutation(self, tasksOrder: List[int]) -> List[int]:
        position1 = np.random.randint(self.NT)
        position2 = np.random.choice(list(range(int(position1), self.NT)))

        newTaskOrder = np.array(tasksOrder)

        for t in range(position1, position2+1):
            newTaskOrder[t] = tasksOrder[position2]
            position2 -= 1

        return newTaskOrder

    def swapMutation(self, tasksOrder: List[int]) -> List[int]:
        position1 = np.random.randint(self.NT)
        position2 = np.random.choice(list(range(int(position1), self.NT)))

        tempTask = tasksOrder[position1]
        tasksOrder[position1] = tasksOrder[position2]
        tasksOrder[position2] = tempTask
        return tasksOrder

    def slideMutation(self, tasksOrder: List[int]) -> List[int]:
        position1 = np.random.randint(self.NT)
        position2 = np.random.choice(list(range(int(position1), self.NT)))

        tempTask = tasksOrder[position1]

        for t in range(position1, position2):
            tasksOrder[t] = tasksOrder[t+1]
        tasksOrder[position2] = tempTask

        return tasksOrder
    
    def timeToStop(self) -> bool:
        if self.iterationNumber < const.LAST_ITER_CHECKED+1:
            return False
        
        if self.iterationNumber == const.MAX_ITER:
            return True
        
        for diff in self.solutionEvaluationsDifferences[-const.LAST_ITER_CHECKED:]:
            if diff < 0 and diff > const.DELTA_TOLERANCE:
                return False
            
        return True

    def saveBestPopulationEvaluation(self):
        self.bestPopulationEvaluations.append(self.takeBestPopulationEvaluation())
    
    def saveSolutionEvaluation(self):
        self.solutionEvaluations.append(self.solution.getEvaluation())
        if self.iterationNumber > 0:
            self.solutionEvaluationsDifferences.append(abs(self.solutionEvaluations[-1] - self.solutionEvaluations[-2]))

    def takeBestPopulationEvaluation(self) -> float:
        return self.takeBestN(self.population, 1)[0].getEvaluation()

    def reset(self):
        self.population = np.empty((const.NP), dtype=Individual)
        self.oppositePopulation = np.empty((const.NP), dtype=Individual)
        self.solution = Individual(Chromosome([0],[0]), 0)
        self.bestPopulationEvaluations = []
        self.solutionEvaluations = []
        self.solutionEvaluationsDifferences = []
        self.iterationNumber = 0
        self.uavsTasksEnergy = np.empty(self.NU)

    def run(self):
        self.reset()
        self.initialPopulationCreation()

        while not self.timeToStop():
            print("Iteration: " + str(self.iterationNumber))
            start = time.process_time()

            self.populationFillsWithRandomIndividuals()

            self.oppositePopulationCreation()
            # oppTime = time.process_time() - start
            # print("opposite population time: " + str(oppTime))

            # oppTime = time.process_time()
            self.populationsCrossover()
            # crossTime = time.process_time() - oppTime
            # print("population crossover time: " + str(crossTime))

            # crossTime = time.process_time()
            self.offspringCreation()
            # offSpringTime = time.process_time() - crossTime
            # print("population offspring time: " + str(offSpringTime))

            # offSpringTime = time.process_time()
            self.constraintsPopulationsSelection()
            # constrTime = time.process_time() - offSpringTime
            # print("population constraint time: " + str(constrTime))

            # constrTime = time.process_time()
            self.addChargingTasksAndEvaluationPopulations()
            # chargeTime = time.process_time() - constrTime
            # print("population charging time: " + str(chargeTime))

            print("Population size: " + str(len(self.population)) + ", Opposite population size: " + str(len(self.oppositePopulation)))
            # chargeTime = time.process_time()
            self.constraintsPopulationsSelection()
            # constr2Time = time.process_time() - chargeTime
            # print("population crossover 2 time: " + str(constr2Time))

            print("Before Population size: " + str(len(self.population)) + ", Opposite population size: " + str(len(self.oppositePopulation)))
            self.newPopulationSelection()
            print("After Population size: " + str(len(self.population)) + ", Opposite population size: " + str(len(self.oppositePopulation)))
            self.saveBestPopulationEvaluation()

            newBestSolution = self.takeBestN(self.population, 1)[0]
            print("New best solution:")
            newBestSolution.toString()
            if newBestSolution.getEvaluation() < self.solution.getEvaluation() or self.iterationNumber == 0:
                self.solution = Individual.fromIndividual(newBestSolution)
            self.saveSolutionEvaluation()
            print("Solution:")
            self.solution.toString()

            self.removeRechargingTaskPopulation()
            print("iteration time: " + str(time.process_time() - start))
            print("")
            self.iterationNumber += 1

        self.printSolution()
        self.graphEvaluations()
        self.graphSolution()

    def getSolution(self):
        return self.solution

    def printSolution(self):
        print("\nSolution")
        print("Evaluation: " + str(self.solution.getEvaluation()))
        n = 0
        for u in self.solution.getChromosome().getTasksPerUav():
            print("\tuav " + str(n) + ": tasks: " + str(u))
            n += 1

    def printPopulationTasksPerUav(self):
        for i in self.population:
            print(i)

    def graphSolution(self):
        solutionChromosome = self.solution.getChromosome()
        plt.figure(2)
        plt.plot([0], [0])
        # startPositions = np.empty(self.NT, 2)
        # endPositions = np.empty(self.NT, 2)
        # chargingPoints = np.empty(self.NT, 2)
        u = 0
        for t in self.tasks:
            # startPosition[i] = [t.getStartPosition().getX(), t.getStartPosition().getY()]
            # endPosition[i] = [t.getEndPosition().getX(), t.getEndPosition().getY()]
            plt.plot([t.getStartPosition().getX()], [t.getStartPosition().getY()], "^") # , label='Start position')
            plt.plot([t.getEndPosition().getX()], [t.getEndPosition().getY()], "v") # , label='End position')
            plt.plot([t.getStartPosition().getX(), t.getEndPosition().getX()], [t.getStartPosition().getY(), t.getEndPosition().getY()], label='Task ' + str(u))
            u+=1

        for cp in self.chargingPoints:
            plt.plot([cp.getStartPosition().getX()], [cp.getStartPosition().getY()], 'P', label='Charging Point')

        for u in range(self.NU):
            droneTask = solutionChromosome.getTasksPerUav()[u]

            prevPosition = Position(0,0)
            for t in droneTask:
                currentTask = self.allTasks[int(t)]
                currentPos = currentTask.getStartPosition()
                plt.plot([prevPosition.getX(), currentPos.getX()], [prevPosition.getY(), currentPos.getY()], '--')
                prevPosition = currentTask.getEndPosition()

        plt.legend()
        plt.show()

    def graphEvaluations(self):
        plt.figure(1)
        plt.plot(range(len(self.bestPopulationEvaluations)), self.bestPopulationEvaluations, label='Best population evaluations')
        plt.plot(range(len(self.solutionEvaluations)), self.solutionEvaluations, label='Solution evaluations')

        plt.legend()