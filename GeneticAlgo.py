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
from Environement import Environement

"""
CONSTRAINTS:
* max payload per drone
* delivery window
* add recharging task depending on the energy used by the drone to complete tasks
"""

class GeneticAlgo:
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

    population = np.empty((const.NP), dtype=Individual)
    oppositePopulation = np.empty((const.NP), dtype=Individual)
    solution = Individual(Chromosome([0],[0]), 0)
    bestPopulationEvaluations = []
    solutionEvaluations = []
    solutionEvaluationsDifferences = []
    iterationNumber = 0
    solutionFound = False
    deliveryFactor = 1

    def __init__(self, environement: Environement, *, printGraph: bool = True): #np.ndarray for typing np array
        self.uavs = environement.getUavs()
        self.tasks = environement.getTasks()
        self.chargingPoints = environement.getChargingPoints()
        self.allTasks = np.append(self.tasks, self.chargingPoints)

        self.NU = len(self.uavs)
        const.NU = self.NU
        self.NT = len(self.tasks)
        const.NT = self.NT
        self.NCP = len(self.chargingPoints)
        const.NCP = self.NCP

        self.uavsTasksEnergy = np.empty(self.NU)

        self.printGraph = printGraph

    def getSolution(self):
        return self.solution

    def getUavs(self):
        return self.uavs

    def fromTasksIndexesToTasks(self, tasksIndexes: List[int]) -> List[Task]:
        # NOT USED, but could be usefull
        tasks = []
        for t in tasksIndexes:
            tasks.append(self.allTasks[int(t)])
        
        return tasks
    
    def individualCreation(self) -> Individual:
        """Creation of a new individual with a random Chromosome which respects max uav payload capabilities
        """
        chromosome = self.chromosomeCreation()
        while not self.respectMaxPayload(chromosome):  # TODO change to Chromosome.respectMaxPayload
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
            tasksOrder = np.concatenate((tasksOrder, uavTask))
            cutPositions[u] = len(uavTask) + offset
            offset = len(tasksOrder)
        
        return Chromosome(tasksOrder, cutPositions)

    def addChargingTasksAndEvaluationPopulations(self):
        # TODO: to place them inside population
        # TODO: rewrite this function with calculate energy and add recharging task separate
        individualsIndexToDelete = []
        for i in range(len(self.population)):
            if const.RC_ONLY_END:
                newChromosome = self.calculateTaskEnergy(self.population[i].getChromosome())
            else:
                newChromosome = self.addChargingTasks(self.population[i].getChromosome())
                if newChromosome == 0:
                    if const.DEBUG:
                        print("")
                    individualsIndexToDelete.append(i)
                    continue

            self.saveEnergiesAndTimeIn(newChromosome)
            self.population[i] = Individual(newChromosome, self.individualEvaluation(newChromosome))
        self.population = np.delete(self.population, individualsIndexToDelete)

        individualsIndexToDelete = []
        for i in range(len(self.oppositePopulation)):
            if const.RC_ONLY_END:
                newChromosome = self.calculateTaskEnergy(self.oppositePopulation[i].getChromosome())
            else:
                newChromosome = self.addChargingTasks(self.oppositePopulation[i].getChromosome())
                if newChromosome == 0:
                    if const.DEBUG:
                        print("")
                    individualsIndexToDelete.append(i)
                    continue

            self.saveEnergiesAndTimeIn(newChromosome)
            self.oppositePopulation[i] = Individual(newChromosome, self.individualEvaluation(newChromosome))
        self.oppositePopulation = np.delete(self.oppositePopulation, individualsIndexToDelete)

    def calculateTaskEnergy(self, chromosome: Chromosome):
        i = 0
        for t in chromosome.getTasksPerUav():
            self.uavs[i].evaluateTasksEnergies(t)
            i += 1

    def addChargingTasksIndividual(self, individual: Individual) -> Individual:
        newChromosome = self.addChargingTasks(individual.getChromosome())
        if newChromosome == 0:
            if const.DEBUG:
                print("")
            raise Exception("Impossible to add recharging task to individual")
        
        self.saveEnergiesAndTimeIn(newChromosome)
        return Individual(newChromosome, self.individualEvaluation(newChromosome))

    def addChargingTasksPerDrone(self, taskOrder: List[int], uavNumber: int) -> List[int]:
        self.uavs[uavNumber].reset()
        uav = self.uavs[uavNumber]

        if len(taskOrder) == 0: 
            return taskOrder

        newTaskOrder = []
        taskIndex = 0
        while taskIndex < len(taskOrder):
            if not uav.canTakeTasks(self.taskAndRechargeTask(uav, taskOrder[taskIndex])):
                # we take the charging task
                if taskIndex == 0 or uav.isFull():
                    if const.DEBUG:
                        print("task " + str(taskOrder[taskIndex]) + " cannot be taken by uav " + str(uavNumber) + " from position: " + str(self.uavs[uavNumber].getPositionId()) + ". Energy not available!")
                        uav.canTakeTasks(self.taskAndRechargeTask(uav, taskOrder[taskIndex]))
                        # uav.taskEnergy(self.tasks[int(taskOrder[0])])
                    return []
                currentCp = self.optimumChargingPoint(uav, uav.getPositionId())
                try:
                    if not uav.canTakeTask(self.chargingPoints[currentCp]) and const.DEBUG:
                        uav.canTakeTask(self.chargingPoints[currentCp])
                        print("")
                    uav.takeTask(self.chargingPoints[currentCp])
                    newTaskOrder.append(currentCp + self.NT)
                except:
                    if const.DEBUG:
                        print("error with task order: " + str(taskOrder) + " at task: " + str(taskIndex) + ", taking cp task: " + str(currentCp))
                    return []
            else:
                uav.takeTask(self.tasks[int(taskOrder[taskIndex])])
                newTaskOrder.append(int(taskOrder[taskIndex]))
                taskIndex += 1

        self.uavs[uavNumber] = uav
        return newTaskOrder

    def addChargingTasks(self, chromosome: Chromosome) -> Chromosome:
        tasksOrder = []

        for u in range(self.NU):
            droneTask = chromosome.getTasksPerUav()[u]
            droneTasksWithCP = self.addChargingTasksPerDrone(droneTask, u)
            if droneTasksWithCP == []:
                return 0
            tasksOrder.append(droneTasksWithCP)

        return Chromosome.fromTasksPerUav(tasksOrder)

    def optimumChargingPoint(self, uav: Uav, fromPositionId: int, *, nextTask: Task = 0):
        # TODO: add the possibility to find the optimum CP also considering the next task
        distances = []
        for cp in self.chargingPoints:
            distances.append(uav.getDistanceToTaskFromPositionId(cp, fromPositionId))
        
        return np.argmin(distances)

    def taskAndRechargeTask(self, uav: Uav, taskNumber: int):
        chargingPoint = self.chargingPoints[self.optimumChargingPoint(uav, self.tasks[int(taskNumber)].getEndPositionId())]
        return [self.tasks[int(taskNumber)], chargingPoint]

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
        """Generate a matrix (NT*NU) indicating which task can be taken by the uav (0 or 1)
        """
        # TODO: add function to control that all the task can be performed (min(sum(righe))>0)
        maxPayload = np.zeros((self.NT, self.NU))

        for t in range(self.NT):
            for u in range(self.NU):
                if self.tasks[t].getPackageMass() <= self.uavs[u].getMaxPayloadMass():
                    maxPayload[t][u] = 1
        
        if self.iterationNumber == 0:
            self.checkAllTasksCanBePerformed(maxPayload)

        return maxPayload

    def checkAllTasksCanBePerformed(self, maxPayloadMatrix):
        for i in maxPayloadMatrix.sum(1):
            if i == 0:
                raise Exception("Not all tasks can be performed by the uav fleet.")
            
    def randomTaskAssignation(self) -> List[int]:
        # not used
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
        # TODO: make it an individual function
        for u in range(self.NU):
            self.uavsTasksEnergy[u] = self.uavs[u].getTotalEnergyUsed()
            if not const.MANDATORY_DELIVERY_WINDOW:
                self.deliveryFactor = 1 / chromosome.getRespectDeliveryPercentage()

        #return self.distanceMassObjectiveFunction(self.getUavsDistances(chromosome), self.getUavsMasses(chromosome))
        # energy optimization
        return self.energyObjectiveFunction()
    
    def saveEnergiesAndTimeIn(self, chromosome: Chromosome):
        timePerTaskPerUav = []
        energyPerTaskPerUav = []

        for u in range(self.NU):
            timePerTaskPerUav.append(self.uavs[u].timeSpentPerTask)
            energyPerTaskPerUav.append(self.uavs[u].energySpentPerTask)

        chromosome.setTimePerTaskPerUav(timePerTaskPerUav)
        chromosome.setEnergyPerTaskPerUav(energyPerTaskPerUav)

    def saveRespectDeliveryPercentage(self, chromosome: Chromosome):
        taskRespecting = 0
        
        uavIndex = 0
        for uavTasks in chromosome.getTasksPerUav():
            timeSpent = 0
            t = 0
            for taskIndex in uavTasks:
                currentTask = self.allTasks[int(taskIndex)]
                timeSpent += chromosome.getTimePerTaskPerUav()[uavIndex][t]

                if currentTask.getMaxDeliveryWindow() >= timeSpent or currentTask.isChargingPoint():
                    taskRespecting += 1
                t += 1
            uavIndex += 1
            
        chromosome.setRespectDeliveryPercentage(taskRespecting / len(chromosome.getTasksOrder()))

    def respectDeliveryWindow(self, chromosome: Chromosome) -> bool:
        uavIndex = 0
        for uavTasks in chromosome.getTasksPerUav():
            timeSpent = 0

            t = 0
            for taskIndex in uavTasks:
                currentTask = self.allTasks[int(taskIndex)]
                timeSpent += chromosome.getTimePerTaskPerUav()[uavIndex][t]
                
                if  not currentTask.isChargingPoint() and currentTask.getMaxDeliveryWindow() < timeSpent:
                    return False
                t += 1
            uavIndex += 1
        return True

    def respectMaxPayload(self, chromosome: Chromosome) -> bool:
        for u in range(self.NU):
            for t in chromosome.getTasksPerUav()[u]:
                if not self.uavs[u].canTakeTaskWeigth(self.allTasks[int(t)]):
                    return False
        return True

    def energyObjectiveFunction(self):
        return self.deliveryFactor * (const.ALPHA * max(self.uavsTasksEnergy) + const.BETA * self.uavsTasksEnergy.sum())

    def initialPopulationCreation(self):
        for i in range(const.NP):
            self.population[i] = self.individualCreation()

    def populationFillsWithRandomIndividuals(self):
        # add new random individuals to the population till we reach the max number of individuals
        for i in range(len(self.population), const.NP):
            self.population = np.append(self.population, self.individualCreation())

    def oppositePopulationCreation(self):
        # TODO: make it a class method in population
        self.oppositePopulation = []

        for individual in self.population:
            oppositeIndividual = self.oppositeIndividualCreation(individual.getChromosome())
            self.oppositePopulation.append(oppositeIndividual)

    def maxPayloadPopulationsSelection(self):
        # TODO: make it population function
        selectedPopulation = np.empty(0)
        selectedOppositePopulation = np.empty(0)

        for i in range(len(self.population)):
            if self.respectMaxPayload(self.population[i].getChromosome()):
                selectedPopulation = np.append(selectedPopulation, self.population[i])

        for i in range(len(self.oppositePopulation)):
            if self.respectMaxPayload(self.oppositePopulation[i].getChromosome()):
                selectedOppositePopulation = np.append(selectedOppositePopulation, self.oppositePopulation[i])
        
        self.population = selectedPopulation
        self.oppositePopulation = selectedOppositePopulation

    def deliveryWindowPopulationsSelection(self):
        # TODO: make it population function
        selectedPopulation = np.empty(0)
        selectedOppositePopulation = np.empty(0)

        for i in range(len(self.population)):
            if self.respectDeliveryWindow(self.population[i].getChromosome()):
                selectedPopulation = np.append(selectedPopulation, self.population[i])

        for i in range(len(self.oppositePopulation)):
            if self.respectDeliveryWindow(self.oppositePopulation[i].getChromosome()):
                selectedOppositePopulation = np.append(selectedOppositePopulation, self.oppositePopulation[i])
        
        self.population = selectedPopulation
        self.oppositePopulation = selectedOppositePopulation

    def newPopulationSelection(self):
        # TODO: rewrite 
        (populationIndividualsNumber, oppositePopulationIndividualsNumber) = self.populationsNewSize()
        offSpring = np.empty(0)

        if len(self.oppositePopulation) > self.NP/2:
            oppositePopulationLeft = self.numberPopulationLeft(oppositePopulationIndividualsNumber)
            offSpring = np.concatenate((self.takeBestN(self.oppositePopulation, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.oppositePopulation, oppositePopulationLeft)))
        else:
        # if len(self.oppositePopulation) > self.BEST_TAKEN and len(self.oppositePopulation) > 0:
            offSpring = np.concatenate((offSpring, self.oppositePopulation))
        # else:
        #     self.population = self.rouletteWheelSelection(self.oppositePopulation, oppositePopulationIndividualsNumber)
        
        # if len(self.population) < self.NP/2:
        #     self.population = np.concatenate(self.population)
        if len(self.population) > self.NP - len(offSpring):
            populationLeft = self.numberPopulationLeft(populationIndividualsNumber)
            offSpring = np.concatenate((offSpring, self.takeBestN(self.population, self.BEST_TAKEN),
                                self.rouletteWheelSelection(self.population, populationLeft)))
        # elif len(self.population) < self.BEST_TAKEN and len(self.population) > self.NP/2:
        #     self.population = self.rouletteWheelSelection(self.population, populationIndividualsNumber)
        else:
            offSpring = np.concatenate((offSpring, self.population))
        

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
        # TODO population function
        populationLeft = populationNumber - self.BEST_TAKEN

        if populationLeft < 0:
            return 0
        
        return populationLeft

    def probabilitiesSelection(self, evaluations):
        # create the proportional probabilities which are used for the roulette wheel selection
        probabilities = []

        revEvaluations = self.reverseEvaluations(evaluations)
        sumEvaluations = sum(revEvaluations)

        for e in revEvaluations:
            probabilities.append(e / sumEvaluations)
        
        return probabilities

    def reverseEvaluations(self, evaluations):
        reverse = []
        for e in evaluations:
            reverse.append(1 / e)
        
        return reverse

    def getAllEvaluations(self, population):
        evaluations = []
        for individual in population:
            evaluations.append(individual.getEvaluation())

        return evaluations

    def populationsCrossover(self):
        # TODO: make it a population function
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
        while forCross == n or forCross >= maxNumber:
                    forCross = np.random.randint(self.NT)

        return forCross

    def removeRechargingTaskPopulation(self):
        # TODO we should not do this but instead store a separate chromosome for genetic operations
        for i in self.population:
            rechargingTasksRemovedPerUav = self.removeRechargingTaskPerUav(i.getChromosome().getTasksPerUav())
            i.setChromosome(Chromosome.fromTasksPerUav(rechargingTasksRemovedPerUav))

    def removeRechargingTaskPerUav(self, tasksOrderPerUav: List):
        # TODO: to put inside chromosome to create a list without RT so that we don't need to do it every time
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
        chromosomeForCross = self.randomChromosomeForCrossDifferentFrom(individualIndex, len(self.population))
        chromosome = self.population[individualIndex].getChromosome()
        
        newCrossedChromosome = chromosome.crossWith(self.population[chromosomeForCross].getChromosome().getTasksOrder())
        self.population = np.append(self.population, Individual(newCrossedChromosome))

    def individualCrossoverInOppositePopulation(self, individualIndex: int):
        # TODO change this method to be a Population class method
        chromosomeForCross = self.randomChromosomeForCrossDifferentFrom(individualIndex, len(self.oppositePopulation))
        chromosome = self.oppositePopulation[individualIndex].getChromosome()
        
        newCrossedChromosome = chromosome.crossWith(self.oppositePopulation[chromosomeForCross].getChromosome().getTasksOrder())
        self.oppositePopulation = np.append(self.oppositePopulation, Individual(newCrossedChromosome))

    def offspringCreation(self):
        for i in range(0, len(self.population), self.GROUP_MUTATION_NUMBER):
            if np.random.rand() < self.PM and i + self.GROUP_MUTATION_NUMBER < self.NP:
                if len(self.population) < const.NP:
                    self.population = np.append(self.population, self.groupMutation(self.population[i:i+self.GROUP_MUTATION_NUMBER]))
                else:
                    self.population[i:i+self.GROUP_MUTATION_NUMBER] = self.groupMutation(self.population[i:i+self.GROUP_MUTATION_NUMBER])

        for i in range(0, len(self.oppositePopulation), self.GROUP_MUTATION_NUMBER):
            if np.random.rand() < self.PM and i + self.GROUP_MUTATION_NUMBER < const.NP:
                if len(self.oppositePopulation) < const.NP:
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
            if diff < 0 or diff > const.DELTA_TOLERANCE:
                return False
            
        return True

    def saveBestPopulationEvaluation(self):
        self.bestPopulationEvaluations.append(self.takeBestPopulationEvaluation())
    
    def saveSolution(self):
        newBestSolution = self.takeBestN(self.population, 1)
        if len(newBestSolution) == 0:
            newBestSolution = Individual(Chromosome([-1],[-1]))
        else:
            self.solutionFound = True
            newBestSolution = newBestSolution[0]

        print("New best solution:")
        print(newBestSolution)
        if (newBestSolution.getEvaluation() < self.solution.getEvaluation() and newBestSolution.getEvaluation() > 0) or self.iterationNumber == 0:
            self.solution = Individual.fromIndividual(newBestSolution)
        self.saveSolutionEvaluation()
    
    def saveSolutionEvaluation(self):
        self.solutionEvaluations.append(self.solution.getEvaluation())
        if self.iterationNumber > 0:
            self.solutionEvaluationsDifferences.append(abs(self.solutionEvaluations[-1] - self.solutionEvaluations[-2]))

    def takeBestPopulationEvaluation(self) -> float:
        if len(self.population) == 0:
            return 0
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
        self.solutionFound = False
        self.deliveryFactor = 1

    def run(self):
        gaStart = time.process_time()
        self.reset()
        self.initialPopulationCreation()
        
        while not self.timeToStop():
            print("Iteration: " + str(self.iterationNumber))
            loopStart = time.process_time()

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
            self.maxPayloadPopulationsSelection()
            # constrTime = time.process_time() - offSpringTime
            # print("population constraint time: " + str(constrTime))
            # print("After maxPayload Population size: " + str(len(self.population)) + ", Opposite population size: " + str(len(self.oppositePopulation)))

            # constrTime = time.process_time()
            self.addChargingTasksAndEvaluationPopulations()
            # chargeTime = time.process_time() - constrTime
            # print("population charging time: " + str(chargeTime))

            # print("Before delivery Population size: " + str(len(self.population)) + ", Opposite population size: " + str(len(self.oppositePopulation)))
            # chargeTime = time.process_time()
            if const.MANDATORY_DELIVERY_WINDOW:
                self.deliveryWindowPopulationsSelection()
            # constr2Time = time.process_time() - chargeTime
            # print("population crossover 2 time: " + str(constr2Time))

            # print("Before Population size: " + str(len(self.population)) + ", Opposite population size: " + str(len(self.oppositePopulation)))
            self.newPopulationSelection()
            print("After Population size: " + str(len(self.population)) + ", Opposite population size: " + str(len(self.oppositePopulation)))
            self.saveBestPopulationEvaluation()

            self.saveSolution()

            self.removeRechargingTaskPopulation()
            print("Iteration time: " + str(time.process_time() - loopStart) + "\n")
            self.iterationNumber += 1

        if const.RC_ONLY_END:
            self.solution = self.addChargingTasksIndividual(self.solution)
            
        print("Total algo time: " + str(time.process_time() - gaStart))

        self.printSolution()

        if self.printGraph:
            self.graphEvaluations()
            self.graphSolution()

    def printSolution(self):
        if not self.solutionFound:
            print("\nSolution not found")
            return
        
        print("\nSolution")
        print("Evaluation: " + str(self.solution.getEvaluation()))
        n = 0
        for t in self.solution.getChromosome().getTasksPerUav():
            print("\tuav " + str(n) + ": tasks: " + str(t))
            n += 1
        n = 0
        print("Times [minutes]:")
        for t in self.solution.getChromosome().getTimePerTaskPerUav():
            print("\tuav " + str(n) + ": tasks: " + str(np.array(t)/60))
            n += 1
        n = 0
        print("Energies [J]:")
        for e in self.solution.getChromosome().getEnergyPerTaskPerUav():
            print("\tuav " + str(n) + ": tasks: " + str(e))
            n += 1

    def printPopulationTasksPerUav(self):
        for i in self.population:
            print(i)

    def graphSolution(self):
        if self.printGraph:
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
                plt.plot([t.getStartPositionId().getX()], [t.getStartPositionId().getY()], "^") # , label='Start position')
                plt.plot([t.getEndPositionId().getX()], [t.getEndPositionId().getY()], "v") # , label='End position')
                plt.plot([t.getStartPositionId().getX(), t.getEndPositionId().getX()], [t.getStartPositionId().getY(), t.getEndPositionId().getY()], label='Task ' + str(u))
                u+=1

            for cp in self.chargingPoints:
                plt.plot([cp.getStartPositionId().getX()], [cp.getStartPositionId().getY()], 'P', label='Charging Point')

            if self.solutionFound:
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