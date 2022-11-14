import math
from random import randint
from re import I
import numpy as np
import matplotlib.pyplot as plt
import constant
import utility
from Task import Task
from Uav import Uav
from Position import Position
"""
First try assumptions:
* for the initial population we could have multiple identical individuals
* infinite energy per drone (no recharging tasks)
* all the tasks in the same day
* minimize the total delivery time and each drone delivery time
* all the drones start from the (0,0) position
* the drone will end in the same spot where started
* all the drones are used

* max payload per ogni drone
* payload di ogni drone
* deadline delle task
* batteria
* preferire drone che consuma meno
"""
"""
DA NOTARE:
    non sempre il migliore prima dell'algoritmo rimane alla fine a causa delle mutazione per popolazione piccole (20)
    e iterazioni 100
"""
# TODO: * rename all variables and methods to be more accurate
#       * better graph visualization with same colors for drone
#       * implement use of class chromosome

print("started")

uav1 = Uav(5000, 2, 1.5, Position(0, 0))
uav2 = Uav(5000, 2, 1.5, Position(0, 0))
task1 = Task(Position(1, 1), Position(1, 3))
task2 = Task(Position(3, 7), Position(2, 3))
task3 = Task(Position(8, 2), Position(8, 4))
task4 = Task(Position(6, 4), Position(4, 6))
task5 = Task(Position(1, 4), Position(7, 3))

uavs = np.array((uav1,uav2))
tasks = np.array([task1, task2, task3, task4, task5])

def checkConstraints(solutionMatrix):
    # not anymore used
    # somma dei valori di tutte le z tabelle per riga e per colonna uguale a 1
    # bidimensional matrix where all the values along z are summed
    bd = solutionMatrix.sum(axis=0)
    cs = bd.sum(axis=0)     # sum for column
    
    # controllo se ogni riga e colonna è uguale a 1
    for ce in cs:
        if ce != 1:
            return 0

    rs = bd.sum(axis=1)     # sum for row
    
    for re in rs:
        if re != 1:
            return 0
    
    for i in range(constant.NU):
        cs_su = solutionMatrix[i].sum(0)
        rs_su = solutionMatrix[i].sum(1)
        
        for j in range(constant.NU):
            if j == i:
                if cs_su[i] != 1 and cs_su[i] != 1:
                    return 0
            else:
                if cs_su[j] != 0 and cs_su[j] != 0:
                    return 0
    return 1

def randomMatrixGenerator():
    # not anymore used
    x = np.zeros((constant.NU, constant.NV, constant.NV))
    d = np.empty((constant.NU))

    while not checkConstraints(x):
        x = np.zeros((constant.NU, constant.NV, constant.NV))
        assignedTask = np.empty((constant.NT))
        for t in range(constant.NT):
            assignedTask[t] = np.random.randint(constant.NU)

        for u in range(constant.NU):
            taskList = np.where(assignedTask == u)[0]
            d[u] = totalTasksDistance(u, taskList)
            np.random.shuffle(taskList)

            lastColumn = u
            for t in taskList:
                x[u,constant.NU+t,lastColumn] = 1
                lastColumn = constant.NU+t
            x[u,u,lastColumn] = 1
    
    return x, d, objectiveFunction(d)

def populationCreationOld():
    # not anymore used
    p = np.empty((constant.NP, constant.NU, constant.NV, constant.NV))
    d = np.empty((constant.NP, constant.NU))
    o = np.empty((constant.NP))

    for i in range(constant.NP):
        p[i],d[i],o[i] = randomMatrixGenerator()

    return p, d, o

def randomDroneTaskAssignation():
    # DEPRECATED
    assignedTask = np.empty((constant.NT))
    for t in range(constant.NT):
        assignedTask[t] = np.random.randint(constant.NU)
    
    return assignedTask

def individualCreation():
    # TODO rewrite better this function
    assignedTask = randomDroneTaskAssignation()
    # chromosome = np.empty(0) # should be NT length
    cutPosition = np.empty(constant.NU)

    # lastIndex = 0
    # for u in range(constant.NU):
    #     uavTaskList = np.where(assignedTask == u)[0]
    #     np.random.shuffle(uavTaskList)
    #     chromosome = np.concatenate((chromosome, uavTaskList))
    #     cutPosition[u] = uavTaskList.shape[0] + lastIndex
    #     lastIndex += uavTaskList.shape[0]

    # more simpler random assignation directly with two separete chromosome
    chromosome = list(range((constant.NT)))
    np.random.shuffle(chromosome)
    
    previousPosition = 0
    for i in range(constant.NU-1):
        cutPosition[i] = np.random.choice(list(range(int(previousPosition), constant.NT)))
        previousPosition = cutPosition[i]
    
    cutPosition[constant.NU - 1] = constant.NT

    return chromosome, cutPosition
    # return Chromosome(chromosome, cutPosition)

def oppositeIndividualCreation(chromosome):
    oppositeChromosome = np.empty((constant.NT))
    for c in range(chromosome.shape[0]):
        oppositeChromosome[c] = constant.NT - chromosome[c] - 1
    return oppositeChromosome

def populationEvaluation(chromosomes, cutPositions):
    evaluations = np.empty((constant.NP))

    for i in range(constant.NP):
        evaluations[i] = individualEvaluation(chromosomes[i], cutPositions[i])
    
    return evaluations

def individualEvaluation(chromosome, cutPosition):
    d = np.empty((constant.NU))

    lastIndex = 0
    for i in range(cutPosition.shape[0]):
        d[i] = totalTasksDistance(i, chromosome[lastIndex : int(cutPosition[i])])
        lastIndex = i
    
    return objectiveFunction(d)

def totalTasksDistance(u, taskList):
    distance = 0
    lastPosition = uavs[u].startPosition

    for t in taskList:
        intT = int(t)
        distance += (tasks[intT].getTrajectDistance() + utility.distance(lastPosition, tasks[intT].getStartPosition()))
        lastPosition = tasks[intT].getEndPosition()
    
    distance += utility.distance(lastPosition, uavs[u].startPosition)
    return distance

def populationCreation():
    chromosomes = np.empty((constant.NP, constant.NT))
    cutPositions = np.empty((constant.NP, constant.NU))
    evaluations = np.empty((constant.NP))

    for i in range(constant.NP):
        chromosomes[i], cutPositions[i]  = individualCreation()
        evaluations[i] = individualEvaluation(chromosomes[i], cutPositions[i])

    return chromosomes, cutPositions, evaluations

def oppositePopulationCreation(chromosomes, cutPositions):
    oppositeChromosomes = np.empty((constant.NP, constant.NT))
    evaluations = np.empty((constant.NP))

    for i in range(chromosomes.shape[0]):
        oppositeChromosomes[i] = oppositeIndividualCreation(chromosomes[i])
        evaluations[i] = individualEvaluation(chromosomes[i], cutPositions[i])

    return oppositeChromosomes, evaluations

def initialPopulationCreation(population, evaluations, populationOp, evaluationsOp):
    # we take BEST_TAKEN chromosomes from population and opposite population and put them directly
    # in the output population. The rest is taken by a roulette wheel selection
    chromosomes = np.empty((constant.NP, constant.NT))

    for i in range(constant.BEST_TAKEN):
        best = np.argmin(evaluations)
        chromosomes[i] = population[best]
        population = np.delete(population, best, 0)
        evaluations = np.delete(evaluations, best, 0)

    for i in range(constant.BEST_TAKEN, int(constant.NP/2)):
        choice = np.random.choice(constant.NP-i, p=probabilitiesSelection(evaluations))
        chromosomes[i] = population[choice]
        population = np.delete(population, choice, 0)
        evaluations = np.delete(evaluations, choice, 0)

    offset = int(constant.NP/2)

    for i in range(constant.BEST_TAKEN):
        best = np.argmin(evaluationsOp)
        chromosomes[i + offset] = populationOp[best]
        populationOp = np.delete(populationOp, best, 0)
        evaluationsOp = np.delete(evaluationsOp, best, 0)

    for i in range(constant.BEST_TAKEN, int(constant.NP/2)):
        choice = np.random.choice(constant.NP-i, p=probabilitiesSelection(evaluationsOp))
        chromosomes[i + offset] = populationOp[choice]
        populationOp = np.delete(populationOp, choice, 0)
        evaluationsOp = np.delete(evaluationsOp, choice, 0)

    return chromosomes

def probabilitiesSelection(evaluations):
    # create the proportional probabilities which are used for the roulette wheel selection
    probabilities = np.empty((evaluations.shape[0]))

    revEvaluations = reverseEvaluations(evaluations)
    sumEvaluations = revEvaluations.sum()

    for i in range(evaluations.shape[0]):
        probabilities[i] = revEvaluations[i] / sumEvaluations
    
    return probabilities

def reverseEvaluations(evaluations):
    reverse = np.empty((evaluations.shape[0]))
    for e in range(evaluations.shape[0]):
        reverse[e] = 1 / evaluations[e]
    
    return reverse

def objectiveFunction(distancesUav):
    return constant.ALPHA * max(distancesUav) + constant.BETA * distancesUav.sum()

def offspringCreation(chromosomes):
    chromosomes = populationCrossover(chromosomes)

    for i in range(0, constant.NP, 4):
        if np.random.rand() < constant.PM and i+4 < constant.NP:
            chromosomes[i:i+8] = groupMutation(chromosomes[i:i+8])
    
    return chromosomes
    
def populationCrossover(chromosomes):
    # per ogni chormosoma in chromosomes, se la probabilità è minore della probabilità di crossover
    # esegui il crossover tra lui e un altro individuo a caso e poi lo aggiungi nella populazione di output,
    # sennò lo metti direttamente nell'output
    crossedChrom = np.empty((constant.NP, constant.NT))

    for c in range(constant.NP):
        if np.random.rand() < constant.PC:
            cUnchanged = np.random.randint(constant.NT)
            while cUnchanged == c:
                cUnchanged = np.random.randint(constant.NT)
            crossedChrom[c] = individualCrossover(chromosomes[c],chromosomes[cUnchanged])
        else:
            crossedChrom[c] = chromosomes[c]

    return crossedChrom

def individualCrossover(cCross, cUncross):
    l = np.random.randint(constant.NT)
    r = np.random.randint(constant.NT)
    while (l >= r):
        l = np.random.randint(constant.NT)
        r = np.random.randint(constant.NT)

    for p in range(l,r):
        i = np.where(cCross == cUncross[p])
        cCross[i] = cCross[p]

    cCross[l:r] = cUncross[l:r]

    return cCross

def groupMutation(chromosomes):
    # TODO implement random cutPosition chromosome as in paper
    # take the best and do the 3 differents mutations, send them all back
    best = np.argmin(populationEvaluation(chromosomes))

    chromosomes[0] = best
    chromosomes[1] = flipMutation(best)
    chromosomes[2] = swapMutation(best)
    chromosomes[3] = slideMutation(best)

    return chromosomes

def flipMutation(chromosome):
    # TODO implement such as int the paper
    position1 = np.random.randint(constant.NT)
    position2 = np.random.choice(list(range(int(position1), constant.NT)))
    return 0

def swapMutation(chromosome):
    # TODO implement such as int the paper
    return 0

def slideMutation(chromosome):
    # TODO implement such as int the paper
    return 0

def geneticAlgorithem():
    population, cutPositions, evaluations = populationCreation()
    print("\nBest starting evaluation")
    print(np.amin(evaluations))
    for i in range(constant.MAX_ITER):
        oppositePopul = oppositePopulationCreation(population, cutPositions)
        initialPopul = initialPopulationCreation(population,evaluations,oppositePopul[0],oppositePopul[1])
        population = populationCrossover(initialPopul)
        evaluations = populationEvaluation(population, cutPositions)
        # TODO: add mutation
        # popul = offspringCreation(initialPopul)

    # print(population)
    # TODO takeBest function
    print("\nBest ending evaluation")
    print(np.amin(evaluations))
    best = np.argmin(evaluations)
    return population[best], cutPositions[best]
    
# print(geneticAlgorithem())
solution = geneticAlgorithem()

plt.plot([0], [0])
for t in tasks:
    plt.plot([t.getStartPosition().getX(), t.getEndPosition().getX()], [t.getStartPosition().getY(), t.getEndPosition().getY()])
    plt.plot([t.getStartPosition().getX()], [t.getStartPosition().getY()], 'o')
    plt.plot([t.getEndPosition().getX()], [t.getEndPosition().getY()], '^')

print("Best solution")
print(solution[0])
print(solution[1])

for i in range(constant.NU):
    droneTask = np.empty(0)

    if i == 0 and solution[1][0] != 0:
        droneTask = solution[0][0:int(solution[1][0])]
    else:
        droneTask = solution[0][int(solution[1][i-1]):int(solution[1][i])]
    
    prevPosition = Position(0,0)
    for t in droneTask:
        currentTask = tasks[int(t)]
        currentPos = currentTask.getStartPosition()
        plt.plot([prevPosition.getX(), currentPos.getX()], [prevPosition.getY(), currentPos.getY()], '--')
        prevPosition = currentTask.getEndPosition()

plt.show()