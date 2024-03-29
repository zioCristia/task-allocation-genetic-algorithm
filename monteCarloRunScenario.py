from GeneticAlgo import GeneticAlgo
from AlgoConstants import AlgoConstants as cst
import statistics
from typing import List

def monteCarloRun(envComplexA, printGraph):
    energies = []
    tasksExecuted = []
    chargeExecuted = []
    iterations = []
    runTime = []

    for i in range(20):
        print("RUN " + str(i))
        ga = GeneticAlgo(envComplexA, printGraph=printGraph)
        ga.run()
        if ga.solutionFound:
            energies.append(totalEnergy(ga.getSolution().getChromosome().getEnergyPerTaskPerUav())/10**6)
            if not cst.MANDATORY_DELIVERY_WINDOW:
                tasksExecuted.append(ga.getSolution().getChromosome().getRespectDeliveryPercentage())
            chargeExecuted.append(numberOfChargingTask(ga.getSolution().getChromosome().getTasksOrder()))
            iterations.append(ga.iterationNumber)
            runTime.append(ga.algoTime)

    print("Number of solutions: " + str(len(energies)))
    print("TOTAL ENERGY [MJ]")
    print("mean: " + str(statistics.mean(energies)))
    print("stdev: " + str(statistics.stdev(energies)))
    if not cst.MANDATORY_DELIVERY_WINDOW:
        print("TASKS EXECUTED")
        print("mean: " + str(statistics.mean(tasksExecuted)))
        print("stdev: " + str(statistics.stdev(tasksExecuted)))
    print("CHARGE EXECUTED")
    print("mean: " + str(statistics.mean(chargeExecuted)))
    print("stdev: " + str(statistics.stdev(chargeExecuted)))
    print("ITERATIONS")
    print("mean: " + str(statistics.mean(iterations)))
    print("stdev: " + str(statistics.stdev(iterations)))
    print("ALGO TIME")
    print("mean: " + str(statistics.mean(runTime)))
    print("stdev: " + str(statistics.stdev(runTime)))

def numberOfChargingTask(taskOrder: List[int]) -> int:
    output = 0
    for t in taskOrder:
        if t > 29 and t < 34:
            output += 1
    
    return output

def totalEnergy(energyPerUav):
    totalEnergy = 0
    for u in energyPerUav:
        for e in u:
            totalEnergy += e
    
    return totalEnergy