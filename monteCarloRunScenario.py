from GeneticAlgo import GeneticAlgo
from AlgoConstants import AlgoConstants as cst
import statistics
from typing import List

def monteCarloRun(envComplexA, printGraph):
    energies = []
    tasksExecuted = []
    chargeExecuted = []

    for i in range(10):
        print("RUN " + str(i))
        ga = GeneticAlgo(envComplexA, printGraph=printGraph)
        ga.run()
        if ga.solutionFound:
            energies.append(totalEnergy(ga.getSolution().getChromosome().getEnergyPerTaskPerUav()))
            if not cst.MANDATORY_DELIVERY_WINDOW:
                tasksExecuted.append(ga.getSolution().getChromosome().getRespectDeliveryPercentage())
            chargeExecuted.append(numberOfChargingTask(ga.getSolution().getChromosome().getTasksOrder()))

    print("Number of solutions: " + str(len(energies)))
    print("TOTAL ENERGY")
    print("mean: " + str(statistics.mean(energies)))
    print("stdev: " + str(statistics.stdev(energies)))
    if not cst.MANDATORY_DELIVERY_WINDOW:
        print("TASKS EXECUTED")
        print("mean: " + str(statistics.mean(tasksExecuted)))
        print("stdev: " + str(statistics.stdev(tasksExecuted)))
    print("CHARGE EXECUTED")
    print("mean: " + str(statistics.mean(chargeExecuted)))
    print("stdev: " + str(statistics.stdev(chargeExecuted)))

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