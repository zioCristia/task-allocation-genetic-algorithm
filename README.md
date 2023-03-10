# A genetic algorithm for a task allocation problem in an urban air mobility scenario
This algorithm is a Master final thesis work for Airspace Engineering in [Politecnico di Torino](https://www.polito.it/en).

### Author: Cristiano Bicchieri
#### Relator: Giorgio Guglieri
#### Correlator: Stefano Primatesta and Marco Rinaldi


#### Date: 07/03/2023


## Abstract
In an aerial package delivery scenario carried out by multiple Unmanned Aerial Vehicles (UAVs), it is important to maximize the collaboration and the resource sharing in the fleet and to satisfy, according to the UAVs' constraints, the largest possible portion of the tasks' requirements' space.
In this thesis work, a UAVs-based parcel delivery problem is formulated and a Genetic Algorithm (GA) is proposed for solving the formulated task assignment problem for a fleet of heterogeneous UAVs.
GAs are a class type of algorithms inspired by the known evolutionary mechanism of populations.
The objective of the proposed task allocation method is to minimize the energy consumed by the fleet for executing the delivery tasks while respecting the time window delivery constraints and the maximum payload capacity of each UAV.
Each task in the task set is considered with the parcel's pick-up point and delivery point, the payload mass, and parcel's time delivery deadline.
The UAVs are assumed to be able to perform only one task at a time. In order to address the service persistency issues, re-charge tasks can also be assigned and added where the UAV's energy is not sufficient to perform the next task.
The initial population of the genetic algorithm is created randomly, and subsequent mutation and crossover operations are performed to increase the diversity of the population and avoid incurring in local minima.
The capability of the proposed solution of efficiently handling the formulated problem is demonstrated with ad-hoc defined scenarios, which represent the algorithm's instances.

## Disclaimer
This is not intended to be either a perfect or a final version of the algorithm, because there is still much to be done and implemented to optimize it and make it easier to use.
Any suggestions and ways for improvement are accepted.

## Index
- [How to use the algo](#how-to-use-the-algo)
- [Next steps](#next-steps)

## How to use the algo
To start go to 'main.py' and create your environnemnt with uavs, tasks and recharging tasks. If needed change the uav's constants inside the 'Uav.py' class.

In the 'AlgoConstants.py' there are the mains constants used by the algorithm, change them for your needs.

Run the 'main.py' to run the genetic algorithm.

## Next steps:
The following are the next steps to do on the algorithm, with no particular order, and in my personal opinion:

- Add individual parameter constraintChecked to avoid re-doing it
- Better use of OOP with for example of a Population class (and Solution class maybe?)
    - Separate Uav with a parent class UavDeployed to differenciate the basic uav characteristic and functionalities and the use itself of the Uav (with for example position, currentTaks, currentTrajectTime,...)
    - Separete the genetic algo logic from the task allocation one
- Rethink of a better way to initialize the population of GA, we do too many operations not anymore needed
- Save in the indiviual the chromosome with and without RT so that we don't need to recreate it at each loop
- Use environement class for stockig of uavs, tasks of ga algo
- Use of Task object insted of number index for tasks and uavs
- Work with lists of Chromosomes insted of Individuals, use them only after the evaluation
- Optimize energy consumption algorithm
- Add a stopping criteria when a key is pressed

