import numpy as np
import random
import scipy.io
import os

# uav1 = scipy.io.loadmat('C:\\Users\\cbicchieri\\Documents\\workspace\\tesi\\task-allocation-genetic-algorithm\\distancesCostMatrix\\locations.mat')['points']
from Positions import Positions
pnt = Positions()
print(pnt.getPoint(2))

print(uav1[0,1])
print(uav1[1,0])

def pmx_crossover(parent1, parent2):
    # select two random cut points
    cut_points = sorted(np.random.choice(range(1, len(parent1)), size=2, replace=False))
    print(cut_points)

    offspring = parent1.copy()
    
    # determine mapping sections and mapping pairs
    section1 = parent1[cut_points[0]:cut_points[1] + 1]
    section2 = parent2[cut_points[0]:cut_points[1] + 1]
    mapping_pairs = list(zip(section1, section2))
    
    # create mapping dictionaries for each parent
    mapping_dict1 = {pair[0]: pair[1] for pair in mapping_pairs}
    mapping_dict2 = {pair[1]: pair[0] for pair in mapping_pairs}
    
    # fill in remaining genes using mappings
    for i in range(len(offspring)):
        if i < cut_points[0] or i > cut_points[1]:
            while offspring[i] in mapping_dict2:
                offspring[i] = mapping_dict2[offspring[i]]
    
    # fill in mapping sections of offspring with values from other parent
    offspring[cut_points[0]:cut_points[1] + 1] = section2
    
    return offspring

# parent1 = [0, 1, 2, 3, 4, 5, 6, 7]
# parent2 = [3, 7, 5, 1, 6, 0, 2, 4]

# print(sum(parent1))

for i in range(40):
    print(int(i/10))