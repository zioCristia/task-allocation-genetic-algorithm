# number of UAV
NU = 2 # uav.length()
# number of tasks
NT = 5 # tasks.length()
# number of possible configurations
NV = NU + NT
# max iterations for genetic algo
MAX_ITER = 100
# number of the fittest chromosomes taken without the roulette wheel selection
BEST_TAKEN = 3
# number of population individuals (paire number at least > 6*BEST_TAKEN)
NP = 60
# crossover probability
PC = 0.3
# mutation probability
PM = 0.3

# minimum time weight factor
ALPHA = 0.5
# minimum consumption weight factor (alpha + beta = 1)
BETA = 0.5