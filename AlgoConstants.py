class AlgoConstants:
    # change the algo to add the recharging tasks only in the end
    RC_ONLY_END = True
    # delivery window constraints not valued as mandatory
    MANDATORY_DELIVERY_WINDOW = False

    # for debugging in the crucial part
    DEBUG = False
    # minimum maximum time weight factor
    ALPHA = 0
    # minimum consumption weight factor (alpha + beta = 1)
    BETA = 1
    # number of population individuals (paire number at least > 6*BEST_TAKEN)
    NP = 30
    # number of the fittest chromosomes taken without the roulette wheel selection
    BEST_TAKEN = 4
    # crossover probability
    PC = 0.3
    # mutation probability
    PM = 0.3
    # max iterations for genetic algo
    MAX_ITER = 15
    # Min tolerance for stopping run loop
    DELTA_TOLERANCE = 0.5
    # Last iteration checked for stopping loop (we avoid local minima)
    LAST_ITER_CHECKED = 5
    # number of the offspring individuals for mutation porpousos
    GROUP_MUTATION_NUMBER = 8

    NU = 0
    NT = 0
    NCP = 0