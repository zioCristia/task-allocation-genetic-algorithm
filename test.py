import numpy as np
import scipy.optimize as sp
import math

# array1 = np.array([1, 2, 3])
# array2 = np.array([4, 5, 6, 7])
# array3 = np.array([7, 8, 9])
# arrayOfArrays = np.array([array1, array2, array3], dtype=object)
# print(arrayOfArrays)

# arr = list(range(9))
# np.random.shuffle(arr)
# print(arr)

# print(np.sort(np.random.randint(5, size=(2))))
# print(list(range(0,5)))
# print(np.random.choice(list(range(3,5))))

# a = np.where(np.array([0,0,1,1,0,1]) == 1)

# for i in np.where(np.array([0,0,1,1,0,1]) == 1)[0]:
#     print(i)

# a = np.array([4, 5, 6, 7])
# b = np.array([2, 4, 5])
# c = np.append(a, b)
# print(np.argmin(c))


eta = 1
cd = 0.3
rho = 1.0125
Ad = 0.1
Ar = 0.1   # total rotor disk area
g = 9.81
Fm = 1  # UAV figure of merit
BLt = 0.3
mass = 2
class Test:
    def energyConsumption(self, v):
        # TODO add the distance from the current drone position and the task start position
        l1 = 1.4142135623730951
        l2 = 2

        mp = 0  # mass package to retrive from task TODO

        A = cd * rho * Ad * (l1 + l2) * v**2
        B = (l2 * math.sqrt(((mass + mp) * g)**3))/(v * Fm * math.sqrt(2 *rho * Ar))
        C = (l1 * math.sqrt((mass * g)**3))/(v * Fm * math.sqrt(2 * rho * Ar))

        return 1/eta * (A + B + C)

    def optimize(self):
        d = 1.4142135623730951
        C1 = sp.Bounds(0, 20)
        C2 = sp.NonlinearConstraint(self.energyConsumption, lb=1500, ub=5000)
        f = lambda x : d/x
        v = sp.minimize(f, (1,), bounds=(C1), constraints=(C2))

        print(v)
        print(self.energyConsumption(3.21716838))
        print(self.energyConsumption(1))
        print(self.energyConsumption(2))
        print(self.energyConsumption(3.21716838))
        print(self.energyConsumption(4))
        print(self.energyConsumption(5))

obj = Test()

obj.optimize()

