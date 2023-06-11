import numpy as np
from runner import run
from itertools import product
from sumolib import checkBinary  # noqa
import traci  # noqa

for (alpha, beta, gamma) in product(np.arange(1, 8, 1), np.arange(0, 1.2, 0.2), np.arange(0, 2.2, 0.2)):
    print((alpha, beta, gamma))
    traci.start([checkBinary('sumo'), "-c", "C:\\Users\\Ayin\\PycharmProjects\\SwarmTraffic\\data2\\grid66.sumocfg",
                 "--tripinfo-output", ".\\stats\\tripinfo_a_{}_b_{}_g_{}_.xml".format(alpha, beta, gamma)])
    run(alpha, beta, gamma)