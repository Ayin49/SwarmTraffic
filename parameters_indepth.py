from runner import run
from sumolib import checkBinary  # noqa
import traci  # noqa

params = [(4.0, 0.2, 1.6), (5.0, 0.8, 2.0), (7.0, 0.4, 0.6)]


for (alpha, beta, gamma) in params:
    print((alpha, beta, gamma))
    for seed in range(0, 105, 5):
        print(seed)
        traci.start([checkBinary('sumo'), "-c", "C:\\Users\\Ayin\\PycharmProjects\\SwarmTraffic\\data2\\grid66.sumocfg",
                 "--tripinfo-output", ".\\stats\\random\\tripinfo_a_{}_b_{}_g_{}_s_{}_.xml".format(alpha, beta, gamma, seed)])
        run(alpha=alpha, beta=beta, gamma=gamma, seed=seed)
