from __future__ import absolute_import
from __future__ import print_function

import os
import string
import sys
import optparse
import random
from itertools import product
from operator import add
from math import exp

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>

def get_neighbours(junction_id):
    letter = ord(junction_id[0])
    number = ord(junction_id[1])
    return chr(letter - 1) + chr(number), chr(letter + 1) + chr(number), chr(letter) + chr(number - 1), chr(
        letter) + chr(number + 1)


def get_inductionloops(junction_id, number_out=-1):
    numbers = [0, 1, 2, 3]
    match number_out:
        case -1:
            return [junction_id + "_0_D" + str(num) + ".0" for num in [0, 2, 1, 3]]
        case _:
            del numbers[number_out]
            return [junction_id + "_0_D" + str(num) + ".0" for num in numbers]


def get_indices():
    ln = traci.trafficlight.getIDList()[-1]
    lastbutoneletter = ord(ln[0])
    lastbutonenumber = ord(ln[1])
    alphabet = list(string.ascii_uppercase)[2: list(string.ascii_uppercase).index(chr(lastbutoneletter))]
    numbers = map(chr, range(ord("2"), lastbutonenumber))
    return list(map(lambda x: x[0] + x[1], product(alphabet, numbers)))


# stworz slownik skrzyzowanie : ilosc samochodow na induction loops 4x4
# {junction : {NS : no vehicles, WE : no vehicles}}
def get_induction_cars(junction_id, beta=0.7, gamma=1.):
    main_indices = get_inductionloops(junction_id)
    neighbours = get_neighbours(junction_id)
    ns_indices = get_inductionloops(neighbours[0], 2) + get_inductionloops(neighbours[1], 0)
    ew_indices = get_inductionloops(neighbours[2], 1) + get_inductionloops(neighbours[3], 3)
    ns = sum(map(traci.inductionloop.getLastStepVehicleNumber, main_indices[:2])) \
         + beta * sum(map(traci.inductionloop.getLastStepVehicleNumber, ns_indices)) \
         - gamma * sum(map(traci.inductionloop.getLastStepVehicleNumber,
                           [neighbours[0] + "_0_D" + str(2) + ".0", neighbours[1] + "_0_D" + str(0) + ".0"]))
    ew = sum(map(traci.inductionloop.getLastStepVehicleNumber, main_indices[2:])) \
         + beta * sum(map(traci.inductionloop.getLastStepVehicleNumber, ew_indices)) \
         - gamma * sum(map(traci.inductionloop.getLastStepVehicleNumber,
                           [neighbours[2] + "_0_D" + str(1) + ".0", neighbours[3] + "_0_D" + str(3) + ".0"]))
    return [ns, ew]


def run(alpha=1, beta=0.7, gamma=1.):
    """init"""
    trafficlights = get_indices()
    # print(trafficlights)
    inductionloops_aggr = {light: [0, 0] for light in trafficlights}
    rand = random
    rand.seed(a=23)
    """execute the TraCI control loop"""
    step = 0
    # print(traci.inductionloop.getIDList())

    switches = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        for light in trafficlights:
            inductionloops_aggr[light] = list(map(add, get_induction_cars(light, beta, gamma), inductionloops_aggr[light]))
        if step % 5 == 0:
            for light in trafficlights:
                ns_ew = list(map(exp, inductionloops_aggr[light]))
                if traci.trafficlight.getPhase(light) == 0:
                    # ns green
                    softmax = ns_ew[1] / sum(ns_ew) / alpha
                    #softmax = (ns_ew[1] / sum(ns_ew) - 0.4) * 10 / 6
                    if rand.random() < softmax:
                        traci.trafficlight.setPhase(light, 1)
                        switches += 1
                else:
                    # ew green
                    softmax = ns_ew[0] / sum(ns_ew) / alpha
                    #softmax = (ns_ew[1] / sum(ns_ew) - 0.4) * 10 / 6
                    if rand.random() < softmax:
                        traci.trafficlight.setPhase(light, 3)
                        switches += 1

                inductionloops_aggr[light] = [0, 0]
        step += 1
    #print(switches / len(trafficlights))
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
"""
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    # traci.start([sumoBinary, "-c", "C:\\Users\\Ayin\\PycharmProjects\\SwarmTraffic\\data2\\grid66.sumocfg",
    #                         "--tripinfo-output", ".\\stats\\tripinfo.xml"])
    traci.start([checkBinary('sumo'), "-c", "C:\\Users\\Ayin\\PycharmProjects\\SwarmTraffic\\data2\\grid66.sumocfg",
                 "--tripinfo-output", ".\\stats\\tripinfo.xml"])
    run()
"""

import numpy as np

for (alpha, beta, gamma) in product(np.arange(1, 8, 1), np.arange(0, 1.2, 0.2), np.arange(0, 2.2, 0.2)):
    print((alpha, beta, gamma))
    traci.start([checkBinary('sumo'), "-c", "C:\\Users\\Ayin\\PycharmProjects\\SwarmTraffic\\data2\\grid66.sumocfg",
                 "--tripinfo-output", ".\\stats\\tripinfo_a_{}_b_{}_g_{}_.xml".format(alpha, beta, gamma)])
    run(alpha, beta, gamma)