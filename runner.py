
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2023 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

from __future__ import absolute_import
from __future__ import print_function

import os
import string
import sys
import optparse
import random

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
    return (chr(letter-1)+chr(number), chr(letter+1)+chr(number), chr(letter)+chr(number-1), chr(letter)+chr(number+1))

def get_inductionloops(junction_id, number_out=-1):
    numbers = [0, 1, 2, 3]
    match number_out:
        case -1:
            return [junction_id + "_0_D"+str(num) + ".0" for num in [0,2,1,3]]
        case _:
            del numbers[number_out]
            return [junction_id + "_0_D" + str(num) + ".0" for num in numbers]


from itertools import product
def get_indices():
    ln = traci.trafficlight.getIDList()[-1]
    lastbutoneletter = ord(ln[0])
    lastbutonenumber = ord(ln[1])
    alphabet = list(string.ascii_uppercase)[2: list(string.ascii_uppercase).index(chr(lastbutoneletter))]
    numbers = map(chr,range(ord("2"), lastbutonenumber))
    return list(map(lambda x: x[0]+x[1], product(alphabet, numbers)))

#stworz slownik skrzyzowanie : ilosc samochodow na induction loops 4x4
# {junction : {NS : no vehicles, WE : no vehicles}}

def get_induction_cars(junction_id, beta=0.7):
    main_indices = get_inductionloops(junction_id)
    neighbours = get_neighbours(junction_id)
    ns_indices = get_inductionloops(neighbours[0], 2) + get_inductionloops(neighbours[1], 0)
    ew_indices = get_inductionloops(neighbours[2], 1) + get_inductionloops(neighbours[3], 3)
    ns = sum(map(traci.inductionloop.getLastStepVehicleNumber, main_indices[:2])) + beta * sum(map(traci.inductionloop.getLastStepVehicleNumber, ns_indices))
    ew = sum(map(traci.inductionloop.getLastStepVehicleNumber, main_indices[2:])) + beta * sum(map(traci.inductionloop.getLastStepVehicleNumber, ew_indices))
    return [ns, ew]

#def update_queque(trafficlights):

from operator import add
from math import exp
def run():
    """init"""
    trafficlights = get_indices()
    print(trafficlights)
    inductionloops_aggr = {light: [0, 0] for light in trafficlights}
    rand = random
    rand.seed(a=23)
    """execute the TraCI control loop"""
    step = 0
    #for label in traci.trafficlight.getIDList():
    #    print(traci.trafficlight.getAllProgramLogics(label))
    print(traci.inductionloop.getIDList())

    #print(traci.inductionloop.getLastStepVehicleNumber(idx))
        #print(traci.inductionloop.get)
    # we start with phase 2 where EW has green
    #traci.trafficlight.setPhase("0", 2)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        for light in trafficlights:
            inductionloops_aggr[light] = list(map(add, get_induction_cars(light), inductionloops_aggr[light]))
        if step % 5 == 0:
            for light in trafficlights:
                ns_ew = list(map(exp, inductionloops_aggr[light]))
                if traci.trafficlight.getPhase(light) == 0:
                # ns green
                    softmax = ns_ew[0] / sum(ns_ew)
                    if rand.random() < softmax:
                        traci.trafficlight.setPhase(light, 1)

                else:
                # ew green
                    softmax = ns_ew[1] / sum(ns_ew)
                    if rand.random() < softmax:
                        traci.trafficlight.setPhase(light, 3)

                inductionloops_aggr[light] = [0,0]
    #        if traci.inductionloop.getLastStepVehicleNumber("0") > 0:
                # there is a vehicle from the north, switch
    #            traci.trafficlight.setPhase("0", 3)
    #        else:
                # otherwise try to keep green for EW
    #            traci.trafficlight.setPhase("0", 2)
        step += 1
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
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
    traci.start([sumoBinary, "-c", "C:\\Users\\Ayin\\PycharmProjects\\SwarmTraffic\\data2\\grid66.sumocfg",
                             "--tripinfo-output", ".\\stats\\tripinfo.xml"])
    run()