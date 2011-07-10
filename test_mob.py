#! /usr/bin/python
from TOSSIM import *
import sys
import time
 
# Matt's classes for managing Network and Radio features
import Network
import Radio


# start out with 2 vehicular nodes, wallking past each other in 10 steps (1 sec intervals)
t = Tossim([]);
r = t.radio();
network = Network.Network();

# initialize update parameters
lastUpdate = t.time();
updateRate = 1 * t.ticksPerSecond();  #update position every 1 second
print "Update rate in ticks is ", updateRate

stopTime = t.time() + 20 * updateRate;

# initialize first mote at (0, 0)
newMote0 = t.getNode(0);
newMote0.bootAtTime(1);
newMote0.attr = {}
newMote0.attr["type"] = "mobile"
newMote0.attr["relays"] = []
newMote0.attr["mobility"] = "mobile"
newMote0.attr["x"] = 0
newMote0.attr["y"] = 0
newMote0.attr["oldX"] = 0
newMote0.attr["oldY"] = 0
newMote0.attr["neighbors"] = []
newMote0.attr["stucktime"] = 0

network.addMote(newMote0);
Radio.initMote(network, r, newMote0);

# initialize second mote at (0, 50)
newMote1 = t.getNode(1);
newMote1.bootAtTime(8);
newMote1.attr = {}
newMote1.attr["type"] = "mobile"
newMote1.attr["relays"] = []
newMote1.attr["mobility"] = "mobile"
newMote1.attr["x"] = 0
newMote1.attr["y"] = 30
newMote1.attr["oldX"] = 0
newMote1.attr["oldY"] = 30
newMote1.attr["neighbors"] = []
newMote1.attr["stucktime"] = 0

network.addMote(newMote1);
Radio.initMote(network, r, newMote1);

Radio.updateNetworkLinkGains(r, network)

# initialize output channels for debugging statements
t.addChannel("CarChat", sys.stdout)
t.addChannel("InfrChat", sys.stdout)
t.addChannel("InfrErr", sys.stdout)
# t.addChannel("Boot", sys.stdout)

# print "Simulation will run from time ", int(t.time()), "until ", stopTime

while newMote0.attr["y"] < 25 and t.time() < stopTime:
  # print "Time is currently ", int(t.time()), ", should update at ", int(lastUpdate + updateRate)
  if (lastUpdate + updateRate < t.time() ) and (1==2):   # time to update
    # set new position of node 0 here.
    newMote0.attr["oldY"] = newMote0.attr["y"];
    newMote0.attr["y"] = newMote0.attr["y"] + 0.5;

    print "*** Moved node 0 to (0, ", newMote0.attr["y"], ") at time ", int(t.time())

    # set new position of node 1 here.
    newMote1.attr["oldY"] = newMote1.attr["y"];
    newMote1.attr["y"] = newMote1.attr["y"] - 0.5;

    print "*** Moved node 1 to (1, ", newMote1.attr["y"], ")"

    # update radio connectivity
    Radio.updateNetworkLinkGains(r, network);

    lastUpdate = t.time();
    

  t.runNextEvent()

