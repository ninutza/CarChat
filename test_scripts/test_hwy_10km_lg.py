#! /usr/bin/env python
from TOSSIM import *
import sys
import time
 
# Matt's classes for managing Network and Radio features
import Network
import Radio

SIM_UNIT = 0.6;	# update every 0.3 seconds for step size = 1 -> 70 mph
                # update every 0.9 seconds for step size = 2 -> 50 mph
                # update every 0.6 seconds for step size = 1 -> 35 mph
NO_VEH = 100;
RUN_TIME = 575; # want to use up whole trajectory file

# start out with NO_VEH vehicular nodes, wallking past each other in (0.3 sec intervals) (at 1 m)
t = Tossim([]);
r = t.radio();
network = Network.Network();

# initialize an array of motes, to be referred to throughout simulation
newMotes = [None] * (NO_VEH+1);

f = open("hwy_10km_lg.txt", "r");
line = f.readline();
s = line.split()

for i in range(1,NO_VEH+1):
  print "Mote ", i, "initialized in position (", int(s[2*i-1]), ", ", int(s[2*i]), ")"
  newMotes[i] = t.getNode(i);
  newMotes[i].bootAtTime(i*7+53470);
  newMotes[i].attr = {}
  newMotes[i].attr["type"] = "mobile"
  newMotes[i].attr["relays"] = []
  newMotes[i].attr["mobility"] = "mobile"
  newMotes[i].attr["x"] = int(s[2*i-1])
  newMotes[i].attr["y"] = int(s[2*i])
  newMotes[i].attr["oldX"] = int(s[2*i-1])
  newMotes[i].attr["oldY"] = int(s[2*i])
  newMotes[i].attr["neighbors"] = []
  newMotes[i].attr["stucktime"] = 0

  network.addMote(newMotes[i]);

  print " --- adding mote ", i, "to radio class"
  Radio.initMote(network, r, newMotes[i]);

print "Infrastructure mote initialized in position (0, 200)"
infrMote = t.getNode(301);
infrMote.bootAtTime(20000);
infrMote.attr = {}
infrMote.attr["type"] = "infrastructure"
infrMote.attr["relays"] = []
infrMote.attr["mobility"] = "fixed"
infrMote.attr["x"] = 0
infrMote.attr["y"] = 200
infrMote.attr["oldX"] = 0
infrMote.attr["oldY"] = 200
infrMote.attr["neighbors"] = []
infrMote.attr["stucktime"] = 0

network.addMote(infrMote);

print " --- adding infrastructure mote to radio class"
Radio.initMote(network, r, infrMote);


#g = open("one_line_hwy.out","w");
h = open("hwy_10km_lg.out","w");
#t.addChannel("CarChat", sys.stdout)
#t.addChannel("CarChat", g)

#t.addChannel("CarData", sys.stdout)
t.addChannel("CarData", h)
#t.addChannel("CarData", g)

#t.addChannel("CarErr",sys.stdout)
#t.addChannel("ActiveChat",sys.stdout)
#t.addChannel("InfrChat", sys.stdout)
#t.addChannel("InfrErr", sys.stdout)
#t.addChannel("CarMisc",sys.stdout) # debugging statements for intermediary steps

# initialize update parameters
lastUpdate = t.time();
updateRate = SIM_UNIT * t.ticksPerSecond();  #update position every SIM_UNIT seconds
stopTime = t.time() + RUN_TIME * updateRate;
#print "Update rate in ticks is ", updateRate

print " ***** -- Simulation will run from time ", int(t.time()), "until ", stopTime, " -- *****"

while int(t.time()) < stopTime :
  # print "Time is currently ", int(t.time()), ", should update at ", int(lastUpdate + updateRate)
  if (lastUpdate + updateRate < t.time() ):   # time to update
    print "Time to update! it's now ", str(t.time()/t.ticksPerSecond()), ", next update at ", str((updateRate+t.time())/t.ticksPerSecond())
    h.write(' ---- UPDATE at time ' + str(int(t.time()/t.ticksPerSecond())) + ' ----\n');
    #g.write(' ---- UPDATE at time ' + str(int(t.time()/t.ticksPerSecond())) + ' ----\n');
    # read next line in file f, update positions of all nodes
    line = f.readline();
    h.write('Motes at positions:' +line)
    #g.write('Motes at positions:' +line)
    s = line.split();
    if len(s) > 0:
      for i in range(1,NO_VEH+1):
        #print "Mote ", i, "moved to position (", int(s[2*i-1]), ", ", int(s[2*i]), ")"
        newMotes[i].attr["oldX"] = newMotes[i].attr["x"];
        newMotes[i].attr["oldY"] = newMotes[i].attr["y"];
        newMotes[i].attr["x"] = int(s[2*i-1]);
        newMotes[i].attr["y"] = int(s[2*i]);

        # update radio connectivity
        Radio.updateNetworkLinkGains(r, network);

        lastUpdate = t.time();
    else:
      break; # reached end of file by accident, quit simulation

  t.runNextEvent();

print " ***** Finished simulation!!! ***** "



