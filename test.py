#! /usr/bin/python
from TOSSIM import *
import sys

# allow for 3 vehicular nodes, and one infrastructure node (id =101)
t = Tossim([])
r = t.radio()
f = open("topo.txt", "r")

lines = f.readlines()
for line in lines:
  s = line.split()
  if (len(s) > 0):
    print " ", s[0], " ", s[1], " ", s[2];
    r.add(int(s[0]), int(s[1]), float(s[2]))

t.addChannel("CarChat", sys.stdout)
t.addChannel("InfrChat", sys.stdout)
t.addChannel("InfrErr", sys.stdout)
# t.addChannel("Boot", sys.stdout)

noise = open("meyer-heavy.txt", "r")
lines = noise.readlines()
for line in lines:
  str1 = line.strip()
  if (str1 != ""):
    val = int(str1)
    for i in range(1, 3):
      t.getNode(i).addNoiseTraceReading(val)
    t.getNode(101).addNoiseTraceReading(val)
  

for i in range(1, 3):
  print "Creating noise model for ",i;
  t.getNode(i).createNoiseModel()
t.getNode(101).createNoiseModel()

t.getNode(101).bootAtTime(99994);
t.getNode(1).bootAtTime(100001);
t.getNode(2).bootAtTime(100008);
#t.getNode(3).bootAtTime(100009);

for i in range(0, 800):
  t.runNextEvent()
  #print "Time is ", t.time()

