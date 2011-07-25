#! /usr/bin/env python

import sys
import time
 
NO_NODES = 100

MoteStatus = [None] * (NO_NODES+2); # initializing array for motes, each item in the array will hold running count of status of a node

f = open('hwy_50km_lg.out','r');
lines = f.readlines();

g = open('temp1.txt','w');

interm = 1

for line in lines:
  words = line.strip();
  if (interm == 1 and 'UPDATE' in line):
    g.write(line)
    interm = 0
  else:
    if ('Have sent' in line):
      g.write(line)
      interm = 1

f.close()
g.close()

g = open('temp1.txt','r')
lines = g.readlines()


# read each line of simplified output, at new time start new line for second output file,
# each subsequent line will update values in MoteStatus array

for i in range(0, NO_NODES+2):
  MoteStatus[i] = 0;	# initialize all motes to no messages sent

curr_time = -1;

for line in lines:
  if ('UPDATE' in line):
    words = line.split()
    curr_time = int(words[4])
    if curr_time > 0:
      MoteStatus[0] = curr_time
      print str(MoteStatus)
    # print current status of MoteStatus array
    # print str(MoteStatus)
  else:
    if ('Have sent' in line): #make sure we're not processing blank lines here
      words = line.split()
      words[1] = words[1].replace('(','')
      words[1] = words[1].replace('):','')
      #print words[1] + ' ... ' + words[12]
      MoteStatus[NO_NODES+1] = MoteStatus[NO_NODES+1] + 20;
      MoteStatus[int(words[1])] = int(words[12])
      



