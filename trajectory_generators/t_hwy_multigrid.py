#! /usr/bin/python
import sys
import random
import math

NO_INTERS_X = 1;
NO_INTERS_Y = 501; # 51 for 5 km, 101 for 10 km, 501 for 50 km

GRAN = 10; #block size (in m) * 10 (scale)
STEP = 2; # how far to advance on each time step

NO_VEH = 10;

#DIST_HWY = 10; # following distance between vehicles on highway 

RANGE_X = (NO_INTERS_X-1) * GRAN + 1;
RANGE_Y = (NO_INTERS_Y-1) * GRAN + 1;

# initialize all arrays needed for later
x_init=range(0,NO_VEH);
y_init=range(0,NO_VEH);
x_curr=range(0,NO_VEH);
y_curr=range(0,NO_VEH);
x_dest=range(0,NO_VEH);
y_dest=range(0,NO_VEH);
prob_turn=range(0,NO_VEH);
x_dir=range(0,NO_VEH);
y_dir=range(0,NO_VEH);

# for general model, allow random starting positions, except those landing between infrastructure
for i in range(0, NO_VEH):
  x_init[i] = (random.randrange(0, NO_INTERS_X)) * GRAN; # should be 0 all the time for highway
  x_dest[i] = (random.randrange(0, NO_INTERS_X)) * GRAN;
  x_dir[i] = 0;

  y_init[i] = (random.randrange(0, NO_INTERS_Y)) * GRAN; # generate between [0km, and (dist)km]
  if y_init[i] >= 200 :
    y_init[i] = y_init[i] + RANGE_Y + 200;
    y_dest[i] = 0;
    y_dir[i] = 0 - STEP;
  else:
    y_dest[i] = 2 * (RANGE_Y - 1) + 200;
    y_dir[i] = STEP;


for i in range(0, NO_VEH):
  x_curr[i] = x_init[i];
  y_curr[i] = y_init[i];

print "Range of X is ",RANGE_X," and range of Y ", RANGE_Y
TARGET_TIME = RANGE_Y / STEP + 75;
print "Target time is ", TARGET_TIME

t_curr = 0;
f = open("hwy_rand_traj.txt", "w")


while (t_curr < TARGET_TIME):

  #print car positions on one line in output file
  f.write(str(t_curr));
  for i in range(0, NO_VEH):
    f.write(' ' + str(x_curr[i]) + ' ' + str(y_curr[i]));
  f.write('\n');

  for i in range(0, NO_VEH):    # each vehicle decides timestep move independently
    #insert small probability of not moving at all, to vary speed from nominal time increment
    move_prob = random.random()

    if move_prob < 0.05: # stay in place with probability 5%
      x_curr[i] = x_curr[i];
      y_curr[i] = y_curr[i];
    else:		      # with probability 90%, choose to move
      x_curr[i] = x_curr[i] + x_dir[i];
      y_curr[i] = y_curr[i] + y_dir[i];

  # time value only updates after all vehicles have moved (silly omission)
  t_curr = t_curr + 1;
      
# print final position of cars
f.write(str(t_curr));
for i in range(0, NO_VEH):
  f.write(' ' + str(x_curr[i]) + ' ' + str(y_curr[i]));
f.write('\n');

