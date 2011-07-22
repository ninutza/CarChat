#! /usr/bin/python
import sys
import random
import math

NO_INTERS_X = 21;	# east-west size (width) of grid
NO_INTERS_Y = 81;       # north-south size (height) of grid

GRAN = 10; #block size (in m) * 10 (scale)
STEP = 2;

NO_VEH = 100;

CITY_PROB = 0.8;
RURAL_PROB = 0.4;
HWY_PROB = 0.1;

# in  city: 0.8 (more often than not, will change direction)
# in rural: 0.4 (change direction at less than half of main road intersections)
# in  hwy : 0.1 (do no change direction unless forced to) 
DEF_PROB = CITY_PROB;


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

# for general model, allow random starting positions 
for i in range(0, NO_VEH):
  x_init[i] = random.randrange(0, NO_INTERS_X) * GRAN;
  y_init[i] = random.randrange(0, NO_INTERS_Y) * GRAN;
  prob_turn[i] = DEF_PROB;
  if i >= NO_VEH/2 : #for the second half on vehicles, make sure y coordinate is AWAY from origin (where infr is located)
    if y_init[i] < RANGE_Y/2: 
      y_init[i] = RANGE_Y - 1 - y_init[i];	# mirror against halfway distance across grid from infr

#for rural/city, initialize the first node at same location as infrastructure (intersection closest to origin) to ensure data gets to ntwk
x_init[0] = 0;
y_init[0] = 0;

for i in range(0, NO_VEH):
  x_curr[i] = x_init[i];
  y_curr[i] = y_init[i];
  x_dest[i] = y_init[i];
  y_dest[i] = y_curr[i];

#for rural/city, initialize dest of the first node at opposite corner to ensure wide mobility
# also, force node to start moving outward
x_dest[0] = RANGE_X-1;
y_dest[0] = RANGE_Y-1;
x_dir[0] = 2;
y_dir[0] = 0;

# for general random simulation
TARGET_TIME = max(NO_INTERS_X,NO_INTERS_Y) * GRAN * 5 / STEP;
# smaller target time for contrived highway case
print "Range of X is ",RANGE_X," and range of Y ", RANGE_Y
#TARGET_TIME = (max(RANGE_X,RANGE_Y)-1-1000) / STEP;
print "Target time is ", TARGET_TIME


t_curr = 0;
f = open("city_traj.txt", "w")


while (t_curr < TARGET_TIME):

  for i in range(0, NO_VEH):
    if (x_dest[i] == x_curr[i]) and (y_dest[i] == y_curr[i]): # if this node has reached destination (or just started), get new destination
      while(x_dest[i] == x_curr[i]) and (y_dest[i] == y_curr[i]):	# to avoid trivial routes
        x_dest[i] = random.randrange(0, NO_INTERS_X) * GRAN;
        y_dest[i] = random.randrange(0, NO_INTERS_Y) * GRAN;
        if i >= NO_VEH/2 : #for the second half on vehicles, make sure y coordinate is AWAY from origin (where infr is located)
          if y_dest[i] < RANGE_Y/2: 
            y_dest[i] = RANGE_Y - 1 - y_dest[i];	# mirror against halfway distance across grid from infr

      x_init[i] = x_curr[i];			# reinitialize source of node to current position
      y_init[i] = y_curr[i];
      prob_turn[i] = DEF_PROB;                  # probability of turning depends on scenario initially (becomes 0 when on final stretch)

      #print "Node ", i, "reassigned from (", x_init[i], ", ", y_init[i], ") to (", x_dest[i], ", ", y_dest[i], ") at time ", t_curr
  
      #establish initial direction axis along longest distance to destination
      if( abs(x_dest[i] - x_curr[i]) > abs(y_dest[i] - y_curr[i]) ):
        x_dir[i] = STEP * (x_dest[i] - x_curr[i])/(abs(x_dest[i] - x_curr[i])); 
        y_dir[i] = 0;
      else:
        y_dir[i] = STEP * (y_dest[i] - y_curr[i])/(abs(y_dest[i] - y_curr[i]));
        x_dir[i] = 0;


  #print car positions on one line in output file
  f.write(str(t_curr));
  for i in range(0, NO_VEH):
    f.write(' ' + str(x_curr[i]) + ' ' + str(y_curr[i]));
  f.write('\n');

  for i in range(0, NO_VEH):    # each vehicle decides timestep move independently
    #insert small probability of not moving at all, to vary speed from nominal time increment
    move_prob = random.random()

    if move_prob < 0.1: # stay in place with probability 10%
      x_curr[i] = x_curr[i];
      y_curr[i] = y_curr[i];
    else:		      # with probability 90%, choose to move
      if (x_curr[i] % GRAN == 0) and (y_curr[i] % GRAN == 0):	#at an intersection
        if (x_curr[i] == x_dest[i]):	# arrived on x coordinate of destination, only direction can be y
          y_dir[i] = STEP * (y_dest[i] - y_curr[i])/(abs(y_dest[i] - y_curr[i]));
          x_dir[i] = 0;
          prob_turn[i] = 0;
        else:
          if (y_curr[i] == y_dest[i]):  # arrived on y coordinate of destination, only direction can be x
            x_dir[i] = STEP * (x_dest[i] - x_curr[i])/(abs(x_dest[i] - x_curr[i])); 
            y_dir[i] = 0;
            prob_turn[i] = 0;
          else:
            choice = random.random()
            #f.write('At intersection ' + str(x_curr[i])  + ',' + str(y_curr[i]) + ' chose ' + str(choice) + ', thus ')
            # print choice;
            if choice < prob_turn[i]: 	# change current direction with prob_turn chance
              #f.write('turning\n')
              if x_dir[i] == 0:        # if current direction is along y
                x_dir[i] = STEP * (x_dest[i] - x_curr[i])/(abs(x_dest[i] - x_curr[i]));
                y_dir[i] = 0;
              else:		      # else, current direction must have been x
                y_dir[i] = STEP * (y_dest[i] - y_curr[i])/(abs(y_dest[i] - y_curr[i]));
                x_dir[i] = 0;
            #else:
              #f.write('not turning\n')

      x_curr[i] = x_curr[i] + x_dir[i];
      y_curr[i] = y_curr[i] + y_dir[i];

  # time value only updates after all vehicles have moved 
  t_curr = t_curr + 1;
      
# print final position of cars
f.write(str(t_curr));
for i in range(0, NO_VEH):
  f.write(' ' + str(x_curr[i]) + ' ' + str(y_curr[i]));
f.write('\n');

