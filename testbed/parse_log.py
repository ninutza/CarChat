#! /usr/bin/python
import sys

f = open('July20/random_med_mote6.log','r')
g = open('July20/random_med_mote6.out','w')

lines = f.readlines()

for line in lines:
  line = line.strip()
  if line != '':
    nums = line.split()
    output = ''
    source = int(nums[8],16)
    if source == 0:
      output = output + "Infrastructure message "
    elif source == 1:
      output = output + "Ping message " + str(int(nums[9],16)) # No_Pings
    elif source == 2:
      output = output + "Advertisement message "
    elif source == 3:
      output = output + "Request message "
    elif source == 4:
      output = output + "Data message "
    elif source == 8:
      output = output + "NEW Data status update: "
    elif source == 9:
      output = output + "Current data status: "

    output = output + " from " + str(int(nums[10]+nums[11],16))  # source address

    if source == 1: # PING
      if nums[12][0] < '8':	  # positive ping value
        output = output + " with ping value " + str(int(nums[12]+nums[13],16))
      else:  # negative ping value
        output = output + " with ping value " + str(int(nums[12]+nums[13],16)-2**16) 
    else: # data ID / data type concatenation
      output = output + " with data ID " + nums[12] + " and data type " + nums[13]

    output = output + " vNum: " + str(int(nums[14],16))            # version number
    output = output + " and pNum: " + str(int(nums[15],16)) + '\n' # packet number

    output = output + "    --- Current time: " + str(int(nums[16]+nums[17]+nums[18]+nums[19],16))
    output = output + ", have sent " + str(int(nums[22]+nums[23],16)) + " packets"
    output = output + " and have ver " + str(int(nums[20],16)) + ", packets up to " + str(int(nums[21],16)) + '\n'
    
    g.write(output)
