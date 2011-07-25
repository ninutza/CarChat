#! /usr/bin/python
import sys
import string

NO_VEH = 100

f = open('hwy_compl_5km_lg.txt','r')
g = open('hwy_msg_5km_lg.txt', 'r')

lines = f.readlines()
curr_msg = 0
msgLine = g.readline()
msgLine = msgLine.strip()
msgLine = string.replace(msgLine,'[','')
msgLine = string.replace(msgLine,']','')
msgLine = string.replace(msgLine,',','')
msg_parts = msgLine.split()
msg_time = int(msg_parts[0])
msg_val = int(msg_parts[NO_VEH + 1])
# print msg_time, ' ', msg_val

for line in lines:
  line = line.strip()
  line = string.replace(line,'[','')
  line = string.replace(line,']','')
  line = string.replace(line,',','')
  numbers = line.split()
  curr_time = int(numbers[0])
  #print curr_time

  #print curr_time, msg_time

  while (curr_time >= msg_time):
    # print curr_time, " is larger than ", msg_time
    msgLine = g.readline()
    msgLine = msgLine.strip()
    if(msgLine != ''):
      msgLine = string.replace(msgLine,'[','')
      msgLine = string.replace(msgLine,']','')
      msgLine = string.replace(msgLine,',','')
      msg_parts = msgLine.split()
      curr_msg = msg_val
      #print msg_parts[0], " new message time, with count ", msg_parts[21]
      msg_time = int(msg_parts[0])
      msg_val = int(msg_parts[NO_VEH + 1])
    else:
      msg_time = curr_time+1
  
  print line, ' ', curr_msg

#print '[1, 2, 3, 4, 5]'.translate(None, '[],')
