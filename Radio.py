# Radio module

import math
import random

# Channel parameters
#n = 3.0 # path loss exponent
n = 3.0 # path loss exponent
sigma = 3.0 # standard deviation shadowing variance
d0 = 1.0 # reference distance
#pld0  = 55.0 # power decay for reference distance d0
pld0  = 35.0 # power decay for reference distance d0

# Radio parameters
pn = -105 # radio noise floor, not required for CPM
wgn = 4 # white gaussian noise, not required for CPM 
# Covariance Matrix for hardware variance
s11 = 3.7
s12 = -3.3
s21 = -3.3
s22 = 6.0

def initMote(network, radio, mote):
	""" Initializes a new mote for the network """
	try:
		mote.noiseModelCreated
		print "The noise model has already been added to the mote."
	except:
		#noise = open("/opt/tinyos-2.x/tos/lib/tossim/noise/meyer-heavy.txt", "r")
		noise = open("meyer-heavy.txt", "r")
		lines = noise.readlines()
		for line in lines:
			str = line.strip()
			if (str != ""):
				val = int(str)
				mote.addNoiseTraceReading(val)
		mote.createNoiseModel()
		mote.noiseModelCreated = True

	setRadioPtPn(mote)

	for othermote in network.motes:
		if (othermote != mote):
			setLinkGain(radio, othermote, mote)

def setRadioPtPn(mote):
	""" Set the output power and noise floor for a mote. """

	global s11, s12, s21, s22

	t11 = math.sqrt(s11)
	t12 = s12/math.sqrt(s11)
	t21 = 0;
	t22 = math.sqrt( (s11*s22 - math.pow(s12, 2)) / s11)

	rn1 = random.gauss(0, 1)
	rn2 = random.gauss(0, 1)
	mote.attr["Pn"] = pn + t11 * rn1
	mote.attr["Pt"] = t12 * rn1 + t22 * rn2

def updateNetworkLinkGains(radio, network):
	for i in range(len(network.motes)-1):
		for j in range(i, len(network.motes)):
                        # print " **** Setting link gain between ",i, " and ",j
			setLinkGain(radio, network.motes[i], network.motes[j])	

def setLinkGain(radio, mote1, mote2):
	""" Sets the gains between two motes. """

	pathloss = expectedPathloss(mote1.attr["x"], mote1.attr["y"], mote2.attr["x"], mote2.attr["y"]) + random.gauss(0, sigma)

        # assymetric links are given by running two different
        # R.V.s for each unidirectional link (output power variance).
	link1 = mote1.attr["Pt"] + pathloss
	link2 = mote2.attr["Pt"] + pathloss
	if link1 > 1.0:
		link1 = 1.0
	if link2 > 1.0:
		link2 = 1.0
        # debugging link connections, slows down program
        #if (mote1.id() != mote2.id()):
        #  print " **** Between links ",mote1.id(), " and ", mote2.id(), " connectivity ", link1, " and ", link2
        radio.add(mote1.id(), mote2.id(), link1)
        radio.add(mote2.id(), mote1.id(), link2)

def expectedGain(mote, x, y):
	""" Returns the expected amount of gain between a mote and another
	    mote hypothetically placed at location (x, y). """

	gain = mote.attr["Pt"] + expectedPathloss(mote.attr["x"], mote.attr["y"], x, y)	

	return gain

def expectedPathloss(x1, y1, x2, y2):
	""" Returns the expected amount of pathloss between a mote placed at
	    location (x1, y1) and another at (x2, y2). """

	global pn, pld0, n, d0, sigma

	Xdist = x1 - x2
	Ydist = y1 - y2

        # distance between a given pair of nodes
        dist = math.pow((Xdist*Xdist + Ydist*Ydist), 0.5)

	if dist < d0:
		pathloss = 0
	else:
        	# mean decay dependent on distance
        	pathloss = -pld0 - 10*n*(math.log(dist/d0)/math.log(10.0)) 
	return pathloss

def sendMessage(msg, packet, destination, time):
	packet.setData(msg.data)
	packet.setType(msg.get_amType())
	packet.setDestination(destination)
	packet.deliver(destination, time)

