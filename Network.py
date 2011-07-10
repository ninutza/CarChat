import Trig

class Network:
	def __init__(self):
		self.motes = []
		self.mobileMotes = []
		self.relayMotes = []
		self.groups = []

	def reset(self, radio):
		# Turn off all motes and blank out attributes
		for mote in self.motes:
			for otherMote in self.motes:
				radio.remove(mote.id(), otherMote.id())
			mote.attr = {}
			print "turing off",mote.id()
			mote.turnOff()
		self.motes = []
		self.mobileMotes = []
		self.relayMotes = []
		self.groups = []

	def addMote(self, mote):
		self.motes.append(mote)
		if (mote.attr["type"] == "mobile"):
			self.mobileMotes.append(mote)
		else:
			self.relayMotes.append(mote)

	def addGroup(self, group):
		self.groups.append(group)
		for mote in group.motes:
			self.addMote(mote)

	def getNumMobileMotes(self):
		return len(self.mobileMotes)

	def getMoteById(self, id):
		mote = None
		for testmote in self.motes:
			if testmote.id() == id:
				mote = testmote	
				break
		return mote

	def areRelaysConnected(self, sim, currentTime):
		if len(self.relayMotes) < 2:
			return True

		moteList = self.relayMotes[1:]
		mote = self.relayMotes[0]
		neighbors = mote.attr["neighbors"]
		while len(neighbors) > 0:
			newNeighbors = []
			for neighbor in neighbors:
				neighborMote = self.getMoteById(neighbor)
				if ((neighborMote in moteList)):
					moteList.remove(neighborMote)
					newNeighbors.append(neighborMote.attr["neighbors"])
			neighbors = newNeighbors
		count = 0
		for motes in moteList:
 			if mote.attr["deploytime"] + (60 * sim.t.ticksPerSecond()) < currentTime:
				count += 1
		if count>0:
			return False
		else:
			return True

class Group:
	# motes, x, y, direction
	def __init__(self):
		self.motes = []
		# Starts at the current location or the last visited landmark.
		self.path = []
		self.speed = 0
		# The group is only mobile if the current time is grater than this time
		self.timeMobile = 0
		self.direction = 0
		self.shift = 0

	def setShift(self, shift):
		self.shift = shift

	def setPath(self, path):
		self.path = path
		self.direction = Trig.direction(path[0].x, path[0].y, path[1].x, path[1].y)

	def setSpeed(self, speed):
		self.speed = speed

	def setCenter(self, x, y):
		self.x = x
		self.y = y

	def setTimeMobile(self, time):
		self.timeMobile = time

	def addMote(self, mote):
		self.motes.append(mote)
