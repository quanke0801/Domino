# -*- coding: utf-8 -*-

import math
import time
import queue

from pybullet import *

from Components import *

class Engine:
	def __init__(self, time_step = 0.01):
		self.time_step = time_step
		self.time_stamp = 0.0
		self.client = connect(GUI)
		resetDebugVisualizerCamera(1, 0, -40, [0.5, 0, 0])
		setGravity(0, 0, -10)
		setTimeStep(time_step)
		ground = createCollisionShape(GEOM_PLANE)
		changeDynamics(ground, -1, lateralFriction = 0.5, restitution = 0.1)
		createMultiBody(0, ground)
	def Start(self):
		while True:
			print(self.time_stamp)
			stepSimulation()
			time.sleep(self.time_step)
			self.time_stamp += self.time_step
	def AddComponent(self, component):
		component.Create()
