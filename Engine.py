# -*- coding: utf-8 -*-

import math
import time

from pybullet import *

from Components import *

class Engine:
	def __init__(self, time_step = 0.01, mode = GUI):
		self.terminate = False
		self.time_step = time_step
		self.elapsed = 0.0
		self.plugins = []
		self.InitializePhysics(mode)
	def InitializePhysics(self, mode):
		connect(mode)
		if mode == GUI:
			resetDebugVisualizerCamera(1, 0, -40, [0.5, 0, 0])
		setGravity(0, 0, -10)
		setTimeStep(self.time_step)
		ground = createCollisionShape(GEOM_PLANE)
		changeDynamics(ground, -1, lateralFriction = 0.5, restitution = 0.1)
		createMultiBody(0, ground)
	def AddPlugin(self, plugin):
		self.plugins += [plugin]
	def AddComponent(self, component):
		for plugin in self.plugins:
			plugin.OnAddComponent(component)
		component.Create()
	def Start(self):
		for plugin in self.plugins:
			plugin.OnStart()
		while not self.terminate:
			for plugin in self.plugins:
				plugin.OnBeforeStep()
			stepSimulation()
			for plugin in self.plugins:
				plugin.OnAfterStep()
			time.sleep(self.time_step)
			self.elapsed += self.time_step
		for plugin in self.plugins:
			plugin.OnTerminate()
	
