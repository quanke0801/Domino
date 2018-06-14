# -*- coding: utf-8 -*-

import math
import time

from pybullet import *

from Utilities import *
from Components import *

class Engine:
	def __init__(self, time_step = 0.01, mode = GUI):
		self.terminate = False
		self.paused = False
		self.time_step = time_step
		self.elapsed = 0.0
		self.plugins = []
		self.record = RecordedStatus(self)
		self.InitializePhysics(mode)
	def InitializePhysics(self, mode):
		connect(mode)
		if mode == GUI:
			resetDebugVisualizerCamera(1, 0, -40, [0.5, 0, 0])
		configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
		configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
		configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
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
	def Pause(self):
		for plugin in self.plugins:
			plugin.OnPause()
	def Unpause(self):
		for plugin in self.plugins:
			plugin.OnUnpause()
	def Start(self):
		self.record.Initialize()
		for plugin in self.plugins:
			plugin.OnStart()
		while not self.terminate:
			for plugin in self.plugins:
				plugin.OnIdle()
			if not self.paused:
				for plugin in self.plugins:
					plugin.OnBeforeStep()
				stepSimulation()
				self.elapsed += self.time_step
				self.record.Update()
				if self.record.GetLatest().time_stamp == self.elapsed:
					for plugin in self.plugins:
						plugin.OnUpdateRecord()
				for plugin in self.plugins:
					plugin.OnAfterStep()
			time.sleep(self.time_step)
		for plugin in self.plugins:
			plugin.OnTerminate()
		disconnect()
	
