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
		self.record = RecordedState(self)
		self.mode = mode
		self.InitializePhysics()
	def InitializePhysics(self):
		connect(self.mode)
		configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
		configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
		configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
		setGravity(0, 0, -10)
		setTimeStep(self.time_step)
		ground_id = createCollisionShape(GEOM_PLANE)
		createMultiBody(0, ground_id)
		changeDynamics(ground_id, -1, lateralFriction = 0.5, restitution = 0.1)
		if self.mode == GUI:
			resetDebugVisualizerCamera(2, 0, -40, [0.5, 0, 0])
			ground_texture_id = loadTexture('ground.jpg')
			changeVisualShape(ground_id, -1, -1, ground_texture_id)
		#print(getPhysicsEngineParameters())
		setPhysicsEngineParameter(numSolverIterations = 50)
	def AddPlugin(self, plugin):
		self.plugins += [plugin]
	def AddComponent(self, component):
		if component is None:
			return
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
		last_time = time.time()
		while not self.terminate:
			for plugin in self.plugins:
				plugin.OnIdle()
			if not self.paused:
				for plugin in self.plugins:
					plugin.OnBeforeStep()
				last_time = time.time()
				stepSimulation()
				self.elapsed += self.time_step
				self.record.Update()
				if self.record.GetLatest().time_stamp == self.elapsed:
					for plugin in self.plugins:
						plugin.OnUpdateRecord()
				for plugin in self.plugins:
					plugin.OnAfterStep()
			if self.mode == GUI:
				until_next = self.time_step - (time.time() - last_time)
				if until_next > 0:
					time.sleep(self.time_step)
		for plugin in self.plugins:
			plugin.OnTerminate()
		disconnect()
	
