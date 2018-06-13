# -*- coding: utf-8 -*-

import math
import time

from pybullet import *

from Engine import *

class PluginBase:
	def __init__(self, engine):
		self.engine = engine
	def OnAddComponent(self, component):
		pass
	def OnStart(self):
		pass
	def OnBeforeStep(self):
		pass
	def OnAfterStep(self):
		pass
	def OnTerminate(self):
		pass

class PrinterPlugin(PluginBase):
	def __init__(self, engine):
		PluginBase.__init__(self, engine)
	def OnStart(self):
		print('start simulation')
	def OnBeforeStep(self):
		if int(self.engine.elapsed * 10) != int((self.engine.elapsed - self.engine.time_step) * 10):
			print('time elapsed: %f' % self.engine.elapsed)
	def OnTerminate(self):
		print('terminated, total time elapsed: %f' % self.engine.elapsed)

class TerminatorPlugin(PluginBase):
	SPEED_THRESHOLD = 1.0E-2
	SPIN_THRESHOLD = 1.0E-1
	class BodiesStatus:
		def __init__(self, speed_threshold = -1, spin_threshold = -1):
			if speed_threshold <= 0:
				speed_threshold = TerminatorPlugin.SPEED_THRESHOLD
			if spin_threshold <= 0:
				spin_threshold = TerminatorPlugin.SPIN_THRESHOLD
			self.num_bodies = getNumBodies()
			self.position = [None] * self.num_bodies
			self.orientation = [None] * self.num_bodies
			self.linear = [None] * self.num_bodies
			self.speed = [0] * self.num_bodies
			self.angular = [None] * self.num_bodies
			self.spin = [0] * self.num_bodies
			self.motion_count = 0
			for i in range(self.num_bodies):
				(self.position[i], self.orientation[i]) = getBasePositionAndOrientation(i)
				(self.linear[i], self.angular[i]) = getBaseVelocity(i)
				self.speed[i] = sum([vi ** 2 for vi in self.linear[i]]) ** 0.5
				self.spin[i] = sum([wi ** 2 for wi in self.angular[i]]) ** 0.5
				if self.speed[i] > speed_threshold or self.spin[i] > spin_threshold:
					self.motion_count += 1
		def Distance(self, position1, position2):
			return sum([(pi1 - pi2) ** 2 for (pi1, pi2) in zip(position1, position2)]) ** 0.5
		def Angle(self, orientation1, orientation2):
			inner_product = sum([qi1 * qi2 for (qi1, qi2) in zip(orientation1, orientation2)])
			return math.acos(max(min(2 * inner_product ** 2 - 1, 1), -1))
		def Displacement(self, other, delta_time):
			if other is None:
				return False
			for i in range(min(self.num_bodies, other.num_bodies)):
				distance = self.Distance(self.position[i], other.position[i])
				if distance > delta_time * TerminatorPlugin.SPEED_THRESHOLD:
					return True
				angle = self.Angle(self.orientation[i], other.orientation[i])
				if angle > delta_time * TerminatorPlugin.SPIN_THRESHOLD:
					return True
			return False
	def __init__(self, engine, time_range = 0.1):
		PluginBase.__init__(self, engine)
		self.time_range = time_range
		self.size = max(2, int(time_range / self.engine.time_step))
		self.status = [None] * self.size
		self.motion_count = 0
		self.index = 0
	def NextIndex(self):
		return (self.index + 1) % self.size
	def LastIndex(self):
		return (self.index + self.size - 1) % self.size
	def OnAfterStep(self):
		self.index = self.NextIndex()
		if self.status[self.index] is not None:
			self.motion_count -= self.status[self.index].motion_count
		self.status[self.index] = TerminatorPlugin.BodiesStatus()
		self.motion_count += self.status[self.index].motion_count
		if self.engine.elapsed < self.time_range:
			return
		if self.motion_count == 0 or not self.status[self.index].Displacement(self.status[self.NextIndex()], self.time_range):
			self.engine.terminate = True
