# -*- coding: utf-8 -*-

import collections
import math
import time

from pybullet import *

from Engine import *

class Plugin:
	def __init__(self, engine):
		self.engine = engine
	def OnAddComponent(self, component):
		pass
	def OnStart(self):
		pass
	def OnPause(self):
		pass
	def OnUnpause(self):
		pass
	def OnIdle(self):
		pass
	def OnBeforeStep(self):
		pass
	def OnUpdateRecord(self):
		pass
	def OnAfterStep(self):
		pass
	def OnTerminate(self):
		pass

class PrintPlugin(Plugin):
	def __init__(self, engine):
		Plugin.__init__(self, engine)
	def OnStart(self):
		print('start simulation')
	def OnPause(self):
		print('simulation paused')
	def OnUnpause(self):
		print('simulation unpaused')
	def OnBeforeStep(self):
		if int(self.engine.elapsed * 10) != int((self.engine.elapsed - self.engine.time_step) * 10):
			print('time elapsed: %f' % self.engine.elapsed)
	def OnTerminate(self):
		print('terminated, total time elapsed: %f' % self.engine.elapsed)

class AutoTerminatePlugin(Plugin):
	SPEED_THRESHOLD = 1.0E-2
	SPIN_THRESHOLD = 1.0E-1
	def __init__(self, engine, window = 0.5):
		Plugin.__init__(self, engine)
		self.window = window
	def OnUpdateRecord(self):
		latest = self.engine.record.GetLatest()
		if latest.time_stamp <= self.window:
			return
		first = self.engine.record.Get(latest.time_stamp - self.window)
		delta_time = latest.time_stamp - first.time_stamp
		for (distance, angle) in latest.Difference(first):
			if distance > AutoTerminatePlugin.SPEED_THRESHOLD * delta_time:
				return
			if angle > AutoTerminatePlugin.SPIN_THRESHOLD * delta_time:
				return
		self.engine.terminate = True

class ComponentTestPlugin(Plugin):
	DISTANCE_THRESHOLD = 1.0E-2
	ANGLE_THRESHOLD = 1.0E-1
	def __init__(self, engine, sequences = [[]], statics = []):
		Plugin.__init__(self, engine)
		self.sequences = sequences
		self.sequence_moments = [[None] * len(sequence) for sequence in self.sequences]
		self.statics = statics
		self.static_moments = [None] * len(self.statics)
		self.initial_states = None
		self.test_success = True
	def OnStart(self):
		self.initial_states = self.engine.record.GetLatest()
	def OnUpdateRecord(self):
		latest = self.engine.record.GetLatest()
		for si in range(len(self.sequences)):
			for i in range(len(self.sequences[si])):
				if self.sequence_moments[si][i] is not None:
					continue
				body_id = self.sequences[si][i]
				(distance, angle) = latest[body_id].Difference(self.initial_states[body_id])
				if distance > ComponentTestPlugin.DISTANCE_THRESHOLD or angle > ComponentTestPlugin.ANGLE_THRESHOLD:
					self.sequence_moments[si][i] = latest.time_stamp
		for i in range(len(self.statics)):
			if self.static_moments[i] is not None:
				continue
			body_id = self.statics[i]
			(distance, angle) = latest[body_id].Difference(self.initial_states[body_id])
			if distance > ComponentTestPlugin.DISTANCE_THRESHOLD or angle > ComponentTestPlugin.ANGLE_THRESHOLD:
				self.static_moments[si][i] = latest.time_stamp
	def OnTerminate(self):
		for si in range(len(self.sequences)):
			for i in range(len(self.sequences[si])):
				if self.sequence_moments[si][i] is None:
					self.test_success = False
					return
				if i > 0 and self.sequence_moments[si][i] < self.sequence_moments[si][i - 1]:
					self.test_success = False
					return
		for i in range(len(self.statics)):
			if self.static_moments[i] is not None:
				self.test_success = False
				return

class KeyboardEventPlugin(Plugin):
	def __init__(self, engine):
		Plugin.__init__(self, engine)
	def OnIdle(self):
		events = getKeyboardEvents()
		for (key, state) in events.items():
			if key == B3G_RETURN and (state & KEY_WAS_TRIGGERED):
				self.engine.paused = not self.engine.paused
