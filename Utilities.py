# -*- coding: utf-8 -*-

import collections
import math

from pybullet import *

class BodyStatus:
	def __init__(self, body_id):
		self.body_id = body_id
		(self.position, self.orientation) = getBasePositionAndOrientation(self.body_id)
		(self.linear, self.angular) = getBaseVelocity(self.body_id)
		self.speed = sum([vi ** 2 for vi in self.linear]) ** 0.5
		self.spin = sum([wi ** 2 for wi in self.angular]) ** 0.5
	def Difference(self, other):
		distance = sum([(pi1 - pi2) ** 2 for (pi1, pi2) in zip(self.position, other.position)]) ** 0.5
		angle = math.acos(max(min(2 * sum([qi1 * qi2 for (qi1, qi2) in zip(self.orientation, other.orientation)]) ** 2 - 1, 1), -1))
		return (distance, angle)

class WorldStatus:
	def __init__(self, engine):
		self.time_stamp = engine.elapsed
		self.status = [BodyStatus(i) for i in range(getNumBodies())]
	def __len__(self):
		return len(self.status)
	def __getitem__(self, index):
		return self.status[index]
	def Difference(self, other):
		return [si1.Difference(si2) for (si1, si2) in zip(self.status, other.status)]

class RecordedStatus:
	def __init__(self, engine, sample_interval = 0.05, time_span = -1):
		self.engine = engine
		self.sample_interval = sample_interval
		self.time_span = time_span
		self.record = collections.deque()
	def __len__(self):
		return len(self.record)
	def Initialize(self):
		self.record.append(WorldStatus(self.engine))
	def Update(self):
		if len(self.record) == 0 or self.engine.elapsed - self.record[-1].time_stamp > self.sample_interval:
			self.record.append(WorldStatus(self.engine))
		if self.time_span > 0 and self.engine.elapsed - self.record[0].time_stamp > self.time_span:
			self.record.popleft()
	def GetLatest(self):
		return self.record[-1]
	def __BinarySearchGet(self, time_stamp, lower_index, upper_index):
		if upper_index == lower_index + 1:
			# TODO: interpolation
			return self.record[lower_index]
		mid_index = (lower_index + upper_index) // 2
		if time_stamp < self.record[mid_index].time_stamp:
			return self.__BinarySearchGet(time_stamp, lower_index, mid_index)
		else:
			return self.__BinarySearchGet(time_stamp, mid_index, upper_index)
	def Get(self, time_stamp):
		if len(self.record) == 0:
			return None
		if time_stamp < self.record[0].time_stamp:
			return None
		if time_stamp > self.record[-1].time_stamp:
			return None
		if time_stamp == self.record[-1].time_stamp:
			return self.record[-1]
		return self.__BinarySearchGet(time_stamp, 0, len(self.record) - 1)
	def __getitem__(self, time_stamp):
		return self.Get(time_stamp)
