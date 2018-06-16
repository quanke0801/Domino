# -*- coding: utf-8 -*-

import math

from scipy.optimize import fsolve
from pybullet import *

DOMINO_SIZE = (SX, SY, SZ) = (0.015, 0.05, 0.1)
DOMINO_MASS = SX * SY * SZ * 1000

class Component:
	def __init__(self, xyz = [0, 0, 0], rpy = [0, 0, 0]):
		(self.translation, self.rotation) = (xyz, getQuaternionFromEuler(rpy))
		self.children = {}
		# self.input = []
		# self.output = []
		self.created = False
	def Create(self, base_translation = [0, 0, 0], base_rotation = [0, 0, 0, 1]):
		if self.created:
			return
		(now_translation, now_rotation) = multiplyTransforms(
			base_translation, base_rotation, self.translation, self.rotation)
		for child in self.children.values():
			child.Create(now_translation, now_rotation)
		self.created = True
	def __getitem__(self, name):
		return self.children[name]
	def Id(self, name = None):
		if name is None:
			assert len(self.children) == 1, 'More than 1 child exist, must provide a name !'
			return list(self.children.values())[0].Id()
		return self.children[name].Id()
	def AllIds(self):
		ids = []
		for child in self.children.values():
			ids += child.AllIds()
		return ids

class SingleDomino(Component):
	ORIENTATION_ALIAS = {
		'xyz': [0, 0, 0],
		'xzy': [math.pi / 2, 0, 0],
		'yxz': [0, 0, math.pi / 2],
		'yzx': [math.pi / 2, 0, math.pi / 2],
		'zxy': [0, math.pi / 2, math.pi / 2],
		'zyx': [0, math.pi / 2, 0]}
	def __init__(self, xyz = [0, 0, 0], rpy = [0, 0, 0]):
		if type(rpy) == str:
			rpy = SingleDomino.ORIENTATION_ALIAS[rpy]
		Component.__init__(self, xyz, rpy)
		self.body_id = -1
	def Create(self, base_translation = [0, 0, 0], base_rotation = [0, 0, 0, 1]):
		if self.created:
			return
		(now_translation, now_rotation) = multiplyTransforms(
			base_translation, base_rotation, self.translation, self.rotation)
		self.body_id = createCollisionShape(
			GEOM_BOX,
			halfExtents = [S / 2 for S in DOMINO_SIZE])
		changeDynamics(self.body_id, -1, lateralFriction = 0.35, restitution = 0)
		createMultiBody(
			DOMINO_MASS,
			self.body_id,
			basePosition = now_translation,
			baseOrientation = now_rotation)
		self.created = True
	def Id(self):
		return self.body_id if self.created else -1
	def AllIds(self):
		return [self.body_id]

class PileDomino(Component):
	def __init__(self, N, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		for i in range(N):
			self.children[i] = SingleDomino([0, 0, SX * (i + 0.5)], 'zyx')
	def BottomId(self):
		return self.children[0].Id()
	def TopId(self):
		return self.children[len(self.children) - 1].Id()

class LineDomino(Component):
	# interval = SZ / INTERVAL_RATIO
	INTERVAL_RATIO = 1.5
	def __init__(self, start_xy, end_xy, contain = (True, True), interval = -1, base_z = 0, side = False):
		delta_xy = [end_xy[i] - start_xy[i] for i in range(2)]
		yaw = math.atan2(delta_xy[1], delta_xy[0])
		Component.__init__(self, [start_xy[0], start_xy[1], 0], [0, 0, yaw])
		if interval < 0:
			interval = SY / LineDomino.INTERVAL_RATIO if side else SZ / LineDomino.INTERVAL_RATIO
		distance = sum([delta_xy[i] ** 2 for i in range(2)]) ** 0.5
		N_interval = int(distance / interval + 0.5)
		interval = distance / N_interval
		N_domino = N_interval - 1 + sum(contain)
		z = SY / 2 if side else SZ / 2
		for i in range(N_domino):
			self.children[i] = SingleDomino([interval * (i + 1 - contain[0]), 0, base_z + z], 'xzy' if side else 'xyz')
	def StartId(self):
		return self.children[0].Id()
	def EndId(self):
		return self.children[len(self.children) - 1].Id()

class CurveDomino(Component):
	SAMPLE_COUNT = 100
	CONTROL_STRETCH = 1
	INTERVAL_RATIO = 2
	def __init__(self, start_xy, start_angle, end_xy, end_angle):
		Component.__init__(self, [0, 0, 0], [0, 0, 0])
		start_stretch = [
			start_xy[0] + CurveDomino.CONTROL_STRETCH * math.cos(start_angle),
			start_xy[1] + CurveDomino.CONTROL_STRETCH * math.sin(start_angle)]
		end_stretch = [
			end_xy[0] - CurveDomino.CONTROL_STRETCH * math.cos(end_angle),
			end_xy[1] - CurveDomino.CONTROL_STRETCH * math.sin(end_angle)]
		curve = self.GetBizierCurve([start_xy, start_stretch, end_stretch, end_xy])
		curve += [self.Interpolate(end_xy, end_stretch, -1)]
		angle = [self.Angle(curve[i], curve[i + 1]) for i in range(len(curve) - 1)] + [end_angle]
		length = [self.Distance(curve[i], curve[i + 1]) for i in range(len(curve) - 1)]
		total_length = sum(length)
		N = total_length / (SZ / CurveDomino.INTERVAL_RATIO)
		interval = total_length / N
		self.children[0] = SingleDomino([start_xy[0], start_xy[1], SZ / 2], [0, 0, start_angle])
		L = 0
		for i in range(len(curve) - 1):
			L += length[i]
			if L >= interval:
				L -= interval
				t = L / interval
				xy = self.Interpolate(curve[i], curve[i + 1], t)
				a = angle[i] * t + angle[i + 1] * (1 - t)
				self.children[len(self.children)] = SingleDomino([xy[0], xy[1], SZ / 2], [0, 0, a])
	def GetBizierCurve(self, points):
		curve = []
		for i in range(CurveDomino.SAMPLE_COUNT + 1):
			interpolated = points
			while len(interpolated) > 1:
				t = 1 - i / CurveDomino.SAMPLE_COUNT
				interpolated = [self.Interpolate(interpolated[j], interpolated[j + 1], t) for j in range(len(interpolated) - 1)]
			curve += interpolated
		return curve
	def Interpolate(self, point1, point2, t):
		return [point1[0] * t + point2[0] * (1 - t), point1[1] * t + point2[1] * (1 - t)]
	def Angle(self, point1, point2):
		return math.atan2(point2[1] - point1[1], point2[0] - point1[0])
	def Distance(self, point1, point2):
		return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

class LeanTrigger(Component):
	# Initially place it with angle = math.atan(SX / SZ) + LEAN_ANGLE.
	# (so that it's unstable and will begin to fall when simulation starts)
	LEAN_ANGLE = 1.0E-2
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		pitch = math.atan(SX / SZ)
		self.children['trigger'] = SingleDomino([0, 0, SZ / math.cos(pitch) / 2], [0, pitch + LeanTrigger.LEAN_ANGLE, 0])

class EdgeTrigger(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be unstable a.k.a less than 0.5)
	CONTACT_RATIO = 0.3
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children['base'] = SingleDomino([-SY / 2, 0, SX / 2], [0, math.pi / 2, math.pi / 2])
		self.children['trigger'] = SingleDomino([SX * (0.5 - EdgeTrigger.CONTACT_RATIO), 0, SX + SZ / 2], 'xyz')

class SideBranch(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be unstable a.k.a less than 0.5)
	CONTACT_RATIO = 0.4
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children['base'] = SingleDomino([0, 0, SX / 2], 'zxy')
		self.children['trigger'] = SingleDomino([0, SZ / 2 + SX * (0.5 - SideBranch.CONTACT_RATIO), SX + SZ / 2], 'yxz')
		self.children['connection'] = SingleDomino([0, 0, SX + SZ / 2], 'xyz')
		self.children['cover'] = SingleDomino([0, SX, SX * 1.5 + SZ], 'zxy')

class MultiBranch(Component):
	def __init__(self, N, xy = [0, 0], angle = 0, gap = SX):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		for i in range(N):
			x = SZ / LineDomino.INTERVAL_RATIO * i
			for j in range(0, i + 1):
				y = (i / 2 - j) * (gap + SY)
				self.children[str(i) + '_' + str(j)] = SingleDomino([x, y, SZ / 2], 'xyz')

class UTurn(Component):
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children['base1'] = SingleDomino([SX * 1.5, 0, SX / 2], 'zyx')
		self.children['base2'] = SingleDomino([SX * 1.5, 0, SY / 2 + SX], 'yzx')
		self.children['lever'] = SingleDomino([SX, 0, SY * 1.5 + SX], 'xzy')
		self.children['support'] = SingleDomino([SX * 1.5 + SY / 2, 0, SY + SZ / 2 + SX], 'yxz')
		self.children['portL'] = SingleDomino([0, SX + SY / 1.5, SZ / 2], 'xyz')
		self.children['portR'] = SingleDomino([0, -SX - SY / 1.5, SZ / 2], 'xyz')

def GenerateCrossingProfile(contact_ratio):
	alpha = fsolve(lambda x: (SZ - SX / math.sin(x)) * math.cos(x) + contact_ratio * SX - SY, math.asin(SX / SZ))
	return (contact_ratio, alpha)

class Crossing(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be stable and can be triggered easily a.k.a slightly larger than 0.5)
	(CONTACT_RATIO, ALPHA) = GenerateCrossingProfile(0.7)
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children['center'] = SingleDomino([0, 0, SX / 2], 'zyx')
		ramp_x = (SZ + SX * math.sin(Crossing.ALPHA)) / 2 + SZ / 2 * math.cos(Crossing.ALPHA)
		ramp_z = SX / 2 * math.cos(Crossing.ALPHA) + SZ / 2 * math.sin(Crossing.ALPHA)
		self.children['rampR'] = SingleDomino([ramp_x, 0, ramp_z], [0, math.pi / 2 - Crossing.ALPHA, 0])
		self.children['rampL'] = SingleDomino([-ramp_x, 0, ramp_z], [0, math.pi / 2 - Crossing.ALPHA, math.pi])
		base_x = (SY + SZ) / 2 + SX * math.sin(Crossing.ALPHA) + SX / math.tan(Crossing.ALPHA)
		self.children['baseR'] = SingleDomino([base_x, 0, SX / 2], 'zxy')
		self.children['baseL'] = SingleDomino([-base_x, 0, SX / 2], 'zxy')
		trigger_x = SX * (0.5 - Crossing.CONTACT_RATIO) + SZ / 2 + SY + SX * math.sin(Crossing.ALPHA) + SX / math.tan(Crossing.ALPHA)
		self.children['triggerR'] = SingleDomino([trigger_x, 0, SX + SZ / 2], 'xyz')
		self.children['triggerL'] = SingleDomino([-trigger_x, 0, SX + SZ / 2], 'xyz')
		self.children['bridge'] = SingleDomino([0, 0, SX + SZ / 2], 'yxz')

class ConditionGate(Component):
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children['base'] = PileDomino(5, [SY / 2 + SX - SZ / 2, 0], 0)
		self.children['connection'] = SingleDomino([-SY - SX / 2, 0, SX * 5.5], 'zxy')
		self.children['condition'] = SingleDomino([-SY * 1.5 - SX, 0, SZ / 2], 'xyz')
		self.children['portL'] = SingleDomino([0, SZ / 2 + SX, SZ / 2], 'yxz')
		self.children['portR'] = SingleDomino([0, -SZ / 2 - SX, SZ / 2], 'yxz')
		(x, y) = (-SY / 2 - SX, SY / 2 + SZ / 2)
		self.children['barrierL1'] = SingleDomino([SY / 2 + SX / 2, SY, SZ / 2], 'xyz')
		self.children['barrierL2'] = LineDomino([x, y], [x - SX * 2, y], interval = SX, side = True)
		self.children['barrierR1'] = SingleDomino([SY / 2 + SX / 2, -SY, SZ / 2], 'xyz')
		self.children['barrierR2'] = LineDomino([x, -y], [x - SX * 2, -y], interval = SX, side = True)

class OrGate(Component):
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children['output'] = SingleDomino([SX, 0, SZ / 2], 'xyz')
		self.children['inputL'] = SingleDomino([-SX / 2, (SX + SY) / 2, SZ / 2], 'xyz')
		self.children['inputR'] = SingleDomino([-SX / 2, -(SX + SY) / 2, SZ / 2], 'xyz')

class AndGate(Component):
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children['barrier'] = LineDomino([0, -SX], [0, SX], interval = SX)
		self.children['base'] = PileDomino(3, [(SX + SY + SZ) / 2, 0], 0)
		self.children['highline'] = LineDomino([SY, 0], [SY + SX * 4, 0], interval = SX * 2, base_z = SX * 3)

class FastPropagation(Component):
	# Initially only CONTACT_RATIO * SZ part will suypport the next one.
	CONTACT_RATIO = 1.0E-2
	def __init__(self, start_xy, end_xy):
		delta_xy = [end_xy[i] - start_xy[i] for i in range(2)]
		yaw = math.atan2(delta_xy[1], delta_xy[0])
		Component.__init__(self, [start_xy[0], start_xy[1], 0], [0, 0, yaw])
		self.N = 0
		distance = sum([delta_xy[i] ** 2 for i in range(2)]) ** 0.5
		self.children[self.N] = SingleDomino([0, 0, SY / 2], 'yzx')
		self.N += 1
		alpha = math.asin(SY / SZ * (1 - FastPropagation.CONTACT_RATIO))
		x = SZ / 2 + SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.cos(alpha) + SY / 2 * math.sin(alpha)
		z = SY - SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.sin(alpha) + SY / 2 * math.cos(alpha)
		while True:
			self.children[self.N] = SingleDomino([x, 0, z], [math.pi / 2 + alpha, 0, math.pi / 2])
			self.N += 1
			contact_x = x + SY / 2 * math.sin(alpha) + SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.cos(alpha)
			contact_z = z + SY / 2 * math.cos(alpha) - SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.sin(alpha)
			support_x = contact_x + (SZ ** 2 - contact_z ** 2) ** 0.5
			support_z = 0
			new_alpha = math.asin(contact_z / SZ)
			new_x = (contact_x + support_x) / 2 + SY / 2 * math.sin(new_alpha)
			new_z = (contact_z + support_z) / 2 + SY / 2 * math.cos(new_alpha)
			if abs(new_x - distance) > abs(x - distance):
				break
			(alpha, x, z) = (new_alpha, new_x, new_z)
		self.children['trigger'] = LeanTrigger([x, -SX / 2 - SZ * math.sin(math.atan(SX / SZ))], math.pi / 2)
	def StartId(self):
		return self.Id(0)
	def EndId(self):
		return self.Id(self.N - 1)
