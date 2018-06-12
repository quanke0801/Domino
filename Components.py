# -*- coding: utf-8 -*-

import math

from scipy.optimize import fsolve
from pybullet import *

DOMINO_SIZE = (SX, SY, SZ) = (0.015, 0.05, 0.1)
DOMINO_MASS = SX * SY * SZ * 1000

class Component:
	def __init__(self, xyz = [0, 0, 0], rpy = [0, 0, 0]):
		(self.translation, self.rotation) = (xyz, getQuaternionFromEuler(rpy))
		self.children = []
		self.created = False
	def Create(self, base_translation = [0, 0, 0], base_rotation = [0, 0, 0, 1]):
		if self.created:
			return
		(now_translation, now_rotation) = multiplyTransforms(
			base_translation, base_rotation, self.translation, self.rotation)
		for child in self.children:
			child.Create(now_translation, now_rotation)
		self.created = True

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
	def Create(self, base_translation = [0, 0, 0], base_rotation = [0, 0, 0, 1]):
		if self.created:
			return
		(now_translation, now_rotation) = multiplyTransforms(
			base_translation, base_rotation, self.translation, self.rotation)
		model_id = createCollisionShape(
			GEOM_BOX,
			halfExtents = [S / 2 for S in DOMINO_SIZE])
		changeDynamics(model_id, -1, lateralFriction = 0.35, restitution = 0)
		createMultiBody(
			DOMINO_MASS,
			model_id,
			basePosition = now_translation,
			baseOrientation = now_rotation)
		self.created = True

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
			self.children += [SingleDomino([interval * (i + 1 - contain[0]), 0, base_z + z], 'xzy' if side else 'xyz')]

class LeanTrigger(Component):
	# Initially place it with angle = math.atan(SX / SZ) + LEAN_ANGLE.
	# (so that it's unstable and will begin to fall when simulation starts)
	LEAN_ANGLE = 1.0E-2
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		pitch = math.atan(SX / SZ)
		self.children = [SingleDomino([0, 0, SZ / math.cos(pitch) / 2], [0, pitch + LeanTrigger.LEAN_ANGLE, 0])]

class EdgeTrigger(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be unstable a.k.a less than 0.5)
	CONTACT_RATIO = 0.3
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children = [
			SingleDomino([-SY / 2, 0, SX / 2], [0, math.pi / 2, math.pi / 2]), # base
			SingleDomino([SX * (0.5 - EdgeTrigger.CONTACT_RATIO), 0, SX + SZ / 2], 'xyz')] # trigger

class SideBranch(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be unstable a.k.a less than 0.5)
	CONTACT_RATIO = 0.4
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children = [
			SingleDomino([0, 0, SX / 2], 'zxy'), # base
			SingleDomino([0, SZ / 2 + SX * (0.5 - SideBranch.CONTACT_RATIO), SX + SZ / 2], 'yxz'), # trigger
			SingleDomino([0, 0, SX + SZ / 2], 'xyz'), # connection
			SingleDomino([0, SX, SX * 1.5 + SZ], 'zxy')] # cover

class UTurn(Component):
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children = [
			SingleDomino([SX * 1.5, 0, SX / 2], 'zyx'), # bottom base
			SingleDomino([SX * 1.5, 0, SY / 2 + SX], 'yzx'), # base
			SingleDomino([SX, 0, SY * 1.5 + SX], 'xzy'), # lever
			SingleDomino([SX * 1.5 + SY / 2, 0, SY + SZ / 2 + SX], 'yxz'), # support
			SingleDomino([0, SX + SY / 1.5, SZ / 2], 'xyz'), # built-in port 1
			SingleDomino([0, -SX - SY / 1.5, SZ / 2], 'xyz')] # built-in port 2

def GenerateCrossingProfile(contact_ratio):
	alpha = fsolve(lambda x: (SZ - SX / math.sin(x)) * math.cos(x) + contact_ratio * SX - SY, math.asin(SX / SZ))
	return (contact_ratio, alpha)

class Crossing(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be stable and can be triggered easily a.k.a slightly larger than 0.5)
	(CONTACT_RATIO, ALPHA) = GenerateCrossingProfile(0.7)
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		center = ([0, 0, SX / 2], 'zyx')
		ramp_x = (SZ + SX * math.sin(Crossing.ALPHA)) / 2 + SZ / 2 * math.cos(Crossing.ALPHA)
		ramp_z = SX / 2 * math.cos(Crossing.ALPHA) + SZ / 2 * math.sin(Crossing.ALPHA)
		ramp1 = ([ramp_x, 0, ramp_z], [0, math.pi / 2 - Crossing.ALPHA, 0])
		ramp2 = ([-ramp1[0][0], ramp1[0][1], ramp1[0][2]], [ramp1[1][0], ramp1[1][1], ramp1[1][2] + math.pi])
		base1 = ([(SY + SZ) / 2 + SX * math.sin(Crossing.ALPHA) + SX / math.tan(Crossing.ALPHA), 0, SX / 2], 'zxy')
		base2 = ([-base1[0][0], base1[0][1], base1[0][2]], 'zxy')
		trigger1 = ([SX * (0.5 - Crossing.CONTACT_RATIO) + SZ / 2 + SY + SX * math.sin(Crossing.ALPHA) + SX / math.tan(Crossing.ALPHA), 0, SX + SZ / 2], 'xyz')
		trigger2 = ([-trigger1[0][0], trigger1[0][1], trigger1[0][2]], 'xyz')
		bridge = ([0, 0, SX + SZ / 2], 'yxz')
		for (xyz, rpy) in [center, ramp1, ramp2, base1, base2, trigger1, trigger2, bridge]:
			self.children += [SingleDomino(xyz, rpy)]

class OrGate(Component):
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children = [
			SingleDomino([SX, 0, SZ / 2], 'xyz'), # output
			SingleDomino([-SX / 2, (SX + SY) / 2, SZ / 2], 'xyz'), # input 1
			SingleDomino([-SX / 2, -(SX + SY) / 2, SZ / 2], 'xyz')] # input 2

class AndGate(Component):
	def __init__(self, xy = [0, 0], angle = 0):
		Component.__init__(self, [xy[0], xy[1], 0], [0, 0, angle])
		self.children += [LineDomino([0, -SX], [0, SX], interval = SX)]
		for i in range(3): # base
			self.children += [SingleDomino([(SX + SY + SZ) / 2, 0, (i + 0.5) * SX], 'zyx')]
		self.children += [LineDomino([SY, 0], [SY + SX * 4, 0], interval = SX * 2, base_z = SX * 3)]

class FastPropagation(Component):
	# Initially only CONTACT_RATIO * SZ part will suypport the next one.
	CONTACT_RATIO = 1.0E-2
	def __init__(self, start_xy, end_xy):
		delta_xy = [end_xy[i] - start_xy[i] for i in range(2)]
		yaw = math.atan2(delta_xy[1], delta_xy[0])
		Component.__init__(self, [start_xy[0], start_xy[1], 0], [0, 0, yaw])
		distance = sum([delta_xy[i] ** 2 for i in range(2)]) ** 0.5
		self.children += [SingleDomino([0, 0, SY / 2], 'yzx')]
		alpha = math.asin(SY / SZ * (1 - FastPropagation.CONTACT_RATIO))
		x = SZ / 2 + SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.cos(alpha) + SY / 2 * math.sin(alpha)
		z = SY - SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.sin(alpha) + SY / 2 * math.cos(alpha)
		while True:
			self.children += [SingleDomino([x, 0, z], [math.pi / 2 + alpha, 0, math.pi / 2])]
			contact_x = x + SY / 2 * math.sin(alpha) + SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.cos(alpha)
			contact_z = z + SY / 2 * math.cos(alpha) - SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.sin(alpha)
			support_x = contact_x + (SZ ** 2 - contact_z ** 2) ** 0.5
			support_z = 0
			new_alpha = math.asin(contact_z / SZ)
			new_x = (contact_x + support_x) / 2 + SY / 2 * math.sin(new_alpha)
			new_z = (contact_z + support_z) / 2 + SY / 2 * math.cos(new_alpha)
			print(x, new_x, distance)
			if abs(new_x - distance) > abs(x - distance):
				break
			(alpha, x, z) = (new_alpha, new_x, new_z)
		self.children += [LeanTrigger([x, -SX / 2 - SZ * math.sin(math.atan(SX / SZ))], math.pi / 2)]
