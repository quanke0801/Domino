# -*- coding: utf-8 -*-

import math

from scipy.optimize import fsolve
from pybullet import *

DOMINO_SIZE = (SX, SY, SZ) = (0.015, 0.05, 0.1)
DOMINO_MASS = SX * SY * SZ * 1000

class Component:
	def __init__(self, pose = ([0, 0, 0], [0, 0, 0])):
		(self.xyz, self.rpy) = pose
		self.pose = (self.xyz, getQuaternionFromEuler(self.rpy))
		self.children = {}
		self.port = {}
		self.created = False
	def Create(self, base_pose = ([0, 0, 0], [0, 0, 0, 1])):
		if self.created:
			return
		now_pose = multiplyTransforms(
			base_pose[0], base_pose[1], self.pose[0], self.pose[1])
		for child in self.children.values():
			child.Create(now_pose)
		self.created = True
	def __getitem__(self, name):
		if name in self.children:
			return self.children[name]
		else:
			return self.port[name]
	def __setitem__(self, name, value):
		self.children[name] = value
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
	def Promote(self, name):
		((x, y), angle) = self.port[name]
		distance = (x ** 2 + y ** 2) ** 0.5
		direction = math.atan2(y, x)
		yaw = self.rpy[2]
		(base_x, base_y) = (self.xyz[0], self.xyz[1])
		(parent_x, parent_y) = (
			base_x + distance * math.cos(direction + yaw),
			base_y + distance * math.sin(direction + yaw))
		parent_angle = angle + yaw
		return ((parent_x, parent_y), parent_angle)
	def __call__(self, child_name, port_name):
		return self.children[child_name].Promote(port_name)
	def Connect(self, child1, port1, child2, port2, name = None):
		if name is None:
			name = '%s_%s_to_%s_%s' % (child1, port1, child2, port2)
		self.children[name] = CurveDomino(self.children[child1].Promote(port1), self.children[child2].Promote(port2))

class SingleDomino(Component):
	INTERVAL_RATIO = 1.8
	ORIENTATION_ALIAS = {
		'xyz': [0, 0, 0],
		'xzy': [math.pi / 2, 0, 0],
		'yxz': [0, 0, math.pi / 2],
		'yzx': [math.pi / 2, 0, math.pi / 2],
		'zxy': [0, math.pi / 2, math.pi / 2],
		'zyx': [0, math.pi / 2, 0]}
	def __init__(self, pose = ([0, 0, 0], [0, 0, 0]), dynamics = {}):
		(xyz, rpy) = pose
		if type(rpy) == str:
			rpy = SingleDomino.ORIENTATION_ALIAS[rpy]
		Component.__init__(self, (xyz, rpy))
		self.dynamics = dynamics
		self.body_id = -1
	def Create(self, base_pose = ([0, 0, 0], [0, 0, 0, 1])):
		if self.created:
			return
		now_pose = multiplyTransforms(
			base_pose[0], base_pose[1], self.pose[0], self.pose[1])
		self.body_id = createCollisionShape(
			GEOM_BOX,
			halfExtents = [S / 2 for S in DOMINO_SIZE])
		createMultiBody(
			DOMINO_MASS,
			self.body_id,
			basePosition = now_pose[0],
			baseOrientation = now_pose[1])
		changeDynamics(
			self.body_id,
			-1,
			lateralFriction = self.dynamics.get('friction', 0.5),
			restitution = self.dynamics.get('restitution', 0))
		self.created = True
	def Id(self):
		return self.body_id if self.created else -1
	def AllIds(self):
		return [self.body_id]

class TargetDomino(Component):
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['target'] = SingleDomino(([0, 0, SZ / 2], [0, 0, 0]))
		self.port['in'] = ([-SY, 0], 0)

class PileDomino(Component):
	def __init__(self, N, pose):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		for i in range(N):
			self.children[i] = SingleDomino(([0, 0, SX * (i + 0.5)], 'zyx'))
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
		Component.__init__(self, ([start_xy[0], start_xy[1], 0], [0, 0, yaw]))
		height = SY if side else SZ
		orientation = 'xzy' if side else 'xyz'
		if interval < 0:
			interval = height / LineDomino.INTERVAL_RATIO
		distance = sum([delta_xy[i] ** 2 for i in range(2)]) ** 0.5
		N = int(distance / interval + 0.5) # number of intervals (rounded)
		interval = distance / N # update true interval
		N = N - 1 + sum(contain) # number of dominoes
		z = SY / 2 if side else SZ / 2
		x = [interval * (i + 1 - contain[0]) for i in range(N)]
		for i in range(N):
			self.children[i] = SingleDomino(([x[i], 0, base_z + height / 2], orientation))
		self.port['in'] = ([x[0] - SY, 0], 0)
		self.port['out'] = ([x[-1] + SY, 0], 0)
	def StartId(self):
		return self.children[0].Id()
	def EndId(self):
		return self.children[len(self.children) - 1].Id()

class CurveDomino(Component):
	SAMPLE_COUNT = 100
	INTERVAL_RATIO = 1.8
	def __init__(self, start_pose, end_pose, interval = -1):
		Component.__init__(self, ([0, 0, 0], [0, 0, 0]))
		(radius, curve) = self.SearchMinCurvature(start_pose, end_pose)
		curve += [self.Interpolate(curve[-2], curve[-1], -1)]
		angle = [self.Angle(curve[i], curve[i + 1]) for i in range(len(curve) - 1)] + [end_pose[1]]
		length = [self.Distance(curve[i], curve[i + 1]) for i in range(len(curve) - 1)] + [radius]
		total_length = sum(length[: -2])
		if interval < 0:
			interval = SZ / CurveDomino.INTERVAL_RATIO
		N = int(total_length / interval + 0.5) + 1
		interval = total_length / (N - 1)
		(L, ci, last) = (interval, -1, curve[0])
		for i in range(N):
			while L < interval:
				ci += 1
				L += length[ci]
			L -= interval
			t = L / length[ci]
			xy = self.Interpolate(curve[ci], curve[ci + 1], t)
			a = angle[ci] * t + angle[ci + 1] * (1 - t)
			self.children[i] = SingleDomino(([xy[0], xy[1], SZ / 2], [0, 0, a]))
	def SearchMinCurvature(self, start_pose, end_pose, lower = 0, upper = 1):
		DIVIDE = 5
		search_radius = [lower + (upper - lower) / (DIVIDE - 1) * i for i in range(DIVIDE)]
		curves = []
		curvatures = []
		for (i, radius) in enumerate(search_radius):
			(start_xy, start_angle) = start_pose
			(end_xy, end_angle) = end_pose
			start_extend = [
				start_xy[0] + radius * math.cos(start_angle),
				start_xy[1] + radius * math.sin(start_angle)]
			end_extend = [
				end_xy[0] - radius * math.cos(end_angle),
				end_xy[1] - radius * math.sin(end_angle)]
			curve = self.GetBizierCurve([start_xy, start_extend, end_extend, end_xy])
			curves += [curve]
			curve = [self.Interpolate(start_xy, start_extend, 1 + 1.0E-2)] + curve + [self.Interpolate(end_xy, end_extend, 1 + 1.0E-2)]
			max_curvature = max([self.Curvature([curve[i], curve[i + 1], curve[i + 2]]) for i in range(len(curve) - 2)])
			curvatures += [max_curvature]
		index = curvatures.index(min(curvatures))
		if index != 0:
			lower = search_radius[index - 1]
		if index != DIVIDE - 1:
			upper = search_radius[index + 1]
		if upper - lower < 0.05:
			return ((lower + upper) / 2, curves[index])
		return self.SearchMinCurvature(start_pose, end_pose, lower, upper)
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
	def Curvature(self, points):
		lengths = [self.Distance(points[i], points[(i + 1) % 3]) for i in range(3)]
		if lengths[0] * lengths[1] * lengths[2] == 0:
			return 1.0E100
		s = sum(lengths) / 2
		area = max(0, s * (s - lengths[0]) * (s - lengths[1]) * (s - lengths[2])) ** 0.5
		return area * 4 / lengths[0] / lengths[1] / lengths[2]
	def Angle(self, point1, point2):
		return math.atan2(point2[1] - point1[1], point2[0] - point1[0])
	def Distance(self, point1, point2):
		return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
	def StartId(self):
		return self.children[0].Id()
	def EndId(self):
		return self.children[len(self.children) - 1].Id()

class LeanTrigger(Component):
	# Initially place it with angle = math.atan(SX / SZ) + LEAN_ANGLE.
	# (so that it's unstable and will begin to fall when simulation starts)
	LEAN_ANGLE = 1.0E-2
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		pitch = math.atan(SX / SZ)
		self.children['trigger'] = SingleDomino(([0, 0, SZ / math.cos(pitch) / 2], [0, pitch + LeanTrigger.LEAN_ANGLE, 0]))
		self.port['out'] = ([SY, 0], 0)

class EdgeTrigger(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be unstable a.k.a less than 0.5)
	CONTACT_RATIO = 0.3
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['base'] = SingleDomino(([-SY / 2, 0, SX / 2], [0, math.pi / 2, math.pi / 2]))
		self.children['trigger'] = SingleDomino(([SX * (0.5 - EdgeTrigger.CONTACT_RATIO), 0, SX + SZ / 2], 'xyz'))
		self.port['out'] = ([SY, 0], 0)

class TapButton(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be stable a.k.a larger than 0.5)
	CONTACT_RATIO = 0.9
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['base'] = SingleDomino(([0, 0, SX / 2], 'zyx'))
		self.children['lever'] = SingleDomino(([-SZ / 2, 0, SX * 1.5], 'zyx'))
		self.children['support'] = SingleDomino(([SY / 2, 0, SX * 1.5], 'zxy'))
		self.children['cover'] = SingleDomino(([0, 0, SX * 2.5], 'zyx'))
		self.children['trigger'] = SingleDomino(([SY + SX * (0.5 - TapButton.CONTACT_RATIO), 0, SZ / 2 + SX * 3], 'xyz'))
		self.port['in'] = ([-SY - SZ, 0], 0)
		self.port['out'] = ([SY * 2, 0], 0)

class SideBranch(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be unstable a.k.a less than 0.5)
	CONTACT_RATIO = 0.4
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['base'] = SingleDomino(([0, 0, SX / 2], 'zxy'))
		self.children['trigger'] = SingleDomino(([0, SZ / 2 + SX * (0.5 - SideBranch.CONTACT_RATIO), SX + SZ / 2], 'yxz'))
		self.children['connection'] = SingleDomino(([0, 0, SX + SZ / 2], 'xyz'))
		self.children['cover'] = SingleDomino(([0, SX, SX * 1.5 + SZ], 'zxy'))
		self.port['inL'] = ([-SY, 0], 0)
		self.port['outL'] = ([-SY, 0], math.pi)
		self.port['inR'] = ([SY, 0], math.pi)
		self.port['outR'] = ([SY, 0], 0)
		self.port['branch'] = ([0, SZ / 2 + SY], math.pi / 2)

class MultiBranch(Component):
	def __init__(self, N, pose = ([0, 0], 0), gap = SX):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		for i in range(N):
			x = SZ / LineDomino.INTERVAL_RATIO * i
			for j in range(0, i + 1):
				y = (i / 2 - j) * (gap + SY)
				self.children[str(i) + '_' + str(j)] = SingleDomino(([x, y, SZ / 2], 'xyz'))
				if i == N - 1:
					self.port['out' + str(j)] = ([x + SY, y], 0)
		self.port['in'] = ([-SY, 0], 0)

class UTurn(Component):
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['base1'] = SingleDomino(([SX * 1.5, 0, SX / 2], 'zyx'))
		self.children['base2'] = SingleDomino(([SX * 1.5, 0, SY / 2 + SX], 'yzx'))
		self.children['lever'] = SingleDomino(([SX, 0, SY * 1.5 + SX], 'xzy'))
		self.children['support'] = SingleDomino(([SX * 1.5 + SY / 2, 0, SY + SZ / 2 + SX], 'yxz'))
		self.children['triggerL'] = SingleDomino(([0, SX + SY / 1.5, SZ / 2], 'xyz'))
		self.children['triggerR'] = SingleDomino(([0, -SX - SY / 1.5, SZ / 2], 'xyz'))
		self.port['inD'] = ([0, -SX - SY], 0)
		self.port['outD'] = ([0, -SX - SY], math.pi)
		self.port['inU'] = ([0, SX + SY], 0)
		self.port['outU'] = ([0, SX + SY], math.pi)

def GenerateCrossingProfile(contact_ratio):
	alpha = fsolve(lambda x: (SZ - SX / math.sin(x)) * math.cos(x) + contact_ratio * SX - SY, math.asin(SX / SZ))
	return (contact_ratio, alpha)

class Crossing(Component):
	# Initially only CONTACT_RATIO * SX part will stand on the base.
	# (should be stable and can be triggered easily a.k.a slightly larger than 0.5)
	(CONTACT_RATIO, ALPHA) = GenerateCrossingProfile(0.7)
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['center'] = SingleDomino(([0, 0, SX / 2], 'zyx'))
		ramp_x = (SZ + SX * math.sin(Crossing.ALPHA)) / 2 + SZ / 2 * math.cos(Crossing.ALPHA)
		ramp_z = SX / 2 * math.cos(Crossing.ALPHA) + SZ / 2 * math.sin(Crossing.ALPHA)
		self.children['rampR'] = SingleDomino(([ramp_x, 0, ramp_z], [0, math.pi / 2 - Crossing.ALPHA, 0]))
		self.children['rampL'] = SingleDomino(([-ramp_x, 0, ramp_z], [0, math.pi / 2 - Crossing.ALPHA, math.pi]))
		base_x = (SY + SZ) / 2 + SX * math.sin(Crossing.ALPHA) + SX / math.tan(Crossing.ALPHA)
		self.children['baseR'] = SingleDomino(([base_x, 0, SX / 2], 'zxy'))
		self.children['baseL'] = SingleDomino(([-base_x, 0, SX / 2], 'zxy'))
		trigger_x = SX * (0.5 - Crossing.CONTACT_RATIO) + SZ / 2 + SY + SX * math.sin(Crossing.ALPHA) + SX / math.tan(Crossing.ALPHA)
		self.children['triggerR'] = SingleDomino(([trigger_x, 0, SX + SZ / 2], 'xyz'))
		self.children['triggerL'] = SingleDomino(([-trigger_x, 0, SX + SZ / 2], 'xyz'))
		self.children['bridge'] = SingleDomino(([0, 0, SX + SZ / 2], 'yxz'))
		self.port['inL'] = ([-trigger_x - SY, 0], 0)
		self.port['outL'] = ([-trigger_x - SY, 0], math.pi)
		self.port['inR'] = ([trigger_x + SY, 0], math.pi)
		self.port['outR'] = ([trigger_x + SY, 0], 0)
		self.port['inD'] = ([0, -SY], math.pi / 2)
		self.port['outD'] = ([0, -SY], -math.pi / 2)
		self.port['inU'] = ([0, SY], -math.pi / 2)
		self.port['outU'] = ([0, SY], math.pi / 2)

class ConditionGate(Component):
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['base'] = PileDomino(5, ([0, -SY / 2 - SX + SZ / 2], math.pi / 2))
		self.children['connection'] = SingleDomino(([0, SY + SX / 2, SX * 5.5], 'zyx'))
		self.children['condition'] = SingleDomino(([0, SY * 1.5 + SX, SZ / 2], 'yxz'))
		self.children['triggerL'] = SingleDomino(([-SZ / 2 - SX, 0, SZ / 2], 'xyz'))
		self.children['triggerR'] = SingleDomino(([SZ / 2 + SX, 0, SZ / 2], 'xyz'))
		self.children['barrierLD'] = SingleDomino(([-SY, -SY / 2 - SX / 2, SZ / 2], 'yxz'))
		self.children['barrierRD'] = SingleDomino(([SY, -SY / 2 - SX / 2, SZ / 2], 'yxz'))
		self.children['barrierLU'] = SingleDomino(([-SY / 2 - SZ / 2, SZ - SY / 2 - SX * 1.5, SY / 2], 'yzx'))
		self.children['barrierRU'] = SingleDomino(([SY / 2 + SZ / 2, SZ - SY / 2 - SX * 1.5, SY / 2], 'yzx'))
		self.port['inL'] = ([-SZ - SX, 0], 0)
		self.port['outL'] = ((-SZ - SX, 0), math.pi)
		self.port['inR'] = ([SZ + SX, 0], math.pi)
		self.port['outR'] = ((SZ + SX, 0), 0)
		self.port['inU'] = ([0, SY * 2 + SX], -math.pi / 2)

class OrGate(Component):
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['output'] = SingleDomino(([SX, 0, SZ / 2], 'xyz'))
		self.children['inputD'] = SingleDomino(([-SX / 2, -(SX + SY) / 2, SZ / 2], 'xyz'))
		self.children['inputU'] = SingleDomino(([-SX / 2, (SX + SY) / 2, SZ / 2], 'xyz'))
		self.port['inD'] = ([-SY, -(SX + SY) / 2], 0)
		self.port['inU'] = ([-SY, (SX + SY) / 2], 0)
		self.port['out'] = ([SX + SY, 0], 0)

class AndGate(Component):
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['barrier'] = LineDomino([0, -SX], [0, SX], interval = SX)
		self.children['base'] = PileDomino(3, ([(SX + SY + SZ) / 2, 0], 0))
		self.children['highline'] = LineDomino([SY, 0], [SY + SX * 4, 0], interval = SX * 2, base_z = SX * 3)
		self.port['inR'] = ([SY * 2 + SX * 4, 0], math.pi)
		self.port['inU'] = ([0, SY], -math.pi / 2)
		self.port['out'] = ([-SY, 0], math.pi)

class FastPropagation(Component):
	# Initially only CONTACT_RATIO * SZ part will suypport the next one.
	CONTACT_RATIO = 1.0E-2
	def __init__(self, start_xy, end_xy):
		delta_xy = [end_xy[i] - start_xy[i] for i in range(2)]
		yaw = math.atan2(delta_xy[1], delta_xy[0])
		Component.__init__(self, ([start_xy[0], start_xy[1], 0], [0, 0, yaw]))
		self.N = 0
		distance = sum([delta_xy[i] ** 2 for i in range(2)]) ** 0.5
		self.children[self.N] = SingleDomino(([0, 0, SY / 2], 'yzx'))
		self.N += 1
		alpha = math.asin(SY / SZ * (1 - FastPropagation.CONTACT_RATIO))
		x = SZ / 2 + SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.cos(alpha) + SY / 2 * math.sin(alpha)
		z = SY - SZ * (0.5 - FastPropagation.CONTACT_RATIO) * math.sin(alpha) + SY / 2 * math.cos(alpha)
		while True:
			self.children[self.N] = SingleDomino(([x, 0, z], [math.pi / 2 + alpha, 0, math.pi / 2]))
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
		self.children['trigger'] = LeanTrigger(([x, -SX / 2 - SZ * math.sin(math.atan(SX / SZ))], math.pi / 2))
		self.port['inD'] = ([0, -SY], math.pi / 2)
		self.port['inU'] = ([0, SY], -math.pi / 2)
		self.port['out'] = ([x, SY], math.pi / 2)
	def StartId(self):
		return self.Id(0)
	def EndId(self):
		return self.Id(self.N - 1)
