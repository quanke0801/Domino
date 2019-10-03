# -*- coding: utf-8 -*-

import time

from Engine import *
from Plugins import *
from Components import *

def Condition():
	component = Component()
	component['trigger1'] = LeanTrigger(([-2, 0], 0))
	component['trigger2'] = LeanTrigger(([0, 1], -math.pi / 2))
	component['gate'] = ConditionGate()
	component['in1'] = CurveDomino(component('trigger1', 'out'), component('gate', 'inL'))
	component['in2'] = CurveDomino(component('trigger2', 'out'), component('gate', 'inU'))
	component['out'] = CurveDomino(component('gate', 'outR'), ((1, 0), 0))
	return component

def Curve():
	component = Component()
	component['trigger'] = LeanTrigger()
	component['curve'] = CurveDomino(component('trigger', 'out'), ([2, 0], -math.pi / 2))
	return component

def ConditionAndGateDemo():
	L = SZ * 5
	component = Component()
	component['triggerL'] = LeanTrigger(([-L, -L * 4], math.pi / 2))
	component['triggerR'] = LeanTrigger(([L, -L * 2], math.pi / 2))
	component['branchL'] = MultiBranch(2, ([-L, -L], math.pi / 2))
	component['branchR'] = MultiBranch(2, ([L, -L], math.pi / 2))
	component['conditionL'] = ConditionGate(([-L * 2, L], -math.pi / 2))
	component['conditionR'] = ConditionGate(([L * 2, L], math.pi / 2))
	component['crossing'] = Crossing(([0, 0], math.pi / 4))
	component['or'] = OrGate(([0, L * 2], math.pi / 2))
	component['target'] = TargetDomino(([0, L * 3], math.pi / 2))
	component.Connect('triggerL', 'out', 'branchL', 'in')
	component.Connect('triggerR', 'out', 'branchR', 'in')
	component.Connect('branchL', 'out0', 'conditionL', 'inR')
	component.Connect('branchL', 'out1', 'crossing', 'inL')
	component.Connect('branchR', 'out0', 'crossing', 'inD')
	component.Connect('branchR', 'out1', 'conditionR', 'inL')
	component.Connect('crossing', 'outU', 'conditionL', 'inU')
	component.Connect('crossing', 'outR', 'conditionR', 'inU')
	component.Connect('conditionL', 'outL', 'or', 'inU')
	component.Connect('conditionR', 'outR', 'or', 'inD')
	component.Connect('or', 'out', 'target', 'in')
	return component

def TapButtonDemo():
	component = Component()
	component['trigger'] = LeanTrigger(([-1, 0], 0))
	component['tap'] = TapButton(([0, 0], 0))
	component['target'] = TargetDomino(([1, 0], 0))
	component.Connect('trigger', 'out', 'tap', 'in')
	component.Connect('tap', 'out', 'target', 'in')
	return component

class NewAndGate(Component):
	# The distance from where the ramp touches ground to the support will be SZ / SEPARATE_RATIO
	SEPARATE_RATIO = 1.5
	def __init__(self, pose = ([0, 0], 0)):
		(xy, angle) = pose
		Component.__init__(self, ([xy[0], xy[1], 0], [0, 0, angle]))
		self.children['tap'] = TapButton(([SZ, 0], 0))
		separation = SZ / NewAndGate.SEPARATE_RATIO
		alpha = math.atan((SY - SX) / separation)
		x = -separation + SZ / 2 * math.cos(alpha) - SX / 2 * math.sin(alpha)
		z = SX + SZ / 2 * math.sin(alpha) + SX / 2 * math.cos(alpha)
		dynamics = {'friction': 10}
		self.children['rampD'] = SingleDomino(([x, -SY / 2, z], [0, math.pi / 2 - alpha, 0]), dynamics)
		self.children['rampU'] = SingleDomino(([x, SY / 2, z], [0, math.pi / 2 - alpha, 0]), dynamics)
		self.children['supportL1'] = SingleDomino(([-separation, 0, SX / 2], 'zxy'), dynamics)
		self.children['supportL2'] = SingleDomino(([-separation - SY, 0, SX / 2], 'zxy'), dynamics)
		self.children['supportL3'] = SingleDomino(([-separation - SX * math.sin(alpha) - SY / 2, 0, SX * 1.5], 'zxy'), dynamics)
		self.children['supportD'] = SingleDomino(([SX / 2, -SY / 2 - SZ / 2, SY / 2], 'xzy'))
		self.children['supportU'] = SingleDomino(([SX / 2, SY / 2 + SZ / 2, SY / 2], 'xzy'))
		self.children['protectD'] = SingleDomino(([-SZ / 2, -SY - SX / 2, SY / 2], 'yzx'))
		self.children['protectU'] = SingleDomino(([-SZ / 2, SY + SX / 2, SY / 2], 'yzx'))
		self.port['inD'] = ([-SX, -SZ], 0)
		self.port['inU'] = ([-SX, SZ], 0)
		self.port['out'] = self.children['tap'].Promote('out')

def NewAndGateDemo():
	component = Component()
	component['tD'] = LeanTrigger(([-1, -0.5], 0))
	component['tU'] = LeanTrigger(([-2, 0.5], 0))
	component['and'] = NewAndGate(([0, 0], 0))
	component['t'] = TargetDomino(([1, 0], 0))
	component.Connect('tD', 'out', 'and', 'inD')
	component.Connect('tU', 'out', 'and', 'inU')
	component.Connect('and', 'out', 't', 'in')
	return component

def Demo(component):
	engine = Engine(0.001)
	engine.AddPlugin(PrintPlugin(engine))
	#engine.AddPlugin(AutoTerminatePlugin(engine))
	engine.AddComponent(component)
	time.sleep(1.0)
	engine.Start()

if __name__ == '__main__':
	Demo(Condition())
