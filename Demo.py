# -*- coding: utf-8 -*-

import time

from Engine import *
from Plugins import *
from Components import *

def Condition():
	component = Component()
	component['trigger1'] = LeanTrigger(([-1, 0], 0))
	component['trigger2'] = LeanTrigger(([0.5, 2], -math.pi / 2))
	component['gate'] = ConditionGate()
	#component['in1'] = CurveDomino(component('trigger1', 'out'), component('gate', 'inL'), 0.1)
	component['in1'] = component.Connect('trigger1', 'out', 'gate', 'inL', 0.1)
	component['in2'] = CurveDomino(component('trigger2', 'out'), component('gate', 'inU'), 0.5)
	component['out'] = CurveDomino(component('gate', 'outR'), ((1, 0), 0), 0.1)
	return component

def AndGateLarge():
	pass

def Demo(component):
	engine = Engine(0.001)
	engine.AddComponent(component)
	time.sleep(2.0)
	engine.Start()

if __name__ == '__main__':
	Demo(Condition())
