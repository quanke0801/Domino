# -*- coding: utf-8 -*-

import time

from Engine import *
from Plugins import *
from Components import *

def ConditionGate():
	component = Component([0.5, 0, 0])
	component.children['trigger1'] = LeanTrigger([-2, 0], 0)
	component.children['condition'] = LineDomino([-2, 0], [-SY * 1.5 - SX / 2, 0], (False, False))
	component.children['cg'] = ConditionGate();
	component.children['trigger2'] = LeanTrigger([0, 1], -math.pi / 2)
	component.children['line1'] = LineDomino([0, -2], [0, -SZ], (False, True))
	component.children['line2'] = LineDomino([0, SZ], [0, 1], (True, False))
	return component

def Curve():
	component = Component()
	component.children['trigger'] = LeanTrigger([0, 0], 0)
	component.children['curve'] = CurveDomino([SY, 0], 0, [SZ * 10, SZ * 10], 0)
	return component

def Demo(component):
	engine = Engine(0.001)
	engine.AddComponent(component)
	time.sleep(2)
	engine.Start()

if __name__ == '__main__':
	Demo(Curve())
