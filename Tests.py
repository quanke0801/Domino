# -*- coding: utf-8 -*-

from Engine import *
from Plugins import *
from Components import *

def GetLineDominoTest():
	test = Component()
	test.children = {
		'trigger1': LeanTrigger([0, 0], 0),
		'line1': LineDomino([0, 0], [1, 0], (False, True), side = True),
		'trigger2': LeanTrigger([0, 1], 0),
		'line2': LineDomino([0, 1], [1, 1], (False, True))}
	return test

def GetUTurnTest():
	test = Component()
	test.children['trigger'] = EdgeTrigger([0, 0], 0)
	for i in range(5):
		y = SZ * i
		test.children['line' + str(i)] = LineDomino([0, y], [SZ * 10, y], (False, False))
		test.children['uturn' + str(i)] = UTurn([SZ * 10 if i % 2 == 0 else 0, y + SZ / 2], 0 if i % 2 == 0 else math.pi)
	return test

def GetOrGateTest():
	test = Component()
	test.children = {
		'trigger1': EdgeTrigger([0, 0], 0),
		'in1': LineDomino([0, 0], [SZ * 10, 0], (False, False)),
		'trigger2': EdgeTrigger([-SZ * 10, SZ], 0),
		'in2': LineDomino([-SZ * 10, SZ], [SZ * 10, SZ], (False, False)),
		'or': OrGate([SZ * 10, SZ / 2], 0),
		'out': LineDomino([SZ * 10, SZ / 2], [SZ * 20, SZ / 2], (False, False))}
	return test

def GetAndGateTest():
	test = Component()
	(d1, d2) = (SZ * 20, SZ * 10)
	test.children = {
		'and': AndGate([0, 0], 0),
		'in1': LineDomino([0, 0], [0, d1], (False, False)),
		'trigger1': EdgeTrigger([0, d1], -math.pi / 2),
		'in2': LineDomino([SZ, 0], [d2, 0], (False, False)),
		'trigger2': EdgeTrigger([d2, 0], -math.pi),
		'out': LineDomino([-SZ / 2, 0], [-SZ * 10, 0], (True, True))}
	return test

def GetCrossingTest():
	test = Component()
	(d1, d2) = (SZ * 20, SZ * 10)
	test.children = {
		'crossing': Crossing([0, 0], 0),
		'in1': LineDomino([-d1, 0], [-SZ * 1.5, 0], (False, False)),
		'trigger1': EdgeTrigger([-d1, 0], 0),
		'out1': LineDomino([SZ * 1.5, 0], [d1, 0], (False, True)),
		'in2': LineDomino([0, d2], [0, 0], (False, False)),
		'trigger2': EdgeTrigger([0, d2], -math.pi / 2),
		'out2': LineDomino([0, 0], [0, -d2], (False, True))}
	return test

def GetSquareTest():
	test = Component()
	test.children['trigger'] = EdgeTrigger([-SZ * 10, 0], 0)
	test.children['line'] = LineDomino([-SZ * 10, 0], [0, 0], (False, False))
	for i in range(10):
		x = i * SZ / LineDomino.INTERVAL_RATIO * 2
		test.children['side' + str(i)] = SideBranch([x, 0], 0)
		test.children['line' +str(i)] = LineDomino([x, SZ], [x, SZ * 10], (True, True))
		test.children['gap' + str(i)] = SingleDomino([x + SZ / LineDomino.INTERVAL_RATIO, 0, SZ / 2], 'xyz')
	return test

def GetFastPropagationTest():
	test = Component()
	test.children = {
		'trigger': EdgeTrigger([0, -SZ * 10], math.pi / 2),
		'line1': LineDomino([0, -SZ * 10], [0, 0], (False, False)),
		'fast': FastPropagation([0, 0], [SZ * 30, 0]),
		'line2': LineDomino([SZ * 30, -SX], [SZ * 30, 1], (False, True))}
	return test

def Test(test):
	engine = Engine(0.001)
	engine.AddPlugin(TerminatorPlugin(engine))
	engine.AddPlugin(PrinterPlugin(engine))
	engine.AddComponent(test)
	engine.Start()

if __name__ == '__main__':
	Test(GetFastPropagationTest())
