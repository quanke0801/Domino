# -*- coding: utf-8 -*-

from Engine import *
from Components import *

def GetLineDominoTest():
	return [
		LeanTrigger([0, 0], 0),
		LineDomino([0, 0], [1, 0], (False, True), side = True),
		LeanTrigger([0, 1], 0),
		LineDomino([0, 1], [1, 1], (False, True))]

def GetUTurnTest():
	components = [EdgeTrigger([0, 0], 0)]
	for i in range(5):
		y = SZ * i
		components += [LineDomino([0, y], [SZ * 10, y], (False, False))]
		components += [UTurn([SZ * 10 if i % 2 == 0 else 0, y + SZ / 2], 0 if i % 2 == 0 else math.pi)]
	return components

def GetOrGateTest():
	return [
		EdgeTrigger([0, 0], 0),
		LineDomino([0, 0], [SZ * 10, 0], (False, False)),
		EdgeTrigger([-SZ * 10, SZ], 0),
		LineDomino([-SZ * 10, SZ], [SZ * 10, SZ], (False, False)),
		OrGate([SZ * 10, SZ / 2], 0),
		LineDomino([SZ * 10, SZ / 2], [SZ * 20, SZ / 2], (False, False))]

def GetAndGateTest():
	(d1, d2) = (SZ * 10, SZ * 20)
	return [
		AndGate([0, 0], 0),
		LineDomino([0, 0], [0, d1], (False, False)),
		EdgeTrigger([0, d1], -math.pi / 2),
		LineDomino([SZ, 0], [d2, 0], (False, False)),
		EdgeTrigger([d2, 0], -math.pi),
		LineDomino([-SZ / 2, 0], [-SZ * 10, 0], (True, True))]

def GetCrossingTest():
	(d1, d2) = (SZ * 20, SZ * 10)
	return [
		Crossing([0, 0], 0),
		LineDomino([-d1, 0], [-SZ * 1.5, 0], (False, False)),
		EdgeTrigger([-d1, 0], 0),
		LineDomino([SZ * 1.5, 0], [d1, 0], (False, True)),
		LineDomino([0, d2], [0, 0], (False, False)),
		EdgeTrigger([0, d2], -math.pi / 2),
		LineDomino([0, 0], [0, -d2], (False, True))]

def GetSquareTest():
	components = [EdgeTrigger([-SZ * 10, 0], 0)]
	components += [LineDomino([-SZ * 10, 0], [0, 0], (False, False))]
	for i in range(10):
		x = i * SZ / LineDomino.INTERVAL_RATIO * 2
		components += [SideBranch([x, 0], 0)]
		components += [LineDomino([x, SZ], [x, SZ * 10], (True, True))]
		components += [SingleDomino([x + SZ / LineDomino.INTERVAL_RATIO, 0, SZ / 2], 'xyz')]
	return components

def GetFastPropagationTest():
	return [
		EdgeTrigger([0, -SZ * 10], math.pi / 2),
		LineDomino([0, -SZ * 10], [0, 0], (False, False)),
		FastPropagation([0, 0], [SZ * 30, 0]),
		LineDomino([SZ * 30, -SX], [SZ * 30, 1], (False, True))]

def Test(components):
	engine = Engine(0.001)
	for component in components:
		engine.AddComponent(component)
	engine.Start()

if __name__ == '__main__':
	Test(GetCrossingTest())
