# -*- coding: utf-8 -*-

from Engine import *
from Plugins import *
from Components import *

class ComponentTest:
	DEFAULT_ARGUMENTS = {}
	@classmethod
	def GenerateArguments(cls):
		return [{}]
	def __init__(self, name, arguments = {}):
		self.name = name
		self.arguments = arguments
		for key in self.DEFAULT_ARGUMENTS:
			if key not in self.arguments:
				self.arguments[key] = self.DEFAULT_ARGUMENTS[key]
		self.component = self.GenerateComponent()
		self.success = True
	def GenerateComponent(self):
		return None
	def GenerateSequences(self):
		return [[]]
	def GenerateStatics(self):
		return []
	def Run(self):
		engine = Engine(0.001, DIRECT)
		engine.AddComponent(self.component)
		sequences = self.GenerateSequences()
		statics = self.GenerateStatics()
		auto_terminate = AutoTerminatePlugin(engine)
		engine.AddPlugin(auto_terminate)
		component_test = ComponentTestPlugin(engine, sequences, statics)
		engine.AddPlugin(component_test)
		engine.Start()
		if not component_test.test_success:
			self.success = False
			print('=' * 30, self.name, '=' * 30)
			print(self.arguments)
			print(component_test.sequence_moments)
			print(component_test.static_moments)

class LineDominoTest(ComponentTest):
	DEFAULT_ARGUMENTS = {
		'side': False}
	@classmethod
	def GenerateArguments(cls):
		return [{'side': True}, {'side': False}]
	def __init__(self, arguments = {}):
		ComponentTest.__init__(self, 'LineDominoTest', arguments)
	def GenerateComponent(self):
		component = Component()
		component.children = {
			'trigger': LeanTrigger([0, 0], 0),
			'line': LineDomino([0, 0], [SZ * 10, 0], (False, True), side = self.arguments['side'])}
		return component
	def GenerateSequences(self):
		return [[
			self.component['trigger'].Id(),
			self.component['line'].StartId(),
			self.component['line'].EndId()]]

class UTurnTest(ComponentTest):
	DEFAULT_ARGUMENTS = {
		'N': 2}
	@classmethod
	def GenerateArguments(cls):
		return [{'N': 2}, {'N': 3}]
	def __init__(self, arguments = {}):
		ComponentTest.__init__(self, 'UTurnTest', arguments)
	def GenerateComponent(self):
		component = Component()
		component.children['trigger'] = LeanTrigger([0, 0], 0)
		for i in range(self.arguments['N']):
			y = SZ * i
			if i % 2 == 0:
				component.children['line' + str(i)] = LineDomino([0, y], [SZ * 10, y], (False, False))
				component.children['uturn' + str(i)] = UTurn([SZ * 10, y + SZ / 2], 0)
			else:
				component.children['line' + str(i)] = LineDomino([SZ * 10, y], [0, y], (False, False))
				component.children['uturn' + str(i)] = UTurn([0, y + SZ / 2], math.pi)
		return component
	def GenerateSequences(self):
		sequence = [self.component['trigger'].Id()]
		for i in range(self.arguments['N']):
			sequence += [self.component['line' + str(i)].StartId()]
			sequence += [self.component['line' + str(i)].EndId()]
			sequence += [self.component['uturn' + str(i)]['lever'].Id()]
		return [sequence]

class AndGateTest(ComponentTest):
	DEFAULT_ARGUMENTS = {
		'exist': (True, True),
		'order12': True}
	@classmethod
	def GenerateArguments(cls):
		return [
			{'exist': (True, True), 'order12': True},
			{'exist': (True, True), 'order12': False},
			{'exist': (True, False)},
			{'exist': (False, True)}]
	def __init__(self, arguments = {}):
		ComponentTest.__init__(self, 'AndGateTest', arguments)
	def GenerateComponent(self):
		component = Component()
		component.children['and'] = AndGate([0, 0], 0)
		L = [SZ * 10, SZ * 20] if self.arguments['order12'] else [SZ * 20, SZ * 10]
		component.children['in1'] = LineDomino([0, L[0]], [0, 0], (False, False))
		component.children['in2'] = LineDomino([L[1], 0], [SZ, 0], (False, False))
		if self.arguments['exist'][0]:
			component.children['trigger1'] = LeanTrigger([0, L[0]], -math.pi / 2)
		if self.arguments['exist'][1]:
			component.children['trigger2'] = LeanTrigger([L[1], 0], -math.pi)
		component.children['out'] = LineDomino([-SZ / 2, 0], [-SZ * 10, 0], (True, True))
		return component
	def GenerateSequences(self):
		sequences = []
		if self.arguments['exist'][0]:
			sequences += [[
				self.component['trigger1'].Id(),
				self.component['in1'].StartId(),
				self.component['in1'].EndId(),
				self.component['and']['barrier'][1].Id()]]
		if self.arguments['exist'][1]:
			sequences += [[
				self.component['trigger2'].Id(),
				self.component['in2'].StartId(),
				self.component['in2'].EndId(),
				self.component['and']['highline'][1].Id()]]
		if self.arguments['exist'][0] and self.arguments['exist'][1]:
			and_sequence = [
				self.component['and']['barrier'][1].Id(),
				self.component['and']['highline'][1].Id()]
			if not self.arguments['order12']:
				and_sequence = and_sequence[: : -1]
			out_sequence = [
				self.component['out'].StartId(),
				self.component['out'].EndId()]
			sequences += [and_sequence + out_sequence]
		return sequences
	def GenerateStatics(self):
		statics = []
		if not self.arguments['exist'][0]:
			statics += [
				self.component['in1'].StartId(),
				self.component['in1'].EndId(),
				self.component['and']['barrier'][1].Id()]
		if not self.arguments['exist'][1]:
			statics += [
				self.component['in2'].StartId(),
				self.component['in2'].EndId(),
				self.component['and']['highline'][1].Id()]
		if not self.arguments['exist'][0] or not self.arguments['exist'][1]:
			statics += [self.component['out'].StartId(), self.component['out'].EndId()]
		return statics

class CrossingTest(ComponentTest):
	DEFAULT_ARGUMENTS = {
		'order12': True}
	@classmethod
	def GenerateArguments(cls):
		return [{'order12': True}, {'order12': False}]
	def __init__(self, arguments = {}):
		ComponentTest.__init__(self, 'CrossingTest', arguments)
	def GenerateComponent(self):
		component = Component()
		L = [SZ * 10, SZ * 20] if self.arguments['order12'] else [SZ * 20, SZ * 10]
		component.children = {
			'crossing': Crossing([0, 0], 0),
			'in1': LineDomino([-L[0], 0], [-SZ * 1.5, 0], (False, False)),
			'trigger1': LeanTrigger([-L[0], 0], 0),
			'out1': LineDomino([SZ * 1.5, 0], [SZ * 10, 0], (False, True)),
			'in2': LineDomino([0, L[1]], [0, 0], (False, False)),
			'trigger2': LeanTrigger([0, L[1]], -math.pi / 2),
			'out2': LineDomino([0, 0], [0, -SZ * 10], (False, True))}
		return component
	def GenerateSequences(self):
		sequences = [[]]
		sequences += [[
			self.component['trigger1'].Id(),
			self.component['in1'].StartId(),
			self.component['in1'].EndId(),
			self.component['crossing']['triggerL'].Id(),
			self.component['crossing']['triggerR'].Id(),
			self.component['out1'].StartId(),
			self.component['out1'].EndId()]]
		sequences += [[
			self.component['trigger2'].Id(),
			self.component['in2'].StartId(),
			self.component['in2'].EndId(),
			self.component['crossing']['bridge'].Id(),
			self.component['out2'].StartId(),
			self.component['out2'].EndId()]]
		return sequences

class SideBranchTest(ComponentTest):
	DEFAULT_ARGUMENTS = {
		'N': 2}
	@classmethod
	def GenerateArguments(cls):
		return [{'N': 2}, {'N': 3}]
	def __init__(self, arguments):
		ComponentTest.__init__(self, 'SideBranchTest', arguments)
	def GenerateComponent(self):
		component = Component()
		component.children['trigger'] = LeanTrigger([0, 0], 0)
		x = 0
		for i in range(self.arguments['N']):
			next_x = x + SZ * 5
			component.children['gap' + str(i)] = LineDomino([x, 0], [next_x, 0], (False, False))
			component.children['side' + str(i)] = SideBranch([next_x, 0], 0)
			component.children['line' + str(i)] = LineDomino([next_x, SZ], [next_x, SZ * 10], (True, True))
			x = next_x
		return component
	def GenerateSequences(self):
		sequences = []
		main_sequence = [self.component['trigger'].Id()]
		for i in range(self.arguments['N']):
			main_sequence += [
				self.component['gap' + str(i)].StartId(),
				self.component['gap' + str(i)].EndId(),
				self.component['side' + str(i)]['connection'].Id()]
		sequences += [main_sequence]
		for i in range(self.arguments['N']):
			sequences += [[
				self.component['side' + str(i)]['connection'].Id(),
				self.component['side' + str(i)]['trigger'].Id(),
				self.component['line' + str(i)].StartId(),
				self.component['line' + str(i)].EndId()]]
		return sequences

class FastPropagationTest(ComponentTest):
	DEFAULT_ARGUMENTS = {}
	@classmethod
	def GenerateArguments(cls):
		return [{}]
	def __init__(self, arguments):
		ComponentTest.__init__(self, 'FastPropagationTest', arguments)
	def GenerateComponent(self):
		component = Component()
		(L, y) = (SZ * 10, -SZ * 2)
		component.children['trigger1'] = LeanTrigger([0, L], -math.pi / 2)
		component.children['head'] = LineDomino([0, L], [0, 0], (False, False))
		component.children['fast'] = FastPropagation([0, 0], [L * 3, 0])
		component.children['tail'] = LineDomino([L * 3, -SX], [L * 3, L], (False, True))
		component.children['trigger2'] = LeanTrigger([-L, y], 0)
		component.children['line'] = LineDomino([-L, y], [L * 4, y], (False, True))
		return component
	def GenerateSequences(self):
		return [
			[
				self.component['trigger1'].Id(),
				self.component['head'].StartId(),
				self.component['head'].EndId(),
				self.component['fast'].StartId(),
				self.component['fast'].EndId(),
				self.component['fast']['trigger'].Id(),
				self.component['tail'].StartId(),
				self.component['tail'].EndId()],
			[
				self.component['trigger2'].Id(),
				self.component['line'].StartId(),
				self.component['line'].EndId()],
			[self.component['tail'].EndId(), self.component['line'].EndId()]]

def GetFastPropagationTest():
	test = Component()
	test.children = {
		'trigger': EdgeTrigger([0, -SZ * 10], math.pi / 2),
		'line1': LineDomino([0, -SZ * 10], [0, 0], (False, False)),
		'fast': FastPropagation([0, 0], [SZ * 30, 0]),
		'line2': LineDomino([SZ * 30, -SX], [SZ * 30, 1], (False, True))}
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

if __name__ == '__main__':
	test_classes = [
		LineDominoTest,
		UTurnTest,
		AndGateTest,
		CrossingTest,
		SideBranchTest,
		FastPropagationTest][: : -1]
	for test_class in test_classes:
		for arguments in test_class.GenerateArguments():
			test = test_class(arguments)
			print(test.name, test.arguments)
			test.Run()
