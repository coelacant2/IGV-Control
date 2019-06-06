from datetime import datetime, timedelta
from Vector2D import Vector2D
from PolarCoordinate import PolarCoordinate
from SabertoothDriver import Sabertooth as SMC
from PID import PID
import math
import numpy as np
import matplotlib.pyplot as plt

import time

class Motion:
	motorControl = None

	plt.ion()
	fig = plt.figure()
	ax = plt.subplot(1,1,1)
	displaySkip = 15
	displayCount = 15
	x = []
	y = []
	pid = None
	robotPosition = Vector2D()
	acceleration = 2.0
	wheelOffset  = 1.0
	minVelocity  = 0.01
	maxVelocity  = 2.0

	robotAngle   = 0.0
	leftRatio    = 0.0
	rightRatio   = 0.0
	velocity     = 0.0
	targetVelocity   = 0.0
	startVelocity    = 0.0
	endVelocity      = 0.0
	distanceToMove   = 0.0
	distanceMoved    = 0.0
	rampUpDistance   = 0.0
	rampDownDistance = 0.0
	isMoving       = False
	accelerate     = True
	robotDirection = 1.0
	rotationBias   = 0.0 # lr + 0.2 | rr - 0.2 = 1.2 -> 0.8  :  -1.0 -> 1.0
	isRotation     = False
	startAngle     = 0.0
	endAngle       = 0.0
	rotationPivot  = 0.0
	rotationAngle  = 0.0
	motorScalar    = 30.0
	leftVelocity   = 0.0
	rightVelocity  = 0.0
	startPosition  = Vector2D()
	endPosition    = Vector2D()
	previousDate   = datetime.now() - timedelta(milliseconds=50)

	def __init__(self, port, position: Vector2D, acceleration: float, wheelOffset: float, minVelocity: float, maxVelocity: float):
		#self.motorControl = SMC(port, baudrate=115200, address=128, timeout=0.1)
		self.pid = PID(0.075, 0.0, 0.05)

		self.acceleration = acceleration
		self.wheelOffset  = wheelOffset
		self.minVelocity  = minVelocity
		self.maxVelocity  = maxVelocity

		self.ax.plot(self.x, self.y, ',r-')
		self.fig.show()

	def setSpeeds(self, left: float, right: float):
		left  = self.constrain(left * self.motorScalar + self.rotationBias,  -30.0, 30.0)
		right = self.constrain(right * self.motorScalar - self.rotationBias,  -30.0, 30.0)

		#self.motorControl.drive(1, left)
		#self.motorControl.drive(2, right)

		self.leftVelocity = left
		self.rightVelocity = right

		#print("Set: " + str(left) + " " + str(right))

		if left == 0.0 and right == 0.0:
			#self.motorControl.stop()
			return None

		return None

	def setSpeedsRaw(self, left: float, right: float):
		#self.motorControl.drive(1, left)
		#self.motorControl.drive(2, right)

		self.leftVelocity = left
		self.rightVelocity = right

		print("Set: " + str(left) + " " + str(right))

		if left == 0.0 and right == 0.0:
			#self.motorControl.stop()
			return None

		return None

	@staticmethod
	def constrain(value: float, min: float, max: float):
		if value >= max:
			return max
		elif value <= min:
			return min
		else:
			return value

	@staticmethod
	def sign(value):
		if value < 0.0:
			return -1.0
		else:
			return 1.0

	@staticmethod
	def lerp(x: float, x0: float, x1: float, y0: float, y1: float):
		if (x1 - x0) == 0:
			return (y0 + y1) / 2.0
		else:
			return y0 + (x - x0) * (y1 - y0) / (x1 - x0)

	@staticmethod
	def map(x, inMin, inMax, outMin, outMax):
		return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

	def getIsMoving(self):
		return self.isMoving

	def shutdown(self):
		plt.close('all')

	def cancelMove(self):
		self.isMoving = False
		self.velocity = 0.0
		self.isRotation = False
		self.rotationBias = 0.0
		self.pid.clear()

		self.setSpeeds(0.0, 0.0)

		return None

	def calculateStep(self, currentYaw: float, skipCompensation):
		if self.isMoving:
			DT = (datetime.now() - self.previousDate).total_seconds()
			self.previousDate = datetime.now()
			self.distanceMoved += self.velocity * DT

			if skipCompensation:
				self.rotationBias = 0.0
			else:
				self.rotationBias = float(self.pid.update(self.robotAngle, currentYaw))

			if self.isRotation:
				self.updatePositionRotationMove()
			else:
				self.updatePositionNormalMove()

			self.plotVector(self.robotPosition)

			#print("TV:" + format(self.targetVelocity, '.2f') + " LR:" + format(self.leftRatio, '.2f') + " RR:" + format(self.rightRatio, '.2f') + " D:" + format(self.robotDirection, '.2f') + " RB:" + format(self.rotationBias, '.2f'))

			if not self.accelerate:
				if self.distanceMoved >= self.distanceToMove:
					self.isMoving = False
					self.velocity = 0.0
					self.isRotation = False
					self.rotationBias = 0.0
					self.pid.clear()

					self.setSpeeds(0.0, 0.0)
				else:
					self.setSpeeds(self.targetVelocity * self.leftRatio * self.robotDirection, self.targetVelocity * self.rightRatio * self.robotDirection)
			else:
				if self.distanceMoved < self.rampUpDistance:
					intVelocity = self.lerp(self.distanceMoved, 0, self.rampUpDistance, self.startVelocity, self.targetVelocity)
					self.velocity = self.constrain(intVelocity, self.minVelocity, self.maxVelocity)

					self.setSpeeds(self.velocity * self.leftRatio * self.robotDirection, self.velocity * self.rightRatio * self.robotDirection)
				elif self.distanceMoved < self.rampDownDistance:
					self.velocity = self.targetVelocity

					self.setSpeeds(self.targetVelocity * self.leftRatio * self.robotDirection, self.targetVelocity * self.rightRatio * self.robotDirection)
				elif self.distanceMoved < self.distanceToMove:
					intVelocity = self.lerp(self.distanceMoved, self.rampDownDistance, self.distanceToMove, self.targetVelocity, self.endVelocity)
					self.velocity = self.constrain(intVelocity, self.minVelocity, self.maxVelocity)

					self.setSpeeds(self.velocity * self.leftRatio * self.robotDirection, self.velocity * self.rightRatio * self.robotDirection)
				else:
					self.isMoving = False
					self.velocity = 0.0
					self.isRotation = False
					self.rotationBias = 0.0
					self.pid.clear()

					self.setSpeeds(0.0, 0.0)


		#print("A:" + format(currentYaw, '.2f') + " TA:" + format(self.robotAngle, '.2f') + " V:" + format(self.velocity, '.2f') + " DC:" + format(self.distanceMoved, '.2f') + " DT:" + format(self.distanceToMove, '.2f') + " LF:" +
		#	  format(self.leftVelocity, '.2f') + " RV:" + format(self.rightVelocity, '.2f') + " RB:" + format(self.rotationBias, '.2f') + " " + str(self.robotPosition))

		return self.isMoving

	def __move(self, tV: float, lr: float, rr: float, d: float):
		if not self.isMoving:
			self.previousDate = datetime.now()

			self.robotDirection = self.sign(tV) * self.sign(d)
			tV = abs(tV)
			d = abs(d)

			tV = self.constrain(tV, self.minVelocity, self.maxVelocity)
			self.targetVelocity = tV

			self.velocity = self.targetVelocity

			self.distanceMoved = 0.0
			self.distanceToMove = d

			self.leftRatio = lr
			self.rightRatio = rr

			self.setSpeeds(self.targetVelocity * self.leftRatio * self.robotDirection, self.targetVelocity * self.rightRatio * self.robotDirection)

			self.accelerate = False
			self.isMoving = True

			return True
		else:
			return False

	def __rampMove(self, tV: float, lr: float, rr: float, d: float):
		if not self.isMoving:
			self.previousDate = datetime.now()

			self.robotDirection = self.sign(tV) * self.sign(d)
			tV = abs(tV)
			d = abs(d)

			tV = self.constrain(tV, self.minVelocity, self.maxVelocity)

			rampTime = tV / self.acceleration
			rampDistance = 0.5 * self.acceleration * pow(rampTime, 2.0)

			if rampDistance * 2 >= d:
				self.rampUpDistance = d / 2
				self.rampDownDistance = d / 2
			else:
				self.rampUpDistance = rampDistance
				self.rampDownDistance = d - rampDistance

			self.distanceMoved = 0.0
			self.distanceToMove = d
			#self.rampUpDistance = tV / self.acceleration #t in seconds for ramp
			#self.rampDownDistance = d - tV / self.acceleration

			self.leftRatio = lr
			self.rightRatio = rr

			self.maxCapableVelocity = math.sqrt((d / 2.0) * self.acceleration * 2.0)

			if self.maxCapableVelocity < tV:
			  tV = self.maxCapableVelocity

			self.targetVelocity = tV

			self.velocity = self.minVelocity

			self.setSpeeds(self.velocity * self.leftRatio * self.robotDirection, self.velocity * self.rightRatio * self.robotDirection)

			self.accelerate = True
			self.isMoving = True

			return True
		else:
			  return False

	def move(self, ramp: bool, velocity: float, distance: float, sV=0.0, eV=0.0):
		self.isRotation = False
		self.startPosition = self.robotPosition
		self.startVelocity  = sV
		self.endVelocity    = eV

		if ramp:
			moveSent = self.__rampMove(velocity, 1.0, 1.0, distance)
		else:
			moveSent = self.__move(velocity, 1.0, 1.0, distance)

		self.endPosition = self.robotPosition + Vector2D.fromPolarCoordinate(distance, 90 - self.robotAngle)

	def rotate(self, ramp: bool, tV: float, a: float, pO: float, sV=0.0, eV=0.0):
		self.isRotation = True
		self.startAngle = self.robotAngle
		self.endAngle   = self.robotAngle + a
		self.rotationPivot  = pO
		self.rotationAngle  = a * self.sign(pO)
		self.startPosition  = self.robotPosition
		self.startVelocity  = sV
		self.endVelocity    = eV

		pivotPosition = Vector2D.fromPolarCoordinate(abs(pO), self.sign(pO) * 90.0 - 90 - self.startAngle)

		changeAngle = self.robotAngle + a

		#centralArcDistance = pO * a * math.pi / 180.0
		leftArcDistance = (pO + self.wheelOffset) * a * math.pi / 180.0
		rightArcDistance = (pO - self.wheelOffset) * a * math.pi / 180.0

		outerArcDistance = max(abs(leftArcDistance), abs(rightArcDistance))

		leftRatio = leftArcDistance / outerArcDistance#scale between 1 and -1
		rightRatio = rightArcDistance / outerArcDistance

		if ramp:
			moveSent = self.__rampMove(tV, leftRatio, rightRatio, outerArcDistance)
		else:
			moveSent = self.__move(tV, leftRatio, rightRatio, outerArcDistance)

		self.endPosition = self.startPosition + pivotPosition + Vector2D.fromPolarCoordinate(pO, 180.0 - a - self.startAngle)

		return moveSent;

	def updatePositionRotationMove(self):
		pivotPosition = Vector2D.fromPolarCoordinate(abs(self.rotationPivot), self.sign(self.rotationPivot) * 90.0 - 90 - self.startAngle)
		completionRatio = self.distanceMoved / self.distanceToMove

		self.__setRobotAngle(self.startAngle + self.rotationAngle * completionRatio)

		self.robotPosition = self.startPosition + pivotPosition + Vector2D.fromPolarCoordinate(self.rotationPivot, 180.0 - self.rotationAngle * completionRatio - self.startAngle)

		#print(format(self.robotAngle, '.2f') + " " + format(180.0 - self.rotationAngle * completionRatio - self.startAngle, '.2f'))

		return None

	def updatePositionNormalMove(self):
		completionRatio = self.distanceMoved / self.distanceToMove
		self.robotPosition = self.startPosition.interpolatePosition(self.endPosition, completionRatio)

		return None

	def __setRobotAngle(self, robotAngle: float):
		robotAngle = (robotAngle + 180.0) % 360.0 - 180.0

		self.robotAngle = robotAngle

	def plotVector(self, vec: Vector2D):
		if self.displayCount >= self.displaySkip:
			self.x.append(self.robotPosition.X)
			self.y.append(self.robotPosition.Y)

			self.ax.lines[0].set_data(self.x, self.y)
			self.ax.set_aspect('equal')
			self.ax.relim()
			self.ax.autoscale_view()
			self.fig.canvas.flush_events()
			self.displayCount = 0
		else:
			self.displayCount += 1

	def savePlot(self, name):
		plt.savefig(name, bbox_inches='tight', dpi=500)

	def getRobotPosition(self):
		return self.robotPosition
