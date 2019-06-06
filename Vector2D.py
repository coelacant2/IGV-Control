import math

class Vector2D:
	X = 0.0
	Y = 0.0

	def __init__(self, X = None, Y = None):
		if isinstance(X, Vector2D):
			self.X = X.X
			self.Y = X.Y
		else:
			if X is None:
				self.X = 0.0
			else:
				self.X = X

			if Y is None:
				self.Y = 0.0
			else:
				self.Y = Y

	def __add__(self, vec):
		if isinstance(vec, Vector2D):
			return Vector2D(self.X + vec.X, self.Y + vec.Y)
		elif isinstance(vec, float) or isinstance(vec, int):
			return Vector2D(self.X + vec, self.Y + vec)
		else:
			return NotImplemented

	def __sub__(self, vec):
		if isinstance(vec, Vector2D):
			return Vector2D(self.X - vec.X, self.Y - vec.Y)
		elif isinstance(vec, float) or isinstance(vec, int):
			return Vector2D(self.X - vec, self.Y - vec)
		else:
			return NotImplemented

	def __rsub__(self, vec):
		if isinstance(vec, Vector2D):
			return Vector2D(vec.X - self.X, vec.Y - self.Y)
		elif isinstance(vec, float) or isinstance(vec, int):
			return Vector2D(vec - self.X, vec - self.Y)
		else:
			return NotImplemented

	def __mul__(self, vec):
		if isinstance(vec, Vector2D):
			return Vector2D(self.X * vec.X, self.Y * vec.Y)
		elif isinstance(vec, float) or isinstance(vec, int):
			return Vector2D(self.X * vec, self.Y * vec)
		else:
			return NotImplemented

	def __div__(self, vec):
		if isinstance(vec, Vector2D):
			return Vector2D(self.X / vec.X, self.Y / vec.Y)
		elif isinstance(vec, float) or isinstance(vec, int):
			return Vector2D(self.X / vec, self.Y / vec)
		else:
			return NotImplemented

	def __rdiv__(self, vec):
		if isinstance(vec, Vector2D):
			return Vector2D(vec.X / self.X, vec.Y / self.Y)
		elif isinstance(vec, float) or isinstance(vec, int):
			return Vector2D(vec / self.X, vec / self.Y)
		else:
			return NotImplemented

	def __pow__(self, vec):
		if isinstance(vec, float) or isinstance(vec, int):
			return Vector2D(self.X ** vec, self.Y ** vec)
		else:
			return NotImplemented

	def __iadd__(self, vec):
		if isinstance(vec, Vector2D):
			self.X += vec.X
			self.Y += vec.Y
		elif isinstance(vec, float) or isinstance(vec, int):
			self.X += vec
			self.Y += vec
			return self
		else:
			return NotImplemented

	def __isub__(self, vec):
		if isinstance(vec, Vector2D):
			self.X -= vec.X
			self.Y -= vec.Y
			return self
		elif isinstance(vec, float) or isinstance(vec, int):
			self.X -= vec
			self.Y -= vec
			return self
		else:
			return NotImplemented

	def __imul__(self, vec):
		if isinstance(vec, Vector2D):
			self.X *= vec.X
			self.Y *= vec.Y
			return self
		elif isinstance(vec, float) or isinstance(vec, int):
			self.X *= vec
			self.Y *= vec
			return self
		else:
			return NotImplemented

	def __idiv__(self, vec):
		if isinstance(vec, Vector2D):
			self.X /= vec.X
			self.Y /= vec.Y
			return self
		elif isinstance(vec, float) or isinstance(vec, int):
			self.X /= vec
			self.Y /= vec
			return self
		else:
			return NotImplemented

	def __ipow__(self, vec):
		if isinstance(vec, float) or isinstance(vec, int):
			self.X **= vec
			self.Y **= vec
			return self
		else:
			return NotImplemented

	def __eq__(self, other):
		if isinstance(other, Vector2D):
			return self.X == other.X and self.Y == other.Y
		else:
			return NotImplemented

	def __ne__(self, other):
		if isinstance(other, Vector2D):
			return self.X != other.X or self.Y != other.Y
		else:
			return NotImplemented

	def __gt__(self, other):
		if isinstance(other, Vector2D):
			return self.getMagnitude() > other.getMagnitude()
		else:
			return NotImplemented
	def __ge__(self, other):
		if isinstance(other, Vector2D):
			return self.getMagnitude() >= other.getMagnitude()
		else:
			return NotImplemented

	def __lt__(self, other):
		if isinstance(other, Vector2D):
			return self.getMagnitude() < other.getMagnitude()
		else:
			return NotImplemented

	def __le__(self, other):
		if isinstance(other, Vector2D):
			return self.getMagnitude() <= other.getMagnitude()
		else:
			return NotImplemented


	def __eq__(self, other):
		if isinstance(other, Vector2D):
			return self.X == other.X and self.Y == other.Y
		else:
			return NotImplemented

	def __len__(self):
		return int(sqrt(self.X**2 + self.Y**2))

	def __getitem__(self, key):
		if key == "X" or key == "X" or key == 0 or key == "0":
			return self.X
		elif key == "y" or key == "Y" or key == 1 or key == "1":
			return self.Y

	def __str__(self):
		return "[X: %(X)f, Y: %(Y)f]" % self

	def __repr__(self):
		return "{'X': %(X)f, 'Y': %(Y)f}" % self

	def __neg__(self):
		return Vector2D(-self.X, -self.Y)

	def getMagnitude(self):
		return sqrt(self.X**2 + self.Y**2)

	def interpolatePosition(self, vec, ratio: float):
		if isinstance(vec, Vector2D):
			return Vector2D(self.X * (1.0 - ratio) + vec.X * ratio, self.Y * (1.0 - ratio) + vec.Y * ratio)
		else:
			return NotImplemented

	@staticmethod
	def fromPolarCoordinate(radius: float, angle: float):
		return Vector2D(radius * math.cos(angle * math.pi / 180.0), radius * math.sin(angle * math.pi / 180.0))

	def rotateVector(self, angle:float):
		return Vector2D(math.cos(angle * self.X * math.pi / 180.0) - math.sin(angle * self.Y * math.pi / 180.0),
						math.sin(angle * self.X * math.pi / 180.0) + math.cos(angle * self.Y * math.pi / 180.0))
