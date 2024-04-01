# Clase para genera el objeto usuario

import numpy as np 
import random
import shapely.geometry

class User(object):

	def __init__(self, position, linearVelocity, angularVelocity, roomLimits, maxTime):

		self.path_ = [position]
		self.linearVelocity_ = linearVelocity
		self.angularVelocity_ = angularVelocity
		self.roomLimits_ = roomLimits
		self.maxTime_ = maxTime

	# Realizar un movimiento aleatorio del usuario
	def Motion(self):

		primitive = bool(random.getrandbits(1))
		# primitive = True

		if primitive:

			self.motion_ = 'Straight'
			# self.motionTime_ = random.uniform(1.5, self.maxTime_)
			self.motionTime_ = self.maxTime_
			self.currentVelocity_ = random.uniform(0.7, self.linearVelocity_)

		else:

			self.motion_ = 'Rotation'
			self.motionTime_ = random.uniform(0.5, self.maxTime_)
			self.currentVelocity_ = random.uniform(0.4, self.linearVelocity_)

	# Funcion para movimiento recto
	def straight_move(self, dt):

		x = self.path_[-1][0] + (self.currentVelocity_*np.cos(self.path_[-1][2]))*dt
		y = self.path_[-1][1] + (self.currentVelocity_*np.sin(self.path_[-1][2]))*dt

		self.path_.append((x, y, self.path_[-1][2]))

		return self.path_[-1]

	# Funcion para rotacion
	def rotation_move(self, dt):

		theta = self.path_[-1][2] + (self.currentVelocity_)*dt

		self.path_.append((self.path_[-1][0], self.path_[-1][1], theta))

		return self.path_[-1]

	# Funcion para forma del usuario
	def Shape(self, pos):

		x = pos[0]
		y = pos[1]
		direction = pos[2]

		p1 = (x + 0.3*np.cos(direction-(np.pi/2+0.1)), y + 0.3*np.sin(direction-(np.pi/2+0.1)))
		p2 = (x + 0.1*np.cos(direction-(np.pi/2)), y + 0.1*np.sin(direction-(np.pi/2)))
		p3 = (x + 0.13*np.cos(direction), y + 0.13*np.sin(direction))
		p5 = (x + 0.3*np.cos(direction+(np.pi/2+0.1)), y + 0.3*np.sin(direction+(np.pi/2+0.1)))
		p4 = (x + 0.1*np.cos(direction+(np.pi/2)), y + 0.1*np.sin(direction+(np.pi/2)))

		shape = shapely.geometry.LineString([p1, p2, p3, p4, p5]).buffer(0.08)

		return shape