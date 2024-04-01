# Clase para genera el objeto robot, modelo DDR

import numpy as np
from copy import copy
import shapely.geometry

class Robot_DDR(object):

	def __init__(self, ID, visualRange, linearVelocity, angularVelocity, roomLimits, user):

		self.ID_ = ID 
		self.visualRange_ = visualRange
		self.linearVelocity_ = linearVelocity
		self.angularVelocity_ = angularVelocity
		self.roomLimits_ = roomLimits
		self.user_ = user
		self.leader_ = []

		self.path_ = []
		# self.controls_ = []
		# self.leader_ = False
		self.radius_ = 0.4 # Medida propuesta 0.3 pero se suma 0.1 por rango

		self.initialize_position()
		self.states_tree_ = {}
		self.timeTraveled_ = 0.0

		self.fixed_direction_ = self.path_[-1][2]

	# Inicializamos la posicion del robot, orientacion y si es lider o no...
	def initialize_position(self):

		radius = (self.visualRange_['Lenght'][0] + self.visualRange_['Lenght'][1])/2

		x = self.user_.path_[-1][0] + radius*np.cos(self.ID_*self.visualRange_['Angle'])
		y = self.user_.path_[-1][1] + radius*np.sin(self.ID_*self.visualRange_['Angle'])
		direction = np.pi + self.ID_*self.visualRange_['Angle']

		self.path_.append((x, y, direction))

		# Determinar se esta dentro de su rango de vista
		# theta = abs(np.arctan2(y - self.user_.path_[-1][1], x - self.user_.path_[-1][0]))# - self.user_.path_[-1][2])
		theta = self.user_.path_[-1][2]
		alpha = theta - self.ID_*self.visualRange_['Angle']# + self.user_.path_[-1][2]

		# print(theta)
		# print(alpha)
		# input()

		if abs(alpha) < self.visualRange_['Angle']/2:
			self.leader_.append(True)
		else:
			self.leader_.append(False)

	# Generar arbol de estados
	def states_generation(self, time, dt, userVelocity, userPosition):

		k = 1/self.angularVelocity_ # Contante de conversion para el costo
		# self.robotsVelocity_ = (userVelocity + self.linearVelocity_)/2 # Una velocidad mayor que el usuario para compensar tiempo de generacion de arbol
		self.robotsVelocity_ = userVelocity

		self.states_tree_ = {}
		self.states_tree_[self.path_[-1]] = {'Parent':None, 'Leafs':[],'Control':(0, 0), 'Cost':dt}

		# Definimos controles
		t = 0.0
		while t <= time:

			current_keys = copy(list(self.states_tree_.keys()))
			for key in current_keys:

				if not self.states_tree_[key]['Leafs']:

					# Generando los nuevos esatdos de acuerdo a los controles
					new_state_1 = self.back_control(key, self.robotsVelocity_, dt)
					new_state_2 = self.back_left_control(key, self.robotsVelocity_, dt)
					new_state_3 = self.back_right_control(key, self.robotsVelocity_, dt)

					userPosition = self.estimate_userPosition(userPosition, userVelocity, dt)

					relativeAngle_1 = new_state_1[2] - np.pi
					# print(relativeAngle_1)
					# print(userPosition[2])
					# input()
					relativeAngle_2 = new_state_1[2] - np.pi
					relativeAngle_3 = new_state_1[2] - np.pi

					# if relativeAngle_1 < 0:
					# 	relativeAngle_1 += 2*np.pi
					# 	relativeAngle_2 += 2*np.pi
					# 	relativeAngle_3 += 2*np.pi

					alpha_1 = userPosition[2] - relativeAngle_3
					alpha_2 = userPosition[2] - relativeAngle_2
					alpha_3 = userPosition[2] - relativeAngle_3

					# Verificnado si se agrega al arbol o no
					if abs(alpha_1) < self.visualRange_['Angle']/2:

						alpha_error = abs((new_state_1[2]-np.pi) - userPosition[2])
						parent = key
						cost = self.states_tree_[parent]['Cost'] + self.isDesirable(parent, userPosition)*dt - k*alpha_error

						self.states_tree_[new_state_1] = {'Parent':parent, 'Leafs':[], 'Control':'BC', 'Cost': cost}
						self.states_tree_[parent]['Leafs'].append(new_state_1)

					if abs(alpha_2) < self.visualRange_['Angle']/2:

						alpha_error = abs((new_state_2[2]-np.pi) - userPosition[2])
						parent = key
						cost = self.states_tree_[parent]['Cost'] + self.isDesirable(parent, userPosition)*dt - k*alpha_error

						self.states_tree_[new_state_2] = {'Parent':parent, 'Leafs':[], 'Control':'BLC', 'Cost': cost}
						self.states_tree_[parent]['Leafs'].append(new_state_2)

					if abs(alpha_3) < self.visualRange_['Angle']/2:

						alpha_error = abs((new_state_3[2]-np.pi) - userPosition[2])
						parent = key
						cost = self.states_tree_[parent]['Cost'] + self.isDesirable(parent, userPosition)*dt - k*alpha_error

						self.states_tree_[new_state_3] = {'Parent':parent, 'Leafs':[], 'Control':'BRC', 'Cost': cost}
						self.states_tree_[parent]['Leafs'].append(new_state_3)

			# print(t)
			t += dt

		# print(self.states_tree_)

	# Funcion que determina si es deseable una posicion
	def isDesirable(self, key, userPosition):

		x, y, theta = key

		theta_error = abs(np.tan((y-userPosition[1])/(x-userPosition[0])))
		dist_error = self.distance([x, y], [userPosition[0], userPosition[1]])

		if theta_error < self.visualRange_['Angle']/2 and dist_error < self.visualRange_['Lenght'][1] and dist_error > self.visualRange_['Lenght'][0]:
			return 1.0

		else:
			return 0.0

	# Control hacia atras
	def back_control(self, key, velocity, dt):

		x, y, theta = key
		
		x = x + (-velocity)*np.cos(theta)*dt
		y = y + (-velocity)*np.sin(theta)*dt

		return (x, y, theta)

	# Control atras-izquierda
	def back_left_control(self, key, velocity, dt):

		x, y, theta = key
		
		x = x + (-velocity)*np.cos(theta)*dt
		y = y + (-velocity)*np.sin(theta)*dt
		theta = theta + (self.angularVelocity_)*dt

		return (x, y, theta)

	# Control atras-derecha
	def back_right_control(self, key, velocity, dt):

		x, y, theta = key
		
		x = x + (-velocity)*np.cos(theta)*dt
		y = y + (-velocity)*np.sin(theta)*dt
		theta = theta + (-self.angularVelocity_)*dt

		return (x, y, theta)

	# Funcion para estimar 
	def estimate_userPosition(self, userPosition, userVelocity, dt):

		x, y, theta = userPosition

		x += userVelocity*np.cos(theta)*dt
		y += userVelocity*np.sin(theta)*dt

		return (x, y, theta)

	# Funcion para encontrar la trayectoria que maximice
	def find_trajectory(self, dt):

		self.controls_ = []

		leafs = self.states_tree_[self.path_[-1]]['Leafs']
		while leafs:

			max_cost = -10000
			for pos in leafs:

				aux_cost = self.states_tree_[pos]['Cost']

				if aux_cost >= max_cost:
					max_cost = aux_cost
					max_pos = pos


			self.path_.append(max_pos)
			self.controls_.append(self.states_tree_[max_pos]['Control'])
			leafs = self.states_tree_[max_pos]['Leafs']
			self.timeTraveled_ += dt

	# Funcion de control para corregir posicion del robot
	def correction_control(self, dt):

		x = self.path_[-1][0]
		y = self.path_[-1][1]
		theta = self.path_[-1][2]
		theta_ref = self.fixed_direction_
		error = theta - theta_ref

		if abs(error) > np.pi:

			aux2error = min([theta_ref, theta])
			index = min(range(len([theta_ref, theta])), key=[theta_ref, theta].__getitem__)


			if index == 0:
				theta_ref += 2*np.pi
			else:
				theta += 2*np.pi

		self.rotationTime_ = 0
		while abs(error) > 0.1:

			if error > 0:
				control = -self.angularVelocity_
			else:
				control = self.angularVelocity_

			theta = theta + control*dt
			error = theta - theta_ref

			self.path_.append((x, y, theta))
			self.rotationTime_ += 1

	# Funcion para determinar empezar o no el desplazamiento del robot
	def start_moving(self, userPosition):

		dist_error = self.distance(self.path_[-1], userPosition)

		if dist_error < self.visualRange_['Lenght'][1] and dist_error > self.visualRange_['Lenght'][0]:
			return False 

		else:
			return True
		
	# Funcion para control de robots (no es lider)
	def position_control(self, leader, dt):

		# if not repeat:
		# 	theta_ref = leaderID*self.visualRange_['Angle']
		theta_ref = leader.ID_*self.visualRange_['Angle']

		# else:
			# theta_ref = leaderPosition[2] + np.pi

		x = self.path_[-1][0]
		y = self.path_[-1][1]
		theta = self.path_[-1][2]

		error = theta - theta_ref

		if abs(error) >= np.pi:

			aux2error = min([theta_ref, theta])
			index = min(range(len([theta_ref, theta])), key=[theta_ref, theta].__getitem__)

			if index == 0:
				theta_ref += 2*np.pi
			else:
				theta += 2*np.pi

		self.rotationTime_ = 0
		while abs(error) > 0.1:

			if error > 0:
				control = -self.angularVelocity_
			else:
				control = self.angularVelocity_

			theta = theta + control*dt
			error = theta - theta_ref

			self.path_.append((x, y, theta))
			self.rotationTime_ += 1

		for control in leader.controls_:

			if control == 'BC':
				new_state = self.back_control((x, y, theta), -leader.robotsVelocity_, dt)

			elif control == 'BLC':
				new_state = self.back_left_control((x, y, theta), -leader.robotsVelocity_, dt)

			elif control == 'BRC':
				new_state = self.back_right_control((x, y, theta), -leader.robotsVelocity_, dt)

			self.path_.append(new_state)
			x, y, theta = self.path_[-1]

	# Funcion para determinar posiciones deseadas
	def generate_reference(self, userPosition, radius):

		x_ref = userPosition[0] + radius*np.cos(self.ID_*self.visualRange_['Angle'])
		y_ref = userPosition[1] + radius*np.sin(self.ID_*self.visualRange_['Angle'])
		theta_ref = np.pi + self.ID_*self.visualRange_['Angle']

		return x_ref, y_ref, theta_ref

	# Funcion para recortar trayectoria
	def trim_path(self, userSteps, userPosition, dt): # Mejorar funcion, cuando no se conoce el tiempo que se mueve el usuario!!!!!!!!!!!!!!!!!!

		# path2trim = abs(len(self.path_) - userSteps)
		path2trim = userSteps
		# print(path2trim)
		# input()

		# if not self.leader_:
		# 	for j in range(1, len(self.path_)):
		# 		if self.path_[j][0] == self.path_[j-1][0] and self.path_[j][1] == self.path_[j-1][1]:
		# 			path2trim -= 1

		# for i in range(path2trim):
		# 	self.path_.pop(-1)

		deletaed = 0
		aux = []
		# while deletaed < path2trim:

		for pos in range(len(self.path_)-1, 0, -1):

			if not self.path_[pos][0] == self.path_[pos-1][0] and not self.path_[pos][1] == self.path_[pos-1][1]:
				aux.append(pos)
				deletaed += 1
				
			if deletaed >= path2trim:
				break

		for i in range(len(aux)):
			self.path_.pop(aux[i])

		# while self.leader_:
		# while 1:

		# 	if self.distance(self.path_[-1], userPosition) < self.visualRange_['Lenght'][1] and self.distance(self.path_[-1], userPosition) > self.visualRange_['Lenght'][0]:
		# 		break

		# 	self.path_.pop(-1)

		self.correction_control(dt)

	# Determinar nuevo lider
	def new_leader(self, userPosition):

		theta = userPosition[2]
		alpha = theta - self.ID_*self.visualRange_['Angle']

		if abs(alpha) < self.visualRange_['Angle']/2:
			self.leader_.append(True)
		else:
			self.leader_.append(False)

	# Forma del robot
	def TriangleShape(self, position):

		p1 = [position[0] + self.radius_*np.cos(position[2]), position[1] + self.radius_*np.sin(position[2])]
		p2 = [position[0] + self.radius_*np.cos(position[2]-(np.pi/2)), position[1] + self.radius_*np.sin(position[2]-(np.pi/2))]
		p3 = [position[0] + self.radius_*np.cos(position[2]+(np.pi/2)), position[1] + self.radius_*np.sin(position[2]+(np.pi/2))]
		shape = shapely.geometry.Polygon([(p1[0], p1[1]), (p2[0], p2[1]), (p3[0], p3[1])])

		return shape

	# Funcion distancia
	def distance(self, p1, p2):
		return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)