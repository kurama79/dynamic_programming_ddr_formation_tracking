'''
Proyecto Robotica II - Implementacion de seguimiento de un usuario con multiples robots.

	Por L. Enrique Ruiz-Fernandez

	V-1.0 - Lo mas parecido al paaper... 
'''

import numpy as np
from copy import copy

import robot
import user
import plot

# Determinar la posicion inicial del usuario
def get_userPosition(limits):

	x = (limits[0][1] - limits[0][0])/2
	y = (limits[1][1] - limits[1][0])/2
	direction = 1.5*np.pi/2
	# direction = 0
	# direction = 2*np.pi/3
	# direction = 4*np.pi/3

	return (x, y, direction)

if __name__ == '__main__':

	# Parametros del problema
	roomLimits = [[0, 15], [0, 15]]

	agentsNumber = 3 # Numero de agentes
	visualRange = {'Angle':(2*np.pi)/agentsNumber, 'Lenght':[1.9, 2.1]} # Rango de vision correspondiente al numero de agentes
	robotsLinearVelocity = 1.8 # Velocidad lineal maxima de los agentes
	robotsAngularVelocity = 1.5 # Velocidad angular maxima de los agentes
	robotsTime2Move = 1.2

	userLinearVelocity = 1.8 # Velocidad lienal del usuario
	userAngularVelocity = 1.0 # Velocidad angula r del usuario
	userMaxTime = 1.2 # Tiempo maximo de recorrido

	# Generacion de usuario y robots
	position = get_userPosition(roomLimits)

	user = user.User(position, userLinearVelocity, userAngularVelocity, roomLimits, userMaxTime)

	robots = []
	for robotID in range(agentsNumber):
		robots.append(robot.Robot_DDR(robotID, visualRange, robotsLinearVelocity, robotsAngularVelocity, roomLimits, user))

	# Inicio de algoritmo...................................................................................................
	dt = 0.1 # Tiempo de muestreo
	inRange = True
	time = 0.0
	user.Motion()
	lastMove = user.motion_

	# while inRange:
	for i in range(5):

		robotMoving = False
		rangeMoves = 0
		steps = len(user.path_)-1

		currentMove = int(user.motionTime_/dt)
		for t in range(currentMove):

			time += dt

			# Inicia movimiento del usuario
			if user.motion_ == 'Straight':
				userPosition = user.straight_move(dt)

			elif user.motion_ == 'Rotation':
				userPosition = user.rotation_move(dt)
				
				for robot in robots:
					aux_position = copy(robot.path_[-1])
					robot.path_.append(aux_position)

				for robot in robots:
					robot.new_leader(userPosition)

			# Movimiento en linea recta
			if not robotMoving and user.motion_ == 'Straight' and robots[0].start_moving(userPosition):

				for index, robot in enumerate(robots):
					if robot.leader_[-1]:
						# Generando estados y trayectorias para el lider
						robot.states_generation(robotsTime2Move, dt, user.currentVelocity_, userPosition)
						robot.find_trajectory(dt)

						leader = index

				for robot in robots:

					if not robot.leader_[-1]:
						# Control para los otros agentes
						robot.position_control(robots[leader], dt)

				maxRotationTime = 0
				for robot in robots:

					if not robot.leader_[-1]:
						if robot.rotationTime_ >= maxRotationTime:

							maxRotationTime = robot.rotationTime_
							robotMaxID = robot.ID_

				for robot in robots:

					if robot.leader_[-1]:

						for i in range(maxRotationTime):

							aux = copy(robot.path_[steps])
							robot.path_.insert(steps, aux)

					elif robot.ID_ != robotMaxID and not robot.leader_[-1]:

						iterations = abs(robot.rotationTime_ - maxRotationTime)
						for i in range(iterations):

							aux = copy(robot.path_[-1])
							robot.path_ = robot.path_ + [aux]

				for robot in robots:

					for i in range(rangeMoves):

						aux = copy(robot.path_[steps])
						robot.path_ = robot.path_[0:steps] + [aux] + robot.path_[steps:-1]
				
				for i in range(maxRotationTime):

					user_aux = copy(user.path_[steps-rangeMoves])
					user.path_.insert(steps-rangeMoves, user_aux)

				# Para definir lider en la animacion
				for robot in robots:

					complementPath = abs(len(robot.path_) - len(robot.leader_))

					for i in range(complementPath):

						aux_leader = copy(robot.leader_[-1])
						robot.leader_.append(aux_leader)

				robotMoving = True

			else:

				rangeMoves += 1

			# Coordinar para tiempos no fijos..................!!!!

			if t == currentMove-1:
				
				maxRotationTime = 0
				for robot in robots:

					# robot.trim_path(steps, userPosition, dt)
					robot.correction_control(dt)

					if robot.rotationTime_ >= maxRotationTime:

						maxRotationTime = robot.rotationTime_
						robotMaxID = robot.ID_

				for i in range(maxRotationTime):

					user_aux = copy(user.path_[-1])
					user.path_ = user.path_ + [user_aux]

				for robot in robots:

					if robot.ID_ != robotMaxID:

						iterations = abs(robot.rotationTime_ - maxRotationTime)
						for i in range(iterations):

							aux = copy(robot.path_[-1])
							user_aux = copy(user.path_[-1])

							robot.path_ = robot.path_ + [aux]

					complementPath = abs(len(robot.path_) - len(robot.leader_))

					for i in range(complementPath):

						aux_leader = copy(robot.leader_[-1])
						robot.leader_.append(aux_leader)

			lastMove = user.motion_
		
		print(user.motion_)
		user.Motion()

	for robot in robots:
		print(len(robot.path_))
	
	print(len(user.path_))

	plot.visualize_state(robots, user)
	input('Ahora se vera animado')
	plot.visualize_dynamic(robots, user)