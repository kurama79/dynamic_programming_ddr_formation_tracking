# Mostramos el movimientos de los robots

import numpy as np
from collections import deque
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import matplotlib.cm as cmx
# import matplotlib.colors as colors
import matplotlib.animation as animation
import matplotlib as mpl

from descartes import PolygonPatch

def visualize_dynamic(robots, user, animated_time=120): # Falta Texto

	history_len = int(len(robots[0].path_) / 5)
	colors = cmx.rainbow(np.linspace(0, 1, len(robots)))
	side = 0.4
	frames = len(user.path_)

	if len(robots[0].path_) < frames:
		frames = len(robots[0].path_)

	fig1, ax1 = plt.subplots()
	ax1.set_title('Mostrando animacion de los robots')
	ax1.set_xlim(robots[0].roomLimits_[0][0], robots[0].roomLimits_[0][1])
	ax1.set_ylim(robots[0].roomLimits_[1][0], robots[0].roomLimits_[1][1])

	# Generamos los puntos y los trazos
	user_shape = user.Shape((0, 0, 0))
	user_patch = PolygonPatch(user_shape, fc='orange', alpha=0.5, zorder=2)
	user_head = plt.Circle((user.path_[0][0], user.path_[0][1]), 0.2, fc='orange', alpha=5)

	user_text = ax1.text(user.path_[-1][0]+0.2, user.path_[-1][1], 'User', fontsize=10, zorder=3)

	trace = []
	histories_x = []
	histories_y = []
	cars = []
	circles = []
	texts = []
	for i, robot in enumerate(robots):
		trace.append(ax1.plot([], [], '-', color=colors[i], lw=1)[0])
		histories_x.append(deque(maxlen=history_len))
		histories_y.append(deque(maxlen=history_len))

		texts.append(ax1.text(robot.path_[0][0]+side, robot.path_[0][1], 'ID:%s'%i,
				fontsize=10, fontweight='bold', zorder=3))

		if robot.leader_[0]:
			leader_text = ax1.text(robot.path_[0][0]+side-0.2, robot.path_[0][1]+side+0.1, 'Leader',
				fontsize=10, zorder=3)

	# obstacles_polygon = []
	# for i in environment.Obstacles:

	# 	obstacles_polygon.append(patches.Polygon(environment.Obstacles.get(i), closed=True, color='black'))
	# 	ax1.add_patch(obstacles_polygon[-1])

		car = robot.TriangleShape((0, 0, 0))

		cars.append(PolygonPatch(car, fc=colors[i], alpha=1.0, zorder=2))
		circles.append(plt.Circle((robot.path_[0][0], robot.path_[0][1]), robot.radius_ , fc=colors[i], alpha=0.7))

	# Funciones para animacion
	def init():

		ax1.add_patch(user_patch)
		ax1.add_patch(user_head)

		for i in range(len(robots)):
			ax1.add_patch(cars[i])
			ax1.add_patch(circles[i])

		return []

	def animate(index):

		print(index)
		# input()

		# Posicion de usuario
		# theta = user.path_[index][2] - (np.pi/2)
		theta = user.path_[index][2]
		x = user.path_[index][0]
		y = user.path_[index][1]

		r = mpl.transforms.Affine2D().rotate(theta)
		t = mpl.transforms.Affine2D().translate(x, y)
		tra = r + t + ax1.transData

		user_patch.set_transform(tra)
		user_head.center = (user.path_[index][0], user.path_[index][1])
		user_text.set_position((user.path_[index][0]+0.2, user.path_[index][1]))

		# Posicion de robots
		for i, robot in enumerate(robots):

			this_x = robot.path_[index][0]
			this_y = robot.path_[index][1]

			if index == 0:

				histories_x[i].clear()
				histories_y[i].clear()

			histories_x[i].appendleft(this_x)
			histories_y[i].appendleft(this_y)

			trace[i].set_data(histories_x[i], histories_y[i])
			texts[i].set_position((robot.path_[index][0]+side, robot.path_[index][1]))

			if robot.leader_[index]:
				leader_text.set_position((robot.path_[index][0]+side-0.2, robot.path_[index][1]+side+0.1))

			# Transformacion de poligono
			theta = robot.path_[index][2]
			# theta = robot.path_[index][2] - robot.path_[index-1][2]
			x = robot.path_[index][0]
			y = robot.path_[index][1]

			r = mpl.transforms.Affine2D().rotate(theta)
			t = mpl.transforms.Affine2D().translate(x, y)
			tra = r + t + ax1.transData

			cars[i].set_transform(tra)		
			circles[i].center = (robot.path_[index][0], robot.path_[index][1])
		
		return cars, circles, trace, user_patch, user_patch

	ani = animation.FuncAnimation(fig1, animate, init_func=init, frames=frames, interval=animated_time)
	ani.save('Simulation_dif_8.mp4', fps=9)
	plt.show()

# Visualizar el ultimo estado del usuario y los robots
def visualize_state(robots, user):

	print('Graficnado...')

	colors = cmx.rainbow(np.linspace(0, 1, len(robots)))
	side = 0.4

	fig, ax = plt.subplots()
	ax.set_title('Mostrando estado actual del usuario y robots')
	ax.set_xlim(user.roomLimits_[0][0], user.roomLimits_[0][1])
	ax.set_ylim(user.roomLimits_[1][0], user.roomLimits_[1][1])

	# obstacles_polygon = []

	# for i in environment.Obstacles:

	# 	obstacles_polygon.append(patches.Polygon(environment.Obstacles.get(i), closed=True, color='black'))
	# 	ax.add_patch(obstacles_polygon[-1])

	# Agregando al usuario a la grafica
	user_shape = user.Shape(user.path_[-1])
	user_patch = PolygonPatch(user_shape, fc='orange', alpha=0.5, zorder=2)
	user_head = plt.Circle((user.path_[-1][0], user.path_[-1][1]), 0.2, fc='orange', alpha=5)

	ax.add_patch(user_head)
	ax.add_patch(user_patch)
	ax.text(user.path_[-1][0]+0.2, user.path_[-1][1], 'User', fontsize=10, zorder=3)

	# Agregando los robots a la grafica
	for robotID, robot in enumerate(robots):

		aux_triangle = robot.TriangleShape(robot.path_[-1])
		aux_patch = PolygonPatch(aux_triangle, fc=colors[robotID], alpha=1, zorder=2)
		aux_circle = plt.Circle((robot.path_[-1][0], robot.path_[-1][1]), robot.radius_, fc=colors[robotID], alpha=0.7)

		ax.add_patch(aux_patch)
		ax.add_patch(aux_circle)

		# Graficando ID de Robot
		ax.text(robot.path_[-1][0]+side, robot.path_[-1][1], 'ID:%s'%robotID,
				fontsize=10, fontweight='bold', zorder=3)

		if robot.leader_[-1]:
			ax.text(robot.path_[-1][0]+side-0.2, robot.path_[-1][1]+side+0.1, 'Leader',
				fontsize=10, zorder=3)

	# Graficar arbol del robot
	# for point in robots[2].states_tree_.keys():

	# 	x = point[0] + 0.01*np.cos(point[2])
	# 	y = point[1] + 0.01*np.sin(point[2])
	# 	ax.plot([point[0], x], [point[1], y], color='black', alpha=alph)
	# 	ax.plot(x, y, '.', color='black', alpha=alph)

	for robot in robots:

		for pos in robot.path_:
			ax.plot(pos[0], pos[1], '.', color='red')

	for point in user.path_:
		ax.plot(point[0], point[1], '.', color='orange')

	plt.show()