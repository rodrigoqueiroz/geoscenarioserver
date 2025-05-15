import numpy as np

from math import cos, radians, sin

def calculate_rectangular_bounding_box(actor):
		center_x     = actor.state.x
		center_y     = actor.state.y
		half_length  = actor.length / 2
		half_width   = actor.width  / 2
		yaw          = -actor.state.yaw

		back_left   = rotate(center_x, center_y, center_x - half_length, center_y + half_width, yaw)
		back_right  = rotate(center_x, center_y, center_x - half_length, center_y - half_width, yaw)
		front_left  = rotate(center_x, center_y, center_x + half_length, center_y + half_width, yaw)
		front_right = rotate(center_x, center_y, center_x + half_length, center_y - half_width, yaw)

		return np.array([front_left, front_right, back_right, back_left])

def rotate(center_x, center_y, x, y, degree_theta):
		radian_theta = radians(degree_theta)
		x2 = center_x + (x - center_x) * cos(radian_theta) + (y - center_y) * sin(radian_theta)
		y2 = center_y - (x - center_x) * sin(radian_theta) + (y - center_y) * cos(radian_theta)

		return np.array([ x2, y2 ])