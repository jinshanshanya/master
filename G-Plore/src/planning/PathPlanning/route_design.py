# license @easydrive

import pandas as pd
import numpy as np
import math
from math import cos, sin, sqrt, log, tan, atan, atan2, floor


k = 4 # k can varying from 2 to 10;
Lf = 1 # Forward looking distance
look_distance = 6  # Preview distance, K * V
pi = np.pi
setsize = 400 # the unit length is 2cm, the setsize is averagely 8m from real wheel where the GPS-RTK locates; the parameter strongly affect the pure pursuit system performance; 
# this number should not be less than 200;


def route(current_x, current_y, waypoint, start_point):
    print(start_point)
    if start_point is None:#initialization to find the starting point
        distance_matrix = waypoint[:, [0, 1]] - np.tile([current_x, current_y], (len(waypoint), 1))
	distance_norm = np.sqrt(np.diag(np.dot(distance_matrix, distance_matrix.T)))
        perm = np.argsort(distance_norm)
        start_point = np.min(perm[0:1])
        LateralPoint = waypoint[start_point:start_point + setsize, [0, 1, 2]]
	#print(1111111)
    else: # update reference points that are closest to current position
        data = waypoint[start_point:start_point + setsize, [0, 1]]
        distance_matrix = data - np.tile([current_x, current_y], (len(data), 1))
        distance_norm = np.sqrt(np.diag(np.dot(distance_matrix, distance_matrix.T)))
        perm = np.argsort(distance_norm)
        start_point_1 = np.min(perm[0:1])
        #print("start_point_1", start_point_1)
        start_point = start_point + start_point_1
        print("start_point", start_point)
        LateralPoint = waypoint[start_point:start_point + setsize, [0, 1, 2]]
    	#print(2222222)
    return LateralPoint, start_point # lateral point: in each circle, we look forward in setsize length, and the lateral points are included in this length;


def LateralPoint_output(current_x, current_y, lateralpoint, start_point, waypoint):
    # print(current_x, current_y)
    # print(lateralpoint)
   dot_angle = np.dot([lateralpoint[1, 0] - lateralpoint[0, 0], lateralpoint[1, 1] - lateralpoint[0, 1]],
                       [lateralpoint[0, 0] - current_x, lateralpoint[0, 1] - current_y])
    # print(dot_angle)
    if dot_angle >= 0:
        i_start = 0
        if start_point == 0:
            LateralPoint_output = waypoint[start_point:start_point + setsize, [0, 1]]
            # start_point = start_point + setsize
        else:
            LateralPoint_output = waypoint[start_point - 1:start_point - 1 + setsize, [0, 1]]
            # start_point = start_point + setsize - 1
    else:
        i_start = 1
        LateralPoint_output = waypoint[start_point:start_point + setsize, [0, 1]]
        # start_point = start_point + setsize
    return i_start, LateralPoint_output # comfirm the inserting angle;


def find_rog(x, y, heading_current):
    L = sqrt(x ** 2 + y ** 2)
    xt = 1
    yt = 1
    if x == 0 and y == 0:
        xt = 0
        yt = 0
        return xt, yt

    if (x > 0 and y > 0) or ((x == 0 and y > 0)):
        alfa = atan(x / y)
        gamma = alfa - heading_current / 180 * pi + pi
        xt = L * sin(gamma)
        yt = L * cos(gamma)
        return xt, yt

    if (x > 0 and y < 0) or (x > 0 and y == 0):
        alfa = -(atan(y / x)) + pi / 2
        gamma = -alfa + heading_current / 180 * pi + pi / 2
        xt = L * -cos(gamma)
        yt = L * -sin(gamma)
        return xt, yt

    if (x < 0 and y < 0) or (x == 0 and y < 0):
        alfa = pi + atan(x / y)
        gamma = alfa - heading_current / 180 * pi + pi
        xt = L * sin(gamma)
        yt = L * cos(gamma)
        return xt, yt

    if (x < 0 and y > 0) or (x < 0 and y == 0):
        alfa = 3 / 2 * pi - atan(y / x)
        gamma = -alfa + heading_current / 180 * pi + pi / 2
        xt = L * -cos(gamma)
        yt = L * -sin(gamma)
        return xt, yt


def coordinate_change(current_x, current_y, heading, LateralPoint_output):
    xt, yt = find_rog(current_x, current_y, heading)
    T = np.array([[xt], [yt]])
    R = np.array([[cos(heading), -sin(heading)], [sin(heading), cos(heading)]])
    trans1 = np.hstack((R, T))
    trans = np.vstack((trans1, np.array([0, 0, 1])))
    later_size = len(LateralPoint_output)
    point = np.hstack((LateralPoint_output[:, [0, 1]], np.tile([1], (later_size, 1))))
    data = np.dot(point, trans)
    vehicle_data = data[:, [0, 1]]
    # print(trans)
    # -------------------------
    # global start_point
    # heading = heading * pi / 180
    # cnb = np.array([[cos(heading), sin(heading)], [-sin(heading), cos(heading)]])
    # vehicle_data = (LateralPoint_output[:, [0, 1]] - np.tile([current_x, current_y], (setsize, 1))) @ cnb

    # print(vehicle_data)
    return vehicle_data


def breakthrough(current_x, current_y, i_start, lateralpoint):# to find the inserting points:
    # print(i_start, current_x, current_y)
    global look_distance
    end = setsize  # end = setsize-1
    if end > len(lateralpoint):
       end = len(lateralpoint)
    for i in range(i_start, end):  # end = end-1
        s1 = sqrt((lateralpoint[i, 0] - current_x) ** 2 + (lateralpoint[i, 1] - current_y) ** 2)
        s2 = sqrt((lateralpoint[i + 1, 0] - current_x) ** 2 + (lateralpoint[i + 1, 1] - current_y) ** 2)
        # print(s1, s2)
        if sqrt((lateralpoint[i_start, 0] - current_x) ** 2 + (lateralpoint[i + 1, 1] - current_y) ** 2) \
                > look_distance:
            x_ref = lateralpoint[i_start+1, 0]
            y_ref = lateralpoint[i_start+1, 1]
 	    heading_ref = lateralpoint[i_start+1,2]
            look_distance = sqrt(
                (lateralpoint[i_start, 0] - current_x) ** 2 + (lateralpoint[i_start, 1] - current_y) ** 2)
            # print(x_ref, y_ref, look_distance)
            return x_ref, y_ref, heading_ref, look_distance
        elif sqrt((lateralpoint[end-1, 0] - current_x) ** 2 + (lateralpoint[end-1, 1] - current_y) ** 2) < look_distance:
            x_ref = lateralpoint[end - 1, 0]
            y_ref = lateralpoint[end - 1, 1]
	    heading_ref = lateralpoint[end - 1,2]
            look_distance = sqrt(
                (lateralpoint[end - 1, 0] - current_x) ** 2 + (lateralpoint[end - 1, 1] - current_y) ** 2)
            # print(x_ref, y_ref, look_distance)
            return x_ref, y_ref, heading_ref, look_distance
        elif s1 < look_distance and s2 > look_distance:
            s_x = (lateralpoint[i + 1, 0] - lateralpoint[i, 0]) / 10
            s_y = (lateralpoint[i + 1, 1] - lateralpoint[i, 1]) / 10
            for j in range(10):
                line_x = lateralpoint[i, 0] + s_x * (j - 1)
                line_x1 = lateralpoint[i, 0] + s_x * (j)

                line_y = lateralpoint[i, 1] + s_y * (j - 1)
                line_y1 = lateralpoint[i, 1] + s_y * (j)
		heading_ref = lateralpoint[i, 2]
                s1 = sqrt((line_x - current_x) ** 2 + (line_y - current_y) ** 2)
                s2 = sqrt((line_x1 - current_x) ** 2 + (line_y1 - current_y) ** 2)
                if s1 < look_distance and s2 > look_distance:
                    x_ref = line_x
                    y_ref = line_y
                else:
                    x_ref = lateralpoint[2, 0]
                    y_ref = lateralpoint[2, 1]
	            heading_ref = lateralpoint[2, 2]
                return x_ref, y_ref, heading_ref, look_distance


def plus_minus(x_ref, y_ref, current_x, current_y, heading, waypoint, start_point):# remember to modify the function;
    ref_angle = math.atan2((y_ref - current_y), (x_ref - current_x))
    if ref_angle > 0.75 * pi and heading < -0.75 * pi:
        diff_angle = ref_angle - (heading + 2 * pi)
    elif ref_angle < -0.75 * pi and heading > 0.75 * pi:
        diff_angle = ref_angle - (heading - 2 * pi)
    else:
        diff_angle = ref_angle - heading

    if diff_angle > pi:
        diff_angle = diff_angle - 2 * pi
    if diff_angle < -pi:
        diff_angle = diff_angle + 2 * pi

    yaw_path = waypoint[start_point, 2]
    if ref_angle > 0.75 * pi and yaw_path < -0.75 * pi:
        diff_angle_1 = ref_angle - (yaw_path + 2 * pi)
    elif ref_angle < -0.75 * pi and yaw_path > 0.75 * pi:
        diff_angle_1 = ref_angle - (yaw_path - 2 * pi)
    else:
        diff_angle_1 = ref_angle - yaw_path

    if diff_angle_1 > pi:
        diff_angle_1 = diff_angle_1 - 2 * pi
    if diff_angle_1 < -pi:
        diff_angle_1 = diff_angle_1 + 2 * pi

    return diff_angle, diff_angle_1


def CalHeading(heading_desire, heading_current):
    index_store = 0
    index_current = 0
    index_store = floor(heading_desire/90)
    index_current = floor(heading_current /90)
    delta_heading = 0

    if abs(index_store - index_current) == 0:
	delta_heading =  heading_desire - heading_current
    elif abs(index_store - index_current) == 1:
	delta_heading =  heading_desire - heading_current
    elif index_store ==0 and index_current ==3:
	delta_heading =  heading_desire - heading_current + 360
    elif index_store ==3 and index_current ==0:
	delta_heading =  heading_desire - heading_current - 360
    return delta_heading

    if  abs(index_store - index_current) == 2:
        if index_store ==0 and index_current ==2:
            if abs(heading_desire - heading_current)> 180:
                delta_heading = - (heading_desire + 360 - heading_current)
            else:
        	delta_heading = heading_current - heading_desire
   
            return delta_heading

        if index_store ==2 and index_current ==0:
	    if abs(heading_desire- heading_current) > 180:
		delta_heading = 360 + heading_current - heading_desire
	    else:
		delta_heading = heading_current - heading_desire
   
            return delta_heading


	if index_store ==1 and index_current ==3:
	    if abs(heading_desire- heading_current) > 180:
		delta_heading = - (heading_desire + 360 - heading_current)
	    else:
		delta_heading = heading_current - heading_desire
	   
	    return delta_heading


	if index_store ==3 and index_current ==1:
	     if abs(heading_desire- heading_current) > 180:
		delta_heading = heading_current + 360 - heading_desire
	     else:
		delta_heading = -(heading_desire - heading_current)
	    
	     return delta_heading


def transformation(lat, lon, B0, L0):
    B = lat * pi / 180
    L = lon * pi / 180
    B0 = B0 * pi / 180
    L0 = L0 * pi / 180
    a = 6378137
    b = 6356752.3142
    e = math.sqrt((1 - (b / a) ** 2))
    ee = sqrt(((a / b) ** 2 - 1))
    K = ((a ** 2 * cos(B0)) / b) / sqrt(1 + (ee) ** 2 * (cos(B0)) ** 2)  # constant
    X = K * (L - L0)
    Y0 = K * log(tan(pi / 4 + B0 / 2) * ((1 - e * sin(B0)) / (1 + e * sin(B0))) ** (e / 2))
    Y = K * log(tan(pi / 4 + B / 2) * ((1 - e * sin(B)) / (1 + e * sin(B))) ** (e / 2)) - Y0
    return X, Y

# lateralpoint = route_design(current_x, current_y, waypoint)
# i_start, LateralPoint_output = LateralPoint_output(current_x, current_y, lateralpoint)
# # print(LateralPoint_output)
# trans_point = coordinate_change(current_x, current_y, heading, LateralPoint_output)
# print(trans_point)
# x_ref, y_ref, look_distance = breakthrough(current_x, current_y, i_start, lateralpoint)
# diff_angle, diff_angle_1 = plus_minus(x_ref, y_ref, current_x, current_y, heading)
#
# LateralPoint_x_out = trans_point[0:setsize, 0]]
# LateralPoint_y_out = trans_point[0:setsize, 1]]
# compute_angle = 2 * sin(diff_angle) / look_distance
# compute_yaw = atan2((y_ref - current_y), (x_ref - current_x)) * 57.3  #
# compute_angle1 = 2 * sin(diff_angle_1) / look_distance
# see_data = diff_angle * 57.3


# way_path_x = waypoint[:, 0]
# way_path_y = waypoint[:, 1]
# lateral_x = LateralPoint_output[:, 0]
# lateral_y = LateralPoint_output[:, 1]
# trans_point_x = trans_point[:, 0]
# trans_point_y = trans_point[:, 1]
# plt.plot(way_path_x, way_path_y, 'r')
# plt.plot(lateral_x, lateral_y, 'b')
# plt.plot(trans_point_x, trans_point_y, 'g')
# plt.plot(x_ref, y_ref, 'r')
# plt.scatter(trans_point_x, trans_point_y)
# plt.show()
