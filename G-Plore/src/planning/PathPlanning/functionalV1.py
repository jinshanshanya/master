# license @Global Carrier Strike Group

import pandas as pd
import numpy as np
import math
from math import cos, sin, sqrt, log, tan, atan, atan2, floor
k = 6  # k can varying from 2 to 10; 6
Lf = 4.5  # Forward looking distance
look_distance = 2 * k / 3.6 + Lf  # Preview distance, K * V
pi = np.pi
setsize = 200  # the unit length is 2cm, the setsize is averagely 8m from real wheel where the GPS-RTK locates; the parameter strongly affect the pure pursuit system performance;
# this number should not be less than 200;
path_waypoint = pd.read_csv("test1227.csv")
#waypoint = path_waypoint.loc[:, ["x", "y", "heading"]].values.astype(float)
waypoint = path_waypoint.loc[:, ["x", "y", "heading"]].values.astype(int)
way = pd.read_csv("waypoint1227.csv")
#way2 = way.loc[:, ["x", "y", "heading"]].values.astype(float)
way2 = way.loc[:, ["x", "y", "heading"]].values.astype(int)
waypoint_x = waypoint[:, 0]
waypoint_y = waypoint[:, 1]
waypoint_heading = waypoint[:, 2]

p = np.empty(shape=(0, 2))  # initialization bezier


def ind_route_generation(current_x, current_y ,current_ind):
    global waypoint
    if current_ind is None:  # initializing to find the starting point

        distance_matrix = waypoint[:, [0, 1]] - np.tile([current_x, current_y], (len(waypoint), 1))
        distance_norm = np.sqrt(np.diag(np.dot(distance_matrix, distance_matrix.T)))
        perm = np.argsort(distance_norm)
        current_ind = np.min(perm[0:2])
        lateral_offset = np.min(distance_norm)
        look_forward_point = waypoint[current_ind: current_ind + setsize, [0, 1, 2]]
    else:  # update reference points that are closest to current position
        prediction_data = waypoint[current_ind: current_ind + setsize, [0, 1]]
        distance_matrix = prediction_data - np.tile([current_x, current_y], (len(prediction_data), 1))
        distance_norm = np.sqrt(np.diag(np.dot(distance_matrix, distance_matrix.T)))
        perm = np.argsort(distance_norm)
        start_point = np.min(perm[0:2])
        current_ind = current_ind + start_point
        lateral_offset = np.min(distance_norm)
        look_forward_point = waypoint[current_ind: current_ind + setsize, [0, 1, 2]]
	print("lateral_offset:",lateral_offset)
	print("current_ind:",current_ind)
	#print("waypoint:",waypoint)

    return look_forward_point, current_ind, lateral_offset  # lateral point: in each circle, we look forward in setsize length, and the lateral points are included in this length;


def breakthrough(current_x, current_y, look_forward_point, lateral_offset):  # to find the inserting points:

    current_x = 0
    current_y = 0

    if abs(lateral_offset) >= 2:

        print('too far from reference line!!!')
        x_ref = 0
        y_ref = 1
    else:
        s = 0
        i = 0
        COND = 1
        while COND == 1: # We can only search in the range of look_forward_point
            s = sqrt((look_forward_point[i, 0] - current_x) ** 2 + (look_forward_point[i, 1] - current_y) ** 2)
            i = i + 1
            if s >= look_distance :
                COND = 2

            if i == len(look_forward_point): # when reach the waypoint end, we use the last point
                COND =3
        # end while
        x_ref = look_forward_point[i - 1, 0]
        y_ref = look_forward_point[i - 1, 1]
        print("x_ref,y_ref",x_ref,y_ref)

    return x_ref, y_ref


def find_rog(x, y, heading_current):  # this function give T matrix transforming from global to huace
    heading_current = heading_current * pi / 180
    L = sqrt(x ** 2 + y ** 2)
    xt = 1
    yt = 1
    if x == 0 and y == 0:
        xt = 0
        yt = 0
        return xt, yt

    if (x > 0 and y > 0) or ((x == 0 and y > 0)):
        alfa = atan(x / y)
        gamma = alfa - heading_current + pi
        xt = L * sin(gamma)
        yt = L * cos(gamma)
        return xt, yt

    if (x > 0 and y < 0) or (x > 0 and y == 0):
        alfa = -(atan(y / x)) + pi / 2
        gamma = -alfa + heading_current + pi / 2
        xt = L * -cos(gamma)
        yt = L * -sin(gamma)
        return xt, yt

    if (x < 0 and y < 0) or (x == 0 and y < 0):
        alfa = pi + atan(x / y)
        gamma = alfa - heading_current + pi
        xt = L * sin(gamma)
        yt = L * cos(gamma)
        return xt, yt

    if (x < 0 and y > 0) or (x < 0 and y == 0):
        alfa = 1.5 * pi - atan(y / x)
        gamma = -alfa + heading_current + pi / 2
        xt = L * -cos(gamma)
        yt = L * -sin(gamma)
        return xt, yt


def global_to_frenet(current_x, current_y, heading, point):
    xt, yt = find_rog(current_x, current_y, heading)
    T = np.array([[xt], [yt]])
    heading = heading * pi / 180
    R = np.array([[cos(heading), -sin(heading)], [sin(heading), cos(heading)]])
    trans1 = np.hstack((R, T))
    trans2 = np.vstack((trans1, np.array([0, 0, 1])))
    # print(trans2)
    # zr = np.array([[cos(pi / 2), -sin(pi / 2), 0], [sin(pi / 2), cos(pi / 2), 0], [0, 0, 1]])  # mapu
    # yr = np.array([[cos(pi), 0, sin(pi)], [0, 1, 0], [-sin(pi), 0, cos(pi)]])
    #
    # zr = np.array([[cos(pi / 2), -sin(pi / 2), 0], [sin(pi / 2), cos(pi / 2), 0], [0, 0, 1]])
    # print(trans)
    later_size = len(point)
    point = np.hstack((point[:, [0, 1]], np.tile([1], (later_size, 1))))
    data = np.dot(trans2, point.T)
    # data = np.hstack((data[:, [0, 1]], np.tile([0], (len(data), 1))))
    # trans = np.dot(yr, np.dot(zr, data)).T  # mapu
    # trans = np.dot(zr, data).T
    # vehicle_data = trans[:, [0, 1]]
    vehicle_data = data.T
    # print(vehicle_data)
    return vehicle_data[:, [0, 1]]  # this func returns a n by 2 matrix


def CalHeading(heading_desire, heading_current):  # this function provide a delta_heading calculation in Stanley model
    index_store = 0
    index_current = 0
    index_store = floor(heading_desire / 90)
    index_current = floor(heading_current / 90)
    delta_heading = 0

    if abs(index_store - index_current) == 2:
        if index_store == 0 and index_current == 2:
            if abs(heading_desire - heading_current) > 180:
                delta_heading = - (heading_desire + 360 - heading_current)
            else:
                delta_heading = heading_current - heading_desire

            return delta_heading

        if index_store == 2 and index_current == 0:
            if abs(heading_desire - heading_current) > 180:
                delta_heading = 360 + heading_current - heading_desire
            else:
                delta_heading = heading_current - heading_desire

            return delta_heading

        if index_store == 1 and index_current == 3:
            if abs(heading_desire - heading_current) > 180:
                delta_heading = - (heading_desire + 360 - heading_current)
            else:
                delta_heading = heading_current - heading_desire

            return delta_heading

        if index_store == 3 and index_current == 1:
            if abs(heading_desire - heading_current) > 180:
                delta_heading = heading_current + 360 - heading_desire
            else:
                delta_heading = -(heading_desire - heading_current)

            return delta_heading


def transformation(lat, lon, B0,L0):  # this function transform earch coord to Cartisian coord with initial Lat and Lon at starting point
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
    #print('x and y is ',X,Y)
    return X, Y

def avoid_design(x, y,L):  # here x is obstacle distance 'd', and y, which is used in point_bezier function
    avoid_point = np.array([[x - x, x + x], [y, y]])

    p = point_bezier(avoid_point,L)

    return p


def point_bezier(avoid_point,L):  # genrate Beizer curve in Frenet coord
    global p
    #L = 2.0  # add a line to expand Bezier curve
    LL = np.array([L, 0])
    Latoff = 0.9  # lateral offset

    xs = avoid_point[0, 0]  # 0
    ys = avoid_point[1, 0]  # 0
    xe = avoid_point[0, -1]  # 34.5844
    ye = avoid_point[1, -1]  # 0

    startp = np.array([xs, ys])
    endp = np.array([xe, ye])

    # print(startp, endp)
    P0 = startp
    P1 = np.array([startp[0] + (endp[0] - startp[0]) / 8, startp[1]])
    P2 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 1, startp[1]])
    P3 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 2, startp[1] + Latoff])
    P4 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 3, startp[1] + Latoff])
    P5 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 4, startp[1] + Latoff])
    P6 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 4 + L, startp[1] + Latoff])
    P7 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 5 + L, startp[1] + Latoff])
    P8 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 6 + L, startp[1] + Latoff])
    P9 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 7 + L, startp[1]])
    P10 = np.array([startp[0] + (endp[0] - startp[0]) / 8 * 7 + L, startp[1]])
    P11 = endp + LL
    i = 1
    half_length = 0.5 * (xe + xs)
    for u in np.arange(startp[0], startp[0] + (endp[0] - startp[0]) / 2 + 0.4, 0.4):
        # for u =startp[0]:0.2: startp[1] + (endp[1] - startp[1]) / 2
        c = (1 - (u - startp[0]) / half_length) ** 5 * P0 + 5 * (1 - (u - startp[0]) / half_length) ** 4 * (
                u - startp[0]) / half_length * P1 + 10 * (1 - (u - startp[0]) / half_length) ** 3 * (
                    (u - startp[0]) / half_length) ** 2 * P2 + 10 * (1 - (u - startp[0]) / half_length) ** 2 * (
                    (u - startp[0]) / half_length) ** 3 * P3 + 5 * (1 - (u - startp[0]) / half_length) * (
                    (u - startp[0]) / half_length) ** 4 * P4 + ((u - startp[0]) / half_length) ** 5 * P5
        i = i + 1
        p = np.append(p, [c], axis=0)

    for u in np.arange(startp[0] + half_length, endp[0], 0.4):
        d = (1 - (u - startp[0] - half_length) / half_length) ** 5 * P6 + 5 * (
                1 - (u - startp[0] - half_length) / half_length) ** 4 * (
                    u - startp[0] - half_length) / half_length * P7 + 10 * (
                    1 - (u - startp[0] - half_length) / half_length) ** 3 * (
                    (u - startp[0] - half_length) / half_length) ** 2 * P8 + 10 * (
                    1 - (u - startp[0] - half_length) / half_length) ** 2 * (
                    (u - startp[0] - half_length) / half_length) ** 3 * P9 + 5 * (
                    1 - (u - startp[0] - half_length) / half_length) * (
                    (u - startp[0] - half_length) / half_length) ** 4 * P10 + (
                    (u - startp[0] - half_length) / half_length) ** 5 * P11
        i = i + 1
        p = np.append(p, [d], axis=0)
    return p  # generate n by 2 array
    # print(p)
    # plt.plot(p[:, 0], p[:, 1], 'r')
    # plt.show()


def frenet_to_global(current_x, current_y, heading, path_delta_heading, a):  # transform from frenet coord to global coord
    T = np.array([[current_x], [current_y]])
    heading = -heading * pi / 180
    # print(heading)
    R = np.array([[cos(heading), -sin(heading)], [sin(heading), cos(heading)]])
    trans1 = np.hstack((R, T))
    rotation2 = np.vstack((trans1, np.array([0, 0, 1])))
    T = np.array([[0], [0]])
    theta = path_delta_heading
    R = np.array([[cos(pi / 2 - theta), -sin(pi / 2 - theta)], [sin(pi / 2 - theta), cos(pi / 2 - theta)]])
    trans1 = np.hstack((R, T))
    rotation1 = np.vstack((trans1, np.array([0, 0, 1])))
    a = np.hstack((a[:, [0, 1]], np.ones((len(a), 1)))).T
    trans = np.dot(rotation2, np.dot(rotation1, a)).T
    trans = trans[:, [0, 1]]
    return trans  # return a n by 2 transfomation matrix


def camera_to_global(current_x, current_y, heading, obstacle_point):
    T1 = np.array([[current_x], [current_y]])  # here T transforms from huace to global
    heading = -heading * pi / 180
    # print(heading)
    R = np.array([[cos(heading), -sin(heading)], [sin(heading), cos(heading)]])
    trans1 = np.hstack((R, T1))  # this is transformation matrix from huace to global
    rotation2 = np.vstack((trans1, np.array([0, 0, 1])))
    T2 = np.array([[3.5], [0]])  # this T transform from caemera to huace
    theta = 2 / 180 * pi
    R = np.array([[cos(pi / 2 + theta), -sin(pi / 2 + theta)], [sin(pi / 2 + theta), cos(pi / 2 + theta)]])
    trans2 = np.hstack((R, T2))
    rotation1 = np.vstack((trans2, np.array([0, 0, 1])))
    a = np.hstack((obstacle_point[:, [0, 1]], np.ones((1, 1)))).T
    trans = np.dot(rotation2, np.dot(rotation1, a)).T
    trans = trans[:, [0, 1]]
    return trans  # return a n by 2 transfomation matrix


def delta_heading_calibration(waypoint_x, waypoint_y, waypoint_heading, current_ind, i):  # this function guide the Bezier curve endpoint to the reference line
    if waypoint_heading[current_ind] >= 0 and waypoint_heading[current_ind] <= 90:
        theta = atan((waypoint_y[current_ind + i] - waypoint_y[current_ind]) / (
                waypoint_x[current_ind + i] - waypoint_x[current_ind]))
        path_delta_heading = -waypoint_heading[current_ind] / 180 * pi + pi / 2 - theta
    # second quadrant
    elif waypoint_heading[current_ind] > 90 and waypoint_heading[current_ind] <= 180:
        theta = atan((waypoint_y[current_ind + i] - waypoint_y[current_ind]) / (
                waypoint_x[current_ind + i] - waypoint_x[current_ind]))
        path_delta_heading = -waypoint_heading[current_ind] / 180 * pi + pi / 2 - theta
    # third quadrant
    elif waypoint_heading[current_ind] > 180 and waypoint_heading[current_ind] <= 270:
        theta = atan((waypoint_x[current_ind + i] - waypoint_x[current_ind]) / (
                waypoint_y[current_ind + i] - waypoint_y[current_ind]))
        path_delta_heading = theta - (waypoint_heading[current_ind] / 180 * pi - pi)
    # fouurth quadrant
    else:
        theta = atan((waypoint_y[current_ind + i] - waypoint_y[current_ind]) / (
                waypoint_x[current_ind + i] - waypoint_x[current_ind]))
        path_delta_heading = waypoint_heading[current_ind + i] / 180 * pi - waypoint_heading[current_ind] / 180 * pi

    return path_delta_heading

def control_v(d):
    if d >= 5 and d <= 10:
        v_des = 0.4 * d - 1
    elif d < 5:
        v_des = 0
    else:
        v_des = 3
    return v_des

def control_steering(x_ref, y_ref,vehicle_length):
    alpha = atan(x_ref / y_ref)                      #**********************************************-atan************************************
    wheel_angle = atan(2 * vehicle_length * sin(alpha) / (k * 2 / 3.6 + Lf)) * 57.3
    steering_wheel_angle = wheel_angle  # the ratio 15 is stil pending!
    steering_wheel_angle = max(-39, min(steering_wheel_angle, 39))
    return  steering_wheel_angle

def avoidance_judgment(current_ind,waypoint_x,waypoint_y,waypoint_heading,v_front,d,L):
    global waypoint
    if v_front <= 0.3 and d <= 8 and d > 6:
    #if  current_ind == 80 : 
        i = 1
        dist = 0
        while dist <= 2 * 8 + L:
            dist = sqrt((waypoint_x[current_ind + i] - waypoint_x[current_ind]) ** 2 + (
                    waypoint_y[current_ind + i] - waypoint_y[current_ind]) ** 2)
            i = i + 1
	print("i=",i)

        path_delta_heading = delta_heading_calibration(waypoint_x, waypoint_y, waypoint_heading, current_ind, i)

        avoid_point = avoid_design(8, 0,L)

        path = frenet_to_global(waypoint_x[current_ind], waypoint_y[current_ind], waypoint_heading[current_ind], path_delta_heading, avoid_point)
        path = np.hstack((path,np.ones((len(path),1))))
        x1 = path[0, [0, 1]]
        x2 = path[-1, [0, 1]]
        distance_matrix = waypoint[:, [0, 1]] - np.tile([x1[0], x1[1]], (len(waypoint), 1))
        distance_norm = np.sqrt(np.diag(np.dot(distance_matrix, distance_matrix.T)))
        perm = np.argsort(distance_norm)
        print('perm is',perm[0])
        distance_matrix = waypoint[:, [0, 1]] - np.tile([x2[0], x2[1]], (len(waypoint), 1))
        distance_norm = np.sqrt(np.diag(np.dot(distance_matrix, distance_matrix.T)))
        perm2 = np.argsort(distance_norm)
        print('perm2 is',perm2[0])
        if abs(waypoint[perm[0], 2] - waypoint[perm2[0] , 2]) <= 10:
            b = np.vstack((waypoint[:perm[0]], waypoint[perm2[0] :]))
            #print('size of b is',b.shape)
            waypoint = np.insert(b, perm[0], values=path, axis=0)
            flag = 1
            print("ChangeWaty_flag:",1)
	    return waypoint
	else:
	    return waypoint
    else:
	flag=2
	print("NoChangeWay_flag=",flag)
	return waypoint

            # end if
        # end while
    # end if
