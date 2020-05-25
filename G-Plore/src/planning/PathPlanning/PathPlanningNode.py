#!/usr/bin/env python
# rosrun DecisionControl PathPlanningNode.py
# @easydrive
import rospy
from InsMsg.msg import ins_p2
#from VehicleMsg.msg import vehicle_download
from VehicleMsg.msg import vehicle_upload
from VehicleMsg.msg import adm_cmd

# from vehicle_logging.msg import logging_msg
from route_design import *
from math import *
#from statemachine import StateMachine, State

from std_msgs.msg import String
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
# from scipy import signal
# right # the TarAcceler Kp should be tuned.
# setsize should be tuned.
# pure pursuit equation k should be tuned.
# ratio from wheel angle to compute angle should be tuned.
pi = np.pi
path_waypoint = pd.read_csv("c.csv")
waypoint = path_waypoint.loc[:, ["x", "y", "heading"]].values.astype(float)


N = len(path_waypoint)
count = 0
T = 0
Vehicle_length = 2.85
start_point = None
#delta_t = 0
v = 0.0
gear = 0
drive_mode = 0
x = list()
y = list()
B0 = 31.3028038738
#B1 = 31.3028037661
L0 = 121.291626427
# pub = rospy.Publisher('PathPlaningNoddPyTopic', vehicle_download, queue_size=100)
#pub = rospy.Publisher('vehcileDownloadMsg', vehicle_download, queue_size=100)

pub1 = rospy.Publisher('AdmCmdMsg', adm_cmd, queue_size=100)





def PathPlaningNode():
    rospy.Subscriber("insMsg", ins_p2, InsCallback)
    rospy.Subscriber("vehicleUploadMsg", vehicle_upload, VehCallback)
    # rospy.Subscriber("logging_msg", logging_msg, LogCallback)
    # pub = rospy.Publisher('PathPlaningNoddPyTopic', vehicle_download, q_ueue_size=10)
    # pub = rospy.Publisher('PathPlaningNoddPyTopic', vehicle_download, queue_size=100)

    rospy.init_node('PathPlanningNodePy', anonymous=True)
    r = rospy.Rate(50)  # 100hz

    # planAngle = 0
    while not rospy.is_shutdown():
        # msg = vehicle_download()
        # planAngle = planAngle + 1
        # msg.Tar_steerangle = planAngle
        # rospy.loginfo(msg)
        # pub.publish(msg)


        r.sleep()
    #plt.plot(x,y)
    #plt.show()


def FPFilter(signal): # not in use here
    b, a = signal.butter(7, 0.01, 'low')
    output = signal.filtfilt(b, a, signal)
    return output


def VehCallback(data):
    global v
    global gear
    global drive_mode
    v = data.Vehicle_speed
    gear = data.Tar_gearFeed
    drive_mode = data.Drive_Mode
    # print("Veh data:", v)


def InsCallback(data):
    global T
    global v
    global gear
    #global delta_t
    global start_point
    
    current_x = data.Lat
    current_y = data.Lon
    heading = data.Heading # remember here that the heading unit is in degree!!!
    #if current_x > B0 and current_x < B1:
	#delta_t = T
    #print('delta_t:', delta_t)
    current_x, current_y = transformation(current_x, current_y, B0, L0)
    print('ins_data', current_x, current_y, data.Heading)  # need
    print('vehicle v:', v)  # need
    
    print("start_point:", start_point)
    lateralpoint, next_start = route(current_x, current_y, waypoint, start_point)
    start_point = next_start
    i_start, lateralPoint_outputs = LateralPoint_output(current_x, current_y, lateralpoint, start_point, waypoint)
    trans_point = coordinate_change(current_x, current_y, heading, lateralPoint_outputs)# not in use!

    # norm_l = sqrt((current_x - waypoint[start_point, 0]) ** 2 + (current_y - waypoint[start_point, 1]) ** 2)
    # compute_angle = atan((waypoint[start_point, 2]/57.3-heading)*3.36/norm_l)*25
    #print("lateralpoint 1",lateralpoint[1, [0, 1]])

    x_ref, y_ref, heading_ref, look_distance = breakthrough(current_x, current_y, i_start, lateralpoint)

#the output of delta_heading is in unit of degree;
    diff_angle, diff_angle_1 = plus_minus(x_ref, y_ref, current_x, current_y, heading, waypoint, start_point) 
    delta_heading = CalHeading(heading_ref, heading)  # 
    x.append(x_ref)
    y.append(y_ref)
    
    
    #LateralPoint_x_out = trans_point[0:setsize, [0]]
    #LateralPoint_y_out = trans_point[0:setsize, [1]]
    
    compute_wheel = atan(2 * Vehicle_length * sin(delta_heading*pi/180) / (k * v / 3.6 + Lf)) * 57.3 
    compute_angle = 14.9 * compute_wheel + 8; # the ratio 15 is stil pending!
    compute_angle = max(-420,min(compute_angle,420))
    compute_angle2 = 2 * sin(diff_angle*pi/180) / look_distance * 14.9
    compute_yaw = atan2((y_ref - current_y), (x_ref - current_x))
    compute_angle3 = compute_angle + (compute_yaw - heading)  # for adjustment angle 

    # compute_angle1 = atan(2 * Vehicle_length * sin(diff_angle_1) / k * v + Lf) * 57.3
    
    print("delta_heading:%s,diff_angle:%s,com_yaw2:%s" % (delta_heading, diff_angle, compute_angle2))
    print('x_ref,y_ref,look_distent', x_ref, y_ref, look_distance)
    msg = vehicle_download()
    
    msg1 = adm_cmd()
    # vehicle machine state

    # if data is not None and drive_mode == 0:
    #     vehicle_state.power_up()
    # if drive_mode == 1 and gear == 1:
    #     vehicle_state.ready()
    # if drive_mode == 0 and gear == 1:
    #     vehicle_state.suspend()
    # if drive_mode == 1 and gear == 4:
    #     vehicle_state.start()
    # if driveoff == 1:
    #     vehicle_state.move()
    # if v == 0 and gear == 1:
    #     vehicle_state.static()

    # ------------------------------
    if T < 100:
        msg.Stoprequest = 0
        msg.Tar_gear = 1
        msg.TarAccler = 0  # 0
        msg.Tar_steerangle = 8
    if T >= 100 and T < 300:
        msg.Stoprequest = 0  # 1
        msg.Tar_gear = 1
        msg.TarAccler = 0  # 0
        msg.Tar_steerangle = 8
    if T >= 300 and T < 400:  # gear = 4
        msg.DiveoffReq = 0
        msg.Tar_gear = 4  # 4
        msg.TarAccler = 0  # 0
        msg.Tar_steerangle = 8
        msg.Stoprequest = 0  # 1
    if T >= 400:
        msg.Stoprequest = 0
        msg.DiveoffReq = 1
        msg.Tar_gear = 4  # 4
        # msg.TarAccler = 0.2  # 0
        # msg.Tar_steerangle = -50
        # if T >= 60:
        # msg.Tar_gear = 4
        # msg.Tar_gear = 1
        # msg.DiveoffReq = 1
        # msg.DiveoffReq = 0
        if T > 420:
            msg.DiveoffReq = 0
        msg.Tar_steerangle = round(compute_angle)  # for reduce error
        # msg.Tar_steerangle = FPFilter(compute_angle)
        msg.TarAccler = 0.04 * (10 - v)/3.6 # right # the TarAcceler Kp should be tuned.
        msg.ADS_mode = 3
        msg.Slope = 0

        if T >= 2300 and T < 2500:
            msg.TarAccler = -0.4  # -0.2
            msg.ADS_mode = 3
        
        if T >= 2300 and v == 0:
            msg.TarAccler = 0  # 0
            msg.DiveoffReq = 0
            msg.Stoprequest = 1
            
            msg.ADS_mode = 2
            msg.Tar_gear = 1

    rospy.loginfo(msg)
#    pub.publish(msg)
    
    pub1.publish(msg)
    

    T = T + 1
    print("T:", T)


if __name__ == '__main__':
    try:
        PathPlaningNode()
    except rospy.ROSInterruptException:
        pass
