#!/usr/bin/env python
#from transitions import Machine
#import time
#import Queue
#import matplotlib.pyplot as plt
#import random
import numpy as np
import rospy
from VehicleMsg.msg import adm_lat  # import end
# from parkingMsg.msg import parking_data#import planning_control
# from DCMsg.msg import DC_data#import DC_command
from VehicleMsg.msg import vehicle_upload  #import upload_complete
# from VehicleMsg.msg import dc  #import upload_complete
from BehaviorDecisionfunction import *
from StatemachineMsg.msg import statemachine
from DispatchMsg.msg import dispatch
from VehicleMsg.msg import cdm_cmd
from VehicleMsg.msg import vehicle_upload  #import upload_complete
from std_msgs.msg import UInt8
# from VehicleMsg.msg import dc #import upload_complete

DC_command = 0
end = 0
velocity = 0
planning_control = 0
route = 0
ADM_command = 0
Compartment_DownLowest = 0
charge_stop = 0

pub = rospy.Publisher('Behavior', statemachine, queue_size=100)


def CommandNode():
    """
    set a python node
    """
    rospy.Subscriber("AdmLatMsg_1", adm_lat, AdmCallback)
    rospy.Subscriber("upload_status", vehicle_upload, vehuploadCallback)
    rospy.Subscriber("dispatchNodes", dispatch, DCback)
    rospy.Subscriber("CdmCmdMsg", cdm_cmd, CdmCallback)
    rospy.Subscriber("/target_controller/target_control_state", UInt8, parkingback)
    rospy.init_node('BehaviorDecision', anonymous=True)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        command()
        r.sleep()


def vehuploadCallback(data):
    """
    this is a callback function of upload_status
    """
    global upload_complete
    upload_complete = data.upload_complete
    print("call back upload_complete is:", upload_complete)


def CdmCallback(data):
    """
    this is a callback function of cdm
    """
    global velocity
    global Compartment_DownLowest
    velocity = data.Veh_Spd
    if data.Compartment_Fd == 4:
        Compartment_DownLowest = 1
    print("call back velocity is:", velocity)


def DCback(data):
    """
    this is a callback function of dispatch
    """
    global DC_command
    DC_command = data.DC_command
    print('call back data DC_command is', DC_command)


def AdmCallback(data):
    """
    this is a callback function of adm
    """
    global end
    # global velocity
    end = data.End
    #v_des=data.v_des
    # velocity = data.Vehicle_speed
    print('call back data end is', end)


def parkingback(data):
    """
    this is a callback function of parking
    """
    global planning_control
    global charge_stop
    if data.data == 4:
        charge_stop = 1
    if data.data == 6:
        planning_control = 1


def command():
    """
    Call execution function
    """
    global DC_command
    global end
    global route_list
    global ADM_command
    global route
    #global v_des
    # global velocity
    global parking_control
    # DC_command=int(input('DC_command='))
    # end=int(input('end='))
    #v_des=int(input('v_des='))
    # velocity=int(input('velocity='))
    DiveroffReq = get_diveroffreq(ADM_command, DC_command, velocity,
                                  Compartment_DownLowest, planning_control)
    print("DriveroffReq is", DiveroffReq)
    vehicle = CommandState(ADM_command, DC_command, route, end, DiveroffReq)
    route_list = vehicle.add_route()
    # planning_control=int(input('planning_control='))
    list = KillRoute(route_list, planning_control)
    route_list = list.kill(end, planning_control)
    route = vehicle.get_info()
    ADM_command = vehicle.get_ADM_command()
    Signal = SignalOutput(end, route, charge_stop, Compartment_DownLowest, velocity, planning_control)
    parking_control, ADM_SubtaskState, dischargeReq = Signal.signal()
    print("ADM_command and current route", (ADM_command, route))
    print("parking_control, ADM_SubtaskState",
          (parking_control, ADM_SubtaskState))
    msg = statemachine()
    msg.route = route
    msg.ADM_command = ADM_command
    msg.parking_control = parking_control
    msg.planning_control = planning_control
    msg.DC_command = DC_command
    msg.DiveroffReq = DiveroffReq
    msg.ADM_SubtaskState = ADM_SubtaskState
    msg.dischargeReq = dischargeReq
    pub.publish(msg)


if __name__ == '__main__':
    try:
        CommandNode()
    except rospy.ROSInterruptException:
        pass
    except SyntaxError as e:
        print(e)
