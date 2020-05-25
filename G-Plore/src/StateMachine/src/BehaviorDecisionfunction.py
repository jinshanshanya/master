#from transitions import Machine
#import time
#import Queue
import random
import numpy as np
#import matplotlib.pyplot as plt
parking_control = 0
discharge_stop = 0
parking_completion = 0
ADM_SubtaskState = 0
dischargeReq = 0


class KillRoute(object):
    """
    in order to calculate the desired route
    """
    route_list = []
    planning_control = 0

    def __init__(self, route_list, planning_control):
        """
        init route_list, planning_control
        """
        self.route_list = route_list

    def kill(self, end,
             planning_control):  # in order to calculate the desired route
        global route_list
        if end == 1 or planning_control == 1:
            route_list = route_list[1:, ]
        return route_list


class CommandState(object):
    """
    ADM command, DC command
    """
    ADM_command = 0
    #ADM command value: 0: initial, 1:cahrge, 2: discharge, 3: refueling,4: parking
    route = 0
    DC_command = 0
    # DC command value: 0: initial, 1:cahrge, 2: discharge, 3: refueling,4: parking
    end = 0  #indicate arriving at the end of a route

    def __init__(self, ADM_command, DC_command, route, end, DiveroffReq):
        self.ADM_command = ADM_command
        self.DC_command = DC_command
        self.route = route
        self.end = end
        self.DiveroffReq = DiveroffReq

    def add_route(self):
        global route_list
        if self.route == 2:
            route_list = np.array([2, 9])
        elif (self.ADM_command != self.DC_command):
            if self.ADM_command == 0 and self.DC_command == 1:
                route_list = self.initial_charge()
            elif self.ADM_command == 1 and self.DC_command == 2:
                route_list = self.charge_discharge()
            elif self.ADM_command == 2 and self.DC_command == 1:
                route_list = self.discharge_charge()
            elif self.ADM_command == 2 and self.DC_command == 3:
                route_list = self.discharge_refueling()
            elif self.ADM_command == 2 and self.DC_command == 4:
                route_list = self.discharge_parking()
            elif self.ADM_command == 3 and self.DC_command == 1:
                route_list = self.refueling_charge()
            elif self.ADM_command == 3 and self.DC_command == 4:
                route_list = self.refueling_parking()
            elif self.ADM_command == 4 and self.DC_command == 1:
                route_list = self.parking_charge()
            elif self.ADM_command == 4 and self.DC_command == 3:
                route_list = self.parking_refueling()
        else:
            route_list = np.array([self.route, self.route])

        return route_list

    def get_ADM_command(self):
        """
        in order to calculate the current ADM_command state, according to route-state table
        """
        if self.route == 1 or self.route == 5 or self.route == 8:
            self.ADM_command = 1  #charge state
        elif self.route == 2 or self.route == 9:
            self.ADM_command = 2  #discharge state
        elif self.route == 3 or self.route == 7:
            self.ADM_command = 3  #refueling state
        elif self.route == 4 or self.route == 6:
            self.ADM_command = 4  #parking state
        else:
            self.ADM_command = 0  #intial state
        return self.ADM_command

    def initial_charge(self):
        global route_list
        if self.route == 0 and self.DiveroffReq == 1:
            route_list = np.array([1])
        return route_list

    def charge_discharge(self):
        global route_list
        if self.route == 1:
            route_list = np.array([1, 2])
        elif self.route == 5:
            route_list = np.array([5, 8])
        elif self.route == 8:
            route_list = np.array([8, 2])
        return route_list

    def discharge_charge(self):
        global route_list
        if self.route == 9:
            route_list = np.array([9, 8])
        return route_list

    def discharge_refueling(self):
        global route_list
        if self.route == 9:
            route_list = np.array([9, 3])
        return route_list

    def discharge_parking(self):
        global route_list
        if self.route == 9:
            route_list = np.array([9, 4])
        return route_list

    def refueling_charge(self):
        global route_list
        route_list = []
        if self.route == 3:
            route_list = np.array([3, 5])
        elif self.route == 7:
            route_list = np.array([7, 5])
        return route_list

    def refueling_parking(self):
        global route_list
        if self.route == 3:
            route_list = np.array([3, 6])
        elif self.route == 7:
            route_list = np.array([7, 6])
        return route_list

    def parking_charge(self):
        global route_list
        if self.route == 4:
            route_list = np.array([4, 1])
        elif self.route == 6:
            route_list = np.array([6, 1])
        return route_list

    def parking_refueling(self):
        global route_list
        if self.route == 4:
            route_list = np.array([4, 7])
        elif self.route == 6:
            route_list = np.array([6, 7])
        return route_list

    def get_info(self):
        global route_list
        self.route = route_list[0]
        return self.route


def get_diveroffreq(ADM_command, DC_command, velocity, Compartment_DownLowest,
                    planning_control):
    """
    set the value of diveroffreq
    d is distance to obstacle
    """
    global diveroffreq
    #print(ADM_command,DC_command)
    #if (ADM_command!=DC_command and velocity==0):
    #diveroffreq=1
    if planning_control == 1 and velocity == 0:
        diveroffreq = 1
    elif (ADM_command != DC_command and velocity == 0
          and Compartment_DownLowest == 1):
        diveroffreq = 1
    elif (ADM_command != DC_command and velocity == 0):
        diveroffreq = 1
    else:
        diveroffreq = 0
    return diveroffreq


class SignalOutput(object):
    """
    set the signal
    :return: parking_control,ADM_SubtaskState
    """
    def __init__(self, end, route, charge_stop, upload_complete, velocity, planning_control):
        self.end = end
        self.route = route
        self.charge_stop = charge_stop
        # self.Compartment_DownLowest = Compartment_DownLowest
        self.upload_complete = upload_complete
        self.velocity = velocity
        self.planning_control = planning_control

    def signal(self):
        """
        signal deal function
        """
        global parking_control
        global ADM_SubtaskState
        global dischargeReq
        #parking_control = 0
        ADM_SubtaskState = 0
        dischargeReq = 0
        # global charge_stop
        # global Compartment_DownLowest
        if self.end == 1 and (self.route == 1 or self.route == 8
                              or self.route == 5) and self.planning_control == 0:
            parking_control = 1
        if self.end == 2 and (self.route == 1 or self.route == 8
                              or self.route == 5):
            parking_control = 1
        if self.planning_control == 1:
            parking_control = 0
        if self.end == 1 and self.route == 9:
            ADM_SubtaskState = 5  #discharge_stop
            dischargeReq = 1
        if (self.end == 1 and self.route == 4) or (self.end == 1
                                                   and self.route == 6):
            ADM_SubtaskState = 3  #parking_completion
        if (self.end == 1 and self.route == 3) or (self.end == 1
                                                   and self.route == 7):
            ADM_SubtaskState = 1  #refueling_truckstop
        if self.charge_stop == 1:
            ADM_SubtaskState = 4  #charge_stop
        if self.upload_complete == 1 and self.route == 2:
            ADM_SubtaskState = 2  #discharge_completion
            dischargeReq = 0
        return parking_control, dischargeReq, ADM_SubtaskState


#############now let us do a test

# route=0
# ADM_command=0
# for dt in range(20):
#     try:
#         parking_control=0
#         discharge_stop=0
#         parking_completion=0
#         refueling_truckstop=0
#         DC_command=int(input('DC_command='))
#         end=int(input('end='))
#         #v_des=int(input('v_des='))
#         velocity=int(input('velocity='))
#         DiveroffReq=get_diveroffreq(ADM_command,DC_command,velocity)
#         print("DriveroffReq is",DiveroffReq)
#         vehicle= commandState(ADM_command,DC_command,route,end,DiveroffReq)
#         route_list = vehicle.add_route()
#         planning_control=int(input('planning_control='))
#         list = kill_route(route_list,planning_control)
#         route_list = list.kill()
#         route = vehicle.get_info()
#         ADM_command = vehicle.get_ADM_command()
#         Signal=signal_output(end,route)
#         parking_control,discharge_stop,parking_completion,refueling_truckstop=Signal.signal()
#         print("ADM_command and current route",(ADM_command, route))
#         print("parking_control,discharge_stop,parking_completion,refueling_truckstop",(parking_control,discharge_stop,parking_completion,refueling_truckstop))
#     except:
#         print("error:not in command-state-machine")
