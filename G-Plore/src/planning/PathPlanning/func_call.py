#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# from InsMsg.msg import *
import matplotlib.pyplot as plt
# from src.DecisionControl.PathPlanning.route_design import *
from route_design import *
import pandas as pd

# path_waypoint = pd.read_csv("test.csv")
path_waypoint = pd.read_csv("a.csv")
waypoint = path_waypoint.loc[:, ["x", "y", "heading"]].values.astype(float)

# way_path_x = waypoint[:, 0]
# way_path_y = waypoint[:, 1]
#
# plt.plot(way_path_x, way_path_y, 'r')
# plt.show()

setsize = 200
N = len(path_waypoint)
count = 0
Vehicle_length = 2.95

start_point = None


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def talker():
    global start_point
    end = 0
    pub = rospy.Publisher('chatter', String, queue_size=10)

    rospy.Subscriber("vehicle_node", String, callback)

    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10)  # 10hz
    current_x, current_y, heading = func2(1)
    while not rospy.is_shutdown() and end < 10:
        # s = msg.
        for i in range(10):
            # print(current_x[i], current_y[i])
            # lateralpoint = route_design(current_x, current_y, waypoint)
            lateralpoint, next_start = route(current_x[i], current_y[i], waypoint, start_point)
            start_point = next_start
            # print(lateralpoint[0, 1])
            # print(lateralpoint[0, 0])
            i_start, lateralPoint_outputs = LateralPoint_output(current_x[i], current_y[i], lateralpoint, start_point, waypoint)
            # trans_point = coordinate_change(current_x, current_y, heading, lateralPoint_outputs)
            trans_point = coordinate_change(current_x[i], current_y[i], heading[i], lateralPoint_outputs)
            # x_ref, y_ref, look_distance = breakthrough(current_x, current_y, i_start, lateralpoint)
            x_ref, y_ref, look_distance = breakthrough(current_x[i], current_y[i], i_start, lateralpoint)
            diff_angle, diff_angle_1 = plus_minus(x_ref, y_ref, current_x[i], current_y[i], heading[i], waypoint, start_point)

            LateralPoint_x_out = trans_point[0:setsize, [0]]
            LateralPoint_y_out = trans_point[0:setsize, [1]]
            compute_angle = atan(2 * Vehicle_length * sin(diff_angle) / k * v + Lf) * 57.3
            # compute_yaw = atan2((y_ref - current_y), (x_ref - current_x)) * 57.3
            compute_yaw = atan2((y_ref - current_y[i]), (x_ref - current_x[i])) * 57.3
            # compute_angle1 = atan(2 * Vehicle_length * sin(diff_angle_1) / k * v + Lf) * 57.3
            # see_data = diff_angle * 57.3
            # print(lateralpoint)
            # print(compute_yaw)
            # hello_str = "hello world %s" % rospy.get_time()
            # message = "%s,%s,%s,%s,%s,%s,%s" % (x_ref, y_ref, LateralPoint_x_out, LateralPoint_y_out,
            #                                     compute_angle, compute_yaw, compute_angle1)
            message = '%s' % (compute_angle)
            rospy.loginfo(message)
            pub.publish(message)

            rate.sleep()

        end = end + 1


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
