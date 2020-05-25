uint32 ID   # 障碍物ID
common_msgs/Point3D Rel_Pos   # 障碍物中心点（车体坐标系）
common_msgs/Point3D[8] Rel_Bbox   # 车体坐标系下障碍物的外接矩形框
common_msgs/Range3D Rel_Range   # 车体坐标系下障碍物的范围
common_msgs/Point3D Rel_Vel   # 障碍物中心点（车体坐标系）速度,单位m/s
common_msgs/Point3D Rel_Acc   # 障碍物中心点（车体坐标系）加速度, 单位m/s^2
common_msgs/Point3D Rel_Pos_Stdev    # 障碍物中心点标准差（车体坐标系）
common_msgs/Point3D Rel_Vel_Stdev   # 障碍物中心点（车体坐标系）速度标准差
common_msgs/Point3D Rel_Acc_Stdev   # 障碍物中心点（车体坐标系）加速度标准差
common_msgs/Angle3D Rel_Angle   # 车体坐标系下障碍物轨迹的横滚角、俯仰角、航向角, 单位deg
common_msgs/Angle3D Rel_Angle_Stdev   # 车体坐标系下障碍物轨迹的横滚角、俯仰角、航向角的标准差
common_msgs/Angle3D Rel_Angle_Vel   # 车体坐标系下障碍物轨迹的横滚角速度、俯仰角速度、航向角速度
common_msgs/Angle3D Rel_Angle_Vel_Stdev   # 车体坐标系下障碍物轨迹的横滚角速度、俯仰角速度、航向角速度的标准差
float32 Rel_Curvature   # 障碍物轨迹的曲率（车体坐标系）
common_msgs/Angle3D Rel_Bbox_Angle   # 车体坐标系下Bounding box的横滚角、俯仰角、航向角的标准差，单位deg
common_msgs/Angle3D Rel_Bbox_Angle_Stdev   # 车体坐标系下Bounding box的横滚角、俯仰角、航向角的标准差
common_msgs/Point3D Abs_Pos   # 障碍物中心点（地图坐标系）
common_msgs/Point3D Abs_Bbox   # 地图坐标系下障碍物的外接矩形框
common_msgs/Range3D Abs_Range   # 地图坐标系下障碍物的范围
common_msgs/Point3D Abs_Vel   # 障碍物中心点的速度, 单位m/s
common_msgs/Point3D Abs_Acc   # 障碍物中心点的加速度, 单位m/s^2
common_msgs/Point3D Abs_Pos_Stdev   # # 障碍物中心点（地图坐标系）的标准差
common_msgs/Point3D Abs_Vel_Stdev   # 障碍物中心点速度（地图坐标系）的标准差
common_msgs/Point3D Abs_Acc_Stdev   # 障碍物中心点加速度（地图坐标系）的标准差
common_msgs/Angle3D Abs_Angle   # 地图坐标系下障碍物轨迹的横滚角、俯仰角、航向角, 单位deg
common_msgs/Angle3D Abs_Angle_Stdev   # 地图坐标系下障碍物轨迹横滚角、俯仰角、航向角的标准差
common_msgs/Angle3D Abs_Angle_Vel   # 地图坐标系下障碍物轨迹的横滚角速度、俯仰角速度、航向角速度, 单位deg/s
common_msgs/Angle3D Abs_Angle_Vel_Stdev   # 地图坐标系下障碍物轨迹横滚角速度、俯仰角速度、航向角速度的标准差
common_msgs/Angle3D Abs_Bbox_Angle   # 地图坐标系下障碍物Bounding Box的横滚角、俯仰角、航向角, 单位deg
common_msgs/Angle3D Abs_Bbox_Angle_Stdev   # 地图坐标系下障碍物Bounding Box横滚角、俯仰角、航向角的标准差
float32 Abs_Curvature   # 地图坐标系下障碍物的曲率
float32 RCS   # 雷达散射截面积，单位:dBm²
float32 Length   # 障碍物长度,单位m
float32 Width   # 障碍物宽度,单位m
float32 Height   # 障碍物高度,单位m
float32 Prob_Exist   # 存在概率。即判断雷达检测到的目标为真实目标的可能性
uint8 Meas_State   # 检测状态指示object是否有效
# 0x0:deleted
# 0x1:new
# 0x2:measured
# 0x3:predicted
# 0x4:deleted for merge
# 0x5 new from merge
uint8 Dyn_Prop # 障碍物动态属性，指示障碍物是否运动
# 0x0: moving 
# 0x1: stationary 
# 0x2: oncoming 
# 0x3: stationary candidate 
# 0x4: unknown 
# 0x5: crossing stationary 
# 0x6: crossing moving 
# 0x7: stopped 
uint8 Obj_Type   # 障碍物类型
# 0x0:point
# 0x1:Car
# 0x2:truck
# 0x3:not in sure
# 0x4:motorcycle
# 0x5: bicycle
# 0x6:wide
# 0x7:reserved
uint8 Status   # radar节点状态
# 0x0:radar节点节点正常，建议车辆正常行驶；
# 0x1:radar节点发现有微小异常，但不影响驾驶，建议继续行驶；
# 0x2:radar节点有较大异常，建议车辆缓慢停车；
# 0x3:radar节点存在严重异常，建议车辆紧急停车。