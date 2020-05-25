uint32 ID   # 障碍物ID
uint8 Obj_Type
# 0x0:UNKNOWN
# 0x1:UNKNOWN_MOVABLE
# 0x2:UNKNOWN_UNMOVABLE
# 0x3:PEDESTRIN
# 0x4:BICYCLE
# 0x5:VEHICLE
# 0x6:TRUNK
# 0x7:MAX_OBJECT_TYPE
# 0x8:reserved
common_msgs/Point3D Rel_Pos   # 障碍物中心点（车体坐标系）
common_msgs/Point3D[8] Rel_Bbox    # 车体坐标系下障碍物的外接矩形框
common_msgs/Range3D Rel_Range   # 车体坐标系下障碍物的范围
common_msgs/Point3D Rel_Vel   # 障碍物中心点（车体坐标系）速度,单位m/s
common_msgs/Point3D Rel_Acc   # 障碍物中心点（车体坐标系）加速度, 单位m/s^2
common_msgs/Point3D Rel_Pos_Stdev   # 障碍物中心点标准差（车体坐标系）
common_msgs/Point3D Rel_Vel_Stdev   # 障碍物中心点（车体坐标系）速度标准差
common_msgs/Point3D Rel_Acc_Stdev   # 障碍物中心点（车体坐标系）加速度标准差
common_msgs/Angle3D Rel_Angle   # 车体坐标系下障碍物轨迹的横滚角、俯仰角、航向角
common_msgs/Angle3D Rel_Angle_Stdev   # 车体坐标系下障碍物轨迹的横滚角、俯仰角、航向角的标准差
common_msgs/Angle3D Rel_Angle_Vel   # 车体坐标系下障碍物轨迹的横滚角速度、俯仰角速度、航向角速度
common_msgs/Angle3D Rel_Angle_Vel_Stdev   # 车体坐标系下障碍物轨迹的横滚角速度、俯仰角速度、航向角速度的标准差
float32 Rel_Curvature   # 障碍物轨迹的曲率（车体坐标系）
common_msgs/Angle3D Rel_Bbox_Angle   # 车体坐标系下Bounding box的横滚角、俯仰角、航向角的标准差，单位deg
common_msgs/Angle3D Rel_Bbox_Angle_Stdev   # 车体坐标系下Bounding box的横滚角、俯仰角、航向角的标准差
common_msgs/Point3D Abs_Pos   # 障碍物中心点（地图坐标系）
common_msgs/Point3D[8] Abs_Bbox   # 地图坐标系下障碍物的外接矩形框
common_msgs/Range3D Abs_Range   # 地图坐标系下障碍物的范围
common_msgs/Point3D Abs_Vel   # 障碍物中心点的速度, 单位m/s
common_msgs/Point3D Abs_Acc   # 障碍物中心点的加速度, 单位m/s^2
common_msgs/Point3D Abs_Pos_Stdev   # 障碍物中心点标准差（地图坐标系）
common_msgs/Point3D Abs_Vel_Stdev   # 障碍物中心点（地图坐标系）速度标准差
common_msgs/Point3D Abs_Acc_Stdev   # 障碍物中心点（地图坐标系）加速度标准差
common_msgs/Angle3D Abs_Angle   # 地图坐标系下障碍物轨迹的横滚角、俯仰角、航向角
common_msgs/Angle3D Abs_Angle_Stdev   # 地图坐标系下障碍物轨迹的横滚角、俯仰角、航向角的标准差
common_msgs/Angle3D Abs_Angle_Vel   # 地图坐标系下障碍物轨迹的横滚角速度、俯仰角速度、航向角速度
common_msgs/Angle3D Abs_Angle_Vel_Stdev   # 地图坐标系下障碍物轨迹的横滚角速度、俯仰角速度、航向角速度的标准差
float32 Abs_Curvature   # 障碍物轨迹的曲率（地图坐标系）
common_msgs/Angle3D Abs_Bbox_Angle   # 地图坐标系下Bounding box的横滚角、俯仰角、航向角的标准差，单位deg
common_msgs/Angle3D Abs_Bbox_Angle_Stdev   # 地图坐标系下Bounding box的横滚角、俯仰角、航向角的标准差
float32 Length   # 目标物长度,单位m
float32 Width   # 目标物高度,单位m
float32 Height   # 目标物宽度,单位m
float32 Prob_Exist   # 存在概率。即判断雷达检测到的目标为真实目标的可能性
uint8 Status   # lidar节点状态
# 0x0:lidar节点节点正常，建议车辆正常行驶；
# 0x1:lidar节点发现有微小异常，但不影响驾驶，建议继续行驶；
# 0x2:lidar节点有较大异常，建议车辆缓慢停车；
# 0x3:lidar节点存在严重异常，建议车辆紧急停车。