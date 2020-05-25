/*
 * Filename: /home/easydrive/project/glb/admsystem/src/IntegratedDriver/Radar/include/Radar/radar.h
 * Path: /home/easydrive/project/glb/admsystem/src/IntegratedDriver/Radar/include/Radar
 * Created Date: Monday, October 14th 2019, 7:28:44 pm
 * Author: easydrive
 * 
 * Copyright (c) 2019 Your Company
 */

#ifndef LIDAR_H
#define LIDAR_H
#include <stdint.h>

#include <array>
#include <limits>
#include <cmath>

#include "types.h"
#include "lidar_msgs/LidarObject.h"

namespace Lidar{


	bool Filter(const lidar_msgs::LidarObject &obj)
	{
		// filter out fov
		if ((obj.Rel_Range.xMax-obj.Rel_Range.xMin)<0.001||
			(obj.Rel_Range.yMax-obj.Rel_Range.yMin)<0.001||
			(obj.Rel_Range.zMax-obj.Rel_Range.zMin)<0.001){
			return true;
		}
		// fliter out abnormal velocity
		if (fabs(obj.Rel_Vel.x) > 20 || fabs(obj.Rel_Vel.y) > 20)
		{
			return true;
		}
		if (fabs(obj.Rel_Pos.x) > 150 || fabs(obj.Rel_Pos.y) > 150)
		{
			return true;
		}
        return false;
	}
}
#endif
