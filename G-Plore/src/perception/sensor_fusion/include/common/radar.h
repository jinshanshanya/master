/*
 * Filename: /home/easydrive/project/glb/admsystem/src/IntegratedDriver/Radar/include/Radar/radar.h
 * Path: /home/easydrive/project/glb/admsystem/src/IntegratedDriver/Radar/include/Radar
 * Created Date: Monday, October 14th 2019, 7:28:44 pm
 * Author: easydrive
 * 
 * Copyright (c) 2019 Your Company
 */

#ifndef _RADAR_H
#define _RADAR_H
#include <stdint.h>

#include <array>
#include <limits>
#include <cmath>

#include "types.h"
#include "radar_msgs/RadarObject.h"

namespace Radar{


	bool Filter(const radar_msgs::RadarObject &obj)
	{
		// filter out fov
		if (abs(obj.Rel_Pos.x)>5)
		{
			return true;
		}
		// fliter out abnormal velocity
		if (fabs(obj.Rel_Vel.x) > 20 || fabs(obj.Rel_Vel.y) > 20)
		{
			return true;
		}
		
        return false;
	}
}
#endif
