

// Created by wss on 7/1/17.
//

#include "Node.h"

UserNode::UserNode(int argc, char **pArgv)
    : m_Init_argc(argc),
      m_pInit_argv(pArgv)
{
    rosRunning = false;
}
UserNode::~UserNode()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
}
/**
 *  ros function
 * */
void UserNode::init()
{
    std::string name;

    ros::init(m_Init_argc, m_pInit_argv, "IFSimu");

    if (!ros::master::check())
        return;

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;

    name = ros::this_node::getName();

    ros_pub_ins_info = nh.advertise<InsMsg::ins_p2>(MSG_TOPIC_GLOBALPOSE, 1);

    p_vehicle_ros_thread = new boost::thread(boost::bind(&UserNode::run, this));
}

void UserNode::run()
{
    ros::Rate loop_rate(20);
    ROS_INFO_STREAM("rosrunning...");
    rosRunning = true;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool UserNode::getReady()
{
    return rosRunning;
}

void UserNode::copy_buff(uint8_t *dest, uint8_t *src, uint8_t len)
{
    uint8_t i;

    for (i = 0; i < len; i++)
    {
        dest[i] = src[i];
    }
}



void UserNode::Imu_Trans()
{

    uint8_t temp[8];
    uint8_t temp_half[4];
    uint8_t temp_six[6];
    memcpy(temp, &imu_buffer[0], 8);
    //0-7 GpsTime 600
    Unpack_DateTime_IFS2000_standard_CAN_(&Time_data, temp, 8);

    //8-15 PosLat&&Lon 601
    memcpy(temp, &imu_buffer[8], 8);
    Unpack_LatitudeLongitude_IFS2000_standard_CAN_(&Postion, temp, 8);
    //16-19 PosAlt 602
    memcpy(temp_half, &imu_buffer[16], 4);
    Unpack_Altitude_IFS2000_standard_CAN_(&Posalt, temp_half, 4);

    //speed 20-27 603
    memcpy(temp, &imu_buffer[20], 8);
    Unpack_Velocity_IFS2000_standard_CAN_(&Velocity, temp, 8);

    //Accel 28-33 605
    memcpy(temp_six, &imu_buffer[28], 6);
    Unpack_AccelVehicle_IFS2000_standard_CAN_(&AccVehicle, temp_six, 6);

    //AngleHeading AnglePitch AngleRoll 34-39 607
    memcpy(temp_six, &imu_buffer[34], 6);
    Unpack_HeadingPitchRoll_IFS2000_standard_CAN_(&Angle_Pos, temp_six, 6);

    ////AngRate XYZ 40-45 608
    memcpy(temp_six, &imu_buffer[40], 6);
    Unpack_AngRateVehicle_IFS2000_standard_CAN_(&Angle_Rate, temp_six, 6);

    //VelForward VelLateral 46-49 604
    memcpy(temp_half, &imu_buffer[46], 4);
    Unpack_VelocityLevel_IFS2000_standard_CAN_(&Vel_Level, temp_half, 4);

    //    std::cout << "TimeYear=" << Time_data.TimeYear << std::endl;

    //    std::cout << "Postion"  << Postion.PosLat << std::endl;
    //    std::cout << "PosLot"  << Postion.PosLon << std::endl;

    //    std::cout << "VelForward=" << Vel_Level.VelForward << std::endl;
    //    std::cout << "VelLateral=" << Vel_Level.VelLateral << std::endl;
}

void UserNode::publishIns(void *data)
{
    uint8_t *ins_data = (uint8_t *)data;

    if (ins_data != nullptr)
    {
        uint8_t i;
        for (i = 0; i < INS_INFO_LENGTH; i++, ins_data++)
        {
            imu_buffer[i] = *ins_data;
        }

        Imu_Trans();

        struct tm tm;
        tm.tm_year = Time_data.TimeYear + (Time_data.TimeCentury * 100);
        //        tm.tm_year=1413;
        //        tm.tm_year=Time_data.TimeYear-1970;
        //        tm.tm_year=2020;
        tm.tm_mon = Time_data.TimeMonth;
        tm.tm_mday = Time_data.TimeDay;
        tm.tm_sec = Time_data.TimeSecond;
        tm.tm_min = Time_data.TimeMinute;
        tm.tm_hour = Time_data.TimeHour;

        //        tm.tm_year -= 1900;
        //        tm.tm_mon--;
        //        global_insMsg.Time=mktime(&tm)+(Time_data.TimeHSecond)*0.01;
        //        global_insMsg.Time=mktime(&tm);
        time_temp = mktime(&tm);
        time_temp_d = time_temp;
        //        time_temp=time_temp-59960828481+6081; //-3
        time_temp = time_temp - 59960822400;
        //        global_insMsg.Time=time_temp+(Time_data.TimeHSecond)*0.01;
        long int tt = 1577811663;
        int16_t error = 0;
        tt = time_temp;
        error = stime(&tt);
        //        std::cout << "TimeYear=" << Time_data.TimeYear << std::endl;
        //        std::cout << "TimeCentury=" << Time_data.TimeCentury << std::endl;
        //        error=stime(&((long int)time_temp));
        //        std::cout << "error=" << error << std::endl;
        std::cout << "d=" << time_temp_d << std::endl;
        global_insMsg.Time = time_temp;
        //        time_s=time_temp+(Time_data.TimeHSecond)*0.01;
        //        global_insMsg.Time=-(global_insMsg.Time)+(Time_data.TimeHSecond)*0.01;
        //        global_insMsg.Time=(Time_data.TimeHSecond)*0.01;

        global_insMsg.Lat = Postion.PosLat * 0.0000001;
        global_insMsg.Lon = Postion.PosLon * 0.0000001;

        global_insMsg.Altitude = Posalt.PosAlt * 0.001;

        //        global_insMsg.Ve =Velocity.VelEast*0.01;
        //        global_insMsg.Vn =Velocity.VelNorth*0.01;
        global_insMsg.Ve = Vel_Level.VelForward * 0.01;
        global_insMsg.Vn = Vel_Level.VelLateral * 0.01;

        global_insMsg.Vu = Velocity.VelDown * 0.01;
        global_insMsg.Base = Velocity.Speed2D * 0.01;

        global_insMsg.acc_x = AccVehicle.AccelX * 0.01;
        global_insMsg.acc_y = AccVehicle.AccelY * 0.01;
        global_insMsg.acc_z = AccVehicle.AccelZ * 0.01;

        global_insMsg.Heading = Angle_Pos.AngleHeading * 0.01;
        global_insMsg.Pitch = Angle_Pos.AnglePitch * 0.01;
        global_insMsg.Roll = Angle_Pos.AngleRoll * 0.01;

        global_insMsg.gyro_x = Angle_Rate.AngRateX * 0.01;
        global_insMsg.gyro_y = Angle_Rate.AngRateY * 0.01;
        global_insMsg.gyro_z = Angle_Rate.AngRateZ * 0.01;

        //        global_insMsg.Week= imu_data.Week;
        //        //global_insMsg.Time= imu_data.Time;
        //        global_insMsg.Time=Time_data.TimeYear;

        //        global_insMsg.Altitude= imu_data.Altitude;

        std::cout << "Time=" << time_temp << std::endl;

        ros_pub_ins_info.publish(global_insMsg);
    }
}


