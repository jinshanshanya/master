#ifndef _PERCEPTION_RADAR_NODE_
#define _PERCEPTION_RADAR_NODE_

// c++
#include <boost/thread/thread.hpp>
#include <mutex>
#include <queue>
#include <thread>

// ros
#include <ros/ros.h>

// ros message
#include "VehicleMsg/cdm_cmd.h"
#include "InsMsg/ins_p2.h"
// #include "RadarMsg/RadarObj.h"
// #include "RadarMsg/RadarObjs.h"
#include "radar_msgs/RadarObjectList.h"
#include "radar_msgs/RadarObject.h"

// perception radar
#include "perception_radar/UDPSocket.h"

// ars408
#include "perception_radar/ars408/ars408_67.h"
#include "perception_radar/ars408/ars408_60.h"
#include "perception_radar/ars408/ars408_66.h"
#include "perception_radar/ars408/ars408_65.h"
#include "perception_radar/ars408/ars408_64.h"
#include "perception_radar/ars408/ars408_63.h"
#include "perception_radar/ars408/ars408_62.h"

// radar tools
#include "perception_radar/radar_tools/radar_tools.h"

#define RADARF_INFO_LENGHT 1300
#define RADARF_CMD_LENGHT 16
#define MSG_TOPIC_GLOBALPOSE               "insMsg"
#define MSG_TOPIC_CDM "CdmCmdMsg"
#define MSG_TOPIC_VEHICLE_STATUS           "vehicleStatusMsg"
#define MSG_TOPIC_VEHICLE_CMD              "vehcileCMDMsg"
#define MSG_TOPIC_VEHICLE_UPLOAD           "vehicleUploadMsg"
#define MSG_TOPIC_VEHICLE_DOWNLOAD         "vehcileDownloadMsg"
#define MSG_TOPIC_RADAROBJ         "RadarObjMsg"
#define IMU_LEN                     100
#define INS_INFO_LENGTH             69
//#define VEHICLE_INFO_LENGHT         21
#define VEHICLE_INFO_LENGHT         1300
#define VEHICLE_CMD_LENGHT          34
#define RAD_ANG_ONJ_NUM 					64
#define RAD_BUS_SIZE						2068
// #define RAD_ANGLE_BUS_SIZE					10086
#define RAD_ANGLE_BUS_SIZE					10598
#define PACKET_TYPE_RADAR1					0
#define PACKET_TYPE_RADAR2					1

const bool print_message = true;

namespace global {
namespace perception {
namespace radar {

class Perception_Radar_Node
{
    public:
        // constructor
        Perception_Radar_Node(ros::NodeHandle& public_node,
        ros::NodeHandle& private_node);
        // destructor
        virtual ~Perception_Radar_Node();
    protected:
        // variable
        // thread
        boost::thread *RadarF_Thread;
        boost::thread *RadarB_Thread;
        boost::thread *RadarFL_Thread;
        boost::thread *RadarFR_Thread;
        boost::thread *RadarBL_Thread;
        boost::thread *RadarBR_Thread;
        // radar port
        uint16_t RadarF_Port = 20004;
        uint16_t RadarB_Port = 20005;
        uint16_t RadarFL_Port = 20006;
        uint16_t RadarFR_Port = 20007;
        uint16_t RadarBL_Port = 20008;
        uint16_t RadarBR_Port = 20009;
        // udp socket
        UDPSocket *RadarF_Socket;
        UDPSocket *RadarB_Socket;
        UDPSocket *RadarFL_Socket;
        UDPSocket *RadarFR_Socket;
        UDPSocket *RadarBL_Socket;
        UDPSocket *RadarBR_Socket;
        // radar url
        string RadarUrl = "192.168.43.104";
        // ros::Publisher ros_pub_RadarObj_info;
        // ros::Publisher ros_pub_RadarObjs_info;
        // radarf message publisher
        ros::Publisher pub_radarf_msgs_;
        // radarb message publisher
        ros::Publisher pub_radarb_msgs_; 
        // radarfl message publisher
        ros::Publisher pub_radarfl_msgs_;
        // radarfr message publisher
        ros::Publisher pub_radarfr_msgs_;
        // radarbl message publisher
        ros::Publisher pub_radarbl_msgs_;
        // radarbr message publisher
        ros::Publisher pub_radarbr_msgs_;
        // radar fusion message publisher
        ros::Publisher pub_radar_fusion_msgs_;
        // Subscriber
        ros::Subscriber Sub_InsMsg;
        ros::Subscriber Sub_CdmMsg;
        // radar message
        uint8_t RadarF_msg[RADARF_INFO_LENGHT];
        uint8_t RadarB_msg[RADARF_INFO_LENGHT];
        uint8_t RadarFL_msg[RADARF_INFO_LENGHT];
        uint8_t RadarFR_msg[RADARF_INFO_LENGHT];
        uint8_t RadarBL_msg[RADARF_INFO_LENGHT];
        uint8_t RadarBR_msg[RADARF_INFO_LENGHT];
        uint8_t RadarF_tx_cmd_msg[RADARF_CMD_LENGHT];
        uint8_t vehicle_tx_cmd_msg[VEHICLE_CMD_LENGHT];
        uint8_t RadarF_tx_msg[RADARF_CMD_LENGHT];
        // ins message
        InsMsg::ins_p2 Global_InsMsg;
        // vehicle message
        VehicleMsg::cdm_cmd Global_CdmMsg;
        // radar message
        Rad_Obj Radar_msg[6];
        // ars408_67 message
        struct ars408_67_obj_0_status_t objb_0;
        struct ars408_67_obj_1_general_t objb_1[50];
        struct ars408_67_obj_2_quality_t objb_2[50];
        struct ars408_67_obj_3_extended_t objb_3[50];
        // ars408_60 message
        struct ars408_60_obj_0_status_t objf_0;
        struct ars408_60_obj_1_general_t objf_1[50];
        struct ars408_60_obj_2_quality_t objf_2[50];
        struct ars408_60_obj_3_extended_t objf_3[50];
        // ars408_66 message
        struct ars408_66_obj_0_status_t objfl_0;
        struct ars408_66_obj_1_general_t objfl_1[50];
        struct ars408_66_obj_2_quality_t objfl_2[50];
        struct ars408_66_obj_3_extended_t objfl_3[50];
        // ars408_65 message
        struct ars408_65_obj_0_status_t objfr_0;
        struct ars408_65_obj_1_general_t objfr_1[50];
        struct ars408_65_obj_2_quality_t objfr_2[50];
        struct ars408_65_obj_3_extended_t objfr_3[50];
        // ars408_64 message
        struct ars408_64_obj_0_status_t objbl_0;
        struct ars408_64_obj_1_general_t objbl_1[50];
        struct ars408_64_obj_2_quality_t objbl_2[50];
        struct ars408_64_obj_3_extended_t objbl_3[50];
        // ars408_63 message
        struct ars408_63_obj_0_status_t objbr_0;
        struct ars408_63_obj_1_general_t objbr_1[50];
        struct ars408_63_obj_2_quality_t objbr_2[50];
        struct ars408_63_obj_3_extended_t objbr_3[50];
        // ars408_67 message
        struct ars408_67_speed_information_t Radar_SpeedInfo;
        struct ars408_67_yaw_rate_information_t Radar_Yaw;
        // function
        void processPendingDatagramsRadarF();
        void processPendingDatagramsRadarB();
        void processPendingDatagramsRadarFL();
        void processPendingDatagramsRadarFR();
        void processPendingDatagramsRadarBL();
        void processPendingDatagramsRadarBR();
        void publishRadarF(void *data);
        void publishRadarB(void *data);
        void publishRadarFL(void *data);
        void publishRadarFR(void *data);
        void publishRadarBL(void *data);
        void publishRadarBR(void *data);
        void publishRadarObjectList(ros::Publisher& pub_radar_msgs, Rad_Obj& radar_object_message);
        void RadarFSendUdp();
        uint8_t getRadarFUdpMessage(void *data);
        // callback function
        void Recv_InsMsg_callack(const InsMsg::ins_p2::ConstPtr &InsMsg);
        void Recv_CdmMsg_callack(const VehicleMsg::cdm_cmd::ConstPtr &CdmMsg);
        // Obj_Rms
        float Obj_Rms(uint8_t num);
        // ObjOri_Rms
        float ObjOri_Rms(uint8_t num);
        // Ros_Show
        // void Ros_Show(Rad_Obj *Radar_msg);
        // RadarObjs_msg
        // RadarMsg::RadarObjs RadarObjs_msg;
        // radar fusion
        void getParameter(ros::NodeHandle& public_node, ros::NodeHandle& private_node);
        void radarFusion();
        void publishRadarFusionObjectList(std::list<Radar_Object>& object_list);
        // variable
        // radar data queue
        std::queue<Rad_Obj> queue_radarf_object_;
        std::mutex mutex_queue_radarf_;
        std::queue<Rad_Obj> queue_radarb_object_;
        std::mutex mutex_queue_radarb_;
        std::queue<Rad_Obj> queue_radarfl_object_;
        std::mutex mutex_queue_radarfl_;
        std::queue<Rad_Obj> queue_radarfr_object_;
        std::mutex mutex_queue_radarfr_;
        std::queue<Rad_Obj> queue_radarbl_object_;
        std::mutex mutex_queue_radarbl_;
        std::queue<Rad_Obj> queue_radarbr_object_;
        std::mutex mutex_queue_radarbr_;  
        // transform matrix
        Eigen::Matrix4f transform_matrix_radarf_;
        Eigen::Matrix4f transform_matrix_radarb_;
        Eigen::Matrix4f transform_matrix_radarfl_;
        Eigen::Matrix4f transform_matrix_radarfr_;
        Eigen::Matrix4f transform_matrix_radarbl_;
        Eigen::Matrix4f transform_matrix_radarbr_;
        // thread
        std::thread radar_fusion_thread_;
        // radar obejct maximum number
        const static uint8_t obj_max_num = 32;
        // remove duplication threshold
        float remove_duplication_threshold_ = 0.8;
};

}   // global
}   // perception
}   // radar

#endif