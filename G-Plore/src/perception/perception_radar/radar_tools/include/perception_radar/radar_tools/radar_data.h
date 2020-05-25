#ifndef  _RADAR_DATA_
#define _RADAR_DATA_

#include <stdint.h>
#include <stdlib.h>
#include <list>

#define RAD_OBJ_NUM 						32

namespace global {
namespace perception {
namespace radar {

template <typename T>
class Point2D
{
    public:
        // constructor
        Point2D(){};
        Point2D(T x, T y)
        : x_(x), y_(y){};
        // destructor
        virtual ~Point2D(){};
        // function
        // inline Point2D<T> operator=(Point2D<T>& point2d);
        // variable
        T x_, y_;
        void transform(Eigen::Matrix<T, 3, 3>& transform_matrix)
        {
                return;
        };
};

template<typename T>
class Point3D : public Point2D<T>
{
    public:
        // constructor
        Point3D(){ };
        Point3D(T x, T y, T z)
        : Point2D<T>(x, y), z_(z) { };
        // destructor
        ~Point3D() { };
        // function
        // inline Point3D<T> operator=(Point3D<T>& point3d);
        void transform(Eigen::Matrix<T, 4, 4>& transform_matrix)
        {
                // Eigen::Vector4f point()
                Eigen::Matrix<T, 4, 1> point;
                point(0, 0) = this->x_;
                point(1, 0) = this->y_;
                point(2, 0) = z_;
                point(3, 0) = 1.0;
                point = transform_matrix * point;
                this->x_ = point(0, 0);
                this->y_ = point(1, 0);
                z_ = point(2, 0);
        };
        // variable
        T z_;

};

template<typename T>
class Angle3D
{
    public:
        // constructor
        Angle3D(){ };
        Angle3D(T roll, T pitch, T yaw):
        roll_(roll),
        pitch_(pitch),
        yaw_(yaw){ };
        // destructor
        virtual ~Angle3D(){ };
        T roll_, pitch_, yaw_;
};

typedef enum _Radar_Obj_Type_
{
        Radar_Obj_Type_Point 		= 0,
        Radar_Obj_Type_Car 			= 1,
        Radar_Obj_Type_Truck 		= 2,
        Radar_Obj_Type_Motorcycle 	= 4,
        Radar_Obj_Type_Bicycle 		= 5,
        Radar_Obj_Type_Wide 		= 6
} Radar_Obj_Type;

typedef enum _Radar_Obj_Meas_State_
{
        Radar_Meas_State_Deleted			= 0,
        Radar_Meas_State_NewCreated			= 1,
        Radar_Meas_State_Measured			= 2,
        Radar_Meas_State_Predicted 			= 4,
        Radar_Meas_State_DeletedForMerged	= 5,
        Radar_Meas_State_NewFromMerged		= 6
} Radar_Meas_State;

typedef enum _Radar_Obj_Exist_Prob_
{
        Radar_Obj_Exist_Prob_Invalid	  = 0,
        Radar_Obj_Exist_Prob_Less_25pct   = 1,
        Radar_Obj_Exist_Prob_Less_50pct   = 2,
        Radar_Obj_Exist_Prob_Less_75pct   = 3,
        Radar_Obj_Exist_Prob_Less_90pct   = 4,
        Radar_Obj_Exist_Prob_Less_99pct   = 5,
        Radar_Obj_Exist_Prob_Less_999pct  = 6,
        Radar_Obj_Exist_Prob_equal_100pct = 7
} Radar_Obj_Exist_Prob;

typedef enum _Radar_Obj_DynProp_
{
        Radar_DynProp_Moving 					= 0,
        Radar_DynProp_Stationary 				= 1,
        Radar_DynProp_Oncoming 					= 2,
        Radar_DynProp_Stationary_Candidate 		= 3,
        Radar_DynProp_DynProp_Unknown 			= 4,
        Radar_DynProp_Crossing_Stationary 		= 5,
        Radar_DynProp_Crossing_Moving 			= 6,
        Radar_DynProp_Stopped 					= 7
} Radar_DynProp;

//Data Packet Format
//Radar Packets
typedef struct _Radar_Obj_Bus_
{
        uint16_t 				versionInfo;
        uint16_t				streamDataLen;
        uint32_t				streamTxCnt;
        uint32_t				sourceTimeStamp;
        uint32_t				streamTimeStamp;
        uint8_t					objNum;
        uint8_t					frameInd;
        uint8_t 				ID[RAD_OBJ_NUM];
        float					Long_Pos[RAD_OBJ_NUM];
        float					Lat_Pos[RAD_OBJ_NUM];
        float					Long_Vel[RAD_OBJ_NUM];
        float					Lat_Vel[RAD_OBJ_NUM];
        float					Long_Acc[RAD_OBJ_NUM];
        float					Lat_Acc[RAD_OBJ_NUM];
        float					Long_Pos_Stdev[RAD_OBJ_NUM];
        float					Lat_Pos_Stdev[RAD_OBJ_NUM];
        float					Long_Vel_Stdev[RAD_OBJ_NUM];
        float					Lat_Vel_Stdev[RAD_OBJ_NUM];
        float					Long_Acc_Stdev[RAD_OBJ_NUM];
        float					Lat_Acc_Stdev[RAD_OBJ_NUM];
        float					Orientation_Angle[RAD_OBJ_NUM];
        float					Orientation_Stdev[RAD_OBJ_NUM];
        float					Length[RAD_OBJ_NUM];
        float					Width[RAD_OBJ_NUM];
        float					RCS[RAD_OBJ_NUM];
        Radar_Obj_Type			Type[RAD_OBJ_NUM];
        Radar_Obj_Exist_Prob	Prob_Exist[RAD_OBJ_NUM];
        Radar_Meas_State		Meas_State[RAD_OBJ_NUM];
        Radar_DynProp 			Dyn_Prop[RAD_OBJ_NUM];
} Rad_Obj;

}   // namespace radar
}   // namespace perception
}   // namespace global

#endif