#ifndef _OBSTACLE_TRACKING_
#define _OBSTACLE_TRACKING_

// c++
#include <vector>
#include <list>

// obstacle_tracking
#include "perception_lidar/obstacle_tracking/association_coefficient.h"
#include "perception_lidar/obstacle_tracking/base_assignment.h"
#include "perception_lidar/obstacle_tracking/hungarian_assignment.h"

// point cloud tools
#include "perception_lidar/point_cloud_tools/feature_extraction.h"

namespace perception
{
    template<typename T>
    class Obstacle_Feature_Tracking
    {
        public:
        // constructor
        Obstacle_Feature_Tracking()
        {
            velocity_.x_ = 0.0;
            velocity_.y_ = 0.0;
            velocity_.z_ = 0.0;
        };
        Obstacle_Feature_Tracking(Cube<T> cube, Point3D<T> dcube[8],
        uint32_t id, double time_stamp):
        cube_(cube),
        id_(id),
        time_stamp_(time_stamp)
        {
            for (size_t i=0; i<8; i++)
                dcube_[i] = dcube[i];
        };
        Obstacle_Feature_Tracking(Cube<T> cube, Point3D<T> dcube[8],
        uint32_t id, double time_stamp, std::list<Cube<T>>& cube_list,
        std::list<double>& time_stamp_list):
        cube_(cube),
        id_(id),
        time_stamp_(time_stamp),
        cube_list_(cube_list),
        time_stamp_list_(time_stamp_list)
        {
            for (size_t i=0; i<8; i++)
                dcube_[i] = dcube[i];
        };
        // destructor
        ~Obstacle_Feature_Tracking() { };
        // function
        void getMeanVelocity(size_t window_size);
        // variable
        uint32_t id_;
        Point3D<T> dcube_[8];
        Cube<T> cube_;
        Point3D<T> velocity_;
        double time_stamp_;
        std::list<Cube<T>> cube_list_;
        std::list<double> time_stamp_list_;
        protected:
        static const size_t buffer_size_ = 101;
    };
    
    template<typename PointT, typename T>
    class Obstacle_Tracking
    {
        public:
            // constructor
            Obstacle_Tracking(const double cost_threshold = 0.05, const double memory_time = 0.35):
            cost_threshold_(cost_threshold),
            memory_time_(memory_time)
            {

            };
            // destructor
            ~Obstacle_Tracking(){ };
            // function
            void obstacleTracking(boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking,
            std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>&  obstacles_feature,
            boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking,
            uint32_t& iterator_id,  const double time_stamp, size_t window_size);

        protected:
            // variable
            // boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking_;
            // perception::Obstacle_Feature_Point_Cloud<PointT, T>  obstacles_feature_[];
            // const size_t obstacles_number_;
            const double cost_threshold_;
            // boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking_;
            // std::vector<std::vector<double>> costMatrix_;
            // std::vector<int> assignment_;
            // uint32_t* iterator_id_pointer_; 
            // const double time_stamp_;
            const double memory_time_;
            // function
            void getAssignment(boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking,
            std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>&  obstacles_feature,
            std::vector<std::vector<double>>& costMatrix, std::vector<int>& assignment);
            void assignID(boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking,
            std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>&  obstacles_feature,
            boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking,
            uint32_t& iterator_id,  const double time_stamp, std::vector<std::vector<double>>& costMatrix,
            std::vector<int>& assignment, size_t window_size);
            void assignIDNoTracking(std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>&  obstacles_feature,
            uint32_t& iterator_id, const double time_stamp,
            boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking);
            void assignIDNoObstacle(boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking,
            boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking,
            const double time_stamp);
    };
}

template<typename T>
void perception::Obstacle_Feature_Tracking<T>::getMeanVelocity(size_t window_size)
{
    if (window_size > buffer_size_)
        window_size = buffer_size_;
    if (window_size > cube_list_.size())
        window_size = cube_list_.size();

    // std::cout << "1. riter_time start" << std::endl;
    // for (std::list<double>::reverse_iterator riter = time_stamp_list_.rbegin();
    // riter != time_stamp_list_.rend(); riter++)
    //     std::cout << "1. riter_time = " << *riter << std::endl;
    // std::cout << "1. riter_time end" << std::endl;

    cube_list_.push_back(cube_);
    time_stamp_list_.push_back(time_stamp_);
    if (cube_list_.size() > buffer_size_)
    {
        cube_list_.pop_front();
        time_stamp_list_.pop_front();
    }

    // std::cout << "2. riter_time start" << std::endl;
    // for (std::list<double>::reverse_iterator riter = time_stamp_list_.rbegin();
    // riter != time_stamp_list_.rend(); riter++)
    //     std::cout << "2. riter_time = " << *riter << std::endl;
    // std::cout << "2. riter_time end" << std::endl;

    double total_time = 0.0;
    std::list<double>::iterator iter;
    typename std::list<Cube<T>>::iterator iter_cube;
    if (window_size >= cube_list_.size())
    {
        iter = time_stamp_list_.begin();
        iter_cube = cube_list_.begin();
        // std::cout << "1. iter_time = " << *iter << std::endl;
    }
    else
    {
        iter = time_stamp_list_.end();
        iter_cube = cube_list_.end();
        for (size_t i=0; i<window_size; i++)
        {
            iter--;
            iter_cube--;
            // std::cout << "2. iter_time = " << *iter << std::endl;
        }
    }
    total_time = *time_stamp_list_.rbegin() - *iter;
    // std::cout << "end x = " << (*cube_list_.rbegin()).xmin_ << std::endl;
    // std::cout << "total_time = " << total_time << std::endl;
    if (total_time != 0.0)
    {
        Cube<T> velocity_cube = (*cube_list_.rbegin() - *iter_cube) / total_time;
        velocity_.x_ = abs(velocity_cube.xmin_) < abs(velocity_cube.xmax_) ? 
        velocity_cube.xmin_ : velocity_cube.xmax_;
        velocity_.y_ = abs(velocity_cube.ymin_) < abs(velocity_cube.ymax_) ? 
        velocity_cube.ymin_ : velocity_cube.ymax_;
        velocity_.z_ = abs(velocity_cube.zmin_) < abs(velocity_cube.zmax_) ? 
        velocity_cube.zmin_ : velocity_cube.zmax_;
    }
}

template<typename PointT, typename T>
void perception::Obstacle_Tracking<PointT, T>::getAssignment(boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking,
std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>&  obstacles_feature,
std::vector<std::vector<double>>& costMatrix, std::vector<int>& assignment)
{
    const int rows = obstacles_featue_tracking->size();
    if (rows == 0)
        return;
    const int cols = obstacles_feature.size();
    if (cols == 0)
        return;
    for (int i=0; i<rows; i++)
    {
        perception::Point2D<T> box0[2];
        box0[0].x_ = (*obstacles_featue_tracking)[i].cube_.xmin_;
        box0[0].y_ = (*obstacles_featue_tracking)[i].cube_.ymin_;
        box0[1].x_ = (*obstacles_featue_tracking)[i].cube_.xmax_;
        box0[1].y_ = (*obstacles_featue_tracking)[i].cube_.ymax_;
        std::vector<double> cost_vec;
        for (int j=0; j<cols; j++)
        {
            perception::Point2D<T> box1[2];
            box1[0].x_ = obstacles_feature[j].cube_.xmin_;
            box1[0].y_ = obstacles_feature[j].cube_.ymin_;
            box1[1].x_ = obstacles_feature[j].cube_.xmax_;
            box1[1].y_ = obstacles_feature[j].cube_.ymax_;
            double cost = perception::getIOU<T>(box0, box1);
            cost_vec.push_back(-cost);
        }
        costMatrix.push_back(cost_vec);
    }
    Base_Assignment<double, int>* base_assignment = new Hungarian_Assignment(cost_threshold_);
    base_assignment->Solve(costMatrix, assignment);
    delete base_assignment;
}

template<typename PointT, typename T>
void perception::Obstacle_Tracking<PointT, T>::assignID(boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking,
std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>&  obstacles_feature,
boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking,
uint32_t& iterator_id,  const double time_stamp, std::vector<std::vector<double>>& costMatrix,
std::vector<int>& assignment,  size_t window_size)
{
    const size_t obstacles_number = obstacles_feature.size();
    const size_t obstacles_tracking_size = obstacles_featue_tracking->size();
    for (size_t i=0; i<obstacles_tracking_size; i++)
        if (assignment[i]>=0)
        {
            perception::Obstacle_Feature_Tracking<T> obstacle_feature_tracking(
            obstacles_feature[assignment[i]].cube_, obstacles_feature[assignment[i]].dcube_,
            (*obstacles_featue_tracking)[i].id_, time_stamp, (*obstacles_featue_tracking)[i].cube_list_,
            (*obstacles_featue_tracking)[i].time_stamp_list_);
            obstacle_feature_tracking.getMeanVelocity(window_size);
            new_obstacles_featue_tracking->push_back(obstacle_feature_tracking);
            obstacles_feature[assignment[i]].label_tracking_ = true;
        }
        else
        {
            double time_diff = time_stamp - (*obstacles_featue_tracking)[i].time_stamp_;
            if (time_diff < memory_time_)
                new_obstacles_featue_tracking->push_back((*obstacles_featue_tracking)[i]);
        }
    
    for (size_t i=0; i<obstacles_number; i++)
        if (!obstacles_feature[i].label_tracking_)
        {
            perception::Obstacle_Feature_Tracking<T> obstacle_feature_tracking(
            obstacles_feature[i].cube_, obstacles_feature[i].dcube_,
            iterator_id, time_stamp);
            iterator_id += 1;
            new_obstacles_featue_tracking->push_back(obstacle_feature_tracking);
        }

}

template<typename PointT, typename T>
void perception::Obstacle_Tracking<PointT, T>::assignIDNoTracking(std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>&  obstacles_feature,
uint32_t& iterator_id, const double time_stamp,
boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking)
{
    const size_t obstacles_number = obstacles_feature.size();
    for (int i=0; i<obstacles_number; i++)
    {
        perception::Obstacle_Feature_Tracking<T> obstacle_feature_tracking(
        obstacles_feature[i].cube_, obstacles_feature[i].dcube_,
        iterator_id, time_stamp);
        iterator_id += 1;
        new_obstacles_featue_tracking->push_back(obstacle_feature_tracking);
    }
}

template<typename PointT, typename T>
void perception::Obstacle_Tracking<PointT, T>::assignIDNoObstacle(boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking,
boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking,
const double time_stamp)
{
    const size_t obstacles_tracking_size = obstacles_featue_tracking->size();
    for (size_t i=0; i<obstacles_tracking_size; i++)
    {
        double time_diff = time_stamp - (*obstacles_featue_tracking)[i].time_stamp_;
        if (time_diff < memory_time_)
            new_obstacles_featue_tracking->push_back((*obstacles_featue_tracking)[i]);
    }
}

template<typename PointT, typename T>
void perception::Obstacle_Tracking<PointT, T>::obstacleTracking(boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> obstacles_featue_tracking,
std::vector<perception::Obstacle_Feature_Point_Cloud<PointT, T>>&  obstacles_feature,
boost::shared_ptr<std::vector<perception::Obstacle_Feature_Tracking<T>>> new_obstacles_featue_tracking,
uint32_t& iterator_id, const double time_stamp, size_t window_size)
{
    const size_t obstacles_number = obstacles_feature.size();
    if (obstacles_featue_tracking->size() == 0 &&
    obstacles_number == 0)
        return;
    if (obstacles_featue_tracking->size() == 0)
    {
        assignIDNoTracking(obstacles_feature, iterator_id, time_stamp, new_obstacles_featue_tracking);
        return;
    }
    if (obstacles_number == 0)
    {
        assignIDNoObstacle(obstacles_featue_tracking, new_obstacles_featue_tracking, time_stamp);
        return;
    }
    std::vector<std::vector<double>> costMatrix;
    std::vector<int> assignment;
    getAssignment(obstacles_featue_tracking, obstacles_feature, costMatrix, assignment);
    assignID(obstacles_featue_tracking, obstacles_feature, new_obstacles_featue_tracking,
    iterator_id,  time_stamp, costMatrix, assignment, window_size);
}

#endif