#include  "stanley_control.h"

Stanley_control::Stanley_control(ros::NodeHandle nh, ros::NodeHandle pri_nh){
    sub_trajectory_= nh.subscribe<autoware_planning_msgs::Trajectory>(
        "input/reference_trajectory",1,&Stanley_control::onTrajectory,this,
    ros::TransportHints().reliable().tcpNoDelay(true));

    sub_current_velocity_ = nh.subscribe<geometry_msgs::TwistStamped>(
        "input/current_velocity",1,&Stanley_control::onCurrentVelocity,this, 
    ros::TransportHints().reliable().tcpNoDelay(true));
    pub_ctrl_cmd_ =
    nh.advertise<autoware_control_msgs::ControlCommandStamped>("output/control_raw", 1);


    current_heading = 0;
    index_store = 0;
    index_current =0;
    Lf = 2.7;
    setsize = 50;
    h_error = 0.1;
    v = 0;
    steering_wheel_offset = 0;
    fd = 1;

}

Stanley_control::~Stanley_control(){};

void Stanley_control::onCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & velocitymsg){

    current_velocity_ = velocitymsg ;
    v = current_velocity_->twist.linear.x;
}

 void Stanley_control::onTimer(const ros::TimerEvent &event){ 
     current_pose_ = tf_utils::getCurrentPose(tf_buffer_);
     current_x = current_pose_->pose.x;
     current_y = current_pose_->pose.y;
 }


 void Stanley_control::onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg){
    //  trajectory_ = msg->points;
    //  int row = trajectory_.size();
    //  //ind_route_generation_
    //  std::vector<float> x;
    //   std::vector<float> y;
    //  for(i = 0; i < row; i++){
    //      x.push_back(trajectory[i].pose.)
    //row &col= the size of the pose
    trajectory_ = msg;
    int pre = 10;
    float current_x = current_pose_.x;
    float current_y = current_pose_.y;
    int row = trajectory_->point.size();//row = the length of x or y point
    Eigen::MatrixXd waypoint(row,2);//inclued x,yposition
    for(int i = 0; i < row; i++){
        waypoint(i, 0) = trajectory_->point[i][0];
        waypoint(i, 1) = trajectory_->point[i][1];
    }
    Eigen::MatrixXd current_pose(row,2);//inclued current_x,current_y position(only one)
    for(int i = 0; i < row; i++){
        current_pose(i, 0) = current_x;
        current_pose(i, 1) = current_y;
    }
    //Eigen::MatrixXd y(row,1);
    //float current_x_F = current_x + Lf * sin(hear * pi/180);//current_x =  the position of  the car;hear = the heading angle of the car.
    //float current_y_F = current_x + Lf * sin(hear * pi/ 180);
    Eigen::MatrixXd distance_matrix_F(row,2);
    Eigen::MatrixXd distance_norm_F(row,row);
    //std::vector<std::vector<float>> distance_matrix_F(row ,std::vector<float>(2));
    distance_matrix_F =  waypoint - current_pose;
    distance_norm_F = distance_matrix_F * distance_matrix_F.transpose();
    std::vector<float> distance_norm_F_1;
    for(int i = 0; i < row; i++){
        distance_norm_F_1.push_back(sqrt(distance_norm_F(i,i)));
    }
    auto minperm_F = min_element(distance_norm_F_1.begin(), distance_norm_F_1.end());
    int current_ind_F = minperm_F - distance_norm_F_1.begin();
    Eigen::MatrixXd look_forward_point(setsize,2);
    for(int i = 0; i <= setsize; i++){
        look_forward_point(i,0) = waypoint(current_ind_F + i, 0);
        look_forward_point(i,1) = waypoint(current_ind_F + i, 1);
    }

    Eigen::MatrixXd p1_F(1, 2);
    Eigen::MatrixXd p2_F(1, 2);
    p1_F = waypoint.row(current_ind_F);
    p2_F = waypoint.row(current_ind_F + 1);
    Eigen::MatrixXd current_point_F(1,2);
    current_point_F(0, 0) = current_x;
    current_point_F(0, 1) = current_y;
    float lateral_offset = find_distance(p1_F, p2_F, current_point_F);
    int look_distance = 9;
    float look_dis = min((0.1 * v + look_distance), 17.5);
    float x_ref,y_ref;
    if(abs(lateral_offset) >= 1500){
        x_ref = 0;
        y_ref = 1;
    }else{
        float s = 0;
        int i = 0; 
        int COND = 1;
        while(COND == 1){
            s = sqrt(pow((look_forward_point(i,0) - current_x),2) + pow((look_forward_point(i,1) - current_y),2));
            i++;
            if(s >= look_dis) COND = 2;
            if(i == setsize) COND = 3;
        }
        x_ref = look_forward_point(i-1,0);
        y_ref = look_forward_point(i-1,1);
    }
    //calCurrentHeading
       if(current_x >= 0 && current_y > 0){
        if(current_x == 0 ){
        current_heading = 0;
        }
        current_heading = atan(current_x / current_y)*180/pi;
    }
    if(current_x > 0 && current_y <= 0){
        if(current_y == 0){
        current_heading = 90;
        }
        current_heading = atan(abs(current_y / current_x))*180/pi + 90;
    }
    if(current_x <= 0 && current_y < 0){
        if(current_x == 0){
            current_heading = 180;
        }
        current_heading = atan(current_x / current_y)*180/pi + 180;
    }
    if(current_x < 0 && current_y >= 0){
        if(current_y == 0){
            current_heading =270;
        }
        current_heading = atan(abs(current_y / current_x))*180/pi + 270;
    }
    //calHeadung
    for(int i=0; i < row; i++){
        float calheading = 0;
        if(waypoint(i, 0) >= 0 && waypoint(i, 1) > 0){
            if(waypoint(i, 0) == 0 ){
            calheading = 0;
            }
            calheading= atan(waypoint(i, 0) / waypoint(i, 1))*180/pi;
         }
        if(waypoint(i, 0) > 0 && waypoint(i, 1) <= 0){
            if(waypoint(i, 1) == 0){
            calheading = 90;
            }
            calheading = atan(abs(waypoint(i, 1) / waypoint(i, 0)))*180/pi +90;
        }    
        if(waypoint(i, 0) <= 0 && waypoint(i, 1) < 0){
            if(waypoint(i, 0) == 0){
                calheading = 180;
            }
            calheading = atan(waypoint(i, 0) / waypoint(i, 1))*180/pi + 180;
        }
        if(waypoint(i, 0) < 0 && waypoint(i, 1) >= 0){
            if(waypoint(i, 1)== 0){
                calheading =270;
            }
            calheading = atan(abs(waypoint(i, 1) / waypoint(i, 0)))*180/pi + 270;
        }
        heading.push_back(calheading);
    }  
    //calDelataHeading
    index_store = floor(heading[current_ind_F] / 90);
    index_current = floor(current_heading / 90);
    float delta_heading = 0;
    if(abs(index_store - index_current) ==0){
        delta_heading = heading[current_ind_F] - current_heading;
    }else if(abs(index_store - index_current) == 1){
        delta_heading =heading[current_ind_F] -current_heading;
    }else if(index_store == 0 && index_current ==3){
        delta_heading =heading[current_ind_F] - current_heading + 360;
    }else if(index_store == 3 && index_current == 0){
        delta_heading =heading[current_ind_F] - current_heading -360;
    }

    if(abs(index_store - index_current) == 2){
        if(index_store == 0 && index_current ==2){
            if(abs(heading[current_ind_F] - current_heading) >180){
                delta_heading =-(heading[current_ind_F] + 360 -current_heading);
            }else delta_heading = current_heading -heading[current_ind_F];
        }

        if(index_store == 2 &&index_current ==0){
            if(abs(heading[current_ind_F] - current_heading) > 180){
                delta_heading = 360 +current_heading - heading[current_ind_F];
            }else delta_heading = current_heading - heading[current_ind_F];
        }

        if(index_store == 1 && index_current == 3){
            if(abs(heading[current_ind_F] - current_heading) > 180){
                delta_heading = -(heading[current_ind_F] + 360 -current_heading);
            }else delta_heading = current_heading - heading[current_ind_F];
        }

        if(index_store == 3 && index_current == 1){
            if(abs(heading[current_ind_F] - current_heading) > 180){
                delta_heading = current_heading + 360 -heading[current_ind_F];
            }else delta_heading = -(heading[current_ind_F] - current_heading);
        }
    }
  
    //kppa
    std::vector<float> x_kppa;
    std::vector<float> y_kppa;
    for(int i = 0; i < row-5; i++){

        x_kppa.push_back(0.2 * (waypoint(i,0) + waypoint(i+1,0) + waypoint(i+2,0) + waypoint(i+3,0) + waypoint(i+4,0)));
        y_kppa.push_back(0.2 * (waypoint(i,1) + waypoint(i+1,1) + waypoint(i+2,1) + waypoint(i+3,1) + waypoint(i+4,1)));
    }
    std::vector<float> kk;
    for(int i = 0; i < row-5; i++){
        float x1 = x_kppa[i];
        float y1 = y_kppa[i];
        float x2 = x_kppa[i+2];
        float y2 = y_kppa[i+2];
        float x3 = x_kppa[i+4];
        float y3 = y_kppa[i+4];
        float a = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
        float b = (pow(x1,2) + pow(y1,2) - pow(x2,2) - pow(y2,2))/2;
        float c = (pow(x1,2) + pow(y1,2) - pow(x3,2) - pow(y3,2))/2;
        float d = b * (y1 - y3) - c * (y1 - y2);
        float e = c * (x1 - x2) - b * (x1 -x3);

        float kppa_kk = sqrt( pow((d/a - x1),2) + pow((e/a - y1),2));

        kk.push_back(kppa_kk);
    }
    float kppa = kk[current_ind_F];
    std::vector<float> alpha;
    alpha.push_back(atan(x_ref / y_ref));
    // float stanley_wheel = atan(2 * vehicle_length * sin(alpha) / look_distance) * 180 / pi;
    float k_error = 0.6;
    float pre_look = max(pre, v/3.6);
    float heading_error_angle = delta_heading * h_error;
    float stanley_wheel = atan(lateral_offset * k_error / pre_look)*180/pi;
    float steer_wheel_angle = fd*(1*stanley_wheel + 0 *heading_error_angle) + steering_wheel_offset;
    cmd.steering_angle  = steer_wheel_angle;
    pub_ctrl_cmd_.publish(cmd);
}

     float find_distance(Eigen::MatrixXd p1_F,Eigen::MatrixXd p2_F,Eigen::MatrixXd current_point_F ){
        float A = p1_F(0,1) - p2_F(0,1);
        float B = p1_F(0,0) - p2_F(0,0);
        float C = p1_F(0,0) * p2_F(0,1) - p1_F(0,1) * p2_F(0,0);
        float dist =  abs((A * current_point_F(0,0) + B * current_point_F(0,1) + C) / (sqrt(pow(A,2) + pow(B,2))));
        float x1 = p1_F(0,0);
        float y1 = p1_F(0,1);
        float x2 = p2_F(0,0);
        float y2 = p2_F(0,1);
        float angle = (x2 - x1) * (current_point_F(0,1) - y1) - (current_point_F(0,0) - x1)*(y2 - y1);
        bool symbo;
        if(angle > 0){
            symbo = 1;
        }else symbo = -1;
        dist = symbo * dist;
        return dist;
     }

