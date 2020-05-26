#include "path_plan_control/dwa_planning.h"

DWAPlanning::DWAPlanning(){

}


DWAPlanning::~DWAPlanning(void){

}

void DWAPlanning::init(std::string config_file_path){
    
    waypoint_id_ = 0;
    robot_pose_id_ = 0;
    first_flag_ = true;
    cv::FileStorage fsSettings(config_file_path, cv::FileStorage::READ);
    max_acc_x_ = fsSettings["max_acc_x"];
    max_speed_ = fsSettings["max_speed"];
    min_speed_ = fsSettings["min_speed"];
    foresee_ = fsSettings["foresee"];  
    region_foresee_ = fsSettings["region_foresee"];
    short_foresee_rate_ = fsSettings["short_foresee_rate"];
    distance_threshold_ = fsSettings["distance_threshold"];
    sim_period_ = fsSettings["sim_period"];
    max_yaw_rate_ = fsSettings["max_yaw_rate"];
    delta_yam_rate_ = fsSettings["delta_yam_rate"];
    delta_speed_ = fsSettings["delta_speed"];  
    

    std::cout << "\nConfigration the DWA parameters: " << std::endl << 
        "   max_acc_x: " << max_acc_x_ << std::endl << 
        "   max_speed: " << max_speed_ << std::endl << 
        "   min_speed: " << min_speed_ << std::endl << 
        "   foresee: " << foresee_ << std::endl <<  
        "   region_foresee: " << region_foresee_ << std::endl << 
        "   distance_threshold: " << distance_threshold_ << std::endl << 
        "   short_foresee_rate: " << short_foresee_rate_ << std::endl << 
        "   sim_period: " << sim_period_ << std::endl  << 
        "   delta_yam_rate: " << delta_yam_rate_ << std::endl  << 
        "   delta_speed: " << delta_speed_ << std::endl  << 
        "   max_yaw_rate: " << max_yaw_rate_ << std::endl 
        << std::endl;
} 


void DWAPlanning::setRobotPose(double x, double y, double yaw){
    robot_pose_(0) = x;
    robot_pose_(1) = y;
    robot_pose_(2) = yaw; 
}

void DWAPlanning::setAngleThresh(double ang){
    angle_threshold_ = ang;
}

void DWAPlanning::setDistanceThresh(double dis){
    distance_threshold_ = dis;
}

float DWAPlanning::getSimPeriod(){
    return sim_period_;
}

// 根据距离误差，后面还需要添加角度误差
bool DWAPlanning::isArriveWayPoint(){ 
    double distance_err = sqrt((robot_pose_(0) - robot_waypoint_(0))*(robot_pose_(0) - robot_waypoint_(0)) +
    (robot_pose_(1) - robot_waypoint_(1))*(robot_pose_(1) - robot_waypoint_(1))); 
    if(distance_err < distance_threshold_){      
        return true;
    }
    return false;
}

bool DWAPlanning::isArriveDestination(){
    double distance_err = sqrt((robot_pose_(0) - robot_waypoint_(0))*(robot_pose_(0) - robot_waypoint_(0)) +
    (robot_pose_(1) - robot_waypoint_(1))*(robot_pose_(1) - robot_waypoint_(1)));
    if(distance_err >= distance_threshold_){      
        return false;
    }

    double angle_err = angle_err_calcu(robot_waypoint_(2), robot_pose_(2));//距离期望角度的偏差
    if(angle_err >= angle_threshold_ || angle_err <= angle_threshold_ * -1.0){      
        return false;
    }
    go_v_ = 0.0;
    turn_v_ = 0.0;
    return true;
}
 
Eigen::VectorXd DWAPlanning::motion(Eigen::VectorXd &x, double v_head, double v_theta){
    x(0) = v_head * sim_time_;
    x(1) = 0;
    x(2) = v_theta * sim_time_;
    x(3) = v_head;
    x(4) = v_theta;
    return x;
}

double DWAPlanning::angle_err_calcu(double dest_angle, double current_angle){
    double angle_err = dest_angle - current_angle;
    
    if(
        ((dest_angle>1.57 && dest_angle < 3.142) && (current_angle<-1.57 && current_angle>-3.142)) //第三象限和第四象限
      ||((current_angle>1.57 && current_angle < 3.142) && (dest_angle<-1.57 && dest_angle>-3.142)) //第三象限和第四象限
    ){
        if(angle_err < 0){
            angle_err = -1.0*(3.14*2-angle_err);
        }else{
            angle_err = 3.14*2 - angle_err;
        } 
    } 
    else if(angle_err > 3.14){ //对角两个象限
        angle_err = angle_err - 3.14*2;
    }
    else if(angle_err < -3.14){
        angle_err = 3.14*2 + angle_err;
    }
    return angle_err;
} 

/**
 * 1 先检测当前目标点是否位于可通行区域内
 *      1.1 如果在通行区域内计算生成速度和转角指令
 *      1.2 如果目标点在未知区域中，目标点在深度相机视角范围外，控制机器人旋转；在视角范围内，发送停止命令
 *      1.3 如果目标点在障碍物区域中，则发送停止命令
 * 
 * 2 在通行区域内计算速度和转角
 *      2.1 动态窗口法生成一些预选的速度和角速度值
 *      2.2 根据运动模型生成轨迹，判断哪条轨迹可以最快靠近目标位置
 */
bool DWAPlanning::dwa_control(const cv::Mat& config_map){
    cv::Mat dwa_image_map = config_map.clone();  
    bool if_dwa_succ = true; 
    
    // --------------------------------------------------------------------
    // 计算最短的角度差值
    // --------------------------------------------------------------------
    double angle_err = angle_err_calcu(robot_waypoint_(2), robot_pose_(2));

    // --------------------------------------------------------------------
    // 计算x,y坐标的欧式距离误差
    // --------------------------------------------------------------------
    double distance_err = sqrt((robot_pose_(0) - robot_waypoint_(0))*(robot_pose_(0) - robot_waypoint_(0)) +
    (robot_pose_(1) - robot_waypoint_(1))*(robot_pose_(1) - robot_waypoint_(1)));

    // --------------------------------------------------------------------
    // x,y坐标位置到达后进行角度调整
    // -------------------------------------------------------------------- 
    if(distance_err <= distance_threshold_){  
        // --------------------------------------------------------------------
        // 在已有加速度限制的情况下计算期望角度
        // --------------------------------------------------------------------
        bool dir_flag = (angle_err > 0);
        bool last_dir_flag = (turn_v_ > 0);
        if(!dir_flag) 
            angle_err = angle_err * -1.0; // 误差取绝对值
        if(!last_dir_flag) 
            turn_v_ = turn_v_ * -1.0; // 速度取绝对值
        if(sim_period_ * max_yaw_rate_ < angle_err)
        {
            if(turn_v_ < max_yaw_rate_ - delta_yam_rate_) //缓慢加速
                turn_v_ += delta_yam_rate_; 
            else
                turn_v_ = max_yaw_rate_; // 饱和速度
        }else{
        turn_v_ = angle_err * 1.0 / sim_period_; // 控制速度
        }
        if(!dir_flag)  // 反向旋转
            turn_v_ = -1.0 * turn_v_;
    }

    // --------------------------------------------------------------------
    // 优先进行位置调整
    // --------------------------------------------------------------------
    else{
        double max_weight_go_speed = 0, max_weight_yaw_speed = 0;
        double 
        // 线速度空间进行采样
        for(){
            // 角速度空间进行采样
            for(){

            }
        }
        
        



    }

    return if_dwa_succ;
}

bool DWAPlanning::move(double & go_v, double & turn_v){

    cv::Mat config_map;
    if(!dwa_control(config_map))
        std::cerr << "DWA Plan is failed!!\n";
    
    go_v = go_v_;
    turn_v = turn_v_;
    
    return false;
}
