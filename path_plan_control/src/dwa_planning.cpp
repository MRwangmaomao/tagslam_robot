#include "path_plan_control/dwa_planning.h"

using namespace std;

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
    max_yaw_rate_ = fsSettings["max_yaw_rate"];
    delta_yam_rate_ = fsSettings["delta_yam_rate"];
    delta_speed_ = fsSettings["delta_speed"];  
    sim_time_ =  fsSettings["sim_time"];
    waypoint_position_tolerance_ = fsSettings["waypoint_position_tolerance"];  
    waypoint_angle_tolerance_ =  fsSettings["waypoint_angle_tolerance"];
    fsSettings["waypoint_path_file_"] >> waypoint_path_file_;
    std::cout << "\nConfigration the DWA parameters: " << std::endl << 
        "   max_acc_x: " << max_acc_x_ << std::endl << 
        "   max_speed: " << max_speed_ << std::endl << 
        "   min_speed: " << min_speed_ << std::endl << 
        "   foresee: " << foresee_ << std::endl <<  
        "   region_foresee: " << region_foresee_ << std::endl << 
        "   distance_threshold: " << distance_threshold_ << std::endl << 
        "   short_foresee_rate: " << short_foresee_rate_ << std::endl <<  
        "   delta_yam_rate: " << delta_yam_rate_ << std::endl  << 
        "   delta_speed: " << delta_speed_ << std::endl  << 
        "   sim_time: " << sim_time_ << std::endl  << 
        "   waypoint_path_file: " << waypoint_path_file_ << std::endl  << 
        "   waypoint_position_tolerance: " << waypoint_position_tolerance_ << std::endl  << 
        "   waypoint_angle_tolerance: " << waypoint_angle_tolerance_ << std::endl  << 
        "   max_yaw_rate: " << max_yaw_rate_ << std::endl 
        << std::endl;

    // readPathWayPoint();
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
    return sim_time_;
}

std::vector<Eigen::Vector3d> DWAPlanning::getAllPathWaypoints(){
    return path_waypoints_;
}


bool DWAPlanning::readPathWayPoint(){
    std::ifstream infile;
    std::cout << "              ------------- 读取路径数据 ------------" << std::endl;
    infile.open(waypoint_path_file_);
    if (infile){
        std::cout << "  文件打开成功！" << std::endl;
    }
    else{
        std::cout << "  不能成功打开！" << std::endl;
    }
    // assert(infile.is_open());
    std::string line_data; 
    while(getline(infile,line_data))
    {
        waypoint_num_++;
        std::string str_data[3];

        stringstream ss_data(line_data); 
        ss_data >> str_data[0] >> str_data[1] >> str_data[2];
        Eigen::Vector3d temp_pose;
        temp_pose(0) = atof(str_data[0].c_str());
        temp_pose(1) = atof(str_data[1].c_str());
        temp_pose(2) = atof(str_data[2].c_str());
        path_waypoints_.push_back(temp_pose);
        std::cout << "      读取第" << waypoint_num_ << "个路标点：" << temp_pose(0) << " " << temp_pose(1) << " " << temp_pose(2) << std::endl; 
    }
    infile.close();             //关闭文件输入流 
    return true;
} 

// 根据距离误差，后面还需要添加角度误差
bool DWAPlanning::isArriveWayPoint(){ 
    double distance_err = sqrt((robot_pose_(0) - robot_waypoint_(0))*(robot_pose_(0) - robot_waypoint_(0)) +
    (robot_pose_(1) - robot_waypoint_(1))*(robot_pose_(1) - robot_waypoint_(1))); 
    if(distance_err > waypoint_position_tolerance_){      
        return false;
    }
    double angle_err = angle_err_calcu(robot_waypoint_(2), robot_pose_(2));//距离期望角度的偏差
    if(angle_err > waypoint_angle_tolerance_){      
        return false;
    } 
    return true;
}

bool DWAPlanning::isArriveDestination(){
    double distance_err = sqrt((robot_pose_(0) - robot_dest_point_(0))*(robot_pose_(0) - robot_dest_point_(0)) +
    (robot_pose_(1) - robot_dest_point_(1))*(robot_pose_(1) - robot_dest_point_(1)));
    if(distance_err >= distance_threshold_){      
        return false;
    }

    double angle_err = angle_err_calcu(robot_dest_point_(2), robot_pose_(2));//距离期望角度的偏差
    if(angle_err >= angle_threshold_ || angle_err <= angle_threshold_ * -1.0){      
        return false;
    }
    go_v_ = 0.0;
    turn_v_ = 0.0;
    return true;
}

 

bool DWAPlanning::move_accurate(double & go_v, double & turn_v){
    double axis_distance_err = 0.1;
    double angle_limit = 0.1;
    double turn_speed = 0.35;
    double go_speed = 0.08;
    if( (robot_pose_(0) - robot_dest_point_(0)) > axis_distance_err ){  //x轴反方向  angle = 1.57
        double angle_err = angle_err_calcu(1.57, robot_pose_(2));
        std::cout << "x轴反方向" << std::endl;
        if(angle_err > angle_limit){
            turn_v = turn_speed;
            go_v = 0.0;
        }
        else if(angle_err < -angle_limit){
            turn_v = -turn_speed;
            go_v = 0.0;
        }
        else // 调整好了方向
        {
            turn_v = 0.0;
            go_v = go_speed;
        } 
    }

    else if( (robot_pose_(0) - robot_dest_point_(0)) < -axis_distance_err){  //x轴正方向 angle = -1.57
        double angle_err = angle_err_calcu(-1.57, robot_pose_(2));
        std::cout << "x轴正方向" << std::endl;
        
        if(angle_err > angle_limit){               
            turn_v = turn_speed;
            go_v = 0.0;
        }
        else if(angle_err < -angle_limit){
            turn_v = -turn_speed;
            go_v = 0.0;
        }
        else // 调整好了方向
        {
            turn_v = 0.0;
            go_v = go_speed;
        }
    }

    else if( (robot_pose_(1) - robot_dest_point_(1)) > axis_distance_err){  //y轴反方向  angle = 3.14
        double angle_err = angle_err_calcu(3.141, robot_pose_(2));
        std::cout << "y轴反方向:" << angle_err << std::endl;
        if(angle_err > angle_limit){
            turn_v = turn_speed;
            go_v = 0.0;
        }
        else if(angle_err < -angle_limit){
            turn_v = -turn_speed;
            go_v = 0.0;
        }
        else // 调整好了方向
        {
            turn_v = 0.0;
            go_v = go_speed;
        }
    }

    else if( (robot_pose_(1) - robot_dest_point_(1)) < -axis_distance_err ){  //y轴正方向  angle = 0
        double angle_err = angle_err_calcu(0, robot_pose_(2));
        std::cout << "y轴正方向" << std::endl;
        if(angle_err > angle_limit){
            turn_v = turn_speed;
            go_v = 0.0;
        }
        else if(angle_err < -angle_limit){
            turn_v = -turn_speed;
            go_v = 0.0;
        }
        else // 调整好了方向
        {
            turn_v = 0.0;
            go_v = go_speed;
        }
    }

    else{ // x,y位置调整好之后, 开始调整方位
        std::cout << "调整方位" << std::endl;
        double angle_err = angle_err_calcu(robot_dest_point_(2), robot_pose_(2));
        if(angle_err > 0.0){
            turn_v = turn_speed;
            go_v = 0.0;
        }
        else{
            turn_v = -turn_speed;
            go_v = 0.0;
        }
    }  
}

Eigen::Vector3d DWAPlanning::motion(Eigen::Vector3d x, double v_head, double v_theta){
    x(0) -= v_head * sim_time_ * sin(x(2));
    x(1) += v_head * sim_time_ * cos(x(2));
    x(2) += v_theta * sim_time_; 
    return x;
}

double DWAPlanning::angle_err_calcu(double dest_angle, double current_angle){
    double angle_err = dest_angle - current_angle;
    
    if(
        ((dest_angle>1.57 && dest_angle < 3.142) && (current_angle<-1.57 && current_angle>-3.142)) //第三象限和第四象限
      ||((current_angle>1.57 && current_angle < 3.142) && (dest_angle<-1.57 && dest_angle>-3.142)) //第四象限和第三象限
    ){
        if(angle_err > 0){ 
            angle_err = 3.14*2 + angle_err;
        }else{ 
            angle_err = angle_err - 3.14*2;
        } 
    }
    else if(
        ((dest_angle>1.57 && dest_angle < 3.142) && (current_angle<0.0 && current_angle>-1.57)) //第三象限和第一象限
      ||((current_angle>1.57 && current_angle < 3.142) && (dest_angle<0.0 && dest_angle>-1.57)) //第一象限和第三象限
    ){
        if(angle_err > 3.14){
            angle_err = angle_err- 3.14*2;
        }
        else if(angle_err < -3.14){
            angle_err = angle_err + 3.14*2;
        }else{

        }
    } 
    else if(
        ((dest_angle>0 && dest_angle < 1.57) && (current_angle<-1.57 && current_angle>-3.142)) //第二象限和第四象限
      ||((current_angle>0 && current_angle < 1.57) && (dest_angle<-1.57 && dest_angle>-3.142)) //第四象限和第二象限
    ){
        if(angle_err > 3.14){
            angle_err = angle_err- 3.14*2; // 反转
        }
        else if(angle_err < -3.14){ // 正转
            angle_err = angle_err + 3.14*2;
        }else{
            
        }
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
        if(sim_time_ * max_yaw_rate_ < angle_err)
        {
            if(turn_v_ < max_yaw_rate_ - delta_yam_rate_) //缓慢加速
                turn_v_ += delta_yam_rate_; 
            else
                turn_v_ = max_yaw_rate_; // 饱和速度
        }else{
            turn_v_ = angle_err * 1.0 / sim_time_; // 控制速度
        }
        if(!dir_flag)  // 反向旋转
            turn_v_ = -1.0 * turn_v_;
    }

    // --------------------------------------------------------------------
    // 优先进行位置调整
    // --------------------------------------------------------------------
    else{
        double max_weight_go_speed = 0, max_weight_yaw_speed = 0;
        std::cout << "位置误差: " << distance_err << ";" << "角度误差：" << angle_err << std::endl;
        double min_sim_dis = 10000;
        double temp_go, temp_turn; 
        double sim_dist = 0.0;
        double last_go_v = go_v_;
        
        // 速度控制
        // 线速度空间进行采样
        for(temp_go = -1.0* max_speed_; temp_go <= max_speed_; temp_go+=delta_speed_){
            // 角速度空间进行采样
            for(temp_turn = -1.0 * max_yaw_rate_; temp_turn <= max_yaw_rate_; temp_turn+=delta_yam_rate_){
                Eigen::Vector3d sim_pose = motion(robot_pose_, temp_go, temp_turn); // 进行一次运动学模拟
                sim_dist = sqrt((sim_pose(0) - robot_waypoint_(0))*(sim_pose(0) - robot_waypoint_(0)) +
                            (sim_pose(1) - robot_waypoint_(1))*(sim_pose(1) - robot_waypoint_(1)));
                // 更新最优值
                if(sim_dist < min_sim_dis){
                    go_v_ = temp_go;
                    turn_v_ = temp_turn;
                    min_sim_dis = sim_dist;
                } 
            }
        }

        // 加速度控制
        if(last_go_v-go_v_ > max_acc_x_){ //减速过快
            go_v_ = last_go_v - max_acc_x_;
        }
        else if(go_v_ - last_go_v > max_acc_x_){ // 增速过快
            go_v_ = last_go_v + max_acc_x_;
        }
        else{

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
