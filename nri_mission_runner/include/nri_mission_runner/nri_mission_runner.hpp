#ifndef NRI_MISSION_RUNNER_HPP_
#define NRI_MISSION_RUNNER_HPP_

#include <mutex>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nri_msgs/msg/nri_waypoint_gps.hpp"
#include "nri_msgs/msg/nri_waypointlist_gps.hpp"
#include "nri_msgs/msg/nri_waypoint_xyz.hpp"
#include "nri_msgs/msg/nri_waypointlist_xyz.hpp"

#include "nri_mission_runner/navsat_conversions.hpp"
#include "nri_mission_runner/nri_mission_util.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#define PI 3.14159265

struct StateMachIdx
{
    int prev;
    int current;
};


struct NriGpsGoal
{
    double latitude_goal;
    double longitude_goal;
    double headang_goal_deg;
    double is_in_field;
};

struct NriXyzGoal
{
    double x_goal;
    double y_goal;
    double headang_goal_deg;
    double is_in_field;
};

namespace nri_mission_runner
{ 

enum class mission_status {
    
    init,
    ready,
    processing,
    sent_goal,
    got_feedback,
    completed
};

class NriMissionRunner : public rclcpp::Node
{
    private:
        
       rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr StatePub_; // monitoring publisher 
       rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
           
       rclcpp::Subscription<nri_msgs::msg::NriWaypointGps>::SharedPtr GpsCmdSub_;
       rclcpp::Subscription<nri_msgs::msg::NriWaypointlistGps>::SharedPtr MultiGpsCmdSub_;
       rclcpp::Subscription<nri_msgs::msg::NriWaypointXyz>::SharedPtr XyzCmdSub_;
       rclcpp::Subscription<nri_msgs::msg::NriWaypointlistXyz>::SharedPtr MultiXyzCmdSub_;
       rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr MissionCancelSub_;
       rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr GpsPosSub_;
       
       rclcpp::AsyncParametersClient::SharedPtr GlobalCostParam_client_;
       rclcpp::AsyncParametersClient::SharedPtr LocalCostParam_client_;
       rclcpp::AsyncParametersClient::SharedPtr ControlSrvParam_client_;
       
       rclcpp::TimerBase::SharedPtr timer_;
       
       std::shared_future<std::vector<rclcpp::Parameter>> parameters_sf_;
       std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> GmapUpdate_Fb_;
       
         rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_action_client_;
       
       unsigned char MissionAct_ = 0;
       unsigned char MissionNum_ = 0;
       
       bool ParamSrv_ready_ = false;
       bool UpdateGCP_ok_ = false;
       long int LoopRate_Milisec_ = 500;
       
       bool wait_param_ = false;
       bool wait_param_update_ = false;
       bool ParamLoad_ok_ = false;
       
       double cur_pos_x_ = 0; //meter
       double cur_pos_y_ = 0; //meter
       double cur_pos_theta_ = 0;  //radian

       std::vector<struct NriGpsGoal> m01_GpsGoal_;
       double m01_waypoint_x_ = 0;
       double m01_waypoint_y_ = 0;
       double m01_waypoint_ang_deg_ = 0;
       double m01_local_goal_x_ = 0;
       double m01_local_goal_y_ = 0;
       bool m01_is_final_local_goal_ = false;   
       bool m01_is_sent_goal_ = false;    
       bool m01_is_accepted_goal_ = false;
       bool m01_is_success_goal_ = false;
       
       mission_status m01_stat_;
       mission_status m02_stat_;
       
       struct StateMachIdx stat_mach_idx ={0, 0};
       

       std::vector<struct NriXyzGoal> m01_XyzGoal_;
       
       int globalmap_width_;
       int globalmap_height_;
       double globalmap_origin_x_;
       double globalmap_origin_y_;
       
       double GpsLat_Init_, GpsLong_Init_;
       double GpsGoalLat_Infield_, GpsGoalLong_Infield_;
       unsigned char GpsInit_SetId_ = 0; // 0: not set, 1: set by msg 2 : set by parameter
       
       void Nav2GoalRespondCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle); 
       void Nav2FbCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
       void Nav2ResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);  
         
       void GpsCmdCallback(const nri_msgs::msg::NriWaypointGps & msg);
       void MultiGpsCmdCallback(const nri_msgs::msg::NriWaypointlistGps & msg);
       void XyzCmdCallback(const nri_msgs::msg::NriWaypointXyz & msg);
       void MultiXyzCmdCallback(const nri_msgs::msg::NriWaypointlistXyz & msg);
       void MissionCancelCallback(const std_msgs::msg::Int8 & msg);
       
       void GpsPosCallback(const sensor_msgs::msg::NavSatFix & msg);
       
       void CurRobotPose();
       void Clear();
       
       void MISSION_01();
       
       void WayPointGen_ID1(bool is_gps_input);
       void SM_MakeAlign_ID2();
       void SM_FollowAlley_ID3();
       void SM_TrunAround_ID4();
                  
        
    public:
       NriMissionRunner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()); /*constructor*/
       ~NriMissionRunner(); /*destructor*/
       void Loop();    
       bool ParamClient(); 
       bool ParamClientReady();   
       bool GetCostMapParam();
       bool UpdateGCP(double origin_x, double origin_y);


};
}// namespace nri_mission_runnner

#endif  // NRI_MISSION_RUNNER_HPP_
