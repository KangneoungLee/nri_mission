#ifndef NRI_MISSION_RUNNER_HPP_
#define NRI_MISSION_RUNNER_HPP_

#include <mutex>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nri_msgs/msg/nri_waypoint_gps.hpp"
#include "nri_msgs/msg/nri_waypointlist_gps.hpp"
#include "nri_msgs/msg/nri_waypoint_xyz.hpp"
#include "nri_msgs/msg/nri_waypointlist_xyz.hpp"

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
class NriMissionRunner : public rclcpp::Node
{
    private:
        
       rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr StatePub_; // monitoring publisher 
           
       rclcpp::Subscription<nri_msgs::msg::NriWaypointGps>::SharedPtr GpsCmdSub_;
       rclcpp::Subscription<nri_msgs::msg::NriWaypointlistGps>::SharedPtr MultiGpsCmdSub_;
       rclcpp::Subscription<nri_msgs::msg::NriWaypointXyz>::SharedPtr XyzCmdSub_;
       rclcpp::Subscription<nri_msgs::msg::NriWaypointlistXyz>::SharedPtr MultiXyzCmdSub_;
       rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr MissionCancelSub_;
       
       rclcpp::AsyncParametersClient::SharedPtr GlobalCostParam_client_;
       rclcpp::AsyncParametersClient::SharedPtr LocalCostParam_client_;
       rclcpp::AsyncParametersClient::SharedPtr ControlSrvParam_client_;
       
       rclcpp::TimerBase::SharedPtr timer_;
       
       std::shared_future<std::vector<rclcpp::Parameter>> parameters_sf_;
       std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> GmapUpdate_Fb_;
       
       unsigned char MissionAct_ = 0;
       unsigned char MissionNum_ = 0;
       
       bool ParamSrv_ready_ = false;
       bool UpdateGCP_ok_ = false;
       long int LoopRate_Milisec_ = 500;
       
       bool wait_param_ = false;
       bool wait_param_update_ = false;
       bool ParamLoad_ok_ = false;
       
       std::vector<struct NriGpsGoal> GpsGoal_;
       std::vector<struct NriXyzGoal> XyzGoal_;
       
       int globalmap_width_;
       int globalmap_height_;
       double globalmap_origin_x_;
       double globalmap_origin_y_;
           
       void GpsCmdCallback(const nri_msgs::msg::NriWaypointGps & msg);
       void MultiGpsCmdCallback(const nri_msgs::msg::NriWaypointlistGps & msg);
       void XyzCmdCallback(const nri_msgs::msg::NriWaypointXyz & msg);
       void MultiXyzCmdCallback(const nri_msgs::msg::NriWaypointlistXyz & msg);
       void MissionCancelCallback(const std_msgs::msg::Int8 & msg);
       
       
       
       void MISSION_01();
       
       void WayPointGen_SM_01();
       void SM_MakeAlign_ID2();
       void SM_FollowAlley_ID3();
       void SM_TrunAround_ID4();
                  
        
    public:
       NriMissionRunner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()); /*constructor*/
       ~NriMissionRunner(); /*destructor*/
       void Loop();    
       void ParamClient(); 
       bool ParamClientReady();   
       bool GetCostMapParam();
       bool UpdateGCP(double origin_x, double origin_y);

};
}// namespace nri_mission_runnner

#endif  // NRI_MISSION_RUNNER_HPP_
