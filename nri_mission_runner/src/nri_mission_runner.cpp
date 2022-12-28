#include "nri_mission_runner/nri_mission_runner.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace nri_mission_runner
{ 
NriMissionRunner::NriMissionRunner(const rclcpp::NodeOptions & options):Node("nri_mission_runner")
{
    std::string GpsCmdSubTopic = "gps_cmd_nri";
    std::string MultiGpsCmdSubTopic = "multi_gps_cmd_nri";
    std::string XyzCmdSubTopic = "xyz_cmd_nri";
    std::string MultiXyzCmdSubTopic = "multi_xyz_cmd_nri";
    std::string MissionCancelSubTopic = "nri_mission_cancel";
    
    auto option = this->get_node_options();
    
    this->declare_parameter("gps_cmd_sub_topic", rclcpp::ParameterValue("gps_cmd_nri"));
    this->declare_parameter("multi_gps_cmd_sub_topic", rclcpp::ParameterValue("multi_gps_cmd_nri"));
    this->declare_parameter("xyz_cmd_sub_topic", rclcpp::ParameterValue("xyz_cmd_nri"));
    this->declare_parameter("multi_xyz_cmd_sub_topic", rclcpp::ParameterValue("multi_xyz_cmd_nri"));
    
    this->declare_parameter("mission_cancel_sub_topic", rclcpp::ParameterValue("nri_mission_cancel"));
    
    this->get_parameter("gps_cmd_sub_topic", GpsCmdSubTopic);
    this->get_parameter("multi_gps_cmd_sub_topic", MultiGpsCmdSubTopic);
    this->get_parameter("xyz_cmd_sub_topic", XyzCmdSubTopic);
    this->get_parameter("multi_xyz_cmd_sub_topic", MultiXyzCmdSubTopic);
    this->get_parameter("mission_cancel_sub_topic", MissionCancelSubTopic);
    
    //(const std::string &topic_name, size_t qos_history_depth, CallbackT &&callback)
    GpsCmdSub_ = this->create_subscription<nri_msgs::msg::NriWaypointGps>(GpsCmdSubTopic, 10, std::bind(&NriMissionRunner::GpsCmdCallback, this, _1));   
    MultiGpsCmdSub_ = this->create_subscription<nri_msgs::msg::NriWaypointlistGps>(MultiGpsCmdSubTopic, 10, std::bind(&NriMissionRunner::MultiGpsCmdCallback, this, _1));
    XyzCmdSub_ = this->create_subscription<nri_msgs::msg::NriWaypointXyz>(XyzCmdSubTopic, 10, std::bind(&NriMissionRunner::XyzCmdCallback, this, _1)); 
    MultiXyzCmdSub_ = this->create_subscription<nri_msgs::msg::NriWaypointlistXyz>(MultiXyzCmdSubTopic, 10, std::bind(&NriMissionRunner::MultiXyzCmdCallback, this, _1));
    MissionCancelSub_ = this->create_subscription<std_msgs::msg::Int8>(MissionCancelSubTopic, 10, std::bind(&NriMissionRunner::MissionCancelCallback, this, _1));
    
    this->declare_parameter("loop_rate_milisec", rclcpp::ParameterValue(500));
    this->get_parameter("loop_rate_milisec", LoopRate_Milisec_);
    
    timer_ = create_wall_timer(std::chrono::milliseconds(LoopRate_Milisec_), std::bind(&NriMissionRunner::Loop, this));

    std::string MissionStatePubTopic = "nri_mission_state";   

    this->declare_parameter("mission_state_pub_topic", rclcpp::ParameterValue("nri_mission_state"));

    this->get_parameter("mission_state_pub_topic", MissionStatePubTopic); 

    StatePub_ = this->create_publisher<std_msgs::msg::Int8>(MissionStatePubTopic, 10);
    

    //ParamClient();
    //Loop();

}

void NriMissionRunner::ParamClient()
{
       auto node = this->shared_from_this();
       GlobalCostParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, "global_costmap/global_costmap");
       LocalCostParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, "local_costmap/local_costmap");
       ControlSrvParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, "controller_server");      
       ParamSrv_ready_ = ParamClientReady();
         
}

bool NriMissionRunner::ParamClientReady()
{

    auto node = this->shared_from_this();
    RCLCPP_INFO(node->get_logger(), "ParamClientReady start...");   
    
    bool GCP_active_ = false;

     while (!(GCP_active_ = GlobalCostParam_client_->wait_for_service(std::chrono::milliseconds(500)))) {
        if (!rclcpp::ok()) {
           RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
           return 0;
        }
        RCLCPP_INFO(node->get_logger(), "global costmap service not available, waiting again...");
    }
    
    bool LCP_active_ = false;

     while (!(LCP_active_ = LocalCostParam_client_->wait_for_service(std::chrono::milliseconds(500)))) {
        if (!rclcpp::ok()) {
           RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
           return 0;
        }
        RCLCPP_INFO(node->get_logger(), "local costmap service not available, waiting again...");
    }
    
    bool CSP_active_ = false;

     while (!(CSP_active_ = ControlSrvParam_client_->wait_for_service(std::chrono::milliseconds(500)))) {
        if (!rclcpp::ok()) {
           RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
           return 0;
        }
        RCLCPP_INFO(node->get_logger(), "controller parameter service not available, waiting again...");
    }
    RCLCPP_INFO(node->get_logger(), "ParamClientReady end... GCP_active_ : %d LCP_active_ : %d CSP_active_ : %d ",GCP_active_, LCP_active_, CSP_active_);          
       return GCP_active_ & LCP_active_ & CSP_active_;
}

bool NriMissionRunner::GetCostMapParam()
{    
     RCLCPP_INFO(this->get_logger(), "GetCostMapParam start...");  
   
     if (wait_param_ == false)
     {
       parameters_sf_ = GlobalCostParam_client_->get_parameters({"width","height","origin_x","origin_y"});
       wait_param_ = true;
     }
     
     auto std_future_status = parameters_sf_.wait_for(2s);
     if ( std_future_status == std::future_status::deferred)
     {
        RCLCPP_ERROR(this->get_logger(), "get_parameters service call deferred. ");
        wait_param_ = true;
        return false;
     }
     else if(std_future_status == std::future_status::timeout)
     {
        RCLCPP_ERROR(this->get_logger(), "get_parameters service call timeout. ");
        wait_param_ = true;
        return false;     
     }
     
     wait_param_ = false;
     auto parameters = parameters_sf_.get();
     globalmap_width_ = (int)parameters[0].as_int();
     globalmap_height_ = (int)parameters[1].as_int();
     globalmap_origin_x_ = parameters[2].as_double();
     globalmap_origin_y_ = parameters[3].as_double();
     
     RCLCPP_INFO(this->get_logger(), "globalmap_width_ : %d globalmap_height_ : %d globalmap_origin_x_ : %f globalmap_origin_y_ : %f",globalmap_width_ , globalmap_height_, globalmap_origin_x_, globalmap_origin_y_);

     return true;
}


bool NriMissionRunner::UpdateGCP(double origin_x, double origin_y)
{
  RCLCPP_INFO(this->get_logger(), "UpdateGCP start...");  
  
  std::vector< rclcpp::Parameter> parameters;
  
  parameters.push_back(rclcpp::Parameter("origin_x", rclcpp::ParameterValue(origin_x)));
  parameters.push_back(rclcpp::Parameter("origin_y", rclcpp::ParameterValue(origin_y)));
  
  if (wait_param_update_ == false)
  {
      GmapUpdate_Fb_ = GlobalCostParam_client_->set_parameters(parameters);
      wait_param_update_ = true;
  }
  auto std_future_status = GmapUpdate_Fb_.wait_for(2s);

  if (std_future_status == std::future_status::deferred)
  {
     RCLCPP_ERROR(this->get_logger(), "set_parameters service call deferred. ");
     wait_param_update_ = true;
     return false;
  }
  else if(std_future_status == std::future_status::timeout)
  {
     RCLCPP_ERROR(this->get_logger(), "set_parameters service call timeout. ");
     wait_param_update_ = true;
     return false;     
  }  
  
  wait_param_update_ = false;
  
  auto feedbacks = GmapUpdate_Fb_.get();
  bool success = true;
  
  for (auto & feedback : feedbacks)
  {
     if(feedback.successful == false) 
     {
       success = false;
       RCLCPP_INFO(this->get_logger(), "parameter update fail");
     } 
  }
  
  if(success == true) 
  {
     RCLCPP_INFO(this->get_logger(), "Updated Global Map origin_x : %f origin_y : %f", origin_x, origin_y);
     return true;
  }
  else return false;

}

void NriMissionRunner::GpsCmdCallback(const nri_msgs::msg::NriWaypointGps & msg)
{
   if(MissionAct_ == 0)
   {
       MissionNum_ = 1;
       MissionAct_ = 1;
       
       GpsGoal_.push_back({msg.latitude_goal, msg.longitude_goal, msg.headang_goal_deg, msg.is_in_field});
   
       RCLCPP_INFO(get_logger(), "****Gps goal recieved****");
       RCLCPP_INFO(get_logger(), "ID : %d Latitude : %f longitude : %f heading angle : %f is in field : %d", 0, GpsGoal_[0].latitude_goal,
       GpsGoal_[0].longitude_goal, GpsGoal_[0].headang_goal_deg, GpsGoal_[0].is_in_field);
   }
   else
   {
       RCLCPP_WARN(get_logger(), "Mission act signal is received but cancel the current mission first <mission number : %d> ", MissionNum_);
   }
}


void NriMissionRunner::MultiGpsCmdCallback(const nri_msgs::msg::NriWaypointlistGps & msg)
{
   if(MissionAct_ == 0)
   {
       MissionNum_ = 1;
       MissionAct_ = 1;
       
       int size = msg.longitude_goal.size();
   
       RCLCPP_INFO(get_logger(), "****Gps goal recieved****");
       for (int i=0; i < size ; i ++)
       {
         GpsGoal_.push_back({msg.latitude_goal[i], msg.longitude_goal[i], msg.headang_goal_deg[i], msg.is_in_field[i]});
         RCLCPP_INFO(get_logger(), "ID : %d Latitude : %f longitude : %f heading angle : %f is in field : %d", i, GpsGoal_[i].latitude_goal,
          GpsGoal_[i].longitude_goal, GpsGoal_[i].headang_goal_deg, GpsGoal_[i].is_in_field);
       }
   }
   else
   {
       RCLCPP_WARN(get_logger(), "Mission act signal is received but cancel the current mission first <mission number : %d> ", MissionNum_);
   }
}

void NriMissionRunner::XyzCmdCallback(const nri_msgs::msg::NriWaypointXyz & msg)
{
   RCLCPP_INFO(get_logger(), "****msg recieved****");
   if(MissionAct_ == 0)
   {
       MissionNum_ = 2;
       MissionAct_ = 1;
       XyzGoal_.push_back({msg.x_goal, msg.y_goal, msg.headang_goal_deg, msg.is_in_field});
       RCLCPP_INFO(get_logger(), "****Xyz goal recieved****");
       RCLCPP_INFO(get_logger(), "ID : %d X : %f Y : %f heading angle : %f is in field : %d", 0, XyzGoal_[0].x_goal,
           XyzGoal_[0].y_goal, XyzGoal_[0].headang_goal_deg, XyzGoal_[0].is_in_field);
   }
   else
   {
       RCLCPP_WARN(get_logger(), "Mission act signal is received but cancel the current mission first <mission number : %d> ", MissionNum_);
   }
}


void NriMissionRunner::MultiXyzCmdCallback(const nri_msgs::msg::NriWaypointlistXyz & msg)
{
   if(MissionAct_ == 0)
   {
       MissionNum_ = 2;
       MissionAct_ = 1;
       
       int size = msg.x_goal.size();
   
       RCLCPP_INFO(get_logger(), "****Xyz goal recieved****");
       for (int i=0; i < size ; i ++)
       {
         XyzGoal_.push_back({msg.x_goal[i], msg.y_goal[i], msg.headang_goal_deg[i], msg.is_in_field[i]});
         RCLCPP_INFO(get_logger(), "ID : %d Latitude : %f longitude : %f heading angle : %f is in field : %d", i, XyzGoal_[i].x_goal,
          XyzGoal_[i].y_goal, XyzGoal_[i].headang_goal_deg, XyzGoal_[i].is_in_field);
       }
   }
   else
   {
       RCLCPP_WARN(get_logger(), "Mission act signal is received but cancel the current mission first <mission number : %d> ", MissionNum_);
   }
}



void NriMissionRunner::MissionCancelCallback(const std_msgs::msg::Int8 & msg)
{
   if(MissionAct_ == 1)
   {
       MissionNum_ = 0;
       MissionAct_ = 0;
       
       /*cancel all action */
       
       GpsGoal_.clear();
       XyzGoal_.clear();
       RCLCPP_INFO(get_logger(), "Successfully cancel");
   }
   else
   {
       RCLCPP_INFO(get_logger(), "No mission is running so cancel is ignored");
   }
}

void NriMissionRunner::Loop()
{
    if (MissionAct_ == 1)
    {
       MISSION_01();
    }

    auto state_msg = std_msgs::msg::Int8();
      
    state_msg.data = MissionNum_;

    StatePub_->publish(state_msg);

}
void NriMissionRunner::MISSION_01()
{
     WayPointGen_SM_01();
}

void NriMissionRunner::WayPointGen_SM_01()
{

    //ParamSrv_ready_= ParamClientReady();
       
    if(ParamSrv_ready_ == false) 
    {
      ParamClient();
      return;
    }
    else 
    {
       if(ParamLoad_ok_ == false) ParamLoad_ok_ = GetCostMapParam();
    }
    if(UpdateGCP_ok_ == false) UpdateGCP_ok_ = UpdateGCP(2.5,2.5);

}


NriMissionRunner::~NriMissionRunner()
{
}


}// namespace nri_mission_runnner
