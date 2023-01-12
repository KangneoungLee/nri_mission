#include "nri_mission_runner/nri_mission_runner.hpp"
#include "rclcpp/rclcpp.hpp"
//#include "boost/bind.hpp>

using namespace std::placeholders;
//using std::placeholders::_1;
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
    
    this->declare_parameter("nri_mission_gps_lat_init", rclcpp::ParameterValue(30.616914));
    this->declare_parameter("nri_mission_gps_long_init", rclcpp::ParameterValue(-96.340988));
    
    this->get_parameter("nri_mission_gps_lat_init", GpsLat_Init_);
    this->get_parameter("nri_mission_gps_long_init", GpsLong_Init_);
    
    GpsPosSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("mavros/global_position", 10, std::bind(&NriMissionRunner::GpsPosCallback, this, _1)); 
    
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
    
    nav2_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, this->get_name());

    //ParamClient();
    //Loop();
    
    m01_stat_ = mission_status::init;

}

void NriMissionRunner::Nav2GoalRespondCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
{
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by NavigateToPose server");
      if(stat_mach_idx.current == 1) m01_is_sent_goal_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by NavigateToPose server, waiting for result");
      if(stat_mach_idx.current == 1) m01_is_accepted_goal_ = true;
    }

}

void NriMissionRunner::Nav2FbCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
}

void NriMissionRunner::Nav2ResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
   switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if(stat_mach_idx.current == 1) m01_is_success_goal_ = true;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose Unknown result code");
        return;
}
}

bool NriMissionRunner::ParamClient()
{
       auto node = this->shared_from_this(); // should not be declared in the class constructor
       GlobalCostParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, "global_costmap/global_costmap");
       LocalCostParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, "local_costmap/local_costmap");
       ControlSrvParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, "controller_server");      
       return ParamClientReady();
         
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

void NriMissionRunner::GpsPosCallback(const sensor_msgs::msg::NavSatFix & msg)
{
   if((MissionAct_ == 1)&&(ParamLoad_ok_ == false)&&(GpsInit_SetId_ == 0))
   {
   
      GpsLat_Init_ = msg.latitude;
      GpsLong_Init_ = msg.longitude;
      GpsInit_SetId_ = 1;
   }
   else if((MissionAct_ == 1)&&(ParamLoad_ok_ == true)&&(GpsInit_SetId_ == 0))
   {
      GpsInit_SetId_ = 2; // just bypass the initial gps position from parameter
   
   }
}

void NriMissionRunner::GpsCmdCallback(const nri_msgs::msg::NriWaypointGps & msg)
{
   if(MissionAct_ == 0)
   {
       MissionNum_ = 1;
       MissionAct_ = 1;
       
       m01_GpsGoal_.push_back({msg.latitude_goal, msg.longitude_goal, msg.headang_goal_deg, msg.is_in_field});
   
       RCLCPP_INFO(get_logger(), "****Gps goal recieved****");
       RCLCPP_INFO(get_logger(), "ID : %d Latitude : %f longitude : %f heading angle : %f is in field : %d", 0, m01_GpsGoal_[0].latitude_goal,
       m01_GpsGoal_[0].longitude_goal, m01_GpsGoal_[0].headang_goal_deg, m01_GpsGoal_[0].is_in_field);
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
         m01_GpsGoal_.push_back({msg.latitude_goal[i], msg.longitude_goal[i], msg.headang_goal_deg[i], msg.is_in_field[i]});
         RCLCPP_INFO(get_logger(), "ID : %d Latitude : %f longitude : %f heading angle : %f is in field : %d", i, m01_GpsGoal_[i].latitude_goal,
          m01_GpsGoal_[i].longitude_goal, m01_GpsGoal_[i].headang_goal_deg, m01_GpsGoal_[i].is_in_field);
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
       m01_XyzGoal_.push_back({msg.x_goal, msg.y_goal, msg.headang_goal_deg, msg.is_in_field});
       RCLCPP_INFO(get_logger(), "****Xyz goal recieved****");
       RCLCPP_INFO(get_logger(), "ID : %d X : %f Y : %f heading angle : %f is in field : %d", 0, m01_XyzGoal_[0].x_goal,
           m01_XyzGoal_[0].y_goal, m01_XyzGoal_[0].headang_goal_deg, m01_XyzGoal_[0].is_in_field);
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
         m01_XyzGoal_.push_back({msg.x_goal[i], msg.y_goal[i], msg.headang_goal_deg[i], msg.is_in_field[i]});
         RCLCPP_INFO(get_logger(), "ID : %d Latitude : %f longitude : %f heading angle : %f is in field : %d", i, m01_XyzGoal_[i].x_goal,
          m01_XyzGoal_[i].y_goal, m01_XyzGoal_[i].headang_goal_deg, m01_XyzGoal_[i].is_in_field);
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
       Clear();
       
       
       RCLCPP_INFO(get_logger(), "Successfully cancel");
       
       
   }
   else
   {
       RCLCPP_INFO(get_logger(), "No mission is running so cancel is ignored");
   }
}
void NriMissionRunner::Clear()
{
  /* mission 01 variables clear */
  m01_GpsGoal_.clear();
  m01_stat_ = mission_status::init;
  m01_waypoint_x_ = 0;
  m01_waypoint_y_ = 0;
  m01_local_goal_x_ = 0;
  m01_local_goal_y_ = 0;
  m01_is_sent_goal_ = false;
  m01_is_accepted_goal_ = false;
  m01_is_success_goal_ = false;
  /* cancel the nav2 */
  
  /* mission 02 variables clear */
  m01_XyzGoal_.clear();
  
  /* cancel the nav2 */
  
  /*common variable clear*/
  MissionAct_ = 0;
  MissionNum_ = 0;
  stat_mach_idx.current = 0;
  stat_mach_idx.prev = 0;
  ParamLoad_ok_ = false;
  UpdateGCP_ok_ = false;
  RCLCPP_INFO(this->get_logger(), "Internal variables are cleared");
}

void NriMissionRunner::CurRobotPose()
{


}

void NriMissionRunner::Loop()
{
    if (MissionNum_ == 1)
    {
       MISSION_01();
    }

    auto state_msg = std_msgs::msg::Int8();
      
    state_msg.data = MissionNum_;

    StatePub_->publish(state_msg);

}
void NriMissionRunner::MISSION_01()
{
     bool is_gps_input = true;
     WayPointGen_ID1(is_gps_input);
}

void NriMissionRunner::WayPointGen_ID1(bool is_gps_input)
{


    if(stat_mach_idx.current == 0) 
    {
       stat_mach_idx.current = 1;
       stat_mach_idx.prev = 0;
    }
    else if(stat_mach_idx.current == 1) { /*do the state machine 1 loop*/}
    else  return;
    
    if(is_gps_input == true){
       if(m01_GpsGoal_.empty() == true)
       {
         Clear();
         return;
       }
    }
    else
    {
       if(m01_XyzGoal_.empty() == true)
       {
         Clear();
         return;
       }
    
    }
    
    auto GpsGoal = m01_GpsGoal_[0];
    auto XyzGoal = m01_XyzGoal_[0];

    if(is_gps_input == true){
       if(GpsGoal.is_in_field == true)
       {
          m01_stat_ = mission_status::completed;
          GpsGoalLat_Infield_ = GpsGoal.latitude_goal;
          GpsGoalLong_Infield_ = GpsGoal.longitude_goal;
          stat_mach_idx.prev = 1;
          stat_mach_idx.current = 3;
          return;
       }
    }
    else{
       if(XyzGoal.is_in_field == true)
       {
          m01_stat_ = mission_status::completed;
          //GpsGoalLat_Infield_ = GpsGoal.latitude_goal;
          //GpsGoalLong_Infield_ = GpsGoal.longitude_goal;
          stat_mach_idx.prev = 1;
          stat_mach_idx.current = 3;
          return;
       }
    }
       
    if(ParamSrv_ready_ == false) 
    {
       ParamSrv_ready_ = ParamClient();
       return;
    }
    else 
    {
       if(ParamLoad_ok_ == false) ParamLoad_ok_ = GetCostMapParam();
       else m01_stat_ = mission_status::ready;
    }
    
    if(m01_stat_ == mission_status::ready)
    {
       if(is_gps_input == true){
          double cartesian_init_x {};
          double cartesian_init_y {};
          double cartesian_init_z {};
          std::string utm_zone_tmp;
    
          navsat_conversions::LLtoUTM(GpsLat_Init_, GpsLong_Init_, cartesian_init_y, cartesian_init_x, utm_zone_tmp);
          
          double cartesian_goal_x {};
          double cartesian_goal_y {};
          double cartesian_goal_z {};
           
          navsat_conversions::LLtoUTM(GpsGoal.latitude_goal, GpsGoal.longitude_goal, cartesian_goal_y, cartesian_goal_x, utm_zone_tmp);
       
          m01_waypoint_x_ = cartesian_goal_x - cartesian_init_x;
          m01_waypoint_y_ = cartesian_goal_y - cartesian_init_y;
          m01_waypoint_ang_deg_ = GpsGoal.headang_goal_deg;
       }
       else{
            m01_waypoint_x_ = XyzGoal.x_goal;
            m01_waypoint_y_ = XyzGoal.y_goal;
            m01_waypoint_ang_deg_ = XyzGoal.headang_goal_deg;
       }
       
       RCLCPP_INFO(this->get_logger(), "m01_waypoint_x_ : %lf m01_waypoint_y_ : %lf", m01_waypoint_x_, m01_waypoint_y_);
       
       m01_stat_ = mission_status::processing;
    }
    

       
    if(m01_stat_ == mission_status::processing)
    {
       m01_is_final_local_goal_ = PruneGoalPath(globalmap_width_, globalmap_height_, cur_pos_x_, cur_pos_y_, m01_waypoint_x_, m01_waypoint_y_, m01_local_goal_x_, m01_local_goal_y_);
          // sent a goal
          
       auto goal_msg = geometry_msgs::msg::PoseStamped();
       goal_msg.header.stamp = this->now();
       goal_msg.pose.position.x = m01_local_goal_x_;
       goal_msg.pose.position.y = m01_local_goal_y_;
       
       double heading_ang_deg_temp = m01_waypoint_ang_deg_;
       tf2::Quaternion q;
       q.setRPY(0, 0, heading_ang_deg_temp*PI/180);
       q.normalize();
       
       geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
       
       goal_msg.pose.orientation = msg_quat;
       
       if(m01_is_sent_goal_ == false)
       {
          if (!this->nav2_action_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
         }
         
         nav2_msgs::action::NavigateToPose::Goal goal;
         goal.pose = goal_msg;
         
         auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
         send_goal_options.goal_response_callback = std::bind(&NriMissionRunner::Nav2GoalRespondCallback, this, _1);
         send_goal_options.feedback_callback = std::bind(&NriMissionRunner::Nav2FbCallback, this, _1, _2);
         send_goal_options.result_callback = std::bind(&NriMissionRunner::Nav2ResultCallback, this, _1);
         nav2_action_client_->async_send_goal(goal, send_goal_options);
         m01_is_sent_goal_ = true;
       }
       
       if((m01_is_sent_goal_ == true)&&(m01_is_accepted_goal_ == true))
       {
         m01_stat_ = mission_status::sent_goal;
       }
    }
      
    if((m01_is_success_goal_ == true)&&(m01_stat_ == mission_status::sent_goal)) m01_stat_ = mission_status::got_feedback;
        
    if((m01_is_final_local_goal_ == false)&&(m01_stat_ == mission_status::got_feedback))
    {
        if(UpdateGCP_ok_ == false) UpdateGCP_ok_ = UpdateGCP(cur_pos_x_,cur_pos_y_);
        else        
        { 
           m01_is_sent_goal_ = false;
           m01_is_accepted_goal_ = false;
           m01_is_success_goal_ = false;
           UpdateGCP_ok_ = false; 
           m01_local_goal_x_ = 0;
           m01_local_goal_y_ = 0;
           
           m01_stat_ = mission_status::processing;
        }
    }
    else if(m01_stat_ == mission_status::got_feedback)
    {    
        m01_is_sent_goal_ = false;
        m01_is_accepted_goal_ = false;
        m01_is_success_goal_ = false;
        UpdateGCP_ok_ = false;
        m01_local_goal_x_ = 0;
        m01_local_goal_y_ = 0;

        ParamLoad_ok_ = false;
        m01_is_final_local_goal_ = false;
        
        m01_waypoint_x_ = 0;
        m01_waypoint_y_ = 0;
        if(is_gps_input == true){
           m01_GpsGoal_.erase(m01_GpsGoal_.begin());
        }
        else{
           m01_XyzGoal_.erase(m01_XyzGoal_.begin());
        }
        
        m01_stat_ = mission_status::init;
    }

}



NriMissionRunner::~NriMissionRunner()
{
}


}// namespace nri_mission_runnner
