#include <memory>

#include "nri_mission_runner/nri_mission_runner.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nri_mission_runner::NriMissionRunner>();
  //node->ParamClient();
  //node->Loop();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
