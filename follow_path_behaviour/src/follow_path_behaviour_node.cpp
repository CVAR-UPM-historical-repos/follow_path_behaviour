// "Copyright [year] <Copyright Owner>"

#include "follow_path_behaviour/follow_path_behaviour.hpp"
#include "as2_core/core_functions.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::cout << "Starting Follow Path Behaviour" << std::endl;

  auto node = std::make_shared<FollowPathBehaviour>();
  node->preset_loop_frequency(30);
  as2::spinLoop(node);  
  
  rclcpp::shutdown();
  return 0;
}
