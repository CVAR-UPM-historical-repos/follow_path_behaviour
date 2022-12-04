
#include "follow_path_plugin_base/follow_path_base.hpp"

class As2FollowPathBaseTest : public follow_path_base::FollowPathBase {
public:
  As2FollowPathBaseTest(){};
  bool on_deactivate(const std::shared_ptr<std::string> &message) override { return false; };
  bool on_pause(const std::shared_ptr<std::string> &message) override { return false; };
  bool on_resume(const std::shared_ptr<std::string> &message) override { return false; };
  as2_behavior::ExecutionStatus own_run() override {
    return as2_behavior::ExecutionStatus::SUCCESS;
  };
  void own_execution_end(const as2_behavior::ExecutionStatus &state) override{};
  Eigen::Vector3d getTargetPosition() override { return Eigen::Vector3d(0, 0, 0); };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<As2FollowPathBaseTest>();
  rclcpp::shutdown();
  return 0;
}