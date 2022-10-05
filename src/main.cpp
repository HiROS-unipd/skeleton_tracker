#include "skeleton_tracker/Tracker.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hiros::skeletons::Tracker>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
