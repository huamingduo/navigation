#include <memory>
#include <ros/ros.h>

#include "amcl.h"
#include "scan_matching/scan_matcher.h"

std::vector<std::pair<int,int>> amcl::AmclNode::free_space_indices; // initialize static class member variable

int main(int argc, char** argv) {
  ros::init(argc, argv, "amcl");
  ros::NodeHandle nh;

  scan_matching::Option option;
  option.map_stack_depth = 5;
  option.min_score = 0.3;
  option.linear_search_window = 2.5;
  option.angular_search_window = M_PI;
  option.shrink_scale = 0.5;
  option.kernal_size = 3;


  std::shared_ptr<amcl::AmclNode> amcl_node_ptr = std::make_shared<amcl::AmclNode>(option);

  ros::Rate rate{50};
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
