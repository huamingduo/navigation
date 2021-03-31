#include <memory>
#include <ros/ros.h>

#include "amcl.h"

std::vector<std::pair<int,int>> amcl::AmclNode::free_space_indices; // initialize static class member variable

int main(int argc, char** argv) {
  ros::init(argc, argv, "amcl");
  ros::NodeHandle nh;

  std::shared_ptr<amcl::AmclNode> amcl_node_ptr = std::make_shared<amcl::AmclNode>();

  ros::Rate rate{50};
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}