#include <memory>
#include <ros/ros.h>

#include "amcl.h"
#include "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"

using namespace cartographer::mapping::scan_matching;

constexpr double PI = 3.141592653;
std::vector<std::pair<int,int>> amcl::AmclNode::free_space_indices; // initialize static class member variable

int main(int argc, char** argv) {
  ros::init(argc, argv, "amcl");
  ros::NodeHandle nh;

  proto::RealTimeCorrelativeScanMatcherOptions options; // TODO: a function for reading params from a file
  options.set_linear_search_window(0.2);
  options.set_angular_search_window(20./180.*PI);
  options.set_translation_delta_cost_weight(1e-1);
  options.set_rotation_delta_cost_weight(1e-1);

  std::shared_ptr<amcl::AmclNode> amcl_node_ptr = std::make_shared<amcl::AmclNode>(options);

  ros::Rate rate{50};
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}