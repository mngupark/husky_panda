#include "husky_panda_scan.h"

int main(int argc, char **argv)
{
  // initialize
  ros::init(argc, argv, "husky_panda_scan_node");
  // HuskyPandaMobileScanMerger husky_panda_mobile_scan_merger;
  auto husky_panda_mobile_scan_merger = boost::make_shared<HuskyPandaMobileScanMerger>();
  return 0;
}