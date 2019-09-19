
#include "icpslam/icpslam.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "icpslam");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();
  IcpSlam icpslam(nh, pnh);
  spinner.stop();

  return EXIT_SUCCESS;
}