#include "LIVMapper.h"
#include <csignal>

int main(int argc, char **argv)
{
  // Ctrl+C in a launch terminal signals the whole foreground process group, so a
  // `... | tee log` wrapper dies at the same instant the mapper starts savePCD().
  // The next std::cout inside savePCD() then hits a broken pipe and SIGPIPE kills
  // the process mid-save -- which is how Log/pcd/ ended up with all_raw_points.pcd
  // but no all_downsampled_points.pcd. Ignoring SIGPIPE turns that into a failed
  // write() that we simply don't care about, so the save runs to completion.
  std::signal(SIGPIPE, SIG_IGN);

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping");
  mapper.initializeSubscribersAndPublishers(nh, it_);
  mapper.run(nh);
  mapper.savePCD();   // also runs if run() returned via rclcpp::ok() going false
  rclcpp::shutdown();
  return 0;
}
