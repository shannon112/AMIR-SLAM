#include <ros/ros.h>
#include <submap3d_visualizer/Submap3dVisualizer.h>

#define USAGE "\nUSAGE: octomap_server <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

using namespace submap3d_visualizer;

int main(int argc, char** argv){
  ros::init(argc, argv, "submap3d_visualizer");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");
  std::string mapFilename(""), mapFilenameParam("");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  Submap3dVisualizer server(private_nh, nh);
  ros::spinOnce();

  try{
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    //ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}