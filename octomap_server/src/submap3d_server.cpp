#include <ros/ros.h>
#include <submap3d_server/Submap3dServer.h>

using namespace submap3d_server;

int main(int argc, char** argv){
  ros::init(argc, argv, "submap3d_server");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

  Submap3dServer server(private_nh, nh);
  ros::spinOnce();

  try{
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    //ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("submap3d_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
