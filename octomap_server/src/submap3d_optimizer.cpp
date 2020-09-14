#include <ros/ros.h>
#include <submap3d_optimizer/Submap3dOptimizer.h>

using namespace submap3d_optimizer;

int main(int argc, char** argv){
  ros::init(argc, argv, "submap3d_optimizer");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

  Submap3dOptimizer server(private_nh, nh);
  ros::spinOnce();

  try{
    ros::MultiThreadedSpinner spinner(6);
    spinner.spin();
    //ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("submap3d_optimizer exception: %s", e.what());
    return -1;
  }

  return 0;
}
