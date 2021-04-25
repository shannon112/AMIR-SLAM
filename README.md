# AMIR-SLAM: Autonomous Mobile Industrial Robot Simultaneous Localization and Mapping

An autonomous mobile industrial robot (AMIR) a.k.a. mobile manipulator, with the eye-in-hand RGB-D camera setting, is able to implement ubiquitous and adaptive mobile manipulation task in a bunch of intelligent services. To perform the autonomous mobile manipulation, the 3D perception of an environment is necessary for the motion planning algorithm to compute collision-free trajectory. We specifically design a 3D SLAM architecture for mobile manipulators, fusing data from sensors on the robot such as 2D laser scan, inertial measurement unit (IMU) data, RGB-D camera data, odometric data and kinematic data to compute an optimized pose, 2D occupancy grid map and 3D occupancy grid map (octomap). Moreover, we also integrate our system with motion planning system, making the robot avoid obstacles autonomously during mobile manipulation in the demonstrations.  
  
The SLAM architecture is based on 2D Cartographer SLAM and extended to have a 3D capability. According to the optimized footprint poses maintained by Cartographer pose graph, we use the kinematic data to transform filtered RGB-D point clouds, building up local submaps which consist of several point clouds by performing the Iterative Closest Point (ICP) technique. To achieve global consistency, the approximate detection is applied to insert a new constraint between two close 3D submap into the pose graph. The non-linear optimization is then conducted on the pose graph with constraints from 3D submaps and Cartographer using Ceres library. Finally, we compose all the submaps by transforming them into world coordinate according to the new optimized poses, getting global 2D and 3D occupancy grid maps.  
  
Keywords: SLAM, 3D Reconstruction, Multi-Sensor Fusion, Mobile Manipulation, Autonomous Mobile Industrial Robot, Obstacle Avoidance, Robot Operating System  
  
<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/system_overview.png" width="840"><img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/mapping_result.png" width="840">

## Related Publication
*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using our work for your own research:*
- R. C. Luo, S. L. Lee, Y. C. Wen and C. H. Hsu, "Modular ROS Based Autonomous Mobile Industrial Robot System for Automated Intelligent Manufacturing Applications," 2020 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM), Boston, MA, USA, 2020, pp. 1673-1678, doi: 10.1109/AIM43001.2020.9158800. pdf: https://ieeexplore.ieee.org/document/9158800, video: https://youtu.be/2rha26UyMa8   
- S. L. Lee, "Autonomous Mobile Industrial Robot with Multi-Sensor Fusion Based Simultaneous Localization and Mapping for Intelligent Service Applications," 2020 National Taiwan University Master Thesis, Taipei, Taiwan, 2020, doi: 10.6342/NTU202003847. pdf: https://hdl.handle.net/11296/q79yre, video: https://youtu.be/Ap8Sr-6QII4, slide: https://www.slideshare.net/secret/C2ACrcWkD41477

- R. C. Luo and S. L. Lee, "Multi-Sensor Based Simultaneous Localization and Mapping System for Autonomous Mobile Industrial Robotic Applications," in IEEE Access, 2020. (Submitted)

### *** ⚠️Warning⚠️ This project is not completed, the code is unarranged and the optimization is needed ***

## 1. Prerequisites

Build from source:
- cartographer: https://github.com/shannon112/cartographer/tree/AMIR-SLAM
- cartographer_ros: https://github.com/shannon112/cartographer_ros/tree/mit_stata_dataset
- ira_laser_tool: https://github.com/shannon112/ira_laser_tools/tree/master
- ceres-solver: https://github.com/shannon112/ceres-solver/tree/testing_1.13.0
- ros_odometry_visualizer: https://github.com/shannon112/ros_odometry_visualizer/tree/master (for ploting fig)
- rgbdslamv2: https://github.com/shannon112/rgbdslamv2_ros_d435/tree/kinetic  (for comparison)
- BLAM: https://github.com/erik-nelson/blam/tree/master (for comparison)
- scorpio_v2: https://github.com/shannon112/scorpio_v2/tree/master (for real robot)
- moveit_extension: https://github.com/shannon112/Moveit-External-Octomap-Updater/tree/master (for autonomous mobile manipulation)


Build with apt:
- ros-kinetic-rtabmap, ros-kinetic-rtabmap-ros, etc.

Dataset:
- MIT Stata Center Data Set: https://projects.csail.mit.edu/stata/downloads.php (2012-04-06-11-15-29.bag)
- Our own dataset: https://drive.google.com/drive/folders/1QCrg-WJSRdKY0BwEpo2rg7WaILU0T1es?usp=sharing

## 2. Building Project
```
 catkin_make_isolated --use-ninja -j8
```

## 3. Examples
Run mapping with MIT STATA Center Data Set or our own dataset:
```
roslaunch cartographer_ros demo_amirslam_mit.launch bag_filenames:=/home/shannon/Downloads/mit19-2012-04-06-11-15-29.bag 
roslaunch cartographer_ros demo_amirslam_our.launch bag_filenames:=/home/shannon/Storage/iros_2.bag

rosrun octomap_server submap3d_server_node cloud_in:=/camera/depth_registered/points frame_id:=/map
rosrun octomap_server submap3d_visualizer_node frame_id:=/map
rosrun octomap_server submap3d_optimizer_node frame_id=/map
python amirslam_extractor.py 
```

Save mapping result:
```
# save cartographer state
rosservice call /finish_trajectory "trajectory_id: 0"
rosservice call /write_state "{filename: '${HOME}/Downloads/AMIRSLAM.bag.pbstream', include_unfinished_submaps: false}"

# save octomap .ot
rosrun octomap_server octomap_saver -f AMIRSLAM.ot

# save pointcloud map .pcd
rosrun pcl_ros pointcloud_to_pcd input:=/map3d 

# save new constraint and pose array as .pkl
rosservice call /save_amir_slam_to_file "{}" 
```

Load mapping result:
```
roslaunch octomap_server result_amir_mit.launch 
python amirslam_loader.py 

rosrun pcl_ros pcd_to_pointcloud AMIRSLAM.pcd  _frame_id:=/map cloud_pcd:=/map_3d
```

## 4. System Overview & Algorithm Walkthrough for Tuning

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/flow_chart_high_level.png" height="200">　<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/flow_chart_low_level.png" height="200">

## 5. Evaluation
We perform the experiments, firstly with an ablation comparison on our architecture, showing how each mechanism improves the performance of the result. Secondly compared with other state-of-art methods with the public SLAM dataset as well as using dataset collected in our own experiment. The result shows that our approach can generate more accurate and more robust maps than other available methods implemented to a mobile manipulator. 

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/evaluation.png" width="840">　

## 6. Demonstration
We demonstrate an autonomous mobile manipulation solution based on our SLAM system, comparing to other available methods without it. The result shows that our system successfully works and enhances motion planning with obstacle avoidance in more comprehensive and convenience way. Last but not the least, we successfully demonstrate an intelligent service scenario of multi-station robotic delivery with our approach from SLAM to autonomous mobile manipulation.

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/demonstration.png" width="840">　

## License
All the packages are distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
AMIR-SLAM is licenced under Apache-2.0 License.

## Acknowledgement
AMIR-SLAM is based on Cartographer SLAM (Hess, Wolfgang, Damon Kohler, Holger Rapp, and Daniel Andor. "Real-time loop closure in 2D LIDAR SLAM."). AMIR-Extension coed is based on octomap-server coding structure.
