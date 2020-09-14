# AMIR-SLAM: Autonomous Mobile Industrial Robot Simultaneous Localization and Mapping

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/system_overview.png?token=AF244QECXSZSGSA4QIKZMY27MW5NE" width="640">

discription

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/mapping_result.png?token=AF244QDAVMPGDCUASHDBAXK7MW5S6" width="840">

## Related Publication
*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using our work for your own research:*
- R. C. Luo, S. L. Lee, Y. C. Wen and C. H. Hsu, "Modular ROS Based Autonomous Mobile Industrial Robot System for Automated Intelligent Manufacturing Applications," 2020 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM), Boston, MA, USA, 2020, pp. 1673-1678, doi: 10.1109/AIM43001.2020.9158800.
- S. L. Lee, "Autonomous Mobile Industrial Robot with Multi-Sensor Fusion Based Simultaneous Localization and Mapping for Intelligent Service Applications," 2020 National Taiwan University Master Thesis, Taipei, Taiwan, 2020, doi: 10.6342/NTU202003847
- R. C. Luo and S. L. Lee, "Multi-Sensor Based Simultaneous Localization and Mapping System for Autonomous Mobile Industrial Robotic Applications," in IEEE Access, 2020. (Submitted)

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

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/flow_chart_high_level.png?token=AF244QB56G7ACOJAAUJBCB27MW5VW" width="410">　<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/flow_chart_low_level.png?token=AF244QC7TNT2MOWBN23J7C27MW5V2" width="410">

## License
All the packages are distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
AMIR-SLAM is licenced under Apache-2.0 License.

## Acknowledgement
AMIR-SLAM is based on Cartographer SLAM (Hess, Wolfgang, Damon Kohler, Holger Rapp, and Daniel Andor. "Real-time loop closure in 2D LIDAR SLAM.")
