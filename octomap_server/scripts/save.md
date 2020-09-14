# save cartographer state
rosservice call /finish_trajectory "trajectory_id: 0"
rosservice call /write_state "{filename: '${HOME}/Downloads/AMIRSLAM.bag.pbstream', include_unfinished_submaps: false}"

# save octomap .ot
rosrun octomap_server octomap_saver -f AMIRSLAM.ot

# save pointcloud map .pcd
rosrun pcl_ros pointcloud_to_pcd input:=/map3d 

# save new constraint and pose array as .pkl
rosservice call /save_amir_slam_to_file "{}" 

roslaunch cartographer_ros demo_amirslam_mit.launch bag_filenames:=/home/shannon/Downloads/mit19-2012-04-06-11-15-29.bag
rosrun octomap_server submap3d_server_node cloud_in:=/camera/depth_registered/points frame_id:=/map
rosrun octomap_server submap3d_visualizer_node frame_id:=/map
rosrun octomap_server submap3d_optimizer_node frame_id:=/map
python amirslam_extractor.py 

roslaunch octomap_server result_amir_mit.launch 
roslaunch cartographer_ros result_amirslam_mit.launch map_filename:=/home/shannon/Documents/master_thesis/v3/AMIRSLAM.bag.pbstream
rosrun pcl_ros pcd_to_pointcloud AMIRSLAM.pcd  _frame_id:=/map cloud_pcd:=/map_3d
python amirslam_loader.py 

frame_id: "/torso_lift_link"
child_frame_id: "/imu_link"
transform: 
translation: 
x: -0.02977
y: -0.1497
z: 0.164

<xacro:pr2_torso_v0 name="torso_lift" parent="base_link">
<origin xyz="-0.05 0 0.739675" rpy="0 0 0" />
</xacro:pr2_torso_v0>

<joint name="${name}_footprint_joint" type="fixed">
    <origin xyz="0 0 0.051" rpy="0 0 0" />
    <child link="${name}_link" />
    <parent link="${name}_footprint"/>
</joint>

total = 0.954675 above