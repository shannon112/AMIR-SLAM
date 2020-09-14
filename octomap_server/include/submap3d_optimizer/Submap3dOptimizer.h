/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_H
#define OCTOMAP_SERVER_OCTOMAPSERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

#include "pose_graph_3d/pose_graph_3d.h"

// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/CollisionMap.h>

// Msg and Srv
#include <octomap_server/PosePointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <std_msgs/ColorRGBA.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

//#define COLOR_OCTOMAP_SERVER // switch color here - easier maintenance, only maintain Submap3dOptimizer. Two targets are defined in the cmake, octomap_server_color and octomap_server. One has this defined, and the other doesn't

#ifdef COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif

namespace submap3d_optimizer {
class Submap3dOptimizer {

public:
#ifdef COLOR_OCTOMAP_SERVER
  typedef pcl::PointXYZRGB PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
  typedef octomap::ColorOcTree OcTreeT;
#else
  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef octomap::OcTree OcTreeT;
#endif
  typedef octomap_msgs::GetOctomap OctomapSrv;
  typedef octomap_msgs::BoundingBoxQuery BBXSrv;
  typedef geometry_msgs::PoseArray PoseArray;
  typedef geometry_msgs::Pose Pose;
  typedef std::pair<Pose,PCLPointCloud::Ptr> PoseCloud;
  typedef std::pair<int,PoseCloud> Node;

  Submap3dOptimizer(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"), const ros::NodeHandle &nh_ = ros::NodeHandle());
  virtual ~Submap3dOptimizer();

  virtual void constraintCallback(const visualization_msgs::MarkerArray::ConstPtr& pose_array);
  virtual void subSubmapPoseCallback(const cartographer_ros_msgs::SubmapList::ConstPtr& pose);
  virtual void subNodePoseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array);
  virtual void subNodeEdgeCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array);
  virtual void subNodeMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud);
  virtual void subSubmapMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud);

protected:

  virtual void publishDebugPC(const ros::Time& rostime = ros::Time::now());
  virtual void publishPoseArray(const ros::Time& rostime = ros::Time::now());
  virtual void publishConstriant(const ros::Time& rostime = ros::Time::now());

  virtual bool PairwiseICP_T(const PCLPointCloud::Ptr &cloud_target, const PCLPointCloud::Ptr &cloud_source, Eigen::Matrix4d &output_trans );
  virtual void PairwiseICP(const PCLPointCloud::Ptr &cloud_target, const PCLPointCloud::Ptr &cloud_source, PCLPointCloud::Ptr &output );
  virtual float TwoPoseDistance(const Pose &pose_target, const Pose &pose_source);
  virtual bool InsertVertex(const int &id, const Pose &vertex);
  virtual void InsertConstraint(const int &id_begin, const int &id_end, const Pose &t_be, const double* info_matrix);
  virtual void InsertConstraint_icp(const int &id_begin, const int &id_end, const Pose &t_be, const double* info_matrix);
  virtual float TwoVectorDistance4f(const Eigen::Vector4f &pose_target, const Eigen::Vector4f &pose_source);
  virtual bool TwoVectorDistance3d(const Eigen::Vector3d &pose_target, const Eigen::Vector3d &pose_source);

  ros::NodeHandle m_nh;
  ros::NodeHandle m_nh_private;

  ros::Publisher  m_poseArrayNewPub, m_constraintNewPub, m_debugPCPub;
  ros::Subscriber m_pointCloudSub;

  message_filters::Subscriber<geometry_msgs::PoseArray>* m_poseArraySub;
  message_filters::Subscriber<geometry_msgs::PoseArray>* m_poseEdgeSub;
  message_filters::Subscriber<cartographer_ros_msgs::SubmapList>* m_poseStampedSub;
  message_filters::Subscriber<octomap_server::PosePointCloud2>* m_submapSub;
  message_filters::Subscriber<octomap_server::PosePointCloud2>* m_nodemapSub;

  tf::MessageFilter<geometry_msgs::PoseArray>* m_tfPoseArraySub;
  tf::MessageFilter<geometry_msgs::PoseArray>* m_tfPoseEdgeSub;
  tf::MessageFilter<cartographer_ros_msgs::SubmapList>* m_tfPoseStampedSub;
  tf::MessageFilter<octomap_server::PosePointCloud2>* m_tfSubmapSub;
  tf::MessageFilter<octomap_server::PosePointCloud2>* m_tfNodemapSub;

  tf::TransformListener m_tfListener;

  double m_maxRange;
  std::string m_worldFrameId; // the map frame
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  bool m_useHeightMap;
  std_msgs::ColorRGBA m_color;
  std_msgs::ColorRGBA m_colorFree;
  double m_colorFactor;

  bool m_latchedTopics;
  bool m_publishFreeSpace;

  double m_res;
  unsigned m_treeDepth;
  unsigned m_maxTreeDepth;

  double m_pointcloudMinX;
  double m_pointcloudMaxX;
  double m_pointcloudMinY;
  double m_pointcloudMaxY;
  double m_pointcloudMinZ;
  double m_pointcloudMaxZ;
  double m_occupancyMinZ;
  double m_occupancyMaxZ;
  double m_minSizeX;
  double m_minSizeY;
  bool m_filterSpeckles;

  bool m_filterGroundPlane;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;

  bool m_compressMap;

  bool m_initConfig;

  // downprojected 2D map:
  bool m_incrementalUpdate;
  nav_msgs::OccupancyGrid m_gridmap;
  bool m_publish2DMap;
  bool m_mapOriginChanged;
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
  bool m_useColoredMap;

  // pose array
  PCLPointCloud::Ptr m_global_pc_map;
  PCLPointCloud::Ptr m_global_pc_map_temp;

  bool isSolved;
  std::vector<Pose> m_Poses; //for debug
  std::unordered_map<int, PoseCloud> NodeGraph;
  std::unordered_set<double> ConstraintCheck;
  ceres::examples::MapOfPoses poses;
  ceres::examples::VectorOfConstraints constraints;
  ceres::examples::VectorOfConstraints constraints_icp;

  ros::WallTime previousTime;
};
}

#endif