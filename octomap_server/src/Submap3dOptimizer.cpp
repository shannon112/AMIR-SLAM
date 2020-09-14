#include <submap3d_optimizer/Submap3dOptimizer.h>
#include <sstream>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace submap3d_optimizer{

Submap3dOptimizer::Submap3dOptimizer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),

  m_poseArraySub(NULL),
  m_poseStampedSub(NULL),
  m_submapSub(NULL),
  m_nodemapSub(NULL),

  m_tfPoseArraySub(NULL),
  m_tfPoseStampedSub(NULL),
  m_tfSubmapSub(NULL),
  m_tfNodemapSub(NULL),

  m_global_pc_map_temp(new PCLPointCloud),
  m_global_pc_map(new PCLPointCloud),
  isSolved(false),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  m_useHeightMap(true),
  m_useColoredMap(false),
  m_colorFactor(0.8),
  m_latchedTopics(true),
  m_publishFreeSpace(false),
  m_res(0.05),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_pointcloudMinX(-std::numeric_limits<double>::max()),
  m_pointcloudMaxX(std::numeric_limits<double>::max()),
  m_pointcloudMinY(-std::numeric_limits<double>::max()),
  m_pointcloudMaxY(std::numeric_limits<double>::max()),
  m_pointcloudMinZ(-std::numeric_limits<double>::max()),
  m_pointcloudMaxZ(std::numeric_limits<double>::max()),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max()),
  m_minSizeX(0.0), m_minSizeY(0.0),
  m_filterSpeckles(false), m_filterGroundPlane(false),
  m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
  m_compressMap(true),
  m_incrementalUpdate(false),
  m_initConfig(true)
{
  double probHit, probMiss, thresMin, thresMax;

  m_nh_private.param("frame_id", m_worldFrameId, m_worldFrameId); //must change: /map
  m_nh_private.param("base_frame_id", m_baseFrameId, m_baseFrameId); //must change: /base_footprint
  m_nh_private.param("height_map", m_useHeightMap, m_useHeightMap);
  m_nh_private.param("colored_map", m_useColoredMap, m_useColoredMap);
  m_nh_private.param("color_factor", m_colorFactor, m_colorFactor);

  m_nh_private.param("pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  m_nh_private.param("pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  m_nh_private.param("pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  m_nh_private.param("pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  m_nh_private.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  m_nh_private.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  m_nh_private.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  m_nh_private.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  m_nh_private.param("min_x_size", m_minSizeX,m_minSizeX);
  m_nh_private.param("min_y_size", m_minSizeY,m_minSizeY);

  m_nh_private.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
  m_nh_private.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
  // distance of points from plane for RANSAC
  m_nh_private.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
  // angular derivation of found plane:
  m_nh_private.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  m_nh_private.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

  m_nh_private.param("sensor_model/max_range", m_maxRange, m_maxRange);

  m_nh_private.param("resolution", m_res, m_res);
  m_nh_private.param("sensor_model/hit", probHit, 0.7);
  m_nh_private.param("sensor_model/miss", probMiss, 0.4);
  m_nh_private.param("sensor_model/min", thresMin, 0.12);
  m_nh_private.param("sensor_model/max", thresMax, 0.97);
  m_nh_private.param("compress_map", m_compressMap, m_compressMap);
  m_nh_private.param("incremental_2D_projection", m_incrementalUpdate, m_incrementalUpdate);

  if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)){
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<m_pointcloudMinZ <<", "<< m_pointcloudMaxZ << "], excluding the ground level z=0. "
              << "This will not work.");
  }

  if (m_useHeightMap && m_useColoredMap) {
    ROS_WARN_STREAM("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
    m_useColoredMap = false;
  }

  if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
    ROS_INFO_STREAM("Using RGB color registration (if information available)");
#else
    ROS_ERROR_STREAM("Colored map requested in launch file - node not running/compiled to support colors, please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
#endif
  }

  // initialize octomap object & params
  previousTime = ros::WallTime::now();


  m_maxTreeDepth = m_treeDepth;
  m_gridmap.info.resolution = m_res;

  double r, g, b, a;
  m_nh_private.param("color/r", r, 0.0);
  m_nh_private.param("color/g", g, 0.0);
  m_nh_private.param("color/b", b, 1.0);
  m_nh_private.param("color/a", a, 1.0);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  m_nh_private.param("color_free/r", r, 0.0);
  m_nh_private.param("color_free/g", g, 1.0);
  m_nh_private.param("color_free/b", b, 0.0);
  m_nh_private.param("color_free/a", a, 1.0);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  m_nh_private.param("publish_free_space", m_publishFreeSpace, m_publishFreeSpace);

  m_nh_private.param("latch", m_latchedTopics, m_latchedTopics);
  if (m_latchedTopics){
    ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
  } else
    ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");

  // publisher
  m_poseArrayNewPub = m_nh.advertise<geometry_msgs::PoseArray>("trajectory_pose_array_new", 1, m_latchedTopics);
  m_constraintNewPub = m_nh.advertise<visualization_msgs::MarkerArray>("constraint_list_new", 1, m_latchedTopics);
  m_debugPCPub = m_nh.advertise<sensor_msgs::PointCloud2>("debug_pointcloud", 1, m_latchedTopics);

  m_pointCloudSub = m_nh.subscribe<visualization_msgs::MarkerArray>("constraint_list", 5, &Submap3dOptimizer::constraintCallback, this);

  // subscriber
  m_poseArraySub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array", 5);
  m_poseEdgeSub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array", 5);
  //m_poseStampedSub = new message_filters::Subscriber<cartographer_ros_msgs::SubmapList> (m_nh, "submap_list", 5);
  //m_submapSub = new message_filters::Subscriber<octomap_server::PosePointCloud2> (m_nh, "submap3d", 5);
  m_nodemapSub = new message_filters::Subscriber<octomap_server::PosePointCloud2> (m_nh, "nodemap3d", 5);

  // tf listener
  m_tfPoseArraySub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseArraySub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseArraySub->registerCallback(boost::bind(&Submap3dOptimizer::subNodePoseCallback, this, _1));
  m_tfPoseEdgeSub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseEdgeSub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseEdgeSub->registerCallback(boost::bind(&Submap3dOptimizer::subNodeEdgeCallback, this, _1));
  //m_tfPoseStampedSub = new tf::MessageFilter<cartographer_ros_msgs::SubmapList> (*m_poseStampedSub, m_tfListener, m_worldFrameId, 5);
  //m_tfPoseStampedSub->registerCallback(boost::bind(&Submap3dOptimizer::subSubmapPoseCallback, this, _1));
  //m_tfSubmapSub = new tf::MessageFilter<octomap_server::PosePointCloud2> (*m_submapSub, m_tfListener, m_worldFrameId, 5);
  //m_tfSubmapSub->registerCallback(boost::bind(&Submap3dOptimizer::subSubmapMapCallback, this, _1));
  m_tfNodemapSub = new tf::MessageFilter<octomap_server::PosePointCloud2> (*m_nodemapSub, m_tfListener, m_worldFrameId, 5);
  m_tfNodemapSub->registerCallback(boost::bind(&Submap3dOptimizer::subNodeMapCallback, this, _1));

}

Submap3dOptimizer::~Submap3dOptimizer(){
  // tf subscriber
  if (m_tfPoseArraySub){
    delete m_tfPoseArraySub;
    m_tfPoseArraySub = NULL;
  }
  if (m_tfPoseEdgeSub){
    delete m_tfPoseArraySub;
    m_tfPoseEdgeSub = NULL;
  }
  //if (m_tfPoseStampedSub){
  //  delete m_tfPoseStampedSub;
  //  m_tfPoseStampedSub = NULL;
  //}
  if (m_tfNodemapSub){
    delete m_tfNodemapSub;
    m_tfNodemapSub = NULL;
  }
  //if (m_tfSubmapSub){
  //  delete m_tfSubmapSub;
  //  m_tfSubmapSub = NULL;
  //}
  
  // subscriber
  if (m_poseArraySub){
    delete m_poseArraySub;
    m_poseArraySub = NULL;
  }
  if (m_poseEdgeSub){
    delete m_poseEdgeSub;
    m_poseEdgeSub = NULL;
  }
  //if (m_poseStampedSub){
  //  delete m_poseStampedSub;
  //  m_poseStampedSub = NULL;
  //}
  //if (m_submapSub){
  //  delete m_submapSub;
  //  m_submapSub = NULL;
  //}
  if (m_nodemapSub){
    delete m_nodemapSub;
    m_nodemapSub = NULL;
  }

}


// no used now
void Submap3dOptimizer::subSubmapPoseCallback(const cartographer_ros_msgs::SubmapList::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("enter subSubmapPoseCallback");
  return;
}


// no used now
void Submap3dOptimizer::subSubmapMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("enter subSubmapMapCallback");
  return;
}


// pose graph solving, but no used now
void Submap3dOptimizer::constraintCallback(const visualization_msgs::MarkerArray::ConstPtr& constraint_list){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("enter constraintCallback");

  /*
  if (constraints.size()>node_id_now-1){
    ceres::Problem problem;
    ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);
    ceres::examples::SolveOptimizationProblem(&problem);
    //ceres::examples::OutputPoses("poses_optimized.txt", poses);

    publishPoseArray();
    publishConstriant();
  }
  */
  return;
}


// constraint building, sub nodemap poses callback, constraint tag to ConstraintGraph
void Submap3dOptimizer::subNodeEdgeCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();

  // pose graph add constraint: pose and next pose from icp
  for (unsigned i=0; i<node_id_now; i=i+2){
    if (i==0) continue; //no edge
    // search now pose and map
    auto nodeNow = NodeGraph.find(i+1);
    if (nodeNow==NodeGraph.end()) continue;
    PCLPointCloud nowMap = *(nodeNow->second.second);
    Pose nowPose = pose_array->poses[i];

    //pc from now frame to world frame
    Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
    Eigen::Vector3d nowPose_v(nowPose.position.x, nowPose.position.y, nowPose.position.z);
    Eigen::Matrix3d nowPose_R = nowPose_q.toRotationMatrix();
    Eigen::Matrix4d Trans_now; 
    Trans_now.setIdentity();
    Trans_now.block<3,3>(0,0) = nowPose_R;
    Trans_now.block<3,1>(0,3) = nowPose_v;
    pcl::transformPointCloud(nowMap, nowMap, Trans_now);
    //centroid of global pc
    Eigen::Vector4f centroid_now; 
    pcl::compute3DCentroid(nowMap,centroid_now);

    //search close neighbor pose
    int counter = 0;
    for (unsigned j=i-1; j<node_id_now; ++j){
      // duplicate with cartographer
      if (j==i|j==i+1) continue;

      // duplicate with old built edge
      double constraint_check_id = i+0.00001*j; //id_begin + 0.00001*id_end;
      if (ConstraintCheck.find(constraint_check_id) != ConstraintCheck.end()) {
        ROS_WARN("Duplicate constraint with ID: %f", constraint_check_id);
        continue;
      }

      // search neighbor pose and map
      auto nodeNeighbor = NodeGraph.find(j+1);
      if (nodeNeighbor==NodeGraph.end()) continue; //no nodemap found
      PCLPointCloud neighborMap = *(nodeNeighbor->second.second);
      Pose neighborPose = pose_array->poses[j];
      //if (TwoPoseDistance(nowPose,neighborPose)>1.0) continue; //distance threshold = 1m

      //pc from neighbor frame to world frame
      Eigen::Quaterniond neighborPose_q(neighborPose.orientation.w, neighborPose.orientation.x, neighborPose.orientation.y, neighborPose.orientation.z);
      Eigen::Vector3d neighborPose_v(neighborPose.position.x, neighborPose.position.y, neighborPose.position.z);
      Eigen::Matrix3d neighborPose_R = neighborPose_q.toRotationMatrix();
      Eigen::Matrix4d Trans_neighbor; 
      Trans_neighbor.setIdentity();
      Trans_neighbor.block<3,3>(0,0) = neighborPose_R;
      Trans_neighbor.block<3,1>(0,3) = neighborPose_v;
      pcl::transformPointCloud(neighborMap, neighborMap, Trans_neighbor);
      //centroid of global pc
      Eigen::Vector4f centroid_neighbor; 
      pcl::compute3DCentroid(neighborMap,centroid_neighbor);

      if (TwoVectorDistance4f(centroid_now,centroid_neighbor)> 0.5) continue; //distance threshold = 1m
      ++counter;
      if (counter%3!=0) continue;

      // find transforamtion from  icp
      Eigen::Matrix4d transformation;
      if (PairwiseICP_T(nowMap.makeShared(), neighborMap.makeShared(), transformation)){
        Eigen::Matrix4d neighborPose_new = transformation*Trans_neighbor;
        Eigen::Matrix3d neighborPose_new_R = neighborPose_new.block<3,3>(0,0);
        Eigen::Vector3d neighborPose_new_v = neighborPose_new.block<3,1>(0,3);
        neighborPose_new_v.z() = neighborPose_v.z();
        Eigen::Vector3d neighborPose_new_e = neighborPose_new_R.eulerAngles(0, 1, 2); 
        Eigen::Vector3d neighborPose_e = neighborPose_R.eulerAngles(0, 1, 2); 
        Eigen::Quaterniond neighborPose_new_q (neighborPose_new_R);

        ROS_INFO("p2 before:  %f, %f, %f. %f, %f, %f.", neighborPose_v.x(), neighborPose_v.y(),neighborPose_v.z(), neighborPose_e.x(),  neighborPose_e.y(),  neighborPose_e.z());
        ROS_INFO("p2 after:  %f, %f, %f. %f, %f, %f.", neighborPose_new_v.x(), neighborPose_new_v.y(),neighborPose_new_v.z(), neighborPose_new_e.x(), neighborPose_new_e.y(), neighborPose_new_e.z());

        //bad matching
        if (TwoVectorDistance3d(neighborPose_v, neighborPose_new_v)==false){
          ConstraintCheck.insert(constraint_check_id);
          constraint_check_id = j+0.00001*i;
          ConstraintCheck.insert(constraint_check_id);
          continue;
        }

        Pose newPose = Pose();
        newPose.position.x = neighborPose_new_v.x();
        newPose.position.y = neighborPose_new_v.y();
        newPose.position.z = neighborPose_new_v.z();
        newPose.orientation.x = neighborPose_new_q.x();
        newPose.orientation.y = neighborPose_new_q.y();
        newPose.orientation.z = neighborPose_new_q.z();
        newPose.orientation.w = neighborPose_new_q.w();

        Pose deltaPose = Pose();

        Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
        Eigen::Quaterniond newPose_q(newPose.orientation.w, newPose.orientation.x, newPose.orientation.y, newPose.orientation.z);
        Eigen::Matrix3d nowPose_R = nowPose_q.toRotationMatrix();
        Eigen::Vector3d deltaPose_v (newPose.position.x - nowPose.position.x, 
            newPose.position.y - nowPose.position.y, newPose.position.z - nowPose.position.z);
        Eigen::Vector3d result_v = nowPose_R.transpose()*deltaPose_v;
        deltaPose.position.x = result_v.x();
        deltaPose.position.y = result_v.y();
        deltaPose.position.z = result_v.z();

        Eigen::Quaterniond deltaPose_q = nowPose_q.inverse()*newPose_q;
        Eigen::Vector3d euler = deltaPose_q.toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Quaterniond q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());
        deltaPose_q = q;
        deltaPose.orientation.x = deltaPose_q.x();
        deltaPose.orientation.y = deltaPose_q.y();
        deltaPose.orientation.z = deltaPose_q.z();
        deltaPose.orientation.w = deltaPose_q.w();

        double *temp_info_matrix = new double[21]{0.075,0,0,0,0,0,
                                                    0.075,0,0,0,0,
                                                      0.075,0,0,0,
                                                        0.0,0,0,
                                                          0.0,0, 
                                                            0.0};
        InsertConstraint_icp(i, j, deltaPose, temp_info_matrix);
        ConstraintCheck.insert(constraint_check_id);
        // add oppsite constraint
        constraint_check_id = j+0.00001*i;
        ConstraintCheck.insert(constraint_check_id);

        ROS_INFO("Node %zu neighbor to %zu, add constraint", i+1, j+1);
      }
      //break; //only try i with i+1
    }
  }

}



// constraint building, sub nodemap poses callback, constraint tag to ConstraintGraph
void Submap3dOptimizer::subNodePoseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();
  
  // for pose array based constraint visualization
  //m_Poses.clear();
  //for (int i=0; i<pose_array->poses.size(); ++i)
  //  m_Poses.push_back(pose_array->poses[i]);

  // pose graph add vertex 
  for (unsigned i=0; i<node_id_now; ++i){
    Pose nowPose = pose_array->poses[i];
    InsertVertex(i, nowPose);
  }

  // pose graph add constraint: each begin_pose(i-1) and end_pose(i) from pose array which maintain by cartographer
  for (unsigned i=0; i<node_id_now; ++i){
    if (i==0) continue; //no edge
    Pose nowPose = pose_array->poses[i];
    Pose prePose = pose_array->poses[i-1];
    Pose deltaPose = Pose();

    Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
    Eigen::Quaterniond prePose_q(prePose.orientation.w, prePose.orientation.x, prePose.orientation.y, prePose.orientation.z);
  	Eigen::Matrix3d prePose_R = prePose_q.toRotationMatrix();
	  Eigen::Vector3d deltaPose_v (nowPose.position.x - prePose.position.x, 
        nowPose.position.y - prePose.position.y, nowPose.position.z - prePose.position.z);
    Eigen::Vector3d result_v = prePose_R.transpose()*deltaPose_v;
    deltaPose.position.x = result_v.x();
    deltaPose.position.y = result_v.y();
    deltaPose.position.z = result_v.z();

    Eigen::Quaterniond deltaPose_q = prePose_q.inverse()*nowPose_q;
    deltaPose.orientation.x = deltaPose_q.x();
    deltaPose.orientation.y = deltaPose_q.y();
    deltaPose.orientation.z = deltaPose_q.z();
    deltaPose.orientation.w = deltaPose_q.w();

    double *temp_info_matrix = new double[21]{1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,4000,0,0,4000,0,4000};
    InsertConstraint(i-1, i, deltaPose, temp_info_matrix);
  }

  // pose graph add constraint: neighbor pose from icp
  for (auto iter=constraints_icp.begin(); iter!=constraints_icp.end(); ++iter){
    constraints.push_back(*iter);
  }

  std::cout << "Finished pose graph building" << '\n';
  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

  // solving pose graph
  ceres::Problem problem;
  ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);
  ceres::examples::SolveOptimizationProblem(&problem);

  // pub optimized result
  publishPoseArray();
  publishConstriant();

  // clean pose graph
  poses.clear();
  constraints.clear();
  //ConstraintCheck.clear();
}

bool Submap3dOptimizer::TwoVectorDistance3d(const Eigen::Vector3d &pose_target, const Eigen::Vector3d &pose_source){
  if (abs(pose_target.z() - pose_source.z())>0.1) return false; //z should not fly
  float x_delta = pow(pose_target.x() - pose_source.x(), 2);
  float y_delta = pow(pose_target.y() - pose_source.y(), 2);
  if (sqrt(x_delta+y_delta)>0.5) return false;
  return true;
}


// Distance between two pose to determine if they are close (neighbor)
float Submap3dOptimizer::TwoVectorDistance4f(const Eigen::Vector4f &pose_target, const Eigen::Vector4f &pose_source){
  float x_delta = pow(pose_target.x() - pose_source.x(), 2);
  float y_delta = pow(pose_target.y() - pose_source.y(), 2);
  float z_delta = pow(pose_target.z() - pose_source.z(), 2);
  return sqrt(x_delta+y_delta+z_delta);
}

// sub nodemap poses callback, constraint tag to ConstraintGraph, output .txt for testing
/*
void Submap3dOptimizer::subNodePoseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();

  //initialize file i/o
  std::fstream fout;
  fout.open("mit_cartographer.g2o",std::ios::out); //write
  if (!fout) return;
  for (unsigned i=0; i<node_id_now; ++i){
    Pose nowPose = pose_array->poses[i];
    fout<<"VERTEX_SE3:QUAT "<<i<<" "<< nowPose.position.x<<" "<<nowPose.position.y<<" "<<nowPose.position.z<<" "<<
          nowPose.orientation.x<<" "<<nowPose.orientation.y<<" "<<nowPose.orientation.z<<" "<<nowPose.orientation.w<<std::endl;
  }
  for (unsigned i=0; i<node_id_now; ++i){
    if (i==0) continue; //no edge
    Pose nowPose = pose_array->poses[i];
    Pose prePose = pose_array->poses[i-1];
    Pose deltaPose = Pose();

    Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
    Eigen::Quaterniond prePose_q(prePose.orientation.w, prePose.orientation.x, prePose.orientation.y, prePose.orientation.z);
  	Eigen::Matrix3d prePose_R = prePose_q.toRotationMatrix();
	  Eigen::Vector3d deltaPose_v (nowPose.position.x - prePose.position.x, 
        nowPose.position.y - prePose.position.y, nowPose.position.z - prePose.position.z);
    Eigen::Vector3d result_v = prePose_R.transpose()*deltaPose_v;
    deltaPose.position.x = result_v.x();
    deltaPose.position.y = result_v.y();
    deltaPose.position.z = result_v.z();

    Eigen::Quaterniond deltaPose_q = prePose_q.inverse()*nowPose_q;
    deltaPose.orientation.x = deltaPose_q.x();
    deltaPose.orientation.y = deltaPose_q.y();
    deltaPose.orientation.z = deltaPose_q.z();
    deltaPose.orientation.w = deltaPose_q.w();

    fout<<"EDGE_SE3:QUAT "<<i-1<<" "<<i<<" "<< deltaPose.position.x<<" "<<deltaPose.position.y<<" "<<deltaPose.position.z<<" "<<deltaPose.orientation.x<<" "<<deltaPose.orientation.y<<" "<<deltaPose.orientation.z<<" "<<deltaPose.orientation.w;
    fout<<" 1 0 0 0 0 0 "<<"1 0 0 0 0 "<<"1 0 0 0 "<<"4000 0 0 "<<"4000 0 "<<"4000"<<std::endl;
  }

  fout.close();
}
*/


// sub nodemap callback, insert Nodemap to NodeGraph
void Submap3dOptimizer::subNodeMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud){
  ROS_INFO("enter subNodeMapCallback");
  ros::WallTime startTime = ros::WallTime::now();
  PCLPointCloud::Ptr pc (new PCLPointCloud);
  pcl::fromROSMsg(pose_pointcloud->cloud, *pc);
  if (pc->size()<100){
    ROS_WARN("NodeMap ignore size=%zu ", pc->size());
    return;
  }
  NodeGraph.insert(Node(pose_pointcloud->node_id, PoseCloud(pose_pointcloud->pose, pc)));
  ROS_INFO("Get node %d, insert to graph size= %d", pose_pointcloud->node_id, NodeGraph.size());
  return;
}


// publish optimized poses from pose graph
void Submap3dOptimizer::publishPoseArray(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  bool publishPoseArray = (m_latchedTopics || m_poseArrayNewPub.getNumSubscribers() > 0);

  geometry_msgs::PoseArray pose_array;
  pose_array.poses.resize(poses.size());

  if (publishPoseArray){
    for (auto poses_iter=poses.begin(); poses_iter != poses.end(); ++poses_iter){
      auto pair = *poses_iter;
      pose_array.poses[pair.first].position.x = pair.second.p.x();
      pose_array.poses[pair.first].position.y = pair.second.p.y();
      pose_array.poses[pair.first].position.z = pair.second.p.z();
      pose_array.poses[pair.first].orientation.x = pair.second.q.x();
      pose_array.poses[pair.first].orientation.y = pair.second.q.y();
      pose_array.poses[pair.first].orientation.z = pair.second.q.z();
      pose_array.poses[pair.first].orientation.w = pair.second.q.w();
    }

    pose_array.header.stamp = rostime;
    pose_array.header.frame_id = "/map";
    m_poseArrayNewPub.publish(pose_array);
  }
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("New poseArray publishing in optimizer took %f sec", total_elapsed);
  return;
}


// publish PointCloud2 for debug
void Submap3dOptimizer::publishDebugPC(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  bool publishDebugPC = (m_latchedTopics || m_debugPCPub.getNumSubscribers() > 0);

  if (publishDebugPC){
    sensor_msgs::PointCloud2 pcloud;
    pcl::toROSMsg (*m_global_pc_map_temp, pcloud);
    pcloud.header.frame_id = m_worldFrameId;//m_worldFrameId;//
    pcloud.header.stamp = rostime;
    m_debugPCPub.publish(pcloud);
  }
  return;
}


// publish constraint MarkerArray for visualization pose graph
void Submap3dOptimizer::publishConstriant(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  bool publishConstriant = (m_latchedTopics || m_constraintNewPub.getNumSubscribers() > 0);
  visualization_msgs::MarkerArray constraint_list_new;

  if (publishConstriant){

    visualization_msgs::Marker line_list, points;
    line_list.header.frame_id = points.header.frame_id = "/map";
    line_list.header.stamp = points.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    points.ns = "points";
    line_list.action = points.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.pose.position.z = 0.1;
    points.pose.orientation.w = 1.0;
    points.pose.position.z = 0.1;

    points.id = 11;
    line_list.id = 10;

    // type of marker!!
    line_list.type = visualization_msgs::Marker::LINE_LIST; 
    points.type = visualization_msgs::Marker::POINTS; 

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.0125;
    points.scale.x = 0.025;
    points.scale.y = 0.025;

    // go through all constraints and draw a edge from begin pose
    for (auto iter=constraints.begin(); iter!=constraints.end(); ++iter){

      /*
      //use pose array edge end-point for debug
      if (iter->id_begin >= m_Poses.size()) continue;
      geometry_msgs::Point p1;
      p1.x = m_Poses[iter->id_begin].position.x;
      p1.y = m_Poses[iter->id_begin].position.y;
      p1.z = m_Poses[iter->id_begin].position.z;
      Eigen::Quaterniond p1_q(m_Poses[iter->id_begin].orientation.w,m_Poses[iter->id_begin].orientation.x,m_Poses[iter->id_begin].orientation.y,m_Poses[iter->id_begin].orientation.z);
      Eigen::Vector3d p1_v(p1.x,p1.y,p1.z);

      Eigen::Matrix3d t_be_R = p1_q.toRotationMatrix();
      Eigen::Vector3d t_be_v = iter->t_be.p;
      Eigen::Vector3d p2_v = p1_v + t_be_R*t_be_v;
      */

      //use vertex as edge end-point
      auto p1_iter = poses.find(iter->id_begin);
      if (p1_iter == poses.end()) continue;  //begin pose not found
      geometry_msgs::Point p1;
      p1.x = p1_iter->second.p.x();
      p1.y = p1_iter->second.p.y();
      p1.z = p1_iter->second.p.z();
      
      /*
      // calculated p2
      //Eigen::Matrix3d t_be_R = p1_iter->second.q.toRotationMatrix();
      //Eigen::Vector3d t_be_v = iter->t_be.p;
      //Eigen::Vector3d p2_v = p1_iter->second.p + t_be_R*t_be_v;//t_be_R*(p1_iter->second.p) + t_be_v;

      geometry_msgs::Point p2;
      p2.x = p2_v.x();
      p2.y = p2_v.y();
      p2.z = p2_v.z(); 
      */

      // direct draw p2
      auto p2_iter = poses.find(iter->id_end);
      if (p2_iter == poses.end()) continue;  //begin pose not found
      geometry_msgs::Point p2;
      p2.x = p2_iter->second.p.x();
      p2.y = p2_iter->second.p.y();
      p2.z = p2_iter->second.p.z();

      // Line list is pink
      std_msgs::ColorRGBA c;
      c.r = 253.0/255.0;
      c.g = 31.0/255.0;
      c.b = 246.0/255.0;
      c.a = 0.83;

      // The line list needs two points for each line
      line_list.points.push_back(p1);
      line_list.points.push_back(p2);
      line_list.colors.push_back(c);
      line_list.colors.push_back(c);

      // The point indicate the begin point of edge with deep pink
      points.points.push_back(p1);
      c.r = 155.0/255.0;
      c.g = 12.0/255.0;
      c.b = 129.0/255.0;
      points.colors.push_back(c);
    }
    constraint_list_new.markers.push_back(line_list);
    constraint_list_new.markers.push_back(points);
    m_constraintNewPub.publish(constraint_list_new);
  }
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("New constraints publishing in optimizer took %f sec", total_elapsed);
}


// Distance between two pose to determine if they are close (neighbor)
float Submap3dOptimizer::TwoPoseDistance(const Pose &pose_target, const Pose &pose_source){
  float x_delta = pow(pose_target.position.x - pose_source.position.x, 2);
  float y_delta = pow(pose_target.position.y - pose_source.position.y, 2);
  float z_delta = pow(pose_target.position.z - pose_source.position.z, 2);
  return sqrt(x_delta+y_delta+z_delta);
}


// ICP for return joined cloud
void Submap3dOptimizer::PairwiseICP(const PCLPointCloud::Ptr &cloud_target, const PCLPointCloud::Ptr &cloud_source, PCLPointCloud::Ptr &output )
{
  ros::WallTime startTime = ros::WallTime::now();
	PCLPointCloud::Ptr src(new PCLPointCloud);
	PCLPointCloud::Ptr tgt(new PCLPointCloud);
 
	tgt = cloud_target;
	src = cloud_source;
 
	pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(0.3); //ignore the point out of distance(m)
  // Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon(1e-6); //converge criterion
  // Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon(1); //diverge threshold
  // Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (5);
	icp.setInputSource (src);
	icp.setInputTarget (tgt);
	icp.align (*output);
	
  if (icp.hasConverged()){
    output->resize(tgt->size()+output->size());
    for (int i=0;i<tgt->size();i++)
    {
      output->push_back(tgt->points[i]);
    }
  }else{
    *output += *tgt;
  }
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("After registration using ICP pointcloud size: %d, time cost: %f(s), converged: %d", output->size(), total_elapsed, icp.hasConverged());
}


// ICP for return transformation matrix 4d
bool Submap3dOptimizer::PairwiseICP_T(const PCLPointCloud::Ptr &cloud_target, const PCLPointCloud::Ptr &cloud_source, Eigen::Matrix4d &output_trans )
{
  ros::WallTime startTime = ros::WallTime::now();
	PCLPointCloud::Ptr src(new PCLPointCloud);
	PCLPointCloud::Ptr tgt(new PCLPointCloud);
 	PCLPointCloud::Ptr src_aligned(new PCLPointCloud);

	tgt = cloud_target;
	src = cloud_source;

  //*m_global_pc_map_temp += *cloud_target;
  //*m_global_pc_map_temp += *cloud_source;
  //publishDebugPC();
  //m_global_pc_map_temp->clear();

	pcl::GeneralizedIterativeClosestPoint<PCLPoint, PCLPoint> icp;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	//icp.setMaxCorrespondenceDistance(0.3); //ignore the point out of distance(m)
  // Set the transformation epsilon (criterion 2)
	//icp.setTransformationEpsilon(1e-6); //converge criterion
  // Set the euclidean distance difference epsilon (criterion 3)
	//icp.setEuclideanFitnessEpsilon(1); //diverge threshold
  // Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (20);
	icp.setInputSource (src);
	icp.setInputTarget (tgt);
	icp.align (*src_aligned);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("After registration using ICP pointcloud, time cost: %f(s), converged: %d, score: %f", total_elapsed, icp.hasConverged(), icp.getFitnessScore());

  if (icp.hasConverged()){
    output_trans = icp.getFinalTransformation().cast<double>();
    return true;
  }else{
    return false;
  }
}


// Pose-graph insert a vertex
bool Submap3dOptimizer::InsertVertex(const int &id, const Pose &vertex){

  // Ensure we don't have duplicate poses.
  if (poses.find(id) != poses.end()) {
    //ROS_WARN("Duplicate vertex with ID: %d", id);
    return false;
  }

  ceres::examples::Pose3d vertex_ceres;
  vertex_ceres.p.x() = vertex.position.x;
  vertex_ceres.p.y() = vertex.position.y;
  vertex_ceres.p.z() = vertex.position.z;

  vertex_ceres.q.w() = vertex.orientation.w;
  vertex_ceres.q.x() = vertex.orientation.x;
  vertex_ceres.q.y() = vertex.orientation.y;
  vertex_ceres.q.z() = vertex.orientation.z;

  poses[id] = vertex_ceres;

  return true;
}


// Pose-graph insert a constraint edge
void Submap3dOptimizer::InsertConstraint(const int &id_begin, const int &id_end, const Pose &t_be, const double* info_matrix) {
  ceres::examples::Constraint3d constraint;
  constraint.id_begin = id_begin;
  constraint.id_end = id_end;

  constraint.t_be.p.x() = t_be.position.x;
  constraint.t_be.p.y() = t_be.position.y;
  constraint.t_be.p.z() = t_be.position.z;

  constraint.t_be.q.w() = t_be.orientation.w;
  constraint.t_be.q.x() = t_be.orientation.x;
  constraint.t_be.q.y() = t_be.orientation.y;
  constraint.t_be.q.z() = t_be.orientation.z;

  int counter = 0;
  for (int i = 0; i < 6; ++i) {
    for (int j = i; j < 6; ++j) {
      constraint.information(i, j) = info_matrix[counter];
      ++counter;
      if (i != j) {
        constraint.information(j, i) = constraint.information(i, j);
      }
    }
  }
  constraints.push_back(constraint);
}


// Pose-graph insert a constraint edge
void Submap3dOptimizer::InsertConstraint_icp(const int &id_begin, const int &id_end, const Pose &t_be, const double* info_matrix) {
  ceres::examples::Constraint3d constraint;
  constraint.id_begin = id_begin;
  constraint.id_end = id_end;

  constraint.t_be.p.x() = t_be.position.x;
  constraint.t_be.p.y() = t_be.position.y;
  constraint.t_be.p.z() = t_be.position.z;

  constraint.t_be.q.w() = t_be.orientation.w;
  constraint.t_be.q.x() = t_be.orientation.x;
  constraint.t_be.q.y() = t_be.orientation.y;
  constraint.t_be.q.z() = t_be.orientation.z;

  int counter = 0;
  for (int i = 0; i < 6; ++i) {
    for (int j = i; j < 6; ++j) {
      constraint.information(i, j) = info_matrix[counter];
      ++counter;
      if (i != j) {
        constraint.information(j, i) = constraint.information(i, j);
      }
    }
  }
  constraints_icp.push_back(constraint);
}

}
