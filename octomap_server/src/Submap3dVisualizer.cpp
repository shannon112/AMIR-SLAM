#include <submap3d_visualizer/Submap3dVisualizer.h>
#include <sstream>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace submap3d_visualizer{

Submap3dVisualizer::Submap3dVisualizer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),
  m_pointCloudSub(NULL),
  m_poseArraySub(NULL),
  m_posePointCloudSub(NULL),
  m_poseStampedSub(NULL),
  m_tfPointCloudSub(NULL),
  m_tfPoseArraySub(NULL),
  m_tfPoseStampedSub(NULL),
  m_tfPosePointCloudSub(NULL),

  refined(false),
  m_SizePoses(0),
  m_global_pc_map_temp(new PCLPointCloud),
  m_global_pc_map(new PCLPointCloud),
  m_local_pc_map(new PCLPointCloud),

  m_octree(NULL),
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

  m_octree = new OcTreeT(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);
  m_treeDepth = m_octree->getTreeDepth();
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
  m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, m_latchedTopics);
  m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1, m_latchedTopics);
  m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, m_latchedTopics);
  m_projectedMapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, m_latchedTopics);
  m_map3dPub = m_nh.advertise<sensor_msgs::PointCloud2>("map3d", 1, m_latchedTopics);

  // subscriber
  m_refineSub = m_nh.subscribe<std_msgs::Empty>("to_refine_map", 5, &Submap3dVisualizer::refineCallback, this);

  m_pointCloudSub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array_new", 5);
  m_poseArraySub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array_new", 5);
  m_poseStampedSub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array_new", 5);
  m_posePointCloudSub = new message_filters::Subscriber<octomap_server::PosePointCloud2> (m_nh, "nodemap3d", 5);

  // tf listener
  m_tfPointCloudSub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
  m_tfPointCloudSub->registerCallback(boost::bind(&Submap3dVisualizer::insertCloudCallback, this, _1));
  m_tfPoseArraySub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseArraySub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseArraySub->registerCallback(boost::bind(&Submap3dVisualizer::subNodePoseCallback, this, _1));
  m_tfPoseStampedSub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseStampedSub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseStampedSub->registerCallback(boost::bind(&Submap3dVisualizer::insertSubmap3dposeCallback, this, _1));
  m_tfPosePointCloudSub = new tf::MessageFilter<octomap_server::PosePointCloud2> (*m_posePointCloudSub, m_tfListener, m_worldFrameId, 5);
  m_tfPosePointCloudSub->registerCallback(boost::bind(&Submap3dVisualizer::subNodeMapCallback, this, _1));


  // service
  m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &Submap3dVisualizer::octomapBinarySrv, this);
  m_octomapFullService = m_nh.advertiseService("octomap_full", &Submap3dVisualizer::octomapFullSrv, this);
  m_clearBBXService = m_nh_private.advertiseService("clear_bbx", &Submap3dVisualizer::clearBBXSrv, this);
  m_resetService = m_nh_private.advertiseService("reset", &Submap3dVisualizer::resetSrv, this);

}

Submap3dVisualizer::~Submap3dVisualizer(){
  if (m_tfPointCloudSub){
    delete m_tfPointCloudSub;
    m_tfPointCloudSub = NULL;
  }

  if (m_pointCloudSub){
    delete m_pointCloudSub;
    m_pointCloudSub = NULL;
  }

  if (m_poseArraySub){
    delete m_poseArraySub;
    m_poseArraySub = NULL;
  }

  if (m_tfPoseArraySub){
    delete m_tfPoseArraySub;
    m_tfPoseArraySub = NULL;
  }

  if (m_tfPoseStampedSub){
    delete m_tfPoseStampedSub;
    m_tfPoseStampedSub = NULL;
  }

  if (m_tfPosePointCloudSub){
    delete m_tfPosePointCloudSub;
    m_tfPosePointCloudSub = NULL;
  }

  if (m_octree){
    delete m_octree;
    m_octree = NULL;
  }

}

void Submap3dVisualizer::refineCallback(const std_msgs::Empty::ConstPtr& command){
  refined = true;
  std::cout<<"start refining map"<<std::endl;

  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("pointcloud get pose vector sized %zu", m_Poses.size());
  unsigned node_id_now = m_Poses.size();

  m_local_pc_map->clear();
  m_global_pc_map->clear();

  //update global map, tuning with live pose_array from cartographer
  int counter = 0;
  for (unsigned i=0; i<node_id_now; ++i){
    auto nodeExist = NodeGraph.find(i+1);
    if (nodeExist!=NodeGraph.end()){
      PCLPointCloud temp = *(nodeExist->second.second);
      Pose nowPose = m_Poses[i];

      Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
      Eigen::Vector3d nowPose_v(nowPose.position.x, nowPose.position.y, nowPose.position.z);
      Eigen::Matrix3d nowPose_R = nowPose_q.toRotationMatrix();
      Eigen::Matrix4d Trans; 
      Trans.setIdentity();
      Trans.block<3,3>(0,0) = nowPose_R;
      Trans.block<3,1>(0,3) = nowPose_v;
      pcl::transformPointCloud(temp, temp, Trans);

      //centroid of global pc
      Eigen::Vector4f centroid; 
      pcl::compute3DCentroid(temp,centroid);
      GlobalGraph.push_back(Global(centroid, temp));

      //insert nodemap to pointcloud map
      //first map
      /*
      if (m_local_pc_map->size()==0) {
        *m_local_pc_map+=temp;
        std::cout<<"@directly  add "<<i<<std::endl;
      }
      //second map
      else{
        PCLPointCloud::Ptr m_local_new(new PCLPointCloud);
        // successfully merge
        if (PairwiseICP(m_local_pc_map, temp.makeShared(), m_local_new)) {
          m_local_pc_map = m_local_new;
          std::cout<<"@icp  add "<<i<<std::endl;
        } 
        // fail to merge
        else {
          std::cout<<"@icp drop "<<i<<std::endl;
        }
        *m_global_pc_map += *m_local_pc_map;
        m_local_pc_map->clear();
      }
      */
    }//if node exist
  }//for

  //search neighbor and icp merge
  for (auto iter=GlobalGraph.begin(); iter!=GlobalGraph.end(); ++iter){
    // find one which is not integrated yet
    if (Integrated.find(iter-GlobalGraph.begin())!=Integrated.end() ) continue; //that map is already integrated
    Integrated.insert(iter-GlobalGraph.begin());

    Eigen::Vector4f centroidNow = iter->first;
    PCLPointCloud::Ptr pcNow = (iter->second).makeShared();
    *m_local_pc_map+= *pcNow;

    // iteratively find neighbor
    //int lastmapId = iter - GlobalGraph.begin();
    for (auto iterN=GlobalGraph.begin(); iterN!=GlobalGraph.end(); ++iterN){
      if (Integrated.find(iterN-GlobalGraph.begin())!=Integrated.end() ) continue; //that map is already integrated

      Eigen::Vector4f centroidNeighbor = iterN->first;
      // see if it is really close enough
      if (TwoVectorDistance(centroidNeighbor, centroidNow) < 1.0){
        std::cout<<"@find ("<< iter-GlobalGraph.begin() <<"/"<<node_id_now <<") neighbor is "<< iterN-GlobalGraph.begin()<<std::endl;

        // see if that is too close
        //if (abs(iterN-GlobalGraph.begin() - lastmapId)<5) continue;
        //lastmapId = iterN-GlobalGraph.begin();

        PCLPointCloud::Ptr pcNeighbor = (iterN->second).makeShared();

        PCLPointCloud::Ptr m_local_new(new PCLPointCloud);
        if (PairwiseICP(m_local_pc_map, pcNeighbor, m_local_new)==true) {
          m_local_pc_map = m_local_new;
          std::cout<<"@integrated "<< iter-GlobalGraph.begin() <<" and its neighbor is "<< iterN-GlobalGraph.begin()<<std::endl;
          Integrated.insert(iterN-GlobalGraph.begin());

          // voxel filter
          PCLPointCloud::Ptr vx_pc (new PCLPointCloud);
          pcl::VoxelGrid<PCLPoint> voxel_filter;
          voxel_filter.setLeafSize( 0.05, 0.05, 0.05);
          voxel_filter.setInputCloud(m_local_pc_map);
          voxel_filter.filter(*vx_pc);
          m_local_pc_map = vx_pc;
        }
        else  {
          std::cout<<"@abort "<< iterN-GlobalGraph.begin()<<std::endl;
        }

        //*m_local_pc_map += iterN->second; //debug
      }
    }
    *m_global_pc_map += *m_local_pc_map;
    m_local_pc_map->clear();
  }

  //publish pointcloud globalmap
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  if ((ros::WallTime::now()-previousTime).toSec() > 2){
    previousTime = ros::WallTime::now();
    ros::Rate r(0.5); // 10 hz
    while (ros::ok())
    {
      ROS_INFO("~Pub refined pointcloud map");
      publishPCmap3d();
      r.sleep();
    }
    GlobalGraph.clear();
    Integrated.clear();
  }
}


// pointcloud map building
void Submap3dVisualizer::insertSubmap3dposeCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("pointcloud get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();

  // for pose array based constraint visualization
  m_Poses.clear();
  for (int i=0; i<pose_array->poses.size(); ++i)
    m_Poses.push_back(pose_array->poses[i]);

  if (refined) return;

  //update global map, tuning with live pose_array from cartographer
  int counter = 0;
  int base_map_id = 0;
  for (unsigned i=0; i<node_id_now; ++i){
    auto nodeExist = NodeGraph.find(i+1);
    if (nodeExist!=NodeGraph.end()){
      PCLPointCloud temp = *(nodeExist->second.second);
      Pose nowPose = pose_array->poses[i];

      Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
      Eigen::Vector3d nowPose_v(nowPose.position.x, nowPose.position.y, nowPose.position.z);
      Eigen::Matrix3d nowPose_R = nowPose_q.toRotationMatrix();
      Eigen::Matrix4d Trans; 
      Trans.setIdentity();
      Trans.block<3,3>(0,0) = nowPose_R;
      Trans.block<3,1>(0,3) = nowPose_v;
      pcl::transformPointCloud(temp, temp, Trans);

      *m_global_pc_map += temp;
    }//if node exist
  }//for

  //publish pointcloud globalmap
  if ((ros::WallTime::now()-previousTime).toSec() > 2){
    previousTime = ros::WallTime::now();
    publishPCmap3d(pose_array->header.stamp);
    m_global_pc_map->clear();
  }
  ROS_INFO("Cleared pointcloud map");
}


// Distance between two pose to determine if they are close (neighbor)
float Submap3dVisualizer::TwoVectorDistance(const Eigen::Vector4f &pose_target, const Eigen::Vector4f &pose_source){
  float x_delta = pow(pose_target.x() - pose_source.x(), 2);
  float y_delta = pow(pose_target.y() - pose_source.y(), 2);
  float z_delta = pow(pose_target.z() - pose_source.z(), 2);
  return sqrt(x_delta+y_delta+z_delta);
}

// octomap map building
void Submap3dVisualizer::insertCloudCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("octomap get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();

  //update global map, tuning with live pose_array from cartographer
  for (unsigned i=0; i<node_id_now; ++i){
    auto nodeExist = NodeGraph.find(i+1);
    if (nodeExist!=NodeGraph.end()){
      PCLPointCloud temp = *(nodeExist->second.second);
      Pose nowPose = pose_array->poses[i];
      Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
      Eigen::Vector3d nowPose_v(nowPose.position.x, nowPose.position.y, nowPose.position.z);
      Eigen::Matrix3d nowPose_R = nowPose_q.toRotationMatrix();
      Eigen::Matrix4d Trans; 
      Trans.setIdentity();
      Trans.block<3,3>(0,0) = nowPose_R;
      Trans.block<3,1>(0,3) = nowPose_v;
      pcl::transformPointCloud(temp, temp, Trans);

      //insert submap to octomap
      //tf::Point sensorOrigin(nowPose.position.x, nowPose.position.y, nowPose.position.z);
      //insertScan(sensorOrigin, temp);
      insertPC(nowPose, temp);
      total_elapsed = (ros::WallTime::now() - startTime).toSec();
      ROS_INFO("Octomap insertion is done, %f sec)", total_elapsed);
    }
  }

  size_t octomapSize = m_octree->size();
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  //publish octomap globalmap
  m_octree->updateInnerOccupancy();
  m_octree->prune();
  //publishOCTmap3d(pose_array->header.stamp);
  publishBinaryOctoMap(pose_array->header.stamp);
  publishFullOctoMap(pose_array->header.stamp);
  m_octree->clear();
  m_gridmap.data.clear();
  m_gridmap.info.height = 0.0;
  m_gridmap.info.width = 0.0;
  m_gridmap.info.resolution = 0.0;
  m_gridmap.info.origin.position.x = 0.0;
  m_gridmap.info.origin.position.y = 0.0;
  ROS_INFO("Cleared octomap & projected 2d map");

  return;
}

// temp map building, maybe good to work as service
void Submap3dVisualizer::subNodePoseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){

}

bool Submap3dVisualizer::PairwiseICP(const PCLPointCloud::Ptr &cloud_target, const PCLPointCloud::Ptr &cloud_source, PCLPointCloud::Ptr &output )
{
  ros::WallTime startTime = ros::WallTime::now();
	PCLPointCloud::Ptr src(new PCLPointCloud);
	PCLPointCloud::Ptr tgt(new PCLPointCloud);
 
	tgt = cloud_target;
	src = cloud_source;
 
	pcl::GeneralizedIterativeClosestPoint<PCLPoint, PCLPoint> icp;
	//icp.setMaxCorrespondenceDistance(1.5); //ignore the point out of distance(m)
	//icp.setTransformationEpsilon(1e-6); //converge criterion
	//icp.setEuclideanFitnessEpsilon(1); //diverge threshold
	icp.setMaximumIterations (20);
	icp.setInputSource (src);
	icp.setInputTarget (tgt);
	icp.align (*output);
	
  if (icp.hasConverged()){
    *output += *tgt;
  }else{
    return false;
  }
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("After registration using ICP pointcloud size: %d, time cost: %f(s), converged: %d", output->size(), total_elapsed, icp.hasConverged());
  return true;
}

// posepointcloud input callback,  insert Node to NodeGraph
void Submap3dVisualizer::subNodeMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud){
  ros::WallTime startTime = ros::WallTime::now();
  PCLPointCloud::Ptr pc (new PCLPointCloud);
  pcl::fromROSMsg(pose_pointcloud->cloud, *pc);
  if (pc->size()<100){
    ROS_WARN("NodeMap ignore size=%zu ", pc->size());
    return;
  }
  //NodeGraph.insert(Node(pose_pointcloud->node_id, pc));
  NodeGraph.insert(Node(pose_pointcloud->node_id, PoseCloud(pose_pointcloud->pose, pc)));
  ROS_INFO("@Get node %d, insert to graph size= %d", pose_pointcloud->node_id, NodeGraph.size());
  return;
}

// processing one pointcloud input
void Submap3dVisualizer::insertPC(const Pose& sensorOrigin, const PCLPointCloud& nonground){
  ros::WallTime startTime = ros::WallTime::now();

  octomap::Pointcloud cloud_octo;
  for (auto p:nonground.points)
    cloud_octo.push_back( p.x, p.y, p.z );
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("To octomap cloud %f", total_elapsed);

  m_octree->insertPointCloud( cloud_octo, 
    octomap::point3d( sensorOrigin.position.x, sensorOrigin.position.y, sensorOrigin.position.z ) );
  total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Insert to octree %f", total_elapsed);

  //for (auto p:temp->points)
  //  m_octree->integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
}

// processing one pointcloud input
void Submap3dVisualizer::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& nonground){
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

  if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
    || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  {
    ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  }

#ifdef COLOR_OCTOMAP_SERVER
  unsigned char* colors = new unsigned char[3];
#endif

  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells, occupied_cells;

  // all other points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it){
    point3d point(it->x, it->y, it->z);

    // free cells
    if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
      free_cells.insert(m_keyRay.begin(), m_keyRay.end());
    }
    // occupied endpoint
    OcTreeKey key;
    if (m_octree->coordToKeyChecked(point, key)){
      occupied_cells.insert(key);

      updateMinKey(key, m_updateBBXMin);
      updateMaxKey(key, m_updateBBXMax);

#ifdef COLOR_OCTOMAP_SERVER // NB: Only read and interpret color if it's an occupied node
      m_octree->averageNodeColor(it->x, it->y, it->z, /*r=*/it->r, /*g=*/it->g, /*b=*/it->b);
#endif
    }

  }

  // mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      m_octree->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    m_octree->updateNode(*it, true);
  }

  if (m_compressMap) //default yes
    m_octree->prune();

#ifdef COLOR_OCTOMAP_SERVER
  if (colors)
  {
    delete[] colors;
    colors = NULL;
  }
#endif
}

// publish stored poses and optimized local submap
void Submap3dVisualizer::publishPCmap3d(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  bool publishPCmap3d = (m_latchedTopics || m_map3dPub.getNumSubscribers() > 0);

  if (publishPCmap3d){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (*m_global_pc_map, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_map3dPub.publish(cloud);
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("PointClouldMap3d publishing in visualizer took %f sec", total_elapsed);
}

void Submap3dVisualizer::publishOCTmap3d(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->size();
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
  bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
  bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
  m_publish2DMap = (m_latchedTopics || m_projectedMapPub.getNumSubscribers() > 0);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> pclCloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
      end = m_octree->end(); it != end; ++it)
  {
    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (inUpdateBBX)
      handleNodeInBBX(it);

    if (m_octree->isNodeOccupied(*it)){
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        // Ignore speckles in the map:
        if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
          ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        } // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (inUpdateBBX)
          handleOccupiedNodeInBBX(it);

        // insert into pointcloud:
        if (publishPointCloud) {
#ifdef COLOR_OCTOMAP_SERVER
          PCLPoint _point = PCLPoint();
          _point.x = x; _point.y = y; _point.z = z;
          _point.r = r; _point.g = g; _point.b = b;
          pclCloud.push_back(_point);
#else
          pclCloud.push_back(PCLPoint(x, y, z));
#endif
        }

      }
    } else{ // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ)
      {
        handleFreeNode(it);
        if (inUpdateBBX)
          handleFreeNodeInBBX(it);
      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  if (publishBinaryMap)
    publishBinaryOctoMap(rostime);

  if (publishFullMap)
    publishFullOctoMap(rostime);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("OctoMap3d publishing in visualizer took %f sec", total_elapsed);
}

void Submap3dVisualizer::publishBinaryOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  if (octomap_msgs::binaryMapToMsg(*m_octree, map))
    m_binaryMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

void Submap3dVisualizer::publishFullOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    m_fullMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");

}


bool Submap3dVisualizer::octomapBinarySrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(*m_octree, res.map))
    return false;

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool Submap3dVisualizer::octomapFullSrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();


  if (!octomap_msgs::fullMapToMsg(*m_octree, res.map))
    return false;

  return true;
}

bool Submap3dVisualizer::clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp){
  point3d min = pointMsgToOctomap(req.min);
  point3d max = pointMsgToOctomap(req.max);

  double thresMin = m_octree->getClampingThresMin();
  for(OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min,max),
      end=m_octree->end_leafs_bbx(); it!= end; ++it){

    it->setLogOdds(octomap::logodds(thresMin));
    //			m_octree->updateNode(it.getKey(), -6.0f);
  }
  // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
  m_octree->updateInnerOccupancy();

  publishOCTmap3d(ros::Time::now());

  return true;
}

bool Submap3dVisualizer::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth +1);
  ros::Time rostime = ros::Time::now();
  m_octree->clear();
  // clear 2D map:
  m_gridmap.data.clear();
  m_gridmap.info.height = 0.0;
  m_gridmap.info.width = 0.0;
  m_gridmap.info.resolution = 0.0;
  m_gridmap.info.origin.position.x = 0.0;
  m_gridmap.info.origin.position.y = 0.0;

  ROS_INFO("Cleared octomap");
  publishOCTmap3d(rostime);
  return true;
}

void Submap3dVisualizer::handlePreNodeTraversal(const ros::Time& rostime){
  if (m_publish2DMap){
    // init projected 2D map:
    m_gridmap.header.frame_id = m_worldFrameId;
    m_gridmap.header.stamp = rostime;
    nav_msgs::MapMetaData oldMapInfo = m_gridmap.info;

    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
    octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

    ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

    // add padding if requested (= new min/maxPts in x&y):
    double halfPaddedX = 0.5*m_minSizeX;
    double halfPaddedY = 0.5*m_minSizeY;
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    minPt = octomap::point3d(minX, minY, minZ);
    maxPt = octomap::point3d(maxX, maxY, maxZ);

    OcTreeKey paddedMaxKey;
    if (!m_octree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)){
      ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }
    if (!m_octree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)){
      ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }

    ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
    m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
    m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;

    int mapOriginX = minKey[0] - m_paddedMinKey[0];
    int mapOriginY = minKey[1] - m_paddedMinKey[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
    double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
    m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
    m_gridmap.info.resolution = gridRes;
    m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
    m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
    if (m_maxTreeDepth != m_treeDepth){
      m_gridmap.info.origin.position.x -= m_res/2.0;
      m_gridmap.info.origin.position.y -= m_res/2.0;
    }

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (m_maxTreeDepth < m_treeDepth)
      m_projectCompleteMap = true;


    if(m_projectCompleteMap){
      ROS_DEBUG("Rebuilding complete 2D map");
      m_gridmap.data.clear();
      // init to unknown:
      m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

    } else {

       if (mapChanged(oldMapInfo, m_gridmap.info)){
          ROS_DEBUG("2D grid map size changed to %dx%d", m_gridmap.info.width, m_gridmap.info.height);
          adjustMapData(m_gridmap, oldMapInfo);
       }
       nav_msgs::OccupancyGrid::_data_type::iterator startIt;
       size_t mapUpdateBBXMinX = std::max(0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0]))/int(m_multires2DScale));
       size_t mapUpdateBBXMinY = std::max(0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1]))/int(m_multires2DScale));
       size_t mapUpdateBBXMaxX = std::min(int(m_gridmap.info.width-1), (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0]))/int(m_multires2DScale));
       size_t mapUpdateBBXMaxY = std::min(int(m_gridmap.info.height-1), (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1]))/int(m_multires2DScale));

       assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
       assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

       size_t numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;

       // test for max idx:
       uint max_idx = m_gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
       if (max_idx  >= m_gridmap.data.size())
         ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, m_gridmap.data.size(), m_gridmap.info.width, m_gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);

       // reset proj. 2D map in bounding box:
       for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j){
          std::fill_n(m_gridmap.data.begin() + m_gridmap.info.width*j+mapUpdateBBXMinX,
                      numCols, -1);
       }

    }



  }

}

void Submap3dVisualizer::handlePostNodeTraversal(const ros::Time& rostime){

  if (m_publish2DMap)
    m_projectedMapPub.publish(m_gridmap);
}

void Submap3dVisualizer::handleOccupiedNode(const OcTreeT::iterator& it){

  if (m_publish2DMap && m_projectCompleteMap){
    update2DMap(it, true);
  }
}

void Submap3dVisualizer::handleFreeNode(const OcTreeT::iterator& it){

  if (m_publish2DMap && m_projectCompleteMap){
    update2DMap(it, false);
  }
}

void Submap3dVisualizer::handleOccupiedNodeInBBX(const OcTreeT::iterator& it){

  if (m_publish2DMap && !m_projectCompleteMap){
    update2DMap(it, true);
  }
}

void Submap3dVisualizer::handleFreeNodeInBBX(const OcTreeT::iterator& it){

  if (m_publish2DMap && !m_projectCompleteMap){
    update2DMap(it, false);
  }
}

void Submap3dVisualizer::update2DMap(const OcTreeT::iterator& it, bool occupied){

  // update 2D map (occupied always overrides):

  if (it.getDepth() == m_maxTreeDepth){
    unsigned idx = mapIdx(it.getKey());
    if (occupied)
      m_gridmap.data[mapIdx(it.getKey())] = 100;
    else if (m_gridmap.data[idx] == -1){
      m_gridmap.data[idx] = 0;
    }

  } else{
    int intSize = 1 << (m_maxTreeDepth - it.getDepth());
    octomap::OcTreeKey minKey=it.getIndexKey();
    for(int dx=0; dx < intSize; dx++){
      int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
      for(int dy=0; dy < intSize; dy++){
        unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/m_multires2DScale);
        if (occupied)
          m_gridmap.data[idx] = 100;
        else if (m_gridmap.data[idx] == -1){
          m_gridmap.data[idx] = 0;
        }
      }
    }
  }


}



bool Submap3dVisualizer::isSpeckleNode(const OcTreeKey&nKey) const {
  OcTreeKey key;
  bool neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        if (key != nKey){
          OcTreeNode* node = m_octree->search(key);
          if (node && m_octree->isNodeOccupied(node)){
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return neighborFound;
}


void Submap3dVisualizer::adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const{
  if (map.info.resolution != oldMapInfo.resolution){
    ROS_ERROR("Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x)/map.info.resolution +0.5);
  int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y)/map.info.resolution +0.5);

  if (i_off < 0 || j_off < 0
      || oldMapInfo.width  + i_off > map.info.width
      || oldMapInfo.height + j_off > map.info.height)
  {
    ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

  for (int j =0; j < int(oldMapInfo.height); ++j ){
    // copy chunks, row by row:
    fromStart = oldMapData.begin() + j*oldMapInfo.width;
    fromEnd = fromStart + oldMapInfo.width;
    toStart = map.data.begin() + ((j+j_off)*m_gridmap.info.width + i_off);
    copy(fromStart, fromEnd, toStart);

//    for (int i =0; i < int(oldMapInfo.width); ++i){
//      map.data[m_gridmap.info.width*(j+j_off) +i+i_off] = oldMapData[oldMapInfo.width*j +i];
//    }

  }

}


std_msgs::ColorRGBA Submap3dVisualizer::heightMapColor(double h) {

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}
}



