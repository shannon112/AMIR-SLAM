#include <submap3d_server/Submap3dServer.h>
#include <sstream>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace submap3d_server{

Submap3dServer::Submap3dServer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),
  m_pointCloudSub(NULL),
  m_poseArraySub(NULL),
  m_submapListSub(NULL),
  m_tfPointCloudSub(NULL),
  m_tfPoseArraySub(NULL),
  m_tfSubmapListSub(NULL),

  m_hasFirstPose(false),
  m_SizePoses(0),
  m_SizeSubmaps(0),
  m_CompleteSubmaps(0),
  m_local_pc_map(new PCLPointCloud),
  submap3d_old(new PCLPointCloud),
  submap3d_new(new PCLPointCloud),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), 
  m_baseFrameId("base_footprint"),
  m_trackingFrameId("imu_link"),
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
  last_pose = Pose();
  last_pose.orientation.w = 1;

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
  m_submap3dPub = m_nh.advertise<octomap_server::PosePointCloud2>("submap3d", 1, m_latchedTopics);
  m_nodemap3dPub = m_nh.advertise<octomap_server::PosePointCloud2>("nodemap3d", 1, m_latchedTopics);
  m_submap3dTestPub = m_nh.advertise<sensor_msgs::PointCloud2>("submap3d_test", 1, m_latchedTopics);

  // subscriber
  m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
  m_poseArraySub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array", 5);
  m_submapListSub = new message_filters::Subscriber<cartographer_ros_msgs::SubmapList> (m_nh, "submap_list", 5);

  // tf listener
  m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
  m_tfPointCloudSub->registerCallback(boost::bind(&Submap3dServer::insertCloudCallback, this, _1));
  m_tfPoseArraySub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseArraySub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseArraySub->registerCallback(boost::bind(&Submap3dServer::insertPoseCallback, this, _1));
  m_tfSubmapListSub = new tf::MessageFilter<cartographer_ros_msgs::SubmapList> (*m_submapListSub, m_tfListener, m_worldFrameId, 5);
  m_tfSubmapListSub->registerCallback(boost::bind(&Submap3dServer::insertSubmapCallback, this, _1));
}

Submap3dServer::~Submap3dServer(){
  if (m_pointCloudSub){
    delete m_pointCloudSub;
    m_pointCloudSub = NULL;
  }

  if (m_poseArraySub){
    delete m_poseArraySub;
    m_poseArraySub = NULL;
  }

  if (m_submapListSub){
    delete m_submapListSub;
    m_submapListSub = NULL;
  }

  if (m_tfPointCloudSub){
    delete m_tfPointCloudSub;
    m_tfPointCloudSub = NULL;
  }

  if (m_tfPoseArraySub){
    delete m_tfPoseArraySub;
    m_tfPoseArraySub = NULL;
  }

  if (m_tfSubmapListSub){
    delete m_tfSubmapListSub;
    m_tfSubmapListSub = NULL;
  }
}

// subscribe submap_list trigger 3dsubmap_list pub
void Submap3dServer::insertSubmapCallback(const cartographer_ros_msgs::SubmapList::ConstPtr& submap_list){
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time latest_stamp = submap_list->header.stamp;

  /*
  //publish debug submap
  // create get new one, pub old one
  if (submap_list->submap.size() > m_SizeSubmaps){
    m_SizeSubmaps = submap_list->submap.size();
    publishSubmap3d(latest_stamp, submap3d_old);
    submap3d_old->clear();
  }

  // building submap3d
  if (nodemap_queue.size()>0){
    PCLPointCloud::Ptr icp_pc(new PCLPointCloud);
    if(submap3d_old->size()==0) {
      *submap3d_old += *(nodemap_queue.front());
      nodemap_queue.pop();
    }else {
      PairwiseICP(submap3d_old, nodemap_queue.front(), icp_pc);
      nodemap_queue.pop();
      submap3d_old = icp_pc;
    }
    ROS_INFO("********Submap3d size %zu pts********", submap3d_old->size()); 
  }
  */
  return;
}

// subscribe trajectory_pose_array trigger 3dnode_list pub
void Submap3dServer::insertPoseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time latest_pa_stamp = pose_array->header.stamp;

  if (pose_array->poses.size() > m_SizePoses){
    m_SizePoses = pose_array->poses.size();
    if (m_hasFirstPose==false){
      last_pose = pose_array->poses[m_SizePoses-1];
      m_hasFirstPose = true;
      return;
    }
    int pc_num_in_node = pc_queue.size();
    int step = pc_num_in_node / 2;

    // accumulate pc to build a 3d map with node ref to global frame
    ROS_INFO("pc_num_in_node size %d", pc_num_in_node);
    for (int i=0; i<pc_num_in_node; ++i){
      if (!(i==0 || i==step || i==pc_num_in_node-1)){
        pc_queue.pop();
        continue;
      }
      if(m_local_pc_map->size()==0) {
        *m_local_pc_map += *(pc_queue.front());
        pc_queue.pop();
      }else {
        PCLPointCloud::Ptr icp_pc(new PCLPointCloud);
        PairwiseICP(m_local_pc_map, pc_queue.front(), icp_pc);
        pc_queue.pop();
        m_local_pc_map = icp_pc;
      }
    }
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("ICP registeration is done %zu pts local_pc_map, time cost: %f sec)", m_local_pc_map->size(), total_elapsed);

    // voxel filter
    pcl::VoxelGrid<PCLPoint> voxel_filter;
    voxel_filter.setLeafSize( 0.05, 0.05, 0.05);
    PCLPointCloud::Ptr vx_pc (new PCLPointCloud);
    voxel_filter.setInputCloud(m_local_pc_map);
    voxel_filter.filter(*vx_pc);
    total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("node map voxel filter, time cost: %f sec)", total_elapsed);

    // enqueue prepare for 3dsubmap_list
    /*
    if (nodemap_queues.size()==1){
      nodemap_queues[-1]->push(vx_pc);
      ROS_INFO("now queue1 size %zu", nodemap_queues[-1]->size());
    }else if (nodemap_queues.size()>1){
      nodemap_queues[-1]->push(vx_pc);
      //nodemap_queues[-2].push(vx_pc);
      ROS_INFO("now queue2 size %zu", nodemap_queues[-1]->size());
    }else{//==0
      ROS_INFO("no queue found");
    }
    */

    //enqueue debug submap
    /*
    if (m_SizeSubmaps>0){
      nodemap_queue.push(vx_pc);
      ROS_INFO("Nodemap_queue size %zu", nodemap_queue.size());
    }
    */

    // transfer to the local ref frame
    PCLPointCloud::Ptr transformed_pc (new PCLPointCloud);
    Eigen::Quaterniond last_pose_q(last_pose.orientation.w, last_pose.orientation.x, last_pose.orientation.y, last_pose.orientation.z);
    Eigen::Vector3d last_pose_v(last_pose.position.x, last_pose.position.y, last_pose.position.z);
    Eigen::Matrix3d last_pose_R = last_pose_q.toRotationMatrix();
    //Eigen::Matrix4d Trans; 
    //Trans.setIdentity();
    //Trans.block<3,3>(0,0) = last_pose_R;
    //Trans.block<3,1>(0,3) = last_pose_v;
    Eigen::Matrix4d Trans_inverse; 
    Trans_inverse.setIdentity();
    Trans_inverse.block<3,3>(0,0) = last_pose_R.transpose();
    Trans_inverse.block<3,1>(0,3) = -last_pose_R.transpose()*last_pose_v;
    pcl::transformPointCloud(*vx_pc, *transformed_pc, Trans_inverse);
    //pcl::transformPointCloud(*m_local_pc_map, *m_local_pc_map, Trans);

    //centroid of local pc
    Eigen::Vector4f centroid; 
    pcl::compute3DCentroid(*transformed_pc,centroid);
    if (centroid.y() >0.05 || centroid.y()< -0.05 || centroid.x()>2 || centroid.x()<-2) {
      std::cout<<"aborted "<<m_SizePoses-1<<std::endl;
      m_local_pc_map->clear();
    }

    // publish 3dnode_list (submap pair with pose)
    publishNodemap3d(latest_pa_stamp, transformed_pc, last_pose);

    // update to new cycle
    m_local_pc_map->clear();
    last_pose = pose_array->poses[m_SizePoses-1];
  }
  return;
}

// pointcloud input callback
void Submap3dServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time latest_pc_stamp = cloud->header.stamp;

  if (m_hasFirstPose==false) return;

  // transform cloud from ros to pcl
  PCLPointCloud::Ptr pc(new PCLPointCloud);
  pcl::fromROSMsg(*cloud, *pc);

  //depth filter, as kinectv1 spec max depth range is around 3.5~4m, min is 0.8m
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-4, 4);
  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-4, 4);
  pcl::PassThrough<PCLPoint> pass_z_p;
  pass_z_p.setFilterFieldName("z");
  pass_z_p.setFilterLimits(-4, 4);
  pcl::PassThrough<PCLPoint> pass_z_n;
  pass_z_n.setFilterFieldName("z");
  pass_z_n.setFilterLimits(-0.8, 0.8);
  pass_z_n.setFilterLimitsNegative (true);
  pass_x.setInputCloud(pc);
  pass_x.filter(*pc);
  pass_y.setInputCloud(pc);
  pass_y.filter(*pc);
  pass_z_p.setInputCloud(pc);
  pass_z_p.filter(*pc);
  pass_z_n.setInputCloud(pc);
  pass_z_n.filter(*pc); 
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Depth filter, time cost: %f sec)", total_elapsed);

  // voxel filter
  pcl::VoxelGrid<PCLPoint> voxel_filter;
  voxel_filter.setLeafSize( 0.05, 0.05, 0.05);
  PCLPointCloud::Ptr vx_pc (new PCLPointCloud);
  voxel_filter.setInputCloud(pc);
  voxel_filter.filter(*vx_pc);
  total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("voxel filter, time cost: %f sec)", total_elapsed);

  // transform pc from cloud frame to global frame
  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }
  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  pcl::transformPointCloud(*vx_pc, *vx_pc, sensorToWorld);
  total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("transformation, time cost: %f sec)", total_elapsed);

  //statistical filter, filtering outlier
  PCLPointCloud::Ptr st_pc(new PCLPointCloud);
  pcl::StatisticalOutlierRemoval<PCLPoint> statistical_filter;
  statistical_filter.setMeanK(50);
  statistical_filter.setStddevMulThresh(1.0);
  statistical_filter.setInputCloud(vx_pc);
  statistical_filter.filter(*st_pc);
  total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("statistical filter, time cost: %f sec)", total_elapsed);

  //push back into queue
  pc_queue.push(st_pc);
  total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Pointcloud enqueue is done (%zu pts st_pc), time cost: %f sec, queue_size: %zu)", st_pc->size(), total_elapsed, pc_queue.size());
  return;
}

// SVD based ICP to align each scan make a submap
void Submap3dServer::PairwiseICP(const PCLPointCloud::Ptr &cloud_target, const PCLPointCloud::Ptr &cloud_source, PCLPointCloud::Ptr &output ){
  ros::WallTime startTime = ros::WallTime::now();
	PCLPointCloud::Ptr src(new PCLPointCloud);
	PCLPointCloud::Ptr tgt(new PCLPointCloud);
 
	tgt = cloud_target;
	src = cloud_source;
 
	pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
	icp.setMaxCorrespondenceDistance(0.3); //ignore the point out of distance(m)
	icp.setTransformationEpsilon(1e-6); //converge criterion
	icp.setEuclideanFitnessEpsilon(0.1); //diverge threshold
	icp.setMaximumIterations (10);
	icp.setInputSource (src);
	icp.setInputTarget (tgt);
  icp.align (*output); //get aligned_source	
  ROS_INFO("ICP pointcloud size src/tgt/output (%zu/%zu/%zu pts)", src->size(), tgt->size(), output->size());
  if (icp.hasConverged()){
    *output += *tgt;
  }else{
    submap3d_old->clear(); //abort old
    //output = src; //abort new
  }
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("After ICP registration aligned pointcloud size: %d, time cost: %f(s), converged: %d, score: %f", output->size(), total_elapsed, icp.hasConverged(), icp.getFitnessScore());
}

// 3dnode_list pub
void Submap3dServer::publishNodemap3d(const ros::Time& rostime, const PCLPointCloud::Ptr &nodemap, const Pose &last_pose){
  ros::WallTime startTime = ros::WallTime::now();
  bool fPublishNodemap3d = (m_latchedTopics || m_nodemap3dPub.getNumSubscribers() > 0);
  int index_now = (int)(m_SizePoses)-1; // start from 1 = corresponding len of pose array
  
  if (fPublishNodemap3d){
    if (index_now>2640) return;
    octomap_server::PosePointCloud2 pcloud;
    pcloud.header.frame_id = m_worldFrameId;//m_worldFrameId;//
    pcloud.header.stamp = rostime;
    pcloud.node_id = index_now;
    pcloud.pose = last_pose;
    pcl::toROSMsg (*nodemap, pcloud.cloud);
    m_nodemap3dPub.publish(pcloud);
    ROS_INFO("--------Published 3dnode_list %d--------", index_now);
  }
}

// 3dsubmap_list pub
void Submap3dServer::publishSubmap3d(const ros::Time& rostime, const PCLPointCloud::Ptr &submapmap){
  ros::WallTime startTime = ros::WallTime::now();
  bool fPublishSubmap3d = (m_latchedTopics || m_submap3dPub.getNumSubscribers() > 0);
  int index_now = (int)(m_SizeSubmaps)-1; // start from 1 = corresponding len of pose array

  /*  
  if (fPublishSubmap3d){
    octomap_server::PosePointCloud2 pcloud;
    pcloud.header.frame_id = m_worldFrameId;//m_worldFrameId;//
    pcloud.header.stamp = rostime;
    pcloud.node_id = index_now;
    pcl::toROSMsg (*submapmap, pcloud.cloud);
    m_submap3dPub.publish(pcloud);
    ROS_INFO("--------Published 3dsubmap_list idx:%d--------", index_now-2);
  }
  */

  if (fPublishSubmap3d){
    sensor_msgs::PointCloud2 pcloud;
    pcl::toROSMsg (*submapmap, pcloud);
    pcloud.header.frame_id = m_worldFrameId;//m_worldFrameId;//
    pcloud.header.stamp = rostime;
    m_submap3dTestPub.publish(pcloud);
    ROS_INFO("--------Published 3dsubmap_list test idx:%d--------", index_now-2);
  }
}

}