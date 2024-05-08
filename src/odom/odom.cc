/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

/*
  @brief 替换了基于GICP的激光里程计，改进VGICP，增加了空间偏差的权重矩阵；
  @addindex 沿用NANO-GICP的策略，子图的kdtree建立非必要；
  @addindex 增加回环检测，使用gtsam优化关键帧位姿集合。
  @addindex 非关键帧位姿由几何观测器优化收敛
  @addindex 支持多种雷达config yaml
  @anchor piluohong
*/

#include "dlio/odom.h"
#include "matplotlib-cpp/matplotlibcpp.h"

using namespace gtsam;
using symbol_shorthand::B; // Bias(ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; 
using symbol_shorthand::V; 
using symbol_shorthand::X; 

namespace plt = matplotlibcpp;

dlio::OdomNode::OdomNode(ros::NodeHandle node_handle) : nh(node_handle) {
  // this->count = 0;
  //gtsam
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.05;
  params.factorization = gtsam::ISAM2Params::QR; 
  params.relinearizeSkip = 1;
  this->isam = new gtsam::ISAM2(params);
  // this->keyframe_pose_corr = Eigen::Isometry3f::Identity();
  this->icpScore = 0.1;
  this->poseCovariance;
  this->addloop_num = this->addodom_num = 0;

  this->getParams();

  this->num_threads_ = omp_get_max_threads();

  this->dlio_initialized = false;
  this->first_valid_scan = false;
  this->first_imu_received = false;
  if (this->imu_calibrate_) {this->imu_calibrated = false;}
  else {this->imu_calibrated = true;}
  this->deskew_status = false;
  this->bigscale = false;
  this->isLoop = false;
  this->iskf = false;
  this->cur_kf = true;
  this->deskew_size = 0;
  this->adaptiveVoxelFilter_size = 0;
  this->keyframe_nn = {0};
  this->ds = {0};
  this->kf_sim_buffer.set_capacity(1);
  
  this->lidar_sub = this->nh.subscribe("pointcloud", 1,
      &dlio::OdomNode::callbackPointCloud, this, ros::TransportHints().tcpNoDelay());
  this->imu_sub = this->nh.subscribe("imu", 2000,
      &dlio::OdomNode::callbackImu, this, ros::TransportHints().tcpNoDelay());
  this->livox_sub = this->nh.subscribe("livox", 1,
      &dlio::OdomNode::callbackLivox, this, ros::TransportHints().tcpNoDelay());
  

  this->livox_pub    = this->nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1, true);
  this->odom_pub     = this->nh.advertise<nav_msgs::Odometry>("odom", 1, true);
  // this->pose_pub     = this->nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);
  this->path_pub     = this->nh.advertise<nav_msgs::Path>("path", 1, true);
  this->kf_pose_pub  = this->nh.advertise<geometry_msgs::PoseArray>("kf_pose", 1, true);
  this->kf_cloud_pub = this->nh.advertise<sensor_msgs::PointCloud2>("kf_cloud", 1, true);
  this->deskewed_pub = this->nh.advertise<sensor_msgs::PointCloud2>("deskewed", 1, true);
  this->global_map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("global_map", 1,true);
  
  this->kf_pose_test = this->nh.advertise<sensor_msgs::PointCloud2>("/kf_pose_test",1,true);
  this->kf_connect_pub = this->nh.advertise<visualization_msgs::Marker>("/kf_connect", 10, true);
  this->convex_connect_pub = this->nh.advertise<visualization_msgs::Marker>("/convex_connect", 10, true);
  this->loop_constraint_pub = this->nh.advertise<visualization_msgs::MarkerArray>("loop_constraint",1,true);
  this->pubHistoryKeyFrames = this->nh.advertise<sensor_msgs::PointCloud2>("/local_map",1,true);
  this->publoopmap = this->nh.advertise<sensor_msgs::PointCloud2>("/vgicp_loop_map",1,true);

  this->odom_time_pub = this->nh.advertise<std_msgs::Float32>("/odom_time",1,true);
  this->submap_time_pub = this->nh.advertise<std_msgs::Float32>("/submap_time",1,true);
  this->each_frame_time_pub = this->nh.advertise<std_msgs::Float32>("/each_frame_time",1,true);
  this->publish_timer = this->nh.createTimer(ros::Duration(0.01), &dlio::OdomNode::publishPose, this);// 100hz

  this->T = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->T_corr = Eigen::Matrix4f::Identity();
  this->lastIncreTransformation = Eigen::Matrix4f::Identity();
  //  std::fill(this->T_fusion,this->T_fusion + 6,0);

  this->origin = Eigen::Vector3f(0., 0., 0.);
  this->state.p = Eigen::Vector3f(0., 0., 0.);
  this->state.q = Eigen::Quaternionf(1., 0., 0., 0.);
  this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);

  this->lidarPose.p = Eigen::Vector3f(0., 0., 0.);
  this->lidarPose.q = Eigen::Quaternionf(1., 0., 0., 0.);

  this->imu_meas.stamp = 0.;
  this->imu_meas.ang_vel[0] = 0.;
  this->imu_meas.ang_vel[1] = 0.;
  this->imu_meas.ang_vel[2] = 0.;
  this->imu_meas.lin_accel[0] = 0.;
  this->imu_meas.lin_accel[1] = 0.;
  this->imu_meas.lin_accel[2] = 0.;

  this->imu_buffer.set_capacity(this->imu_buffer_size_);
  this->first_imu_stamp = 0.;
  this->prev_imu_stamp = 0.;

  this->original_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
  this->deskewed_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
  this->current_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
  this->current_scan_w = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  // this->current_scan_lidar = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
  this->submap_cloud = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
  this->cloudKeyPoses6D = pcl::PointCloud<PointTypePose>::Ptr (boost::make_shared<pcl::PointCloud<PointTypePose>>());
  this->copy_cloudKeyPoses6D = pcl::PointCloud<PointTypePose>::Ptr (boost::make_shared<pcl::PointCloud<PointTypePose>>());
  // this->kdtreeHistoryKeyPoses = pcl::KdTreeFLANN<PointTypePose>::Ptr (boost::make_shared<pcl::KdTreeFLANN<PointTypePose>>());
  this->kdtreeHistoryKeyPoses.reset(new nanoflann::KdTreeFLANN<PointTypePose>());
  // this->kf_kdtree = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr (boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>());
  // this->kf_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
  this->global_map = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

  this->num_processed_keyframes = 0;

  this->submap_hasChanged = true;
  this->submap_kf_idx_prev.clear();

  this->first_scan_stamp = 0.;
  this->elapsed_time = 0.;
  this->length_traversed;
  this->timelaserCur = 0.;

  this->convex_hull.setDimension(3);
  this->concave_hull.setDimension(3);
  this->concave_hull.setAlpha(this->keyframe_thresh_dist_);
  this->concave_hull.setKeepInformation(true);

/*
    @brief 超参数
    # k_correspondences           : Number of points used for covariance estimation
    # max_correspondence_distance : Maximum distance for corresponding point search
    # voxel_resolution            : Resolution of voxel-based algorithms
    # neighbor_search_method      : DIRECT1, DIRECT7, DIRECT27, or DIRECT_RADIUS
    # neighbor_search_radius      : Neighbor voxel search radius (for GPU-based methods)
    # num_threads                 : Number of threads
*/
//GICP配准
  this->gicp.setNumThreads(this->num_threads_);
  this->gicp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_ep_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_ep_);
  this->gicp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);
  this->gicp.setDistVar(0.0);
  this->gicp.setAzimuthVar(0.0);
  this->gicp.setElevationVar(0.0);

  this->gicp_temp.setNumThreads(this->num_threads_);
  this->gicp_temp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
  this->gicp_temp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp_temp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp_temp.setTransformationEpsilon(this->gicp_transformation_ep_);
  this->gicp_temp.setRotationEpsilon(this->gicp_rotation_ep_);
  this->gicp_temp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);

//DWVGICP激光里程计
this->vgicp_odom.setResolution(this->downsamplerate);
this->vgicp_odom.setNumThreads(this->num_threads_);
this->vgicp_odom.setCorrespondenceRandomness(this->gicp_k_correspondences_);
this->vgicp_odom.setMaximumIterations(this->gicp_max_iter_);
this->vgicp_odom.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
this->vgicp_odom.setTransformationEpsilon(this->gicp_transformation_ep_);
// this->vgicp_odom.setRegularizationMethod(fast_gicp::RegularizationMethod::NORMALIZED_MIN_EIG);
this->vgicp_odom.setDistVar(0.001); // Avia: 0.001 Mid360: 0.002
this->vgicp_odom.setAzimuthVar(0.05); // Avia: 0.05° Mid360: 0.15°
this->vgicp_odom.setElevationVar(0.05);

this->vgicp_loop.setResolution(this->downsamplerate);
this->vgicp_loop.setNumThreads(this->num_threads_);
this->vgicp_loop.setMaximumIterations(this->gicp_max_iter_);
this->vgicp_loop.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
this->vgicp_loop.setTransformationEpsilon(this->gicp_transformation_ep_);
this->vgicp_loop.setDistVar(0.0);
this->vgicp_loop.setAzimuthVar(0.0);
this->vgicp_loop.setElevationVar(0.0);

if (this->neighborsearch == 1){
  this->vgicp_odom.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1); 
}
if(this->neighborsearch == 7){
  this->vgicp_odom.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7); 
}
// 降低kdtree重新建立的频率
 pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
 this->gicp.setSearchMethodSource(temp, true);
 this->gicp.setSearchMethodTarget(temp, true);
 this->gicp_temp.setSearchMethodSource(temp,true);
 this->gicp_temp.setSearchMethodTarget(temp,true);

  this->geo.first_opt_done = false;
  this->geo.prev_vel = Eigen::Vector3f(0., 0., 0.);

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  this->crop.setNegative(true);
  this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
  this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));

  this->voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);
  this->voxel_global.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);
  this->downsampleloop.setLeafSize(this->loop_size,this->loop_size,this->loop_size);

  this->metrics.spaciousness.push_back(0.);
  this->metrics.density.push_back(this->gicp_max_corr_dist_);

  // CPU Specs
  char CPUBrandString[0x40];
  memset(CPUBrandString, 0, sizeof(CPUBrandString));

  this->cpu_type = "";

  #ifdef HAS_CPUID
  unsigned int CPUInfo[4] = {0,0,0,0};
  __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
  unsigned int nExIds = CPUInfo[0];
  for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
    __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    if (i == 0x80000002)
      memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000003)
      memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000004)
      memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
  }
  this->cpu_type = CPUBrandString;
  boost::trim(this->cpu_type);
  #endif

  FILE* file;
  struct tms timeSample;
  char line[128];

  this->lastCPU = times(&timeSample);
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;

  file = fopen("/proc/cpuinfo", "r");
  this->numProcessors = 0;
  while(fgets(line, 128, file) != nullptr) {
      if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
  }
  fclose(file);
}

dlio::OdomNode::~OdomNode() {
  // if(this->savefile){
    // this->getkfCovs();
  //   // 保存因子图的节点为g2o文件
  //   {
  //     gtsam::writeG2o(this->gtSAMgraph,this->initial_cache,"/home/hhh/project_hhh/temp/slam/nonlinear_opt/src/GLIOM_HONG_1.21/doc/pose_graph/graph_v.g2o");
  //     std::ofstream file("/home/hhh/project_hhh/temp/slam/nonlinear_opt/src/GLIOM_HONG_1.21/doc/pose_graph/graph_e.g2o");
  //     int i = 0;
  //     printf(">>> %d\n",this->pose3_vector.size());
  //     for (const auto& betweenFactor : this->pose3_vector) {  
  //       // 提取双边因子的信息
  //       gtsam::Pose3 measured = betweenFactor;
  //       gtsam::Rot3 rotation = measured.rotation();
        
  //       // 将信息写入 g2o 格式文件
  //       file << "EDGE_SE3:QUAT " << this->noise_vector[i].first.first << " " << this->noise_vector[i].first.second << " "
  //             << measured.x() << " " << measured.y() << " " << measured.z() << " "
  //             << rotation.toQuaternion().x() << " " << rotation.toQuaternion().y() << " "
  //             << rotation.toQuaternion().z() << " " << rotation.toQuaternion().w() << " "
  //             << this->noise_vector[i].second->sigmas().transpose() << "\n";
          
  //         ++i;
  //     }

  //     file.close();
  //     printf(">>> 成功保存.\n");
  //   }
  // }
}

void dlio::OdomNode::getParams() {

  // Version
  ros::param::param<std::string>("~dlio/version", this->version_, "0.0.0");
  ros::param::param<bool>("~dlio/loopflag",this->loopflag,false);
  ros::param::param<bool>("~dlio/saveFile",this->savefile,false);
  ros::param::param<string>("~dlio/matchMethod",this->matchMethod,"VGICP");

  // Frames
  ros::param::param<std::string>("~dlio/frames/odom", this->odom_frame, "odom");
  ros::param::param<std::string>("~dlio/frames/baselink", this->baselink_frame, "base_link");
  ros::param::param<std::string>("~dlio/frames/lidar", this->lidar_frame, "lidar");
  ros::param::param<std::string>("~dlio/frames/imu", this->imu_frame, "imu");
  ros::param::param<bool>("~dlio/pointcloud/dense", this->global_dense, false);
  // Get Node NS and Remove Leading Character
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,1);

  // Concatenate Frame Name Strings
  this->odom_frame = ns + "/" + this->odom_frame;
  this->baselink_frame = ns + "/" + this->baselink_frame;
  this->lidar_frame = ns + "/" + this->lidar_frame;
  this->imu_frame = ns + "/" + this->imu_frame;

  // Deskew FLag
  ros::param::param<bool>("~dlio/pointcloud/deskew", this->deskew_, true);

  // Gravity
  ros::param::param<double>("~dlio/odom/gravity", this->gravity_, 1.0);
  ros::param::param<bool>("~dlio/odom/normlize", this->gyronormlized, false);
  // ros::param::param<double>("~dlio/odom/gravity_norm", this->gravity_delete,9.80);

  ros::param::param<bool>("~dlio/odom/computeTimeOffset", this->time_offset_, false);

  // Keyframe Threshold
  ros::param::param<double>("~dlio/odom/keyframe/threshD", this->keyframe_thresh_dist_, 0.1);
  ros::param::param<double>("~dlio/odom/keyframe/threshR", this->keyframe_thresh_rot_, 1.0);
  ros::param::param<double>("~dliom/odom/keyframe/threshJaccardCorr", this->jaccard_corr_thresh_, 2.0);
  ros::param::param<double>("~dliom/odom/keyframe/threshJaccardSim", this->jaccard_sim_thresh_, 0.2);
  ros::param::param<float>("~dlio/odom/keyframe/envir_thresh_", this->envir_thresh_, 5.0);
  ros::param::param<float>("~dlio/odom/keyframe/loop_score_", this->loop_score_, 0.03);
  ros::param::param<int>("~dlio/odom/keyframe/loop_search_R", this->loop_search_R, 15);
  ros::param::param<int>("~dlio/odom/keyframe/loop_search_Num", this->loop_search_Num, 8);
  ros::param::param<int>("~dlio/odom/keyframe/loop_frequeency", this->loop_frequeency, 1);
  ros::param::param<int>("~dlio/odom/keyframe/downsamplerate", this->downsamplerate, 1);
  ros::param::param<float>("~dlio/odom/keyframe/loop_size", this->loop_size, 0.4);

  // Submap
  ros::param::param<int>("~dlio/odom/submap/keyframe/knn", this->submap_knn_, 10);
  ros::param::param<int>("~dlio/odom/submap/keyframe/kcv", this->submap_kcv_, 10);
  ros::param::param<int>("~dlio/odom/submap/keyframe/kcc", this->submap_kcc_, 10);
  ros::param::param<bool>("~dlio/odom/submap/useJaccard", this->useJaccard, true);

  // Dense map resolution
  ros::param::param<bool>("~dlio/map/dense/filtered", this->densemap_filtered_, true);

  // Wait until movement to publish map
  ros::param::param<bool>("~dlio/map/waitUntilMove", this->wait_until_move_, false);

  // Crop Box Filter
  ros::param::param<double>("~dlio/odom/preprocessing/cropBoxFilter/size", this->crop_size_, 1.0);

  // Voxel Grid Filter
  ros::param::param<bool>("~dlio/pointcloud/voxelize", this->vf_use_, true);
  ros::param::param<double>("~dlio/odom/preprocessing/voxelFilter/res", this->vf_res_, 0.25);

  // Adaptive Parameters
  ros::param::param<bool>("~dlio/adaptive", this->adaptive_params_, true);

  // Extrinsics
  std::vector<float> t_default{0., 0., 0.};
  std::vector<float> R_default{1., 0., 0., 0., 1., 0., 0., 0., 1.};

  // center of gravity to imu
  std::vector<float> baselink2imu_t, baselink2imu_R;
  ros::param::param<std::vector<float>>("~dlio/extrinsics/baselink2imu/t", baselink2imu_t, t_default);
  ros::param::param<std::vector<float>>("~dlio/extrinsics/baselink2imu/R", baselink2imu_R, R_default);
  this->extrinsics.baselink2imu.t =
    Eigen::Vector3f(baselink2imu_t[0], baselink2imu_t[1], baselink2imu_t[2]);
  this->extrinsics.baselink2imu.R =
    Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(baselink2imu_R.data(), 3, 3);

  this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2imu_T.block(0, 3, 3, 1) = this->extrinsics.baselink2imu.t;
  this->extrinsics.baselink2imu_T.block(0, 0, 3, 3) = this->extrinsics.baselink2imu.R;

  // center of gravity to lidar
  std::vector<float> baselink2lidar_t, baselink2lidar_R;
  ros::param::param<std::vector<float>>("~dlio/extrinsics/baselink2lidar/t", baselink2lidar_t, t_default);
  ros::param::param<std::vector<float>>("~dlio/extrinsics/baselink2lidar/R", baselink2lidar_R, R_default);

  this->extrinsics.baselink2lidar.t =
    Eigen::Vector3f(baselink2lidar_t[0], baselink2lidar_t[1], baselink2lidar_t[2]);
  this->extrinsics.baselink2lidar.R =
    Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(baselink2lidar_R.data(), 3, 3);

  this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2lidar_T.block(0, 3, 3, 1) = this->extrinsics.baselink2lidar.t;
  this->extrinsics.baselink2lidar_T.block(0, 0, 3, 3) = this->extrinsics.baselink2lidar.R;

  // IMU
  ros::param::param<bool>("~dlio/odom/imu/calibration/accel", this->calibrate_accel_, true);
  ros::param::param<bool>("~dlio/odom/imu/calibration/gyro", this->calibrate_gyro_, true);
  ros::param::param<double>("~dlio/odom/imu/calibration/time", this->imu_calib_time_, 3.0);
  ros::param::param<int>("~dlio/odom/imu/bufferSize", this->imu_buffer_size_, 2000);

  std::vector<float> accel_default{0., 0., 0.}; std::vector<float> prior_accel_bias;
  std::vector<float> gyro_default{0., 0., 0.}; std::vector<float> prior_gyro_bias;

  ros::param::param<bool>("~dlio/odom/imu/approximateGravity", this->gravity_align_, true);
  ros::param::param<bool>("~dlio/imu/calibration", this->imu_calibrate_, true);
  ros::param::param<std::vector<float>>("~dlio/imu/intrinsics/accel/bias", prior_accel_bias, accel_default);
  ros::param::param<std::vector<float>>("~dlio/imu/intrinsics/gyro/bias", prior_gyro_bias, gyro_default);

  // scale-misalignment matrix
  std::vector<float> imu_sm_default{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  std::vector<float> imu_sm;

  ros::param::param<std::vector<float>>("~dlio/imu/intrinsics/accel/sm", imu_sm, imu_sm_default);

  if (!this->imu_calibrate_) {
    this->state.b.accel[0] = prior_accel_bias[0];
    this->state.b.accel[1] = prior_accel_bias[1];
    this->state.b.accel[2] = prior_accel_bias[2];
    this->state.b.gyro[0] = prior_gyro_bias[0];
    this->state.b.gyro[1] = prior_gyro_bias[1];
    this->state.b.gyro[2] = prior_gyro_bias[2];
    this->imu_accel_sm_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(imu_sm.data(), 3, 3);
  } else {
    this->state.b.accel = Eigen::Vector3f(0., 0., 0.);
    this->state.b.gyro = Eigen::Vector3f(0., 0., 0.);
    this->imu_accel_sm_ = Eigen::Matrix3f::Identity();
  }

  // GICP
  ros::param::param<int>("~dlio/odom/gicp/minNumPoints", this->gicp_min_num_points_, 100);
  ros::param::param<int>("~dlio/odom/gicp/kCorrespondences", this->gicp_k_correspondences_, 20);
  ros::param::param<double>("~dlio/odom/gicp/maxCorrespondenceDistance", this->gicp_max_corr_dist_,
  std::sqrt(std::numeric_limits<double>::max()));
  ros::param::param<int>("~dlio/odom/gicp/maxIterations", this->gicp_max_iter_, 64);
  ros::param::param<double>("~dlio/odom/gicp/transformationEpsilon", this->gicp_transformation_ep_, 0.0005);
  ros::param::param<double>("~dlio/odom/gicp/rotationEpsilon", this->gicp_rotation_ep_, 0.0005);
  ros::param::param<double>("~dlio/odom/gicp/initLambdaFactor", this->gicp_init_lambda_factor_, 1e-9);
  ros::param::param<int>("~dlio/odom/gicp/neighborSearch", this->neighborsearch, 1);


  // Geometric Observer
  ros::param::param<double>("~dlio/odom/geo/Kp", this->geo_Kp_, 1.0);
  ros::param::param<double>("~dlio/odom/geo/Kv", this->geo_Kv_, 1.0);
  ros::param::param<double>("~dlio/odom/geo/Kq", this->geo_Kq_, 1.0);
  ros::param::param<double>("~dlio/odom/geo/Kab", this->geo_Kab_, 1.0);
  ros::param::param<double>("~dlio/odom/geo/Kgb", this->geo_Kgb_, 1.0);
  ros::param::param<double>("~dlio/odom/geo/abias_max", this->geo_abias_max_, 1.0);
  ros::param::param<double>("~dlio/odom/geo/gbias_max", this->geo_gbias_max_, 1.0);


}

void dlio::OdomNode::start() {

  printf("\033[2J\033[1;1H");
  std::cout << std::endl
            << "+-------------------------------------------------------------------+" << std::endl;
  std::cout << "|            \033[32mThanks the DLIO scheme very much !!! from \033[0m " << this->version_  << "   |"
            << std::endl;
  std::cout << "+-------------------------------------------------------------------+" << std::endl;

}

void dlio::OdomNode::publishPose(const ros::TimerEvent& e) {

  // nav_msgs::Odometry
  // this->odom_ros.header.stamp = this->imu_stamp;
  this->odom_ros.header.stamp = this->imu_stamp;
  this->odom_ros.header.frame_id = this->odom_frame;
  this->odom_ros.child_frame_id = this->baselink_frame;

  this->odom_ros.pose.pose.position.x = this->state.p[0];
  this->odom_ros.pose.pose.position.y = this->state.p[1];
  this->odom_ros.pose.pose.position.z = this->state.p[2];

  this->odom_ros.pose.pose.orientation.w = this->state.q.w();
  this->odom_ros.pose.pose.orientation.x = this->state.q.x();
  this->odom_ros.pose.pose.orientation.y = this->state.q.y();
  this->odom_ros.pose.pose.orientation.z = this->state.q.z();

  this->odom_ros.twist.twist.linear.x = this->state.v.lin.w[0];
  this->odom_ros.twist.twist.linear.y = this->state.v.lin.w[1];
  this->odom_ros.twist.twist.linear.z = this->state.v.lin.w[2];

  this->odom_ros.twist.twist.angular.x = this->state.v.ang.b[0];
  this->odom_ros.twist.twist.angular.y = this->state.v.ang.b[1];
  this->odom_ros.twist.twist.angular.z = this->state.v.ang.b[2];

  this->odom_pub.publish(this->odom_ros); // 发布里程计 频率100hz


  // //geometry_msgs::PoseStamped
  // this->pose_ros.header.stamp = this->imu_stamp;
  // this->pose_ros.header.frame_id = this->odom_frame;

  // this->pose_ros.pose.position.x = this->state.p[0];
  // this->pose_ros.pose.position.y = this->state.p[1];
  // this->pose_ros.pose.position.z = this->state.p[2];

  // this->pose_ros.pose.orientation.w = this->state.q.w();
  // this->pose_ros.pose.orientation.x = this->state.q.x();
  // this->pose_ros.pose.orientation.y = this->state.q.y();
  // this->pose_ros.pose.orientation.z = this->state.q.z();

  // this->pose_pub.publish(this->pose_ros);

}

void dlio::OdomNode::publishToROS(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud) {
  //T_cloud = this->T_corr
  this->publishCloud(published_cloud, T_cloud);

  // 发布nav_msgs::Path 
  this->path_ros.header.stamp = this->imu_stamp;
  this->path_ros.header.frame_id = this->odom_frame;

  geometry_msgs::PoseStamped p;
  p.header.stamp = this->imu_stamp;
  p.header.frame_id = this->odom_frame;
  p.pose.position.x = this->state.p[0];
  p.pose.position.y = this->state.p[1];
  p.pose.position.z = this->state.p[2];
  p.pose.orientation.w = this->state.q.w();
  p.pose.orientation.x = this->state.q.x();
  p.pose.orientation.y = this->state.q.y();
  p.pose.orientation.z = this->state.q.z();

  this->path_ros.poses.push_back(p);
  this->path_pub.publish(this->path_ros);

  // transform: odom to baselink
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->odom_frame;
  transformStamped.child_frame_id = this->baselink_frame;

  transformStamped.transform.translation.x = this->state.p[0];
  transformStamped.transform.translation.y = this->state.p[1];
  transformStamped.transform.translation.z = this->state.p[2];

  transformStamped.transform.rotation.w = this->state.q.w();
  transformStamped.transform.rotation.x = this->state.q.x();
  transformStamped.transform.rotation.y = this->state.q.y();
  transformStamped.transform.rotation.z = this->state.q.z();

  br.sendTransform(transformStamped);

  // transform: baselink to imu
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->baselink_frame;
  transformStamped.child_frame_id = this->imu_frame;

  transformStamped.transform.translation.x = this->extrinsics.baselink2imu.t[0];
  transformStamped.transform.translation.y = this->extrinsics.baselink2imu.t[1];
  transformStamped.transform.translation.z = this->extrinsics.baselink2imu.t[2];

  Eigen::Quaternionf q(this->extrinsics.baselink2imu.R);
  transformStamped.transform.rotation.w = q.w();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();

  br.sendTransform(transformStamped);

  // transform: baselink to lidar
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->baselink_frame;
  transformStamped.child_frame_id = this->lidar_frame;

  transformStamped.transform.translation.x = this->extrinsics.baselink2lidar.t[0];
  transformStamped.transform.translation.y = this->extrinsics.baselink2lidar.t[1];
  transformStamped.transform.translation.z = this->extrinsics.baselink2lidar.t[2];

  Eigen::Quaternionf qq(this->extrinsics.baselink2lidar.R);
  transformStamped.transform.rotation.w = qq.w();
  transformStamped.transform.rotation.x = qq.x();
  transformStamped.transform.rotation.y = qq.y();
  transformStamped.transform.rotation.z = qq.z();

  br.sendTransform(transformStamped);

}

void dlio::OdomNode::publishCloud(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud) {

  if (this->wait_until_move_) {
    if (this->length_traversed < 0.1) { return; }
  }

  // 发布用state矫正的原始点云
  pcl::PointCloud<PointType>::Ptr deskewed_scan_t_ (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::PointCloud<PointType>::Ptr current_scan_t_ (boost::make_shared<pcl::PointCloud<PointType>>());
  // auto T = Eigen::Isometry3f::Identity();
  // T.translate(this->state.p);
  // T.rotate(this->state.q);
  pcl::transformPointCloud (*published_cloud, *deskewed_scan_t_, T_cloud);
  // published deskewed cloud
  sensor_msgs::PointCloud2 deskewed_ros;
  pcl::toROSMsg(*deskewed_scan_t_, deskewed_ros);
  deskewed_ros.header.stamp = this->scan_header_stamp;
  deskewed_ros.header.frame_id = this->odom_frame;
  this->deskewed_pub.publish(deskewed_ros);

}

void dlio::OdomNode::publishKeyframe(std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::ConstPtr> kf, ros::Time timestamp) {

  //更新关键帧位姿
  // for (int i = 0;i<this->keyframes.size();i++)
  // {
  // geometry_msgs::Pose p;
  // p.position.x = this->keyframes[i].first.first[0];
  // p.position.y = this->keyframes[i].first.first[1];
  // p.position.z = this->keyframes[i].first.first[2];
  // p.orientation.w = this->keyframes[i].first.second.w();
  // p.orientation.x = this->keyframes[i].first.second.x();
  // p.orientation.y = this->keyframes[i].first.second.y();
  // p.orientation.z = this->keyframes[i].first.second.z();
  // this->kf_pose_ros.poses.push_back(p);
  // }

  // 优化后的lidarPose
   geometry_msgs::Pose p;
  p.position.x = kf.first.first[0];
  p.position.y = kf.first.first[1];
  p.position.z = kf.first.first[2];
  p.orientation.w = kf.first.second.w();
  p.orientation.x = kf.first.second.x();
  p.orientation.y = kf.first.second.y();
  p.orientation.z = kf.first.second.z();
  this->kf_pose_ros.poses.push_back(p);
  // Publish
  this->kf_pose_ros.header.stamp = timestamp;
  this->kf_pose_ros.header.frame_id = this->odom_frame;
  this->kf_pose_pub.publish(this->kf_pose_ros);

  //kf_pose_test == this->state
  sensor_msgs::PointCloud2 cloudKeyPoses3D_ros;
  pcl::PointCloud<pcl::PointXYZI> cloud_;
  for(auto &p : this->cloudKeyPoses6D->points)
  {
    pcl::PointXYZI pt;
    pt.x = p.x;pt.y = p.y;pt.z = p.z;pt.intensity = p.intensity;
    cloud_.push_back(pt);
  }
  pcl::toROSMsg(cloud_,cloudKeyPoses3D_ros);
  cloudKeyPoses3D_ros.header.stamp = timestamp;
  cloudKeyPoses3D_ros.header.frame_id = this->odom_frame;
  this->kf_pose_test.publish(cloudKeyPoses3D_ros);


  // publish keyframe scan for map
  if (this->vf_use_){
    if (kf.second->points.size() == kf.second->width * kf.second->height) {
      sensor_msgs::PointCloud2 keyframe_cloud_ros;
      pcl::toROSMsg(*kf.second, keyframe_cloud_ros);
      keyframe_cloud_ros.header.stamp = timestamp;
      keyframe_cloud_ros.header.frame_id = this->odom_frame;
      this->kf_cloud_pub.publish(keyframe_cloud_ros);
    }
  } else {
    sensor_msgs::PointCloud2 keyframe_cloud_ros;
    pcl::toROSMsg(*kf.second, keyframe_cloud_ros);
    keyframe_cloud_ros.header.stamp = timestamp;
    keyframe_cloud_ros.header.frame_id = this->odom_frame;
    this->kf_cloud_pub.publish(keyframe_cloud_ros);
  }

}

void dlio::OdomNode::getScanFromROS(const sensor_msgs::PointCloud2ConstPtr& pc) {

  pcl::PointCloud<PointType>::Ptr original_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*pc, *original_scan_);

  // Remove NaNs
  std::vector<int> idx;
  original_scan_->is_dense = false;
  pcl::removeNaNFromPointCloud(*original_scan_, *original_scan_, idx);

  // Crop Box Filter
  this->crop.setInputCloud(original_scan_);
  this->crop.filter(*original_scan_);

  // automatically detect sensor type
  this->sensor = dlio::SensorType::UNKNOWN;
  for (auto &field : pc->fields) {
    if (field.name == "t") {
      this->sensor = dlio::SensorType::OUSTER;
      break;
    } else if (field.name == "time") {
      this->sensor = dlio::SensorType::VELODYNE;
      break;
    } else if (field.name == "offset_time") {
      this->sensor = dlio::SensorType::LIVOX;
      break;
    } else if (field.name == "timestamp") {
      this->sensor = dlio::SensorType::HESAI;
      break;
    }
  }

  if (this->sensor == dlio::SensorType::UNKNOWN) {
    this->deskew_ = false;
  }

  this->scan_header_stamp = pc->header.stamp;
  this->timelaserCur = pc->header.stamp.toSec();
  //保存时间戳
  this->frametimes.push_back(this->timelaserCur);
  
 this->original_scan = original_scan_;
//  this->current_scan_lidar->clear();
//  pcl::transformPointCloud(*this->original_scan, *this->current_scan_lidar, this->extrinsics.baselink2lidar_T);
//  this->voxel.setInputCloud(this->current_scan_lidar);
//  this->voxel.filter(*this->current_scan_lidar);
}

void dlio::OdomNode::preprocessPoints() {

  // 对点云进行去畸变
  // Deskew the original dlio-type scan
  if (this->deskew_) {
//    std::cout << "Deskew!" << std::endl;
    this->deskewPointcloud(); // 得到deskewed_scan

    if (!this->first_valid_scan) {
      return;
    }

  } else {

    // 不去畸变的情况 scan_stamp为消息头的时间
    //std::cout << "No deskew!" << std::endl;
    this->scan_stamp = this->scan_header_stamp.toSec();
    // 第一帧不去畸变
    if (!this->first_valid_scan) {

      if (this->imu_buffer.empty() || this->scan_stamp <= this->imu_buffer.back().stamp) {
        return;
      }

      this->first_valid_scan = true;
      this->T_prior = this->T; // assume no motion for the first scan

    } else {

      // IMU prior for second scan onwards
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
      frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q, this->lidarPose.p,
                                this->geo.prev_vel.cast<float>(), {this->scan_stamp});
    // 保存最后一个点的状态作为T_prior
    if (frames.size() > 0) {
      this->T_prior = frames.back();
    } else {
      this->T_prior = this->T;
    }

    }


    // 转换到结束时刻
    pcl::PointCloud<PointType>::Ptr deskewed_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::transformPointCloud (*this->original_scan, *deskewed_scan_,
                            this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = false;
  }


  //Voxel Grid Filter
  if (this->vf_use_ && this->deskewed_scan->size() > 14000) {
    pcl::PointCloud<PointType>::Ptr current_scan_
      (boost::make_shared<pcl::PointCloud<PointType>>(*this->deskewed_scan));
    this->voxel.setInputCloud(current_scan_);
    this->voxel.filter(*current_scan_);
    this->current_scan = current_scan_;}else{
    this->current_scan = this->deskewed_scan;
    }

}

void dlio::OdomNode::deskewPointcloud() {

  pcl::PointCloud<PointType>::Ptr deskewed_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
  deskewed_scan_->points.resize(this->original_scan->points.size());

  // individual point timestamps should be relative to this time
  double sweep_ref_time = this->scan_header_stamp.toSec();

  // sort points by timestamp and build list of timestamps
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;
  std::function<bool(boost::range::index_value<PointType&, long>,
                     boost::range::index_value<PointType&, long>)> point_time_neq;
  std::function<double(boost::range::index_value<PointType&, long>)> extract_point_time;

  if (this->sensor == dlio::SensorType::OUSTER) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.t < p2.t; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().t != p2.value().t; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return sweep_ref_time + pt.value().t * 1e-9f; };

  } else if (this->sensor == dlio::SensorType::VELODYNE) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.time < p2.time; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().time != p2.value().time; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return sweep_ref_time + pt.value().time; };

  } else if (this->sensor == dlio::SensorType::LIVOX) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.offset_time < p2.offset_time; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().offset_time != p2.value().offset_time; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return sweep_ref_time + pt.value().offset_time * 1e-9f; };

  } else if (this->sensor == dlio::SensorType::HESAI) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.timestamp < p2.timestamp; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().timestamp != p2.value().timestamp; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return pt.value().timestamp; };

  }

  // copy points into deskewed_scan_ in order of timestamp
  std::partial_sort_copy(this->original_scan->points.begin(), this->original_scan->points.end(),
                         deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

  // filter unique timestamps
  auto points_unique_timestamps = deskewed_scan_->points
                                  | boost::adaptors::indexed()
                                  | boost::adaptors::adjacent_filtered(point_time_neq);
 
  std::vector<double> timestamps;
  std::vector<int> unique_time_indices;
    // offset time
  double offset = 0.0;
  if (this->time_offset_) {
    offset = sweep_ref_time - extract_point_time(*points_unique_timestamps.begin());
  }

   // extract timestamps from points and put them in their own list
  for (auto it = points_unique_timestamps.begin(); it != points_unique_timestamps.end(); it++) {
    timestamps.push_back(extract_point_time(*it) + offset);
    unique_time_indices.push_back(it->index());
  }
  unique_time_indices.push_back(deskewed_scan_->points.size());

  int median_pt_index = timestamps.size() / 2;
  this->scan_stamp = timestamps[median_pt_index]; // set this->scan_stamp to the timestamp of the median point

  // don't process scans until IMU data is present
  if (!this->first_valid_scan) {
    if (this->imu_buffer.empty() || this->scan_stamp <= this->imu_buffer.back().stamp) {
      return;
    }

    this->first_valid_scan = true;
    this->T_prior = this->T; // assume no motion for the first scan
    pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = true;
    return;
  }

  // IMU prior & deskewing for second scan onwards
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
  frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q, this->lidarPose.p,
                              this->geo.prev_vel.cast<float>(), timestamps);
  this->deskew_size = frames.size(); // if integration successful, equal to timestamps.size()

  // if there are no frames between the start and end of the sweep
  // that probably means that there's a sync issue
  if (frames.size() != timestamps.size()) {
    ROS_FATAL("Bad time sync between LiDAR and IMU!");

    this->T_prior = this->T;
    pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = false;
    return;
  }

  // update prior to be the estimated pose at the median time of the scan (corresponds to this->scan_stamp)
  this->T_prior = frames[median_pt_index];

#pragma omp parallel for num_threads(this->num_threads_)
  for (int i = 0; i < timestamps.size(); i++) {

    Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;

    // transform point to world frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
      auto &pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }

  this->deskewed_scan = deskewed_scan_;
  this->deskew_status = true;

}

void dlio::OdomNode::initializeInputTarget() {
  /*
    @brief 第一帧默认为关键帧处理，默认参数初始化
  */
  // 构建当前关键帧
  this->prev_scan_stamp = this->scan_stamp;
  this->FirstGTSAMUpdateFactor();
  // 保存关键帧的姿态 以及去畸变降采样后的点云
  this->keyframes.push_back(std::make_pair(std::make_pair(this->lidarPose.p, this->lidarPose.q), this->current_scan));
  this->keyframesInfo.push_back(this->current_scan);
  // 保存关键帧消息时间戳,协方差
  this->keyframe_timestamps.push_back(this->scan_header_stamp);
  this->keyframe_normals_v.push_back(this->gicp.getSourceCovariances());
  this->keyframe_transformations.push_back(this->T_corr);
  this->keyframe_transformations_prior.push_back(this->T_prior);
  this->keyframe_stateT.push_back(this->T);
  // auto tree = boost::make_shared<pcl::octree::OctreePointCloudSearch<PointType>>(this->jaccard_corr_thresh_);
  // this->kf_tree.push_back(tree);

    PointTypePose pt_rpy;
    pt_rpy.x = this->lidarPose.p[0];
    pt_rpy.y = this->lidarPose.p[1];
    pt_rpy.z = this->lidarPose.p[2];
    pt_rpy.intensity = this->cloudKeyPoses6D->size();
    pt_rpy.roll = this->lidarPose.q.normalized().toRotationMatrix().eulerAngles(2,1,0)[2];
    pt_rpy.pitch = this->lidarPose.q.normalized().toRotationMatrix().eulerAngles(2,1,0)[1];
    pt_rpy.yaw = this->lidarPose.q.normalized().toRotationMatrix().eulerAngles(2,1,0)[0];
    pt_rpy.time = this->scan_header_stamp.toSec();
    this->cloudKeyPoses6D->push_back(pt_rpy);
  
}

void dlio::OdomNode::setInputSource() {
      //vgicp & gicp
    this->vgicp_odom.setInputSource(this->current_scan);
    this->gicp.setInputSource(this->current_scan);
    this->gicp.calculateSourceCovariances();
}

void dlio::OdomNode::initializeDLIO() {

  // Wait for IMU
  if (!this->first_imu_received || !this->imu_calibrated) {
    return;
  }

  this->dlio_initialized = true;
  std::cout << std::endl << " GLIOM initialized!" << std::endl;
  std::cout << std::endl << " Laser odom method(front): " << this->matchMethod << std::endl;
}

void dlio::OdomNode::callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& pc) {
  // this->count++;
  // static double total_time = 0;
  
  std::unique_lock<decltype(this->main_loop_running_mutex)> lock(main_loop_running_mutex);
  this->main_loop_running = true;
  lock.unlock();

  double then = ros::Time::now().toSec();
 
  if (this->first_scan_stamp == 0.) {
    this->first_scan_stamp = pc->header.stamp.toSec();
  }

  // 检查初始化
  // DLIO Initialization procedures (IMU calib, gravity align)
  if (!this->dlio_initialized) {
    this->initializeDLIO();
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  this->getScanFromROS(pc);
  // 点云预处理 去畸变  剔除离群点 降采样
  this->preprocessPoints();

  if (!this->first_valid_scan) {
    return;
  }
  if (this->current_scan->points.size() <= this->gicp_min_num_points_) {
    ROS_FATAL("Low number of points in the cloud!");
    return;
  }
 
  // 计算sparsity
  this->metrics_thread = std::thread( &dlio::OdomNode::computeMetrics, this);
  this->metrics_thread.detach();

  if (this->adaptive_params_) {
    this->setAdaptiveParams();
  }

  this->setInputSource();

  if (this->keyframes.size() == 0) 
  {
    this->initializeInputTarget();
    this->main_loop_running = false;
    this->submap_future = std::async( std::launch::async, &dlio::OdomNode::buildKeyframesAndSubmap, this, this->state );
    this->submap_future.wait(); // wait until completion ; c++14
    return;
  }
  this->getNextPose();
  // Update next global pose
  this->propagateGICP();

  // Update current keyframe pose and state
  this->GTSAMUpdateFactor();
   //用于姿态-惯性融合的收缩分层观测器
    this->updateState();
   

  // Build keyframe normals and submap if needed (and if we're not already waiting) KNN and pcl::Convexhull
  if (this->new_submap_is_ready) {
    this->main_loop_running = false;
    this->submap_future =
      std::async( std::launch::async, &dlio::OdomNode::buildKeyframesAndSubmap, this, this->state);// publish kf_pose/kf_cloud
  }else{
    lock.lock();
    this->main_loop_running = false;
    lock.unlock();
    this->submap_build_cv.notify_one();
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  float time_total =  std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
  this->each_frame_time_pub.publish(time_total);
  // this->adaptive_voxel.publish(time_total);
  this->time_total_time.push_back(time_total);
  // 更新时间戳
  this->lidar_rates.push_back( 1. / (this->scan_stamp - this->prev_scan_stamp) );
  this->prev_scan_stamp = this->scan_stamp;
  this->elapsed_time = this->scan_stamp - this->first_scan_stamp;

  this->trajectory.push_back( std::make_pair(this->state.p, this->state.q));// correctPoses and getNextPose() 

  // 在ROS内发布出去
  pcl::PointCloud<PointType>::ConstPtr published_cloud;
  if (this->densemap_filtered_) {
    published_cloud = this->current_scan; 
  } else {
    published_cloud = this->deskewed_scan;
  }
  
  this->publish_thread = std::thread( &dlio::OdomNode::publishToROS, this, published_cloud,this->T_corr);// path ; TF
  this->publish_thread.detach();

  // Update some statistics
  this->comp_times.push_back(ros::Time::now().toSec() - then);
  this->gicp_hasConverged = this->vgicp_odom.hasConverged();

  // Debug statements and publish custom DLIO message
  // this->debug_thread = std::thread( &dlio::OdomNode::debug, this);
  // this->debug_thread.detach();

  this->geo.first_opt_done = true;
  
  // total_time += time;
}

void dlio::OdomNode::callbackLivox(const livox_ros_driver2::CustomMsgConstPtr& livox) {
  // convert custom livox message to pcl pointcloud
  pcl::PointCloud<LivoxPoint>::Ptr cloud (new pcl::PointCloud<LivoxPoint>);
  for (int i = 0; i < livox->point_num; i++) {
    LivoxPoint p;
    p.x = livox->points[i].x;
    p.y = livox->points[i].y;
    p.z = livox->points[i].z;
    p.intensity = livox->points[i].reflectivity;
    p.offset_time = livox->points[i].offset_time;
    cloud->push_back(p);
  }
  // publish converted livox pointcloud
  sensor_msgs::PointCloud2 cloud_ros;
  pcl::toROSMsg(*cloud, cloud_ros);

  cloud_ros.header.stamp = livox->header.stamp;
  cloud_ros.header.seq = livox->header.seq;
  cloud_ros.header.frame_id = this->lidar_frame;
  this->livox_pub.publish(cloud_ros);
}

void dlio::OdomNode::callbackImu(const sensor_msgs::Imu::ConstPtr& imu_raw) {
  // 第一帧标志位
  this->first_imu_received = true;
  // 将原始的IMU数据转换到base_link下
  sensor_msgs::Imu::Ptr imu = this->transformImu( imu_raw );
  // 获取IMU的时间戳
  this->imu_stamp = imu->header.stamp;

  // 分别获取角速度和加速度
  Eigen::Vector3f lin_accel;
  Eigen::Vector3f ang_vel;

  // Get IMU samples
  ang_vel[0] = imu->angular_velocity.x ;
  ang_vel[1] = imu->angular_velocity.y ;
  ang_vel[2] = imu->angular_velocity.z ;
  // mid360内置imu，加速度单位为g
  if(this->gyronormlized){
    lin_accel[0] = (imu->linear_acceleration.x * this->gravity_);
    lin_accel[1] = (imu->linear_acceleration.y * this->gravity_);
    lin_accel[2] = (imu->linear_acceleration.z * this->gravity_);
    // ROS_WARN(">>> %lf\n",this->gravity_);
  }else{
    lin_accel[0] = imu->linear_acceleration.x ;
    lin_accel[1] = imu->linear_acceleration.y ;
    lin_accel[2] = imu->linear_acceleration.z ;
  }
  // 保留第一帧IMU的时间戳
  if (this->first_imu_stamp == 0.) {
    this->first_imu_stamp = imu->header.stamp.toSec();
  }

  // IMU静态初始化
  // IMU calibration procedure - do for three seconds
  if (!this->imu_calibrated) {
    // 采样数量
    static int num_samples = 0;
    // 角速度和加速度
    static Eigen::Vector3f gyro_avg (0., 0., 0.);
    static Eigen::Vector3f accel_avg (0., 0., 0.);
    static bool print = true;
    // 默认的IMU初始化时间为3秒 时间戳与第一帧IMU时间戳相差3秒以内的都要进行初始化
    if ((imu->header.stamp.toSec() - this->first_imu_stamp) < this->imu_calib_time_) {
      // IMU采样数量+1
      num_samples++;
      // 累加加速度和加速度测量值
      gyro_avg[0] += ang_vel[0];
      gyro_avg[1] += ang_vel[1];
      gyro_avg[2] += ang_vel[2];

      accel_avg[0] += lin_accel[0];
      accel_avg[1] += lin_accel[1];
      accel_avg[2] += lin_accel[2];

      if(print) {
        std::cout << std::endl << " Calibrating IMU for " << this->imu_calib_time_ << " seconds... ";
        std::cout.flush();
        print = false;
      }

    } else {

      std::cout << "done" << std::endl << std::endl;
      // 计算角速度和加速度的平均值
      gyro_avg /= num_samples;
      accel_avg /= num_samples;
      // 重力加速度
      Eigen::Vector3f grav_vec (0., 0., this->gravity_);
      // 对重力进行对齐
      if (this->gravity_align_) {

        // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
        // 取加速度平均值与ba差的方向作为重力方向 重力大小由参数给出
        grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
        // 计算得到的重力方向与标准z轴的偏转 作为初始的姿态
        Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., this->gravity_));
        // 初始化T的旋转和lidarPose的旋转
        // set gravity aligned orientation
        this->state.q = grav_q;
        this->T.block(0,0,3,3) = this->state.q.toRotationMatrix();
        this->lidarPose.q = this->state.q;
        // 计算欧拉角
        // rpy
        auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
        double yaw = euler[0] * (180.0/M_PI);
        double pitch = euler[1] * (180.0/M_PI);
        double roll = euler[2] * (180.0/M_PI);

        // use alternate representation if the yaw is smaller
        if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
          yaw   = remainder(yaw + 180.0,   360.0);
          pitch = remainder(180.0 - pitch, 360.0);
          roll  = remainder(roll + 180.0,  360.0);
        }
        std::cout << " Estimated initial attitude:" << std::endl;
        std::cout << "   Roll  [deg]: " << to_string_with_precision(roll, 4) << std::endl;
        std::cout << "   Pitch [deg]: " << to_string_with_precision(pitch, 4) << std::endl;
        std::cout << "   Yaw   [deg]: " << to_string_with_precision(yaw, 4) << std::endl;
        std::cout << std::endl;
      }

      if (this->calibrate_accel_) {

        // subtract gravity from avg accel to get bias
        // 初始化ba
        this->state.b.accel = accel_avg - grav_vec;

        std::cout << " Accel biases [xyz]: " << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                                             << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                                             << to_string_with_precision(this->state.b.accel[2], 8) << std::endl;
      }

      if (this->calibrate_gyro_) {
        // 角速度均值用来初始化bg
        this->state.b.gyro = gyro_avg;

        std::cout << " Gyro biases  [xyz]: " << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                                             << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                                             << to_string_with_precision(this->state.b.gyro[2], 8) << std::endl;
      }
      // IMU初始化完成
      this->imu_calibrated = true;

    }

  } else {
    // IMU初始化已经完成 : 外部工具测得imu偏差
    double dt = imu->header.stamp.toSec() - this->prev_imu_stamp;
    if (dt == 0) { dt = 1.0/200.0; }
    this->imu_rates.push_back(1./dt);
    

    // Apply the calibrated bias to the new IMU measurements
    this->imu_meas.stamp = imu->header.stamp.toSec();
    this->imu_meas.dt = dt;
    this->prev_imu_stamp = this->imu_meas.stamp;

    Eigen::Vector3f lin_accel_corrected = (this->imu_accel_sm_ * lin_accel) - this->state.b.accel;
    Eigen::Vector3f ang_vel_corrected = ang_vel - this->state.b.gyro;

    this->imu_meas.lin_accel = lin_accel_corrected;
    this->imu_meas.ang_vel = ang_vel_corrected;
    // 加入IMU缓存
    // Store calibrated IMU measurements into imu buffer for manual integration later.
    this->mtx_imu.lock();
    this->imu_buffer.push_front(this->imu_meas);
    this->mtx_imu.unlock();

    // Notify the callbackPointCloud thread that IMU data exists for this time
    this->cv_imu_stamp.notify_one();

    if (this->geo.first_opt_done) {
      // Geometric Observer: Propagate State
      // IMU积分
      this->propagateState();
    }

  }

}

void dlio::OdomNode::getNextPose() {
  // Check if the new submap is ready to be used // 判断线程是否在getNextPose()前已经执行完
  this->new_submap_is_ready = (this->submap_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
  // auto t1 = std::chrono::high_resolution_clock::now();
  this->vgicp_odom.reuse_voxelmap_ = this->submap_hasChanged; 
  if (this->new_submap_is_ready && this->submap_hasChanged) { 
  // vgicp
  this->vgicp_odom.setTargetCovariances(*this->submap_normals_v);
  this->gicp.setTargetCovariances(this->submap_normals_v);
  this->gicp.target_kdtree_ = this->submap_kdtree_v;
  this->vgicp_odom.setInputTarget(this->submap_cloud);
  this->gicp.setInputTarget(this->submap_cloud);
  this->submap_hasChanged = false; //重置该条件，判断submap vector内关键帧是否在两时刻内有没有变化
  // this->vgicp_odom.reuse_voxelmap_ = false; //重置，需要重新建立体素地图
  }
  pcl::PointCloud<PointType>::Ptr aligned (boost::make_shared<pcl::PointCloud<PointType>>());
  
  if (this->matchMethod == "VGICP")
  {
    this->vgicp_odom.align(*aligned);
    this->T_corr = this->vgicp_odom.getFinalTransformation();
    this->lastIncreTransformation = this->T_corr;
    this->icpScore = 0.1;
    // printf(">>> score, and converged: %d, %f, %d\n",this->icpScore, this->vgicp_odom.hasConverged());
  }else
  {
    this->gicp.align(*aligned);
    this->T_corr = this->gicp.getFinalTransformation();
    this->lastIncreTransformation = this->T_corr;
    this->icpScore = 0.1;//this->gicp.getFitnessScore();
  }
  this->T = this->T_corr * this->T_prior;
  // auto t2 = std::chrono::high_resolution_clock::now();
  // float time_odom =  std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6; 
  // this->time_odom.push_back(time_odom);
  // this->odom_time_pub.publish(time_odom);

}

bool dlio::OdomNode::imuMeasFromTimeRange(double start_time, double end_time,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) {
  // imu_buffer中 front为最新的消息 back为最老的消息 这里等待 使得buffer中最新的消息时间戳大于end_time
  if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
    // Wait for the latest IMU data
    std::unique_lock<decltype(this->mtx_imu)> lock(this->mtx_imu);
    this->cv_imu_stamp.wait(lock, [this, &end_time]{ return this->imu_buffer.front().stamp >= end_time; });
  }

  auto imu_it = this->imu_buffer.begin();

  auto last_imu_it = imu_it;
  imu_it++;
  // 将last_imu_it移动到end_time处
  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
  }

  // 将imu_it移动到start_time处
  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
    imu_it++;
  }
  // IMU测量数据不足的情况
  if (imu_it == this->imu_buffer.end()) {
    // not enough IMU measurements, return false
    return false;
  }
  imu_it++;

  // Set reverse iterators (to iterate forward in time)
  end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
  begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
dlio::OdomNode::integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                             Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps) {

  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;
  if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
    // invalid input, return empty vector
    return empty;
  }
  // 使用反向迭代器的原因是在imu_buffer中时间顺序是从大到小的 start_time在后 end_time在前
  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
  boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
  // 获得start_time和end_time对应的 IMU测量数据段 分别保存在begin_imu_it和end_imu_it
  if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it) == false) {
    // not enough IMU measurements, return empty vector
    return empty;
  }

  // Backwards integration to find pose at first IMU sample
  const ImuMeas& f1 = *begin_imu_it;
  const ImuMeas& f2 = *(begin_imu_it+1);

  // 两帧IMU之间的时间差
  // Time between first two IMU samples
  double dt = f2.dt;
  // begin_imu与start_time之间的时间差 因为肯定不是完全相等的 还存在一点差异
  // Time between first IMU sample and start_time
  double idt = start_time - f1.stamp;

  // 前两次IMU采样中的角加速度
  // Angular acceleration between first two IMU samples
  Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
  Eigen::Vector3f alpha = alpha_dt / dt;

  // 这里的时间关系是
  // ------|-----------|----------|----------
  //      IMU_i    Start_time   IMU_i+1
  // 获得前半段时间内的平均角速度
  // Average angular velocity (reversed) between first IMU sample and start_time
  Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5*alpha*idt);
  // 将q_init转换到第一帧IMU时的状态 即已知start_time的状态 往前回溯 得到IMU_i的状态 从这里开始积分
  // 角速度为body坐标系下 并不是世界坐标系下 因此采用右乘    q_init = q_init x [1  0.5*\omega*idt]
  // Set q_init to orientation at first IMU sample
  q_init = Eigen::Quaternionf (
        q_init.w() - 0.5*( q_init.x()*omega_i[0] + q_init.y()*omega_i[1] + q_init.z()*omega_i[2] ) * idt,
        q_init.x() + 0.5*( q_init.w()*omega_i[0] - q_init.z()*omega_i[1] + q_init.y()*omega_i[2] ) * idt,
        q_init.y() + 0.5*( q_init.z()*omega_i[0] + q_init.w()*omega_i[1] - q_init.x()*omega_i[2] ) * idt,
        q_init.z() + 0.5*( q_init.x()*omega_i[1] - q_init.y()*omega_i[0] + q_init.w()*omega_i[2] ) * idt
  );
  q_init.normalize();

  // Average angular velocity between first two IMU samples
  // 前两帧IMU之间的平均角速度
  Eigen::Vector3f omega = f1.ang_vel + 0.5*alpha_dt;
  // 得到第二帧IMU时的旋转
  // Orientation at second IMU sample
  Eigen::Quaternionf q2 (
    q_init.w() - 0.5*( q_init.x()*omega[0] + q_init.y()*omega[1] + q_init.z()*omega[2] ) * dt,
    q_init.x() + 0.5*( q_init.w()*omega[0] - q_init.z()*omega[1] + q_init.y()*omega[2] ) * dt,
    q_init.y() + 0.5*( q_init.z()*omega[0] + q_init.w()*omega[1] - q_init.x()*omega[2] ) * dt,
    q_init.z() + 0.5*( q_init.x()*omega[1] - q_init.y()*omega[0] + q_init.w()*omega[2] ) * dt
  );
  q2.normalize();

  // 将第一帧和第二帧的加速度转换到世界坐标系
  // Acceleration at first IMU sample
  Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
  a1[2] -= this->gravity_;

  // Acceleration at second IMU sample
  Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
  a2[2] -= this->gravity_;

  // 计算加加速度
  // Jerk between first two IMU samples
  Eigen::Vector3f j = (a2 - a1) / dt;

  // 将速度和位置都回溯到第一帧IMU的状态
  // 认为加加速度是常量 有a = kt + c
  // Set v_init to velocity at first IMU sample (go backwards from start_time)
  v_init -= a1*idt + 0.5*j*idt*idt;

  // Set p_init to position at first IMU sample (go backwards from start_time)
  p_init -= v_init*idt + 0.5*a1*idt*idt + (1/6.)*j*idt*idt*idt;

  return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, begin_imu_it, end_imu_it);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
dlio::OdomNode::integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                                     const std::vector<double>& sorted_timestamps,
                                     boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                                     boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;
  // 获取初始状态
  // Initialization
  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  // 加速度为世界坐标系下
  Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
  a[2] -= this->gravity_;

  // Iterate over IMU measurements and timestamps
  auto prev_imu_it = begin_imu_it;
  auto imu_it = prev_imu_it + 1;

  auto stamp_it = sorted_timestamps.begin();

  for (; imu_it != end_imu_it; imu_it++) {

    const ImuMeas& f0 = *prev_imu_it;
    const ImuMeas& f = *imu_it;
    // 相邻两帧IMU之间的时间差
    // Time between IMU samples
    double dt = f.dt;
    // 计算角加速度
    // Angular acceleration
    Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
    Eigen::Vector3f alpha = alpha_dt / dt;
    // 计算平均角速度
    // Average angular velocity
    Eigen::Vector3f omega = f0.ang_vel + 0.5*alpha_dt;

    // 旋转姿态传播
    // Orientation
    q = Eigen::Quaternionf (
      q.w() - 0.5*( q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2] ) * dt,
      q.x() + 0.5*( q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2] ) * dt,
      q.y() + 0.5*( q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2] ) * dt,
      q.z() + 0.5*( q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2] ) * dt
    );
    q.normalize();

    // Acceleration
    // 根据刚刚传播的状态 将后IMU加速度转换到世界坐标系下
    Eigen::Vector3f a0 = a;
    a = q._transformVector(f.lin_accel);
    a[2] -= this->gravity_;

    // Jerk
    // 计算加加速度
    Eigen::Vector3f j_dt = a - a0;
    Eigen::Vector3f j = j_dt / dt;

    // 对给定时间戳状态进行插值求解
    // -------------|-----------+---------+-----------+-------------|-------------------------
    //             f0           p1        p2          p3             f
    // Interpolate for given timestamps
    while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
      // Time between previous IMU sample and given timestamp
      // 计算时间差
      double idt = *stamp_it - f0.stamp;
      // f0 与 待插值时间点内的平均角速度
      // Average angular velocity
      Eigen::Vector3f omega_i = f0.ang_vel + 0.5*alpha*idt;
      // 旋转传播
      // Orientation
      Eigen::Quaternionf q_i (
        q.w() - 0.5*( q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2] ) * idt,
        q.x() + 0.5*( q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2] ) * idt,
        q.y() + 0.5*( q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2] ) * idt,
        q.z() + 0.5*( q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2] ) * idt
      );
      q_i.normalize();
      // 位置传播
      // Position
      Eigen::Vector3f p_i = p + v*idt + 0.5*a0*idt*idt + (1/6.)*j*idt*idt*idt;
      // 写入齐次变换
      // Transformation
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
      T.block(0, 3, 3, 1) = p_i;

      imu_se3.push_back(T);

      stamp_it++;
    }

    // p和v向下一帧IMU进行传播 q在上面已经算过了
    // Position
    p += v*dt + 0.5*a0*dt*dt + (1/6.)*j_dt*dt*dt;

    // Velocity
    v += a0*dt + 0.5*j_dt*dt;

    prev_imu_it = imu_it;

  }

  return imu_se3;

}

void dlio::OdomNode::propagateGICP() {

  this->lidarPose.p << this->T(0,3), this->T(1,3), this->T(2,3);

  Eigen::Matrix3f rotSO3;
  rotSO3 << this->T(0,0), this->T(0,1), this->T(0,2),
            this->T(1,0), this->T(1,1), this->T(1,2),
            this->T(2,0), this->T(2,1), this->T(2,2);

  Eigen::Quaternionf q(rotSO3);

  // Normalize quaternion
  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  q.w() /= norm; q.x() /= norm; q.y() /= norm; q.z() /= norm;
  this->lidarPose.q = q;


}

// callbackImu
void dlio::OdomNode::propagateState() {

  // Lock thread to prevent state from being accessed by UpdateState
  std::lock_guard<std::mutex> lock( this->geo.mtx );

  double dt = this->imu_meas.dt;
  // 获取当前的姿态 
  Eigen::Quaternionf qhat = this->state.q, omega;
  Eigen::Vector3f world_accel;
  // 将加速度转换到世界坐标系
  // Transform accel from body to world frame
  world_accel = qhat._transformVector(this->imu_meas.lin_accel);

  // 世界坐标系下的加速度传播 得到pos "p = vt + 1/2 a t^2"
  // Accel propogation
  this->state.p[0] += this->state.v.lin.w[0]*dt + 0.5*dt*dt*world_accel[0];
  this->state.p[1] += this->state.v.lin.w[1]*dt + 0.5*dt*dt*world_accel[1];
  this->state.p[2] += this->state.v.lin.w[2]*dt + 0.5*dt*dt*(world_accel[2] - this->gravity_);
  // 世界坐标系下的速度传播 v = v + at
  this->state.v.lin.w[0] += world_accel[0]*dt;
  this->state.v.lin.w[1] += world_accel[1]*dt;
  this->state.v.lin.w[2] += (world_accel[2] - this->gravity_)*dt;
  this->state.v.lin.b = this->state.q.toRotationMatrix().inverse() * this->state.v.lin.w;

  // 姿态传播
  // Gyro propogation
  omega.w() = 0;
  omega.vec() = this->imu_meas.ang_vel;
  Eigen::Quaternionf tmp = qhat * omega;
  this->state.q.w() += 0.5 * dt * tmp.w();
  this->state.q.vec() += 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  this->state.q.normalize();

  this->state.v.ang.b = this->imu_meas.ang_vel;
  this->state.v.ang.w = this->state.q.toRotationMatrix() * this->state.v.ang.b;

}

void dlio::OdomNode::updateState() {

  // Lock thread to prevent state from being accessed by PropagateState 与imu数据呼应
  std::lock_guard<std::mutex> lock( this->geo.mtx );

  Eigen::Vector3f pin = this->lidarPose.p;
  Eigen::Quaternionf qin = this->lidarPose.q;
  double dt = this->scan_stamp - this->prev_scan_stamp;

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Constuct error quaternion 共轭
  qe = qhat.conjugate()*qin;

  double sgn = 1.;
  if (qe.w() < 0) {
    sgn = -1;
  }

  // Construct quaternion correction 旋转校正
  qcorr.w() = 1 - abs(qe.w());
  qcorr.vec() = sgn*qe.vec();
  qcorr = qhat * qcorr;

  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  double abias_max = this->geo_abias_max_;
  double gbias_max = this->geo_gbias_max_;

  // Update accel bias
  this->state.b.accel -= dt * this->geo_Kab_ * err_body;
  this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);
  // Update gyro bias
  this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
  this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
  this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
  this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);
  // Update state
  // if(this->iskf)
  // {
  //   //keyframes
  //   this->state.p = this->lidarPose.p;
  //   this->state.q = this->lidarPose.q;
  //   this->iskf = false;
  // }else{
    
    //common frames
    this->state.p += dt * this->geo_Kp_ * err;
    this->state.v.lin.w += dt * this->geo_Kv_ * err;
    this->state.q.w() += dt * this->geo_Kq_ * qcorr.w();
    this->state.q.x() += dt * this->geo_Kq_ * qcorr.x();
    this->state.q.y() += dt * this->geo_Kq_ * qcorr.y();
    this->state.q.z() += dt * this->geo_Kq_ * qcorr.z();
    this->state.q.normalize();
  // }

  // store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;

}



sensor_msgs::Imu::Ptr dlio::OdomNode::transformImu(const sensor_msgs::Imu::ConstPtr& imu_raw) {
  sensor_msgs::Imu::Ptr imu (new sensor_msgs::Imu);
  // 复制消息头
  // Copy header
  imu->header = imu_raw->header;
  // 获取第一帧IMU的时间戳
  static double prev_stamp = imu->header.stamp.toSec();
  // 计算当前帧IMU与上一帧IMU之间的时间差
  double dt = imu->header.stamp.toSec() - prev_stamp;
  prev_stamp = imu->header.stamp.toSec();
  
  if (dt == 0) { dt = 1.0/200.0; }

  // Transform angular velocity (will be the same on a rigid body, so just rotate to ROS convention)
  // 获取角速度
  Eigen::Vector3f ang_vel(imu_raw->angular_velocity.x,
                          imu_raw->angular_velocity.y,
                          imu_raw->angular_velocity.z);
  // 获取base_link下的角速度
  Eigen::Vector3f ang_vel_cg = this->extrinsics.baselink2imu.R * ang_vel;
  // 用base_link下的角速度赋值
  imu->angular_velocity.x = ang_vel_cg[0];
  imu->angular_velocity.y = ang_vel_cg[1];
  imu->angular_velocity.z = ang_vel_cg[2];

  static Eigen::Vector3f ang_vel_cg_prev = ang_vel_cg;

  // Transform linear acceleration (need to account for component due to translational difference)
  // 获取加速度
  Eigen::Vector3f lin_accel(imu_raw->linear_acceleration.x,
                            imu_raw->linear_acceleration.y,
                            imu_raw->linear_acceleration.z);
  // 考虑旋转
  Eigen::Vector3f lin_accel_cg = this->extrinsics.baselink2imu.R * lin_accel;
  // 考虑平移
  lin_accel_cg = lin_accel_cg
                 + ((ang_vel_cg - ang_vel_cg_prev) / dt).cross(-this->extrinsics.baselink2imu.t)
                 + ang_vel_cg.cross(ang_vel_cg.cross(-this->extrinsics.baselink2imu.t));

  ang_vel_cg_prev = ang_vel_cg;
  // 赋值
  imu->linear_acceleration.x = lin_accel_cg[0];
  imu->linear_acceleration.y = lin_accel_cg[1];
  imu->linear_acceleration.z = lin_accel_cg[2];

  return imu;

}

void dlio::OdomNode::computeMetrics() {
  // 根据当前帧点云空间密度计算距离阈值
  this->computeDensity();
  this->computeSpaciousness();
  
}

void dlio::OdomNode::computeSpaciousness() {

  // 遍历计算每个点的range
  std::vector<float> ds;
  //original_scan 当前帧
  // auto temp_scan = this->current_scan;
  for (int i = 0; i <= this->original_scan->points.size(); i++) {
    float d = std::sqrt(pow(this->original_scan->points[i].x, 2) +
                        pow(this->original_scan->points[i].y, 2) +
                        pow (this->original_scan->points[i].z,2)); // mid360: 40m 10%反射率 70m 80%反射率
      ds.push_back(d);
  }

  // float max_range = *std::max_element(ds.begin(),ds.end());
  // this->sp_pub.publish(max_range);
  // 这里用的是没有去畸变的点云 取距离的中位数, 找到range中位数 并将其置于ds索引为ds.size()/2的位置 其之前都比该值小 之后都比该值大
  std::nth_element(ds.begin(), ds.begin() + ds.size()/2, ds.end());
  float median_curr = ds[ds.size()/2];
  static float median_prev = median_curr;
  // models  sparsity
  float den = this->metrics.density.back();
  float median_lpf = 0.9*median_prev + 0.1*median_curr - std::floor(den);
  median_prev = median_lpf;

  // push
  this->metrics.spaciousness.push_back( median_lpf );

}
void dlio::OdomNode::computeDensity() {
      float density;
      if (!this->geo.first_opt_done) {
        density = 0.;
      } else {
        density = this->gicp.source_density_; 
      }

      static float density_prev = density;
      float density_lpf = 0.95*density_prev + 0.05*density;
      density_prev = density_lpf;

      this->metrics.density.push_back( density_lpf );
    }

void dlio::OdomNode::setAdaptiveParams() {
    // 获取当前帧点云的spacious和density
    float sp = this->metrics.spaciousness.back();
    if (sp < 0.5) { sp = 0.5; }
    if (sp > this->envir_thresh_) { sp = this->envir_thresh_; }

    this->keyframe_thresh_dist_ = sp;
    float den = this->metrics.density.back();

    // 进行限制 判断属于小环境还是大环境
    if (den < 0.5*this->gicp_max_corr_dist_) { den = 0.5*this->gicp_max_corr_dist_;this->bigscale = false;}
    if (den > 2.0*this->gicp_max_corr_dist_) { den = 2.0*this->gicp_max_corr_dist_;this->bigscale = true; }
    
    if (sp < this->envir_thresh_) { den = 0.5*this->gicp_max_corr_dist_; };
    if (sp > this->envir_thresh_) { den = 2.0*this->gicp_max_corr_dist_; };
    
    this->concave_hull.setAlpha(this->keyframe_thresh_dist_);
    this->gicp.setMaxCorrespondenceDistance(den);
    this->vgicp_odom.setMaxCorrespondenceDistance(den);

}

// 筛选出与当前帧最近的k个关键帧
void dlio::OdomNode::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames) {

  // make sure dists is not empty
  if (dists.empty()) { return; }
  // 
  if (dists.size() == 1) { this->submap_kf_idx_curr.push_back(Similarity {frames[0], 1.f}); return; };

  // maintain max heap of at most k elements
  std::priority_queue<float> pq;

  for (auto d : dists) {
    if (pq.size() >= k && pq.top() > d) {
      pq.push(d);
      pq.pop();
    } else if (pq.size() < k) {
      pq.push(d);
    }
  }

  // get the kth smallest element, which should be at the top of the heap
  float kth_element = pq.top();

  // get all elements smaller or equal to the kth smallest element
  for (int i = 0; i < dists.size(); ++i) {
    if (dists[i] <= kth_element)
      this->submap_kf_idx_curr.push_back(Similarity { frames[i], dists[i] / kth_element });
  }

}
/**
 * @brief 构建submap： 基于欧式空间
 * @param vehicle_state
 * @note 该函数较为重要 对于第一帧而言 由于num_processed_keyframes=1 所以该函数只完成了将第一帧的点云加入submap 构建vgicp target
 */
void dlio::OdomNode::buildSubmap(State vehicle_state) 
{ //初始化 已经将第一帧设为关键帧，作为子图
  static int max_num = 2;
  
  auto t1 = std::chrono::high_resolution_clock::now();
  // clear vector of keyframe indices to use for submap
  this->submap_kf_idx_curr.clear();
  this->keyframe_nn.clear();
  this->ds.clear();
  // 基于空间欧式距离的关键帧选择
  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  //遍历刚刚遍历过的关键帧
  for (int i = 0; i < this->num_processed_keyframes ; i++) {
    // 当前帧状态与历史关键帧空间位置差
    float d = sqrt( pow(vehicle_state.p[0] - this->keyframes[i].first.first[0], 2) +
                    pow(vehicle_state.p[1] - this->keyframes[i].first.first[1], 2) +
                    pow(vehicle_state.p[2] - this->keyframes[i].first.first[2], 2) );
    this->ds.push_back(d);
    this->keyframe_nn.push_back(i);
  }
  lock.unlock();
  // 在上述关键帧中筛选出与当前帧位置最近的submap_knn_个关键帧 submap_kf_idx_curr #20
  this->pushSubmapIndices(this->ds, this->submap_knn_, this->keyframe_nn);
  
  this->computeConvexHull();
  std::vector<float> convex_ds;
  for (const auto& c : this->keyframe_convex) {
    convex_ds.push_back(this->ds[c]);
  }

  // get indices for top kNN for convex hull
  this->pushSubmapIndices(convex_ds, this->submap_kcv_, this->keyframe_convex);
  
  // get concave hull indices
  this->computeConcaveHull();
  // get distances for each keyframe on concave hull
  std::vector<float> concave_ds;
  for (const auto& c : this->keyframe_concave) {
    concave_ds.push_back(this->ds[c]);
  }
  // get indices for top kNN for concave hull
  this->pushSubmapIndices(concave_ds, this->submap_kcc_, this->keyframe_concave);

  //sort current and previous submap kf list of indices
  std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end(),[](const auto& a, const auto& b) { return a.index > b.index; });
  // std::sort(this->submap_kf_idx_prev.begin(), this->submap_kf_idx_prev.end(),[](const auto& a, const auto& b) { return a.index > b.index; });

  auto index_equality = [](const auto& a, const auto& b) { return a.index == b.index; };
  // std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end(),index_equality);
  this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());
  // 检查当前搜索得到的关键帧vector是否与前一帧使用的关键帧vector相同
  // if (this->submap_kf_idx_curr != this->submap_kf_idx_prev)
  bool new_kf = this->submap_kf_idx_curr.size() != this->submap_kf_idx_prev.size();
  if(new_kf || !std::equal(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end(), this->submap_kf_idx_prev.begin(), index_equality))
  {
    // std::cout << "submap_kf.size() = " << this->submap_kf_idx_curr.size() << std::endl;
    this->submap_hasChanged = true;
    // Pause to prevent stealing resources from the main loop if it is running.
    this->pauseSubmapBuildIfNeeded();

    // 如果不同 更新submap及其协方差
    std::shared_ptr<nano_gicp::CovarianceList> submap_normals_ (std::make_shared<nano_gicp::CovarianceList>());
    pcl::PointCloud<PointType>::Ptr submap_cloud_ (boost::make_shared<pcl::PointCloud<PointType>>());
    // std::shared_ptr<nano_gicp::CovarianceList> submap_normals_ (std::make_shared<nano_gicp::CovarianceList>());

    for (auto [k, _]: this->submap_kf_idx_curr) {
      //构建submap
      lock.lock();
        *submap_cloud_ += *this->keyframes[k].second;
      lock.unlock();
        submap_normals_->insert(std::end(*submap_normals_),
        std::begin(*this->keyframe_normals_v[k]), std::end(*this->keyframe_normals_v[k]));
    }

    this->submap_cloud = submap_cloud_;
    this->submap_normals_v = submap_normals_;
    this->submap_kf_idx_prev = this->submap_kf_idx_curr;
      this->pauseSubmapBuildIfNeeded();
    if(this->matchMethod == "GICP")
    {
        this->gicp_temp.setInputTarget(this->submap_cloud);
        this->submap_kdtree_v = this->gicp_temp.target_kdtree_;
    }
    std::unique_lock<decltype(this->kf_sim_mutex)> lock(this->kf_sim_mutex);
      this->kf_sim_buffer.push_back(this->submap_kf_idx_prev);
    if (this->pubHistoryKeyFrames.getNumSubscribers() != 0) // 有订阅者就发布
      this->publishCloud(this->pubHistoryKeyFrames, submap_cloud_, ros::Time::now(),this->odom_frame);// 返回submap点云
    
  }
}

// void dlio::OdomNode::buildSubmapViaJaccard(State vehicle_state,pcl::PointCloud<PointType>::ConstPtr cloud)
// {
//       this->submap_kf_idx_curr.clear();
//       this->keyframe_nn.clear();
//       this->ds.clear();

//       std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
//       for (int i = 0; i < this->num_processed_keyframes ; i++) {
//       // 当前帧状态与历史关键帧空间位置差
//         float d = sqrt( pow(vehicle_state.p[0] - this->keyframes[i].first.first[0], 2) +
//                 pow(vehicle_state.p[1] - this->keyframes[i].first.first[1], 2) +
//                 pow(vehicle_state.p[2] - this->keyframes[i].first.first[2], 2) );
//         this->ds.push_back(d);
//         this->keyframe_nn.push_back(i);
//       }
//       lock.unlock();

//       // get indices for top K nearest neighbor keyframe poses
//       this->pushSubmapIndices(this->ds,this->submap_knn_,this->keyframe_nn);
      
//       auto size_submap = this->submap_kf_idx_curr.size();
//       std::vector<int> intersection_nums(size_submap);
//       std::vector<int> union_nums(size_submap);
//       std::vector<Similarity> jaccardian_candidates;jaccardian_candidates.reserve(size_submap);

//       if(this->submap_kf_idx_curr.size() > 3)
//       {
//           for(int i = 0; i< this->submap_kf_idx_curr.size();i++)
//           {
//                 auto kf_idx = this->submap_kf_idx_curr[i].index;

//                 lock.lock();
//                 const auto& tree =  this->kf_tree[kf_idx];
//                 const auto& size = this->keyframes[kf_idx].second->size();
//                 lock.unlock();

//                 auto& intersection = intersection_nums[i];
//                 auto& union_ = union_nums[i];

//                 int id;
//                 float dis;

//                 for (const auto& query: cloud->points){
//                       tree->approxNearestSearch(query,id,dis);
//                       intersection_nums[i] += (dis < this->jaccard_corr_thresh_) * 1;
//                 }

//                 union_ = cloud->size() + size - intersection;
//                 auto similarity = float(intersection) / float(union_);
//                 if(similarity > this->jaccard_sim_thresh_) //0.2
//                     jaccardian_candidates.emplace_back(Similarity{kf_idx,similarity});
//           }

//           if (jaccardian_candidates.size() > 3)
//           {
//             this->submap_kf_idx_curr = jaccardian_candidates;
//           }else{
//             ROS_DEBUG("Bad jaccardian match !");
//           }
//       }

//           // sort by index to make it comparable with prev
//           std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end(), [](const auto& a, const auto& b) { return a.index > b.index; });
//           auto index_equality = [](const auto& a, const auto& b) { return a.index == b.index; };

//           auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end(), index_equality);
//           this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());

//           bool new_kf = this->submap_kf_idx_curr.size() != this->submap_kf_idx_prev.size();
//           if(new_kf || !std::equal(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end(), this->submap_kf_idx_prev.begin(), index_equality))
//           {
//             this->submap_hasChanged = true;
//             this->pauseSubmapBuildIfNeeded();

//             // 如果不同 更新submap及其协方差
//             std::shared_ptr<fast_gicp::CovariancesList> submap_normals_ (std::make_shared<fast_gicp::CovariancesList>());
//             pcl::PointCloud<PointType>::Ptr submap_cloud_ (boost::make_shared<pcl::PointCloud<PointType>>());
//             for (auto [k, sim]: this->submap_kf_idx_curr) {
//               //构建submap
//               lock.lock();
//               *submap_cloud_ += *this->keyframes[k].second;
//               lock.unlock();

//               // grab corresponding submap cloud's normals
//               submap_normals_->insert( std::end(*submap_normals_),
//                                 std::begin((this->keyframe_normals_v[k])), std::end((this->keyframe_normals_v[k])));
//             }

//             this->submap_cloud = submap_cloud_;
//             this->submap_normals_v = submap_normals_;
            
//             this->pauseSubmapBuildIfNeeded();
            
//             this->submap_kf_idx_prev = this->submap_kf_idx_curr;

//             std::unique_lock<decltype(this->kf_sim_mutex)> lock(this->kf_sim_mutex);
//             this->kf_sim_buffer.push_back(this->submap_kf_idx_prev);
//             // lock.unlock();

//           }



// }
  


void dlio::OdomNode::visualizeSubmap(State vehicle_state){
  // 可视化组成submap的关键帧位置
  visualization_msgs::Marker kf_connect_marker, convex_connect_marker;
  kf_connect_marker.ns = "kf_extraction";
  kf_connect_marker.header.stamp = this->imu_stamp;
  kf_connect_marker.header.frame_id = this->odom_frame;
  kf_connect_marker.id = 0;
  kf_connect_marker.type = visualization_msgs::Marker::LINE_LIST;
  kf_connect_marker.scale.x = 0.1;
  kf_connect_marker.color.r = 1.0;
  kf_connect_marker.color.g = 0.0;
  kf_connect_marker.color.b = 0.0;
  kf_connect_marker.color.a = 1.0;
  kf_connect_marker.action = visualization_msgs::Marker::ADD;
  kf_connect_marker.pose.orientation.w = 1.0;

  convex_connect_marker.ns = "convex_extraction";
  convex_connect_marker.header.stamp = this->imu_stamp;
  convex_connect_marker.header.frame_id = this->odom_frame;
  convex_connect_marker.id = 0;
  convex_connect_marker.type = visualization_msgs::Marker::LINE_LIST;
  convex_connect_marker.scale.x = 0.1;
  convex_connect_marker.color.r = 0.0;
  convex_connect_marker.color.g = 1.0;
  convex_connect_marker.color.b = 0.0;
  convex_connect_marker.color.a = 1.0;
  convex_connect_marker.action = visualization_msgs::Marker::ADD;
  convex_connect_marker.pose.orientation.w = 1.0;
  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  
  for (auto [i,_] : this->submap_kf_idx_curr)
  {
      auto vex = std::find(this->keyframe_convex.begin(),this->keyframe_convex.end(),i);
      if(vex != this->keyframe_convex.end())
      {
        geometry_msgs::Point point1;
        point1.x = vehicle_state.p[0]; 
        point1.y = vehicle_state.p[1];
        point1.z = vehicle_state.p[2];
        convex_connect_marker.points.push_back(point1);
        geometry_msgs::Point point2;
        point2.x = this->cloudKeyPoses6D->points[i].x; //this->keyframes[i].first.first.x();
        point2.y = this->cloudKeyPoses6D->points[i].y;
        point2.z = this->cloudKeyPoses6D->points[i].z;
        convex_connect_marker.points.push_back(point2);
      }else{
        geometry_msgs::Point point1;
        point1.x = vehicle_state.p[0]; 
        point1.y = vehicle_state.p[1];
        point1.z = vehicle_state.p[2];
        kf_connect_marker.points.push_back(point1);
        geometry_msgs::Point point2;
        point2.x = this->cloudKeyPoses6D->points[i].x; //this->keyframes[i].first.first.x();
        point2.y = this->cloudKeyPoses6D->points[i].y;
        point2.z = this->cloudKeyPoses6D->points[i].z;
        kf_connect_marker.points.push_back(point2);
      }
      
  }
  lock.unlock();
  
  // if (kf_connect_marker.points.size() > 0)
      this->kf_connect_pub.publish(kf_connect_marker);
      this->convex_connect_pub.publish(convex_connect_marker);

}

void dlio::OdomNode::buildKeyframesAndSubmap(State vehicle_state) {
  auto t1 = std::chrono::high_resolution_clock::now();
  // transform the new keyframe(s) and associated covariance list(s)
  std::shared_ptr<const fast_gicp::CovarianceList> raw_covariances_v;

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  // 遍历关键帧 num_processed_keyframes初始化为0，发布关键帧信息
  for (int i = this->num_processed_keyframes; i < this->keyframes.size(); i++) {
   
    // 拿到关键帧对应的点云
    pcl::PointCloud<PointType>::ConstPtr raw_keyframe = this->keyframes[i].second;
    raw_covariances_v = this->keyframe_normals_v[i];
  
    // 拿到关键帧的齐次变换
    Eigen::Matrix4f T_kf = this->keyframe_transformations[i];
    lock.unlock();

    Eigen::Matrix4d Td = T_kf.cast<double>();

    pcl::PointCloud<PointType>::Ptr transformed_keyframe (boost::make_shared<pcl::PointCloud<PointType>>());
    // submap点云转换到世界坐标系下
    pcl::transformPointCloud (*raw_keyframe, *transformed_keyframe, Td);


     std::shared_ptr<fast_gicp::CovarianceList> transformed_covariances_v (std::make_shared<fast_gicp::CovarianceList>(raw_covariances_v->size()));
    std::transform(raw_covariances_v->begin(), raw_covariances_v->end(), transformed_covariances_v->begin(),
                [&Td](Eigen::Matrix4d cov) { return Td * cov * Td.transpose(); }); 
    ++this->num_processed_keyframes;
    lock.lock();

    if(this->cur_kf)
      this->keyframes[i].second = transformed_keyframe;

    // this->kf_tree[i]->setInputCloud(this->keyframes[i].second);
    // this->kf_tree[i]->addPointsFromInputCloud();
    this->keyframe_normals_v[i] = transformed_covariances_v;
 
    this->publish_keyframe_thread = std::thread( &dlio::OdomNode::publishKeyframe, this, this->keyframes[i], this->keyframe_timestamps[i] );
    this->publish_keyframe_thread.detach();
   }
  lock.unlock();
  this->cur_kf = true;
  // Pause to prevent stealing resources from the main loop if it is running.
  this->pauseSubmapBuildIfNeeded();
  // 维护一个20个kf范围的localmap，构建submap
  this->buildSubmap(vehicle_state);
  auto t2 = std::chrono::high_resolution_clock::now();
  float built_map_time =  std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6; 
  this->time_built_submap.push_back(built_map_time);
  this->submap_time_pub.publish(built_map_time);
  // this->buildSubmapViaJaccard(vehicle_state,this->current_scan);
  this->visualizeSubmap(vehicle_state);
}

void dlio::OdomNode::computeConvexHull() {

  // at least 4 keyframes for convex hull
  if (this->num_processed_keyframes < 4) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    PointType pt;
    pt.x = this->keyframes[i].first.first[0];
    pt.y = this->keyframes[i].first.first[1];
    pt.z = this->keyframes[i].first.first[2];
    cloud->push_back(pt);
  }
  lock.unlock();

  // calculate the convex hull of the point cloud
  this->convex_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the convex hull
  pcl::PointCloud<PointType>::Ptr convex_points =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  this->convex_hull.reconstruct(*convex_points);

  pcl::PointIndices::Ptr convex_hull_point_idx = pcl::PointIndices::Ptr (boost::make_shared<pcl::PointIndices>());
  this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

  this->keyframe_convex.clear();
  for (int i=0; i<convex_hull_point_idx->indices.size(); ++i) {
    this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
  }

}
// 凹
void dlio::OdomNode::computeConcaveHull() {

  // at least 5 keyframes for concave hull
  if (this->num_processed_keyframes < 5) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    PointType pt;
    pt.x = this->keyframes[i].first.first[0];
    pt.y = this->keyframes[i].first.first[1];
    pt.z = this->keyframes[i].first.first[2];
    cloud->push_back(pt);
  }
  lock.unlock();

  // calculate the concave hull of the point cloud
  this->concave_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the concave hull
  pcl::PointCloud<PointType>::Ptr concave_points =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  this->concave_hull.reconstruct(*concave_points);

  pcl::PointIndices::Ptr concave_hull_point_idx = pcl::PointIndices::Ptr (boost::make_shared<pcl::PointIndices>());
  this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

  this->keyframe_concave.clear();
  for (int i=0; i<concave_hull_point_idx->indices.size(); ++i) {
    this->keyframe_concave.push_back(concave_hull_point_idx->indices[i]);
  }

}


void dlio::OdomNode::pauseSubmapBuildIfNeeded() {
  std::unique_lock<decltype(this->main_loop_running_mutex)> lock(this->main_loop_running_mutex);
  this->submap_build_cv.wait(lock, [this]{ return !this->main_loop_running; });
}


// void dlio::OdomNode::debug() {

//   // Total length traversed
//   double length_traversed = 0.;
//   Eigen::Vector3f p_curr = Eigen::Vector3f(0., 0., 0.);
//   Eigen::Vector3f p_prev = Eigen::Vector3f(0., 0., 0.);
//   for (const auto& t : this->trajectory) {
//     if (p_prev == Eigen::Vector3f(0., 0., 0.)) {
//       p_prev = t.first;
//       continue;
//     }
//     p_curr = t.first;
//     double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));

//     if (l >= 0.1) {
//       length_traversed += l;
//       p_prev = p_curr;
//     }
//   }
//   this->length_traversed = length_traversed;

//   // Average computation time
//   double avg_comp_time =
//     std::accumulate(this->comp_times.begin(), this->comp_times.end(), 0.0) / this->comp_times.size();

//   // Average sensor rates
//   int win_size = 100;
//   double avg_imu_rate;
//   double avg_lidar_rate;
//   if (this->imu_rates.size() < win_size) {
//     avg_imu_rate =
//       std::accumulate(this->imu_rates.begin(), this->imu_rates.end(), 0.0) / this->imu_rates.size();
//   } else {
//     avg_imu_rate =
//       std::accumulate(this->imu_rates.end()-win_size, this->imu_rates.end(), 0.0) / win_size;
//   }
//   if (this->lidar_rates.size() < win_size) {
//     avg_lidar_rate =
//       std::accumulate(this->lidar_rates.begin(), this->lidar_rates.end(), 0.0) / this->lidar_rates.size();
//   } else {
//     avg_lidar_rate =
//       std::accumulate(this->lidar_rates.end()-win_size, this->lidar_rates.end(), 0.0) / win_size;
//   }

  // RAM Usage
//   double vm_usage = 0.0;
//   double resident_set = 0.0;
//   std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
//   std::string pid, comm, state, ppid, pgrp, session, tty_nr;
//   std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
//   std::string utime, stime, cutime, cstime, priority, nice;
//   std::string num_threads, itrealvalue, starttime;
//   unsigned long vsize;
//   long rss;
//   stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
//               >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
//               >> utime >> stime >> cutime >> cstime >> priority >> nice
//               >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
//   stat_stream.close();
//   long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
//   vm_usage = vsize / 1024.0;
//   resident_set = rss * page_size_kb;

//   // CPU Usage
//   struct tms timeSample;
//   clock_t now;
//   double cpu_percent;
//   now = times(&timeSample);
//   if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
//       timeSample.tms_utime < this->lastUserCPU) {
//       cpu_percent = -1.0;
//   } else {
//       cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
//       cpu_percent /= (now - this->lastCPU);
//       cpu_percent /= this->numProcessors;
//       cpu_percent *= 100.;
//   }
//   this->lastCPU = now;
//   this->lastSysCPU = timeSample.tms_stime;
//   this->lastUserCPU = timeSample.tms_utime;
//   this->cpu_percents.push_back(cpu_percent);
//   double avg_cpu_usage =
//     std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();

//   // Print to terminal
//   printf("\033[2J\033[1;1H");

//   std::cout << std::endl
//             << "+-------------------------------------------------------------------+" << std::endl;
//   std::cout << "|   Gerenral LiDAR-Inertial Odometry And Mpping  " << this->version_  << "            |"
//             << std::endl;
//   std::cout << "+-------------------------------------------------------------------+" << std::endl;

//   std::time_t curr_time = this->scan_stamp;
//   std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
//   std::cout << "| " << std::left << asc_time;
//   std::cout << std::right << std::setfill(' ') << std::setw(42)
//     << "Elapsed Time: " + to_string_with_precision(this->elapsed_time, 2) + " seconds "
//     << "|" << std::endl;

//   if ( !this->cpu_type.empty() ) {
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//       << this->cpu_type + " x " + std::to_string(this->numProcessors)
//       << "|" << std::endl;
//   }

//   if (this->sensor == dlio::SensorType::OUSTER) {
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//       << "Sensor Rates: Ouster @ " + to_string_with_precision(avg_lidar_rate, 2)
//                                    + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
//       << "|" << std::endl;
//   } else if (this->sensor == dlio::SensorType::VELODYNE) {
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//       << "Sensor Rates: Velodyne @ " + to_string_with_precision(avg_lidar_rate, 2)
//                                      + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
//       << "|" << std::endl;
//   }else if (this->sensor == dlio::SensorType::LIVOX) {
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//       << "Sensor Rates: Livox @ " + to_string_with_precision(avg_lidar_rate, 2)
//                                   + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
//       << "|" << std::endl; 
//   }
//   else if (this->sensor == dlio::SensorType::HESAI) {
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//       << "Sensor Rates: Hesai @ " + to_string_with_precision(avg_lidar_rate, 2)
//                                   + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
//       << "|" << std::endl;
//   } else {
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//       << "Sensor Rates: Unknown LiDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
//                                           + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
//       << "|" << std::endl;
//   }

//   std::cout << "|===================================================================|" << std::endl;

//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Position     {W}  [xyz] :: " + to_string_with_precision(this->state.p[0], 4) + " "
//                                 + to_string_with_precision(this->state.p[1], 4) + " "
//                                 + to_string_with_precision(this->state.p[2], 4)
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Orientation  {W} [wxyz] :: " + to_string_with_precision(this->state.q.w(), 4) + " "
//                                 + to_string_with_precision(this->state.q.x(), 4) + " "
//                                 + to_string_with_precision(this->state.q.y(), 4) + " "
//                                 + to_string_with_precision(this->state.q.z(), 4)
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Lin Velocity {B}  [xyz] :: " + to_string_with_precision(this->state.v.lin.b[0], 4) + " "
//                                 + to_string_with_precision(this->state.v.lin.b[1], 4) + " "
//                                 + to_string_with_precision(this->state.v.lin.b[2], 4)
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Ang Velocity {B}  [xyz] :: " + to_string_with_precision(this->state.v.ang.b[0], 4) + " "
//                                 + to_string_with_precision(this->state.v.ang.b[1], 4) + " "
//                                 + to_string_with_precision(this->state.v.ang.b[2], 4)
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Accel Bias        [xyz] :: " + to_string_with_precision(this->state.b.accel[0], 8) + " "
//                                 + to_string_with_precision(this->state.b.accel[1], 8) + " "
//                                 + to_string_with_precision(this->state.b.accel[2], 8)
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Gyro Bias         [xyz] :: " + to_string_with_precision(this->state.b.gyro[0], 8) + " "
//                                 + to_string_with_precision(this->state.b.gyro[1], 8) + " "
//                                 + to_string_with_precision(this->state.b.gyro[2], 8)
//     << "|" << std::endl;

//   std::cout << "|                                                                   |" << std::endl;

//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Distance Traveled  :: " + to_string_with_precision(length_traversed, 4) + " meters"
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Distance to Origin :: "
//       + to_string_with_precision( sqrt(pow(this->state.p[0]-this->origin[0],2) +
//                                        pow(this->state.p[1]-this->origin[1],2) +
//                                        pow(this->state.p[2]-this->origin[2],2)), 4) + " meters"
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "Registration       :: keyframes: " + std::to_string(this->keyframes.size()) + ", "
//                                + "deskewed points: " + std::to_string(this->deskew_size)
//     << "|" << std::endl;
//   std::cout << "|                                                                   |" << std::endl;

//   std::cout << std::right << std::setprecision(2) << std::fixed;
//   std::cout << "| Computation Time :: "
//     << std::setfill(' ') << std::setw(6) << this->comp_times.back()*1000. << " ms    // Avg: "
//     << std::setw(6) << avg_comp_time*1000. << " / Max: "
//     << std::setw(6) << *std::max_element(this->comp_times.begin(), this->comp_times.end())*1000.
//     << "     |" << std::endl;
//   // std::cout << "| Cores Utilized   :: "
//   //   << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: "
//   //   << std::setw(6) << (avg_cpu_usage/100.) * this->numProcessors << " / Max: "
//   //   << std::setw(6) << (*std::max_element(this->cpu_percents.begin(), this->cpu_percents.end()) / 100.)
//   //                      * this->numProcessors
//   //   << "     |" << std::endl;
//   // std::cout << "| CPU Load         :: "
//   //   << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
//   //   << std::setw(6) << avg_cpu_usage << " / Max: "
//   //   << std::setw(6) << *std::max_element(this->cpu_percents.begin(), this->cpu_percents.end())
//   //   << "     |" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
//     << "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66) 
//     << "AdaptiveFilter_size  :: " + std::to_string(this->adaptiveVoxelFilter_size) 
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66) 
//     << "Loop number  :: " + to_string_with_precision(this->addloop_num) 
//     << "|" << std::endl;
//   std::cout << "| " << std::left << std::setfill(' ') << std::setw(66) 
//     << "Odom number  :: " + to_string_with_precision(this->addodom_num) 
//     << "|" << std::endl;
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66) 
//     << "Submap_kf_size  :: " + to_string_with_precision(this->submap_kf_idx_curr.size()) 
//     << "|" << std::endl;
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66) 
//     << "Num_usedthreads  :: " + to_string_with_precision(this->num_threads_) 
//     << "|" << std::endl;
//     // std::cout << "| " << std::left << std::setfill(' ') << std::setw(66) 
//     // << "VGICP_Time  :: " + to_string_with_precision(this->time_odom) 
//     // << "|" << std::endl;
//     std::cout << "| " << std::left << std::setfill(' ') << std::setw(66) 
//     << "Submap_covs  :: " + to_string_with_precision(this->submap_normals_v->size()) 
//     << "|" << std::endl;
//   std::cout << "+-------------------------------------------------------------------+" << std::endl;

// }


gtsam::Pose3 dlio::OdomNode::state2Pose3(Eigen::Quaternionf rot, Eigen::Vector3f pos)
{
    rot.normalize();
    return gtsam::Pose3(gtsam::Rot3(rot.cast<double>()), gtsam::Point3(pos.cast<double>()));
}
Eigen::Affine3f dlio::OdomNode::pclPointToAffine3f(PointTypePose thisPoint)
{
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}
gtsam::Pose3 dlio::OdomNode::pclPointTogtsamPose3(PointTypePose thisPoint)
{
      return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                    gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}



void dlio::OdomNode::performLoopThread()
{

  ros::Rate rate(this->loop_frequeency); 
  while (this->nh.ok())
  {
    rate.sleep();
    if (this->loopflag == true){
    this->performLoopClosure();
    this->visualizeLoopClosure();}
    this->updatemap();
  }
  
}
/*
  @brief 执行map2map（2-20）
*/
void dlio::OdomNode::performLoopClosure()
{
  if (this->cloudKeyPoses6D->size() < 2)
    return;
  
  std::unique_lock<decltype(this->keyframes_mutex)> lock_kf(this->keyframes_mutex);
  // std::vector<pcl::PointCloud<PointType>::ConstPtr> his_kf_scan = this->keyframesInfo;
  // std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> kf_T = this->keyframe_transformations;
  *this->copy_cloudKeyPoses6D = *this->cloudKeyPoses6D;
  auto keyfra_ = this->keyframes;
  lock_kf.unlock();
   // 当前关键帧索引
  int loopKeyCur;
  // 候选帧索引
  int loopKeyPre;
  
  // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧 (可近邻搜索也可半径搜索)
  if (this->detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) {return;}

  pcl::PointCloud<PointType>::Ptr cureKeyframeCloud (new pcl::PointCloud<PointType>());//= pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::PointCloud<PointType>::Ptr prevKeyframeCloud (new pcl::PointCloud<PointType>());// = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  // ROS_WARN("DEBUG");
  // 提取当前关键帧相邻的2帧点集合，并降采样
  this->loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur,0,keyfra_);
  // 提取闭环匹配关键帧前后 相邻若干帧的关键帧的点集合，并降采样
  this->loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre,this->loop_search_Num,keyfra_);
  
  // 发布闭环匹配关键帧局部map的点云
  // if (this->publoopmap.getNumSubscribers() != 0) // 有订阅者就发布
  //     this->publishCloud(this->publoopmap, prevKeyframeCloud,ros::Time::now(),this->odom_frame);
  pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());

  auto t1 = std::chrono::high_resolution_clock::now();
  this->vgicp_loop.setInputSource(cureKeyframeCloud);
  this->vgicp_loop.setInputTarget(prevKeyframeCloud);
  // this->vgicp_loop.setTargetCovariances(*this->loop_local_map);
  this->vgicp_loop.align(*unused_result);
  float noiseScore = this->vgicp_loop.getFitnessScore();
  auto t2 = std::chrono::high_resolution_clock::now();
  
  if (this->vgicp_loop.hasConverged() == false || this->vgicp_loop.getFitnessScore() > this->loop_score_)
    return;
  this->vgicp_loop_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
  // Get pose transformation
  
  Eigen::Transform<float, 3, Eigen::Affine> correctionLidarFrame(this->vgicp_loop.getFinalTransformation());
  // Eigen::Affine3f tWrong = this->pclPointToAffine3f(this->copy_cloudKeyPoses6D->points[loopKeyCur]);
  Eigen::Affine3f tWrong= Eigen::Affine3f::Identity();
  tWrong.translate(keyfra_[loopKeyCur].first.first);
  tWrong.rotate(keyfra_[loopKeyCur].first.second);

  // transform from world origin to corrected pose
  std::cout << "before loop cur & curid: \n" << tWrong.translation() << std::endl;
  printf("roll:%f, pitch:%f, yaw:%f \n",tWrong.rotation().eulerAngles(2,1,0)[2],tWrong.rotation().eulerAngles(2,1,0)[1],tWrong.rotation().eulerAngles(2,1,0)[0]);
  std::cout << loopKeyCur << std::endl; 
  Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // 当前帧补偿后的结果
  
  std::cout << "after loop cur & preid:  \n" << tCorrect.translation() << std::endl;
  std::cout << loopKeyPre<< std::endl;
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles (tCorrect,x, y, z, roll, pitch, yaw);
  printf("x,y,z,roll,pitch,yaw: %f, %f, %f, %f, %f, %f\n",x,y,z,roll, pitch, yaw);
  gtsam::Pose3 poseFrom = gtsam::Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  // gtsam::Pose3 poseTo = this->pclPointTogtsamPose3(this->copy_cloudKeyPoses6D->points[loopKeyPre]);
  gtsam::Pose3 poseTo = this->state2Pose3(keyfra_[loopKeyPre].first.second,keyfra_[loopKeyPre].first.first);
  ROS_WARN("******************回环对生成****************,id1 & id2: %d,%d",loopKeyCur,loopKeyPre);
  gtsam::Vector Vector6(6);
  Vector6 << 1e-8,1e-8,1e-8,1e-6,1e-6,1e-6;
  // Vector6 << noiseScore,noiseScore,noiseScore, noiseScore, noiseScore, noiseScore;
  gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6*noiseScore);
  
  std::unique_lock<decltype(this->loop_factor_mutex)> lock(this->loop_factor_mutex);
  this->loopIndexQueue.push_back(make_pair(loopKeyCur,loopKeyPre)); // 存储回环索引；
  this->loopPoseQueue.push_back(poseFrom.between(poseTo));           // 存储回环间相对位姿；
  this->loopNoiseQueue.push_back(constraintNoise);
  lock.unlock();
 

  this->loopIndexContainer[loopKeyCur] = loopKeyPre;

}

bool dlio::OdomNode::detectLoopClosureDistance(int *latestID, int *closestID)
{
  // loopKeyCur初始化为copy_cloudKeyPoses3D 对应的最大索引
  int loopKeyCur = this->copy_cloudKeyPoses6D->size() - 1;
  int loopKeyPre = -1;

  //当前帧已经添加过闭环对应关系，将不再继续添加
  auto it = this->loopIndexContainer.find(loopKeyCur);
  if (it != this->loopIndexContainer.end())
    return false;
  // 在历史关键帧中查找与当前关键帧 距离最近 的关键帧 集合 作为历史候选关键帧集合
  std::vector<int> pointSearchIndLoop; 
  std::vector<float> pointSearchSqDisLoop;
  this->kdtreeHistoryKeyPoses->setInputCloud(this->copy_cloudKeyPoses6D);
  // ROS_WARN(">>> %d\n",this->kdtreeHistoryKeyPoses->getInputCloud()->points.size());
  // 可K近邻，R半径
  this->kdtreeHistoryKeyPoses->nearestKSearch(this->copy_cloudKeyPoses6D->back(),this->loop_search_R, pointSearchIndLoop, pointSearchSqDisLoop);//0);
  for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
  {
      int id = pointSearchIndLoop[i];
      if (abs(this->copy_cloudKeyPoses6D->points[id].time - this->timelaserCur) > 35.0) //30
      {
          loopKeyPre = id;
          break;
      }
  }

  if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
      return false;

  *latestID = loopKeyCur;
  *closestID = loopKeyPre;

  return true;
  
}

// void dlio::OdomNode::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum,std::vector<pcl::PointCloud<PointType>::ConstPtr> &his_kf_lidar,
// std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &kf_T)
void dlio::OdomNode::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum,  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                        pcl::PointCloud<PointType>::ConstPtr>>& keyframes){
  // extract near keyframes
    // 提取key索引的关键帧前后相邻若干帧的关键帧的点集合
    std::vector<int> index;
    nearKeyframes->clear();
    int cloudSize = this->copy_cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i) // 通过-searchNum 到 +searchNum。(-15,15) 搜索Key两侧内容
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize) // 判断 帧范围
          continue;
        // index.push_back(keyNear);
        //int select_loop_index = (loop_index != -1) ? loop_index : key + i;
        pcl::PointCloud<PointType>::Ptr temp (new pcl::PointCloud<PointType>());
        // pcl::transformPointCloud(*his_kf_lidar[keyNear], *temp, kf_T[keyNear]);
        
        *nearKeyframes += *keyframes[keyNear].second;
    }
    if (nearKeyframes->empty())
      return;
    // if(index.size() > 5)
    // { 
    //   std::shared_ptr<fast_gicp::CovariancesList> loop_normals_v (std::make_shared<fast_gicp::CovariancesList>());
    //   for(auto& k:index)
    //   {
    //     loop_normals_v->insert( std::end(*loop_normals_v),
    //       std::begin((this->loop_v[k])), std::end((this->loop_v[k])));
    //   }
    //   this->loop_local_map = loop_normals_v;
    // }

    pcl::PointCloud<PointType>::Ptr cloud_temp = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());;
    // cloud_temp.reset(new pcl::PointCloud<PointType>());
    if (nearKeyframes->size() > 60000)
    {
      this->downsampleloop.setInputCloud(nearKeyframes);
      this->downsampleloop.filter(*cloud_temp);
      *nearKeyframes = *cloud_temp;
    }
    
}

// 发布回环匹配的关键帧图
sensor_msgs::PointCloud2 dlio::OdomNode::publishCloud(const ros::Publisher& thisPub, const pcl::PointCloud<PointType>::Ptr & thisCloud,ros::Time thisStamp, std::string thisFrame){
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;;
    tempCloud.header.frame_id = thisFrame;
    // 如果发布者 thisPub 的订阅者数量不为零，则通过发布者发布tempCloud。
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

bool dlio::OdomNode::isKeyframe(){
  // get closest pose and corresponding rotation
  Eigen::Vector3f closest_pose = this->keyframes.back().first.first;
  Eigen::Quaternionf closest_pose_r = this->keyframes.back().first.second;

  // calculate distance between current pose and closest pose from above
  float dd = sqrt( pow(this->lidarPose.p[0] - closest_pose[0], 2) +
                   pow(this->lidarPose.p[1] - closest_pose[1], 2) +
                   pow(this->lidarPose.p[2] - closest_pose[2], 2) );

  // calculate difference in orientation using SLERP
  Eigen::Quaternionf dq;

  if (this->state.q.dot(closest_pose_r) < 0.) {
    Eigen::Quaternionf lq = closest_pose_r;
    lq.w() *= -1.; lq.x() *= -1.; lq.y() *= -1.; lq.z() *= -1.;
    dq = this->state.q * lq.inverse();
  } else {
    dq = this->state.q * closest_pose_r.inverse();
  }

  double theta_rad = 2. * atan2(sqrt( pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2) ), dq.w());
  double theta_deg = theta_rad * (180.0/M_PI);

  // update keyframes
  bool newKeyframe = false;
  if (abs(dd) >= this->keyframe_thresh_dist_ || abs(theta_deg) >= this->keyframe_thresh_rot_) {
    newKeyframe = true;
  }

  if (abs(dd) <this->keyframe_thresh_dist_) {
    newKeyframe = false;
  }

  if (abs(dd) < this->keyframe_thresh_dist_ && abs(theta_deg) > this->keyframe_thresh_rot_)
  {
    newKeyframe = true;
  }

  if(newKeyframe)
  {    
      this->updateCurrentInfo();
  }
  this->iskf=newKeyframe;
  return newKeyframe;
    
}

void dlio::OdomNode::updateCurrentInfo(){
    std::unique_lock<decltype(this->keyframes_mutex)> lock_kf(this->keyframes_mutex);
      // auto tree = boost::make_shared<pcl::octree::OctreePointCloudSearch<PointType>>(this->jaccard_corr_thresh_);
      // this->kf_tree.push_back(tree);
      this->keyframes.push_back(std::make_pair(std::make_pair(this->lidarPose.p, this->lidarPose.q), this->current_scan));
      this->keyframesInfo.push_back(this->current_scan);
      this->keyframe_timestamps.push_back(this->scan_header_stamp);
      this->keyframe_normals_v.push_back(this->gicp.getSourceCovariances());
      this->keyframe_transformations_prior.push_back(this->T_prior);
      this->keyframe_transformations.push_back(this->T_corr);
      this->keyframe_stateT.push_back(this->T);
      this->v_kf_time.push_back(this->scan_stamp);
      lock_kf.unlock();
}

void dlio::OdomNode::addOdomFactor(){
  static int num_factor =  0;
  
  if (num_factor == 0)
  {
      printf("第一帧 ! ! ! !\n");
      gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(
              (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
      this->gtSAMgraph.addPrior(0, this->state2Pose3(this->lidarPose.q, this->lidarPose.p), priorNoise);
      this->initialEstimate.insert(0, this->state2Pose3(this->lidarPose.q, this->lidarPose.p));
      // this->initial_cache.insert(0, this->state2Pose3(this->lidarPose.q, this->lidarPose.p));
      this->noise_vector.push_back(std::make_pair(std::make_pair(0,0),priorNoise));
  }
  else{
    std::cout << "########## Curid & Lastid: " << this->keyframes.size() - 1<<", " << this->cloudKeyPoses6D->size() - 1 << std::endl;
    // std::unique_lock<decltype(this->kf_sim_mutex)> sim_lock(this->kf_sim_mutex);
    // if (this->kf_sim_buffer.empty()) {
    // sim_lock.unlock();
    //   return;
    // }
    // auto kfs = this->kf_sim_buffer[0];
    // this->kf_sim_buffer.clear();
    // sim_lock.unlock();

    // if (kfs.size() >=2){kfs.erase(kfs.begin(),kfs.end() -2);}
    // ROS_WARN(">>> %d",kfs.size());
    gtsam::Pose3 poseTo = this->state2Pose3(this->lidarPose.q,this->lidarPose.p);
    // auto max_it = std::max_element(kfs.begin(), kfs.end(), [](const auto& a, const auto& b) { return a.similarity < b.similarity; });
    // auto max_sim = max_it->similarity;
    // 计算submap_kf噪声权重
    float noiseScore = this->icpScore;
    // auto Vector6 = (gtsam::Vector(6) << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore).finished(); 
    auto Vector6 = (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished();
    // for (const auto& [kf_idx, similarity]: kfs)
    // for(size_t kf_idx = this->cloudKeyPoses6D->size() - 1; kf_idx < this->cloudKeyPoses6D->size(); ++kf_idx)
    
        std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
        int kf_idx = this->cloudKeyPoses6D->size() - 1;
        // PointTypePose point = this->cloudKeyPoses6D->points[kf_idx];
        gtsam::Pose3 poseFrom = this->state2Pose3(this->keyframes[kf_idx].first.second,this->keyframes[kf_idx].first.first);
        lock.unlock();
        // gtsam::Pose3 poseFrom = this->pclPointTogtsamPose3(point);
        float weight = this->icpScore;
        // if (similarity > 0) {
        //   float normalized_sim = similarity / max_sim;
        //   weight = normalized_sim; // / gicp_score; 
        // }
        gtsam::Pose3  betweenFactor = poseFrom.between(poseTo);
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6*weight); 
        this->pose3_vector.push_back(betweenFactor);
        this->noise_vector.push_back(std::make_pair(std::make_pair(kf_idx,num_factor),odometryNoise));
         this->gtSAMgraph.emplace_shared<BetweenFactor<Pose3>>(kf_idx,num_factor, betweenFactor, odometryNoise);
    
    this->initialEstimate.insert(num_factor, poseTo);
    // this->initial_cache.insert(num_factor, poseTo);
    // ROS_WARN("******************里程计因子加入****************");
    }
    ++num_factor;
    this->addodom_num++;
}

void dlio::OdomNode::addLoopFactor(){    
    if (this->loopIndexQueue.empty())
        return;
    std::unique_lock<decltype(this->loop_factor_mutex)> lock(this->loop_factor_mutex);
    for (int i = 0; i < (int)this->loopIndexQueue.size(); ++i)
    {
       
        int indexFrom = this->loopIndexQueue[i].first;
        int indexTo = this->loopIndexQueue[i].second;
        
        gtsam::Pose3 poseBetween = this->loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = this->loopNoiseQueue[i];
        this->pose3_vector.push_back(poseBetween);
         this->noise_vector.push_back(std::make_pair(std::make_pair(indexFrom,indexTo),noiseBetween));
        this->gtSAMgraph.emplace_shared<BetweenFactor<Pose3>>(indexFrom, indexTo, poseBetween, noiseBetween);
        ROS_WARN("******************回环因子加入****************,curid & kfid: %d,%d",indexFrom,indexTo);
      
    }
    this->loopIndexQueue.clear();
    this->loopPoseQueue.clear();
    this->loopNoiseQueue.clear();
    lock.unlock();

    this->isLoop = true;
    this->addloop_num++;
    
}

void dlio::OdomNode::addIMUFctor(){}
void dlio::OdomNode::addGravityFactor(){}

void dlio::OdomNode::visualizeLoopClosure()
{
  if (loopIndexContainer.empty())
    return;

      visualization_msgs::MarkerArray markerArray;
      // loop nodes
      visualization_msgs::Marker markerNode;
      markerNode.header.frame_id = this->odom_frame;
      markerNode.header.stamp = this->imu_stamp;
      markerNode.action = visualization_msgs::Marker::ADD;
      markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
      markerNode.ns = "loop_nodes";
      markerNode.id = 0;
      markerNode.pose.orientation.w = 1;
      markerNode.scale.x = 0.3;
      markerNode.scale.y = 0.3;
      markerNode.scale.z = 0.3;
      markerNode.color.r = 0;
      markerNode.color.g = 0.8;
      markerNode.color.b = 1;
      markerNode.color.a = 1;
      // loop edges
      visualization_msgs::Marker markerEdge;
      markerEdge.header.frame_id = this->odom_frame;
      markerEdge.header.stamp = this->imu_stamp;
      markerEdge.action = visualization_msgs::Marker::ADD;
      markerEdge.type = visualization_msgs::Marker::LINE_LIST;
      markerEdge.ns = "loop_edges";
      markerEdge.id = 1;
      markerEdge.pose.orientation.w = 1;
      markerEdge.scale.x = 0.15;
      markerEdge.color.r = 1.0;
      markerEdge.color.g = 0.5;
      markerEdge.color.b = 1.0;
      markerEdge.color.a = 1;
      // this->loop_factor_mutex.lock();
      for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
      {
          int key_cur = it->first;
          int key_pre = it->second;
          geometry_msgs::Point p;
          p.x = this->copy_cloudKeyPoses6D->points[key_cur].x;
          p.y = this->copy_cloudKeyPoses6D->points[key_cur].y;
          p.z = this->copy_cloudKeyPoses6D->points[key_cur].z;
          markerNode.points.push_back(p);
          markerEdge.points.push_back(p);
          p.x = this->copy_cloudKeyPoses6D->points[key_pre].x;
          p.y = this->copy_cloudKeyPoses6D->points[key_pre].y;
          p.z = this->copy_cloudKeyPoses6D->points[key_pre].z;
          markerNode.points.push_back(p);
          markerEdge.points.push_back(p);
      }
      // this->loop_factor_mutex.unlock();

      markerArray.markers.push_back(markerNode);
      markerArray.markers.push_back(markerEdge);
      this->loop_constraint_pub.publish(markerArray);
    
}

void dlio::OdomNode::meetloopAndcorrect()
{
  // 更新历史关键帧位置
    if (this->isLoop == true)
    {       
        std::unique_lock<decltype(this->keyframes_mutex)> lock_kf(this->keyframes_mutex);
#pragma omp parallel for num_threads(omp_get_max_threads()) 
        for (int i = 0; i < this->iSAMCurrentEstimate.size(); i++)
        {
              this->keyframes[i].first.first = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).translation().cast<float>();
              this->keyframes[i].first.second = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).rotation().toQuaternion().cast<float>().normalized();
              
              
              // this->cloudKeyPoses6D->points[i].x = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
              // this->cloudKeyPoses6D->points[i].y = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
              // this->cloudKeyPoses6D->points[i].z = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).translation().z();
              // this->cloudKeyPoses6D->points[i].roll = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
              // this->cloudKeyPoses6D->points[i].pitch = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
              // this->cloudKeyPoses6D->points[i].yaw = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

              Eigen::Affine3f T_g = Eigen::Affine3f::Identity();
              T_g.translate(this->keyframes[i].first.first);
              T_g.rotate(this->keyframes[i].first.second);
              auto temT = T_g.matrix() * this->keyframe_transformations_prior[i].inverse();
              this->keyframe_transformations[i] = temT.matrix();

              pcl::PointCloud<PointType>::Ptr cloud_(boost::make_shared<pcl::PointCloud<PointType>>());
              pcl::transformPointCloud(*this->keyframesInfo[i],*cloud_,temT.matrix());
              this->keyframes[i].second = cloud_; 

              // #pragma omp critical
              
           
              
        } 
        lock_kf.unlock();   
      this->isLoop = false;
      this->cur_kf = false;

      //更新this->T
      Eigen::Affine3f tempT;
      tempT.translate(this->lidarPose.p);
      tempT.rotate(this->lidarPose.q);
      this->T = tempT.matrix();
      this->keyframe_stateT.back() = this->T;
  }
  
}


void dlio::OdomNode::GTSAMUpdateFactor(){
    if (this->isKeyframe())
    { 
            std::cout << "\033[32m===============================================================\033[0m" << std::endl;
            std::cout << "*****Before opt_pose*****:\n>>> t: [" << this->lidarPose.p(0,3) <<" "<< this->lidarPose.p(1,3)<<" "<< this->lidarPose.p(2,3) <<"]\n";
            std::cout << ">>> q: [" << this->lidarPose.q.w()<<" "<<this->lidarPose.q.x() << " "<<this->lidarPose.q.y() <<" "<<this->lidarPose.q.z()<<"]\n";
            // 添加里程计因子
            this->addOdomFactor();
              // 添加闭环因子
            this->addLoopFactor();
            // 重力因子
            this->addGravityFactor();
            
            // 添加GPS因子
            //this->addGPSFactorWithoutAlign();

            this->isam->update(this->gtSAMgraph, this->initialEstimate);
            this->isam->update();

            if (this->isLoop == true)
            {
                this->isam->update();
                this->isam->update();
                this->isam->update();
                this->isam->update();
                this->isam->update();
            }

            this->gtSAMgraph.resize(0);
            this->initialEstimate.clear();
            this->iSAMCurrentEstimate = this->isam->calculateEstimate();
            // this->poseCovariance = this->isam->marginalCovariance(this->iSAMCurrentEstimate.size() - 1);//更新位姿协方差
            int n_ = this->iSAMCurrentEstimate.size() - 1;
            this->lidarPose.p = this->iSAMCurrentEstimate.at<gtsam::Pose3>(n_).translation().cast<float>();
            this->lidarPose.q = this->iSAMCurrentEstimate.at<gtsam::Pose3>(n_).rotation().toQuaternion().cast<float>();

            PointTypePose pt_rpy;
            pt_rpy.x = this->lidarPose.p[0];
            pt_rpy.y = this->lidarPose.p[1];
            pt_rpy.z = this->lidarPose.p[2];
            pt_rpy.intensity = this->cloudKeyPoses6D->size();
            pt_rpy.roll = this->iSAMCurrentEstimate.at<gtsam::Pose3>(n_).rotation().roll();
            pt_rpy.pitch = this->iSAMCurrentEstimate.at<gtsam::Pose3>(n_).rotation().pitch();
            pt_rpy.yaw = this->iSAMCurrentEstimate.at<gtsam::Pose3>(n_).rotation().yaw(); 
            pt_rpy.time = this->scan_header_stamp.toSec();
            this->cloudKeyPoses6D->push_back(pt_rpy);
            if(this->isLoop)
            {
                  std::cout <<"$$$$$$$$ curid & isamid: " << this->cloudKeyPoses6D->size() - 1 << ", " << this->iSAMCurrentEstimate.size() - 1<< std::endl;
                  std::cout << "*****After opt_pose *****:\n>>> t: [" << this->lidarPose.p(0,3) <<" "<< this->lidarPose.p(1,3)<<" "<< this->lidarPose.p(2,3) <<"]\n";
                  std::cout << ">>> q: [" << this->lidarPose.q.w()<<" "<<this->lidarPose.q.x() << " "<<this->lidarPose.q.y() <<" "<<this->lidarPose.q.z()<<"]\033[32m ########\033[0m\n";
                  std::cout << "\033[32m==============================================================\033[0m" << std::endl;
            }
            this->meetloopAndcorrect();
    }  
    return;
  }

void dlio::OdomNode::FirstGTSAMUpdateFactor(){
      this->addOdomFactor();

      this->isam->update(this->gtSAMgraph, this->initialEstimate);
      this->isam->update();

      this->gtSAMgraph.resize(0);
      this->initialEstimate.clear();
      this->iSAMCurrentEstimate = this->isam->calculateEstimate();
      // this->poseCovariance = this->isam->marginalCovariance(iSAMCurrentEstimate.size() - 1);//更新位姿协方

      int i = this->iSAMCurrentEstimate.size() - 1;
      this->lidarPose.p = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).translation().cast<float>();
      this->lidarPose.q = this->iSAMCurrentEstimate.at<gtsam::Pose3>(i).rotation().toQuaternion().cast<float>();


}




void dlio::OdomNode::updatemap()
{
  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  auto kfs = this->keyframes;
  // auto kf_cor = this->keyframe_transformations;
  lock.unlock();
  // ROS_WARN("!!!!!!!!!! %d",kfs.size());

  pcl::PointCloud<PointType>::Ptr globalkf_cloud (new pcl::PointCloud<PointType>());
  // pcl::PointCloud<PointType>::Ptr tempkf_cloud (new pcl::PointCloud<PointType>());
  for (size_t i = 0; i < kfs.size(); i++)
  {
    // pcl::transformPointCloud(*kfs[i],*tempkf_cloud, kf_cor[i].matrix());
    *globalkf_cloud += *kfs[i].second;
  }

  sensor_msgs::PointCloud2 global_ros;
  pcl::toROSMsg(*globalkf_cloud, global_ros);
  global_ros.header.stamp = ros::Time::now();
  global_ros.header.frame_id = this->odom_frame;
  this->global_map_pub.publish(global_ros);
}

void dlio::OdomNode::saveTimefile(std::vector<float>& data,const std::string& path)
{
  std::ofstream outfile(path);
  if (!outfile.is_open()) {
      std::cerr << "Error opening the file." << std::endl;
      return;
  }

  for (const double& time : data) {
      outfile << time << std::endl;
  }
  outfile.close();

  std::cout << "Time file saved to " << path << std::endl;
}