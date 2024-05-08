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
#include "dlio/dlio.h"

// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/lago.h>
#include <gtsam/slam/dataset.h>

#include <nano_gicp/nanoflann_adaptor.h>

// GeographicLib
// #include <GeographicLib/Geocentric.hpp>
// #include <GeographicLib/LocalCartesian.hpp>
// #include <GeographicLib/Geoid.hpp>
// #include <GeographicLib/GeoCoords.hpp>

#include<fvgicp/dwvgicp.hpp>
#include "ikd-Tree/ikd_Tree.h"
class dlio::OdomNode {

public:

  OdomNode(ros::NodeHandle node_handle);
  ~OdomNode();

  void start();
  int count;
 
  void FirstGTSAMUpdateFactor();
  void GTSAMUpdateFactor();
  void updateCurrentInfo();
  bool isKeyframe();
  void addIMUFctor();
  void addOdomFactor();
  void addGravityFactor();
  void correctPoses();
  
  // void addGPSFactor();
  // void addGPSFactorWithoutAlign();

  // LOOP
  void performLoopThread();
  void performLoopClosure();
  void visualizeLoopClosure();
  void addLoopFactor();
  bool detectLoopClosureDistance(int *latestID, int *closestID);
  // void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum,std::vector<pcl::PointCloud<PointType>::ConstPtr> & his_kf_lidar, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &kf_T);
   void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum,  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                        pcl::PointCloud<PointType>::ConstPtr>>& keyframes);
  sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const pcl::PointCloud<PointType>::Ptr &thisCloud, ros::Time thisStamp, std::string thisFrame);
  void meetloopAndcorrect();

 // 输出轨迹和地图文件
  std::vector<double> frametimes;
  bool iskf;
  bool cur_kf;
  bool savefile;
  std::vector<float> time_odom;
  std::vector<float> time_built_submap;
  std::vector<float> time_total_time;
  void saveTimefile(std::vector<float>& data,const std::string& path);
  void updatemap();

private:

  // optimization && loop correct
  //位姿致信度
  float icpScore;
  std::fstream f;
  Eigen::Isometry3f keyframe_pose_corr;
  visualization_msgs::Marker loop_marker;
  bool kf_update;
  bool isLoop;
  std::vector<std::pair<int, int>> history_loop_id;
  pcl::PointCloud<PointType>::Ptr global_map;

  std::mutex global_map_update_mutex;
  std::condition_variable global_map_update_cv;
  bool global_map_update_finish = true;


  std::mutex update_map_info_mutex;
  std::queue<std::pair<bool, gtsam::Values>> update_map_info;

 

  struct State;
  struct ImuMeas;
  struct Similarity;


  // GPS
  // Eigen::Matrix3d R_M_G = Eigen::Matrix3d::Identity();
  // Eigen::Vector3d t_M_G = Eigen::Vector3d::Zero();
  // GeographicLib::LocalCartesian geo_converter;


  // struct GPSMeas
  // {
  //     GPSMeas() { mathced_id = -1; };
  //     GPSMeas(double x_, double y_, double z_, double time_) : x(x_), y(y_), z(z_), time(time_) { mathced_id = -1; };
  //     double x;
  //     double y;
  //     double z;
  //     double time;
  //     Eigen::Matrix3d cov;
  //     int mathced_id;
  // };

  // std::vector<GPSMeas> v_gps_meas;
  // std::vector<GPSMeas> v_gps_init;
  // std::vector<GPSMeas> v_gps_state;

  // std::mutex gps_buffer_mutex;
  // std::vector<GPSMeas> gps_buffer;

  // std::unordered_set<int> gps_node_id;
  // std::mutex val_gps_mutex;
  // std::vector<GPSMeas> v_val_gps;

  // std::mutex gps_mutex;
  // GPSMeas gps_meas;
  // GPSMeas last_gps_meas;
  // bool gps_init = false;

  // DLIO
  void getParams();

  void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
  void callbackImu(const sensor_msgs::Imu::ConstPtr& imu);
  void callbackLivox(const livox_ros_driver2::CustomMsgConstPtr& livox);
  // void callbackGPS(const sensor_msgs::NavSatFixConstPtr& gps);
  // void callbackGPSWithoutAlign(const sensor_msgs::NavSatFixConstPtr& gps);
  // bool matchGPSWithKf(GPSMeas& gps);



  void publishPose(const ros::TimerEvent& e);

  void publishToROS(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud);
  void publishCloud(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud);
  void publishKeyframe(std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                       pcl::PointCloud<PointType>::ConstPtr> kf, ros::Time timestamp);

  void getScanFromROS(const sensor_msgs::PointCloud2ConstPtr& pc);
  void preprocessPoints();
  void deskewPointcloud();
  void initializeInputTarget();
  void setInputSource();

  void initializeDLIO();

  void getNextPose();
  bool imuMeasFromTimeRange(double start_time, double end_time,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                 const std::vector<double>& sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                         const std::vector<double>& sorted_timestamps,
                         boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                         boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
  void getEulerAngles(boost::circular_buffer<ImuMeas> &imu_buffers, float &Rosroll,float &Rospitch, float &Rosyaw,double dt);
  float constraintTransformation(float value, float limit);
  void propagateGICP();

  void propagateState();
  void updateState();
  void updateState_kf();

  void setAdaptiveParams();

 // keyframe threshold
  void computeMetrics();
  void computeSpaciousness();
  void computeDensity();

  // void computeJaccard();
  sensor_msgs::Imu::Ptr transformImu(const sensor_msgs::Imu::ConstPtr& imu);

  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
  void buildSubmap(State vehicle_state);
  void visualizeSubmap(State vehicle_state);

  void buildSubmapViaJaccard(State vehicle_state,pcl::PointCloud<PointType>::ConstPtr cloud);

  void buildKeyframesAndSubmap(State vehicle_state);
  void pauseSubmapBuildIfNeeded();

  // tool func
  gtsam::Pose3 state2Pose3(Eigen::Quaternionf rot, Eigen::Vector3f pos);
  Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
  gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

  void debug();

  ros::NodeHandle nh;
  ros::Timer publish_timer;

  // Subscribers
  ros::Subscriber lidar_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber gps_sub;
  ros::Subscriber livox_sub;

  // Publishers
  ros::Publisher livox_pub;
  ros::Publisher odom_pub;
  ros::Publisher pose_pub;
  ros::Publisher path_pub;
  ros::Publisher kf_pose_pub;
  ros::Publisher kf_cloud_pub;
  ros::Publisher deskewed_pub;
  ros::Publisher kf_connect_pub;
  ros::Publisher convex_connect_pub;
  ros::Publisher global_map_pub;
  ros::Publisher global_pose_pub;
  ros::Publisher loop_constraint_pub;

  ros::Publisher gps_pub_test;

  ros::Publisher kf_pose_test;
  ros::Publisher pubHistoryKeyFrames;
  ros::Publisher publoopmap;

  //debug time cost
  ros::Publisher odom_time_pub;
  ros::Publisher submap_time_pub;
  ros::Publisher each_frame_time_pub;

  //test octomap 
  ros::Publisher octomap_cloud;
  ros::Publisher octomap_tf;

  // ROS Msgs
  nav_msgs::Odometry odom_ros;
  geometry_msgs::PoseStamped pose_ros;
  nav_msgs::Path path_ros;
  geometry_msgs::PoseArray kf_pose_ros;
  geometry_msgs::PoseArray global_pose;

  bool global_dense;
  // Flags
  std::atomic<bool> dlio_initialized;
  std::atomic<bool> first_valid_scan;
  std::atomic<bool> first_imu_received;
  std::atomic<bool> imu_calibrated;
  std::atomic<bool> submap_hasChanged;
  std::atomic<bool> gicp_hasConverged;
  std::atomic<bool> deskew_status;
  std::atomic<int> deskew_size;
  std::atomic<int> adaptiveVoxelFilter_size;
  bool gyronormlized;
  bool time_offset_;

  // Threads
  std::thread publish_thread;
  std::thread publish_keyframe_thread;
  std::thread metrics_thread;
  std::thread debug_thread;
  // std::thread mapping_thread;
  std::thread loop_thread;
  // std::thread updatemap_thread;


  // Trajectory
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  double length_traversed;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> kf_trajectory;

  std::vector<double> v_kf_time;

  std::vector<pcl::PointCloud<PointType>::ConstPtr> keyframesInfo;
  
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                        pcl::PointCloud<PointType>::ConstPtr>> keyframes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr kf_cloud;
  std::vector<ros::Time> keyframe_timestamps;
  // std::vector<std::shared_ptr<const nano_gicp::CovarianceList>> keyframe_normals;
   std::vector<std::shared_ptr<const fast_gicp::CovarianceList>> keyframe_normals_v;
  // std::vector<fast_gicp::CovarianceList> temp_loop_v;
  // std::vector<fast_gicp::CovarianceList> loop_v;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_stateT;
  std::vector<Eigen::Matrix4f> kf_sT;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations_prior;

  std::vector<double> time_2f;

  std::mutex keyframes_mutex;
  std::mutex kf_sim_mutex;

  // Sensor Type
  dlio::SensorType sensor;

  // Frames
  std::string odom_frame;
  std::string baselink_frame;
  std::string lidar_frame;
  std::string imu_frame;

  // Preprocessing
  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> voxel;
  pcl::VoxelGrid<PointType> voxel_2;
  pcl::VoxelGrid<PointType> voxel_global;
  pcl::VoxelGrid<PointType> downsampleloop;

  // Point Clouds
  pcl::PointCloud<PointType>::ConstPtr original_scan;       // 原始点云
  pcl::PointCloud<PointType>::ConstPtr deskewed_scan;       // 去畸变后的点云
  pcl::PointCloud<PointType>::ConstPtr current_scan;        // 去畸变且降采样后的点云
  pcl::PointCloud<PointType>::Ptr current_scan_w;        // 去畸变且降采样后的点云


    // Keyframes
  pcl::PointCloud<PointType>::ConstPtr keyframe_cloud;
  int num_processed_keyframes;

  pcl::ConvexHull<PointType> convex_hull;
  pcl::ConcaveHull<PointType> concave_hull;
  std::vector<int> keyframe_convex;
  std::vector<int> keyframe_concave;

  // Submap
  pcl::PointCloud<PointType>::ConstPtr submap_cloud;
  // std::shared_ptr<const nano_gicp::CovarianceList> submap_normals;
  std::shared_ptr<const nano_gicp::CovarianceList> submap_normals_v;
  // std::shared_ptr<const fast_gicp::CovarianceList> loop_local_map;
  // std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree;
  std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree_v;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kf_kdtree;
  
  
  struct Similarity{
      int index;
      float similarity;
  };

  boost::circular_buffer<std::vector<Similarity>> kf_sim_buffer;
  std::vector<Similarity> submap_kf_idx_curr;
  std::vector<Similarity> submap_kf_idx_prev;
    std::vector<pcl::octree::OctreePointCloudSearch<PointType>::Ptr> kf_tree;

  std::vector<int> keyframe_nn;
  std::vector<float> ds;

  

  // Loop
  pcl::PointCloud<PointType>::Ptr current_scan_lidar;
  std::vector<pcl::PointCloud<PointType>::ConstPtr> history_pointcloud_lidar;

  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;  // 存储关键帧6D位姿 信息 的点云 // 到全局坐标系的变换信息
  
  map<int, int> loopIndexContainer;
  vector<pair<int, int>> loopIndexQueue;
  vector<gtsam::Pose3> loopPoseQueue;
  vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
  // pcl::KdTreeFLANN<PointTypePose>::Ptr kdtreeHistoryKeyPoses;
  std::shared_ptr<nanoflann::KdTreeFLANN<PointTypePose>> kdtreeHistoryKeyPoses;

  bool loopflag;
  float loop_score_;
  int loop_search_R;
  int loop_search_Num;
  int loop_frequeency;
  int downsamplerate;
  float loop_size;
  double submap_concave_alpha_;
  bool bigscale;
  double timelaserCur;



  std::mutex loop_info_mutex;
  struct loop_info
  {
      bool loop = false;
      bool loop_candidate = false;
      int current_id = -1;
      std::vector<int> candidate_key;
      std::vector<float> candidate_dis;
      std::vector<float> candidate_sim;
      std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
              pcl::PointCloud<PointType>::ConstPtr>> candidate_frame;
      std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
              pcl::PointCloud<PointType>::ConstPtr> current_kf;
      std::vector<std::shared_ptr<nano_gicp::CovarianceList>> candidate_frame_normals;
      
  };


  std::mutex loop_factor_mutex;
  struct loop_factor_info
  {
      // curr target
      bool loop = false;
      // curr target
      std::pair<int, int> factor_id;
      Eigen::Isometry3f T_current;
      Eigen::Isometry3f T_target;
      float sim;
      float dis;

  };
  loop_info curr_loop_info;
  loop_factor_info curr_factor_info;



  bool new_submap_is_ready;
  std::future<void> submap_future;
  std::condition_variable submap_build_cv;  // 条件变量
  bool main_loop_running;
  std::mutex main_loop_running_mutex;

  bool useJaccard;
  // Timestamps
  ros::Time scan_header_stamp;
  double scan_stamp;
  double prev_scan_stamp;
  double scan_dt;
  std::vector<double> comp_times;
  std::vector<double> imu_rates;
  std::vector<double> lidar_rates;

  double first_scan_stamp;
  double elapsed_time;

 std::string matchMethod;
  // GICP
  nano_gicp::NanoGICP<PointType, PointType> gicp;
  nano_gicp::NanoGICP<PointType,PointType> gicp_temp;
  // nano_gicp::NanoGICP<PointType, PointType> vgicp_loop;

  //VGICP
  int neighborsearch;
  fast_gicp::FastVGICP<PointType,PointType> vgicp_loop;
  fast_gicp::FastVGICP<PointType,PointType> vgicp_odom;

  Eigen::Matrix4f  lastIncreTransformation;
  double vgicp_loop_time;
  
  pcl::StatisticalOutlierRemoval<PointType> sor;

  // Transformations
  Eigen::Matrix4f T, T_prior, T_corr;float T_fusion[6];
  Eigen::Quaternionf q_final;

  Eigen::Vector3f origin;

  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 baselink2lidar;
    Eigen::Matrix4f baselink2imu_T;
    Eigen::Matrix4f baselink2lidar_T;
  }; Extrinsics extrinsics;

  // IMU
  ros::Time imu_stamp;
  double first_imu_stamp;
  double prev_imu_stamp;
  double imu_dp, imu_dq_deg;

  struct ImuMeas {
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  }; ImuMeas imu_meas;

  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;
  float imuRollInit,imuPitchInit,imuYawInit;
  double timeinfo;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  // Geometric Observer
  struct Geo {
    bool first_opt_done;
    std::mutex mtx;
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  }; Geo geo;

  // State Vector
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b;
    Eigen::Vector3f w;
  };

  struct Velocity {
    Frames lin;
    Frames ang;
  };

  struct State {
    Eigen::Vector3f p; // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
    Velocity v;
    ImuBias b; // imu biases in body frame
  }; State state;

  

  State currentFusionState;
  Eigen::Affine3f currentFusionT;


  struct Pose {
    Eigen::Vector3f p; // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
  }lidarPose, imuPose;
  

  // Metrics
  struct Metrics {
    std::vector<float> spaciousness;
    std::vector<float> density;
  }; Metrics metrics;

  std::string cpu_type;
  std::vector<double> cpu_percents;
  clock_t lastCPU, lastSysCPU, lastUserCPU;
  int numProcessors;

  // Parameters
  std::string version_;
  int num_threads_;

  bool deskew_;

  double gravity_;
  double gravity_delete;

  bool adaptive_params_;

  // double obs_submap_thresh_;
  // double obs_keyframe_thresh_;
  // double obs_keyframe_lag_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;
  double jaccard_corr_thresh_;
  double jaccard_sim_thresh_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  float envir_thresh_;



  bool densemap_filtered_;
  bool wait_until_move_;

  double crop_size_;

  bool vf_use_;
  double vf_res_;

  bool imu_calibrate_;
  bool calibrate_gyro_;
  bool calibrate_accel_;
  bool gravity_align_;
  double imu_calib_time_;
  int imu_buffer_size_;
  Eigen::Matrix3f imu_accel_sm_;

  int gicp_min_num_points_;
  int gicp_k_correspondences_;
  double gicp_max_corr_dist_;
  int gicp_max_iter_;
  double gicp_transformation_ep_;
  double gicp_rotation_ep_;
  double gicp_init_lambda_factor_;
  double dw_dist_, dw_azimuth_, dw_elevation_;

  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double geo_abias_max_;
  double geo_gbias_max_;

  // gtasm
  gtsam::NonlinearFactorGraph gtSAMgraph;
  gtsam::Values initialEstimate;
  gtsam::Values initial_cache;
  // gtsam::Values optimizedEstimate;
  gtsam::ISAM2 *isam;
  gtsam::Values iSAMCurrentEstimate;
  Eigen::MatrixXd poseCovariance;

  int addloop_num, addodom_num, num_factor;
  State last_kf_state;
  std::vector<gtsam::Pose3> pose3_vector;
  std::vector<std::pair<std::pair<int,int>,gtsam::noiseModel::Diagonal::shared_ptr>> noise_vector;

  // //ikd-Tree
  // KD_TREE ikdtree;
  // PointVector pointToAdd;
  // std::vector<BoxPointType> cub_needrm; // ikd-tree中，地图需要移除的包围盒序列
  // std::vector<PointVector> Nearest_Points; //每个点的最近点序列
  // pcl::PointCloud<PointType>::Ptr _featsArray; // ikd-tree中，map需要移除的点云序列
  // pcl::PointCloud<PointType>::Ptr feats_down_world;// 去畸变后降采样的单帧点云，W系
  // int feats_down_size;// 记录滤波后的点云数量
  // int process_increments;

  // void submap_incremental(); //地图的增量更新，主要完成对ikd-tree的地图建立
  
 

  // std::vector<pcl::PointCloud<PointType>::Ptr> all_keyframes;

  //自适应体素滤波
  // float max_range = 40;
  // float setVoxel(float &d, float &den,bool &scale);
  // void adaptiveVoxelFilter(pcl::PointCloud<PointType>::ConstPtr &cloud_, float& v);
  // void AdaptiveVoxelFilter_mean(pcl::PointCloud<PointType>::ConstPtr  & pcl_in, pcl::PointCloud<PointType>::Ptr & pcl_out);

};
