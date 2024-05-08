1.将VGICP与GICP内部集成在一起
idea1: 根据每帧法向量的分布特性判定是否为退化环境。

// void dlio::OdomNode::getEulerAngles(boost::circular_buffer<ImuMeas> &imu_buffers, float &Rosroll,flosetvoxel60at &Rospitch, float  &Rosyaw,double dt)
// {
//   auto imu_meas = imu_buffers.front();
//       _Float32 roll;    // 横滚角
//     _Float32 pitch;   // 俯仰角
//     _Float32 yaw;     // 偏航角
//     _Float32 rollAcc; // 加速度计计算的横滚角
//     _Float32 pitchAcc; // 加速度计计算的俯仰角
//     //imu_meas: 经过采样后矫正的imu数据； 角速度积分得到姿态角变化量
//     roll+= imu_meas.ang_vel[0] * dt;
//     pitch += imu_meas.ang_vel[1] * dt;
//     yaw += imu_meas.ang_vel[2] * dt;

//     // 加速度计算得到姿态角
//     rollAcc = std::atan2(imu_meas.lin_accel[1], imu_meas.lin_accel[2]);
//     pitchAcc = std::atan2(-imu_meas.ang_vel[0], std::sqrt(imu_meas.lin_accel[1] * imu_meas.lin_accel[1] 
//                                                                         + imu_meas.lin_accel[2] *imu_meas.lin_accel[2]));

//     // 融合加速度计和陀螺仪的姿态角
//     Rosroll =  0.98 * roll + 0.02 * rollAcc;
//     Rospitch = 0.98 * pitch + 0.02 * pitchAcc;
//     Rosyaw = yaw;
// }

/* 
  @brief  根据“空间度 ”和“空间密度”确定下采样的体素大小
   @param d: 空间度：统计每个点的距离
*/
// float dlio::OdomNode::setVoxel(float& d, float& den, bool& scale)
// {

//   //  ROS_WARN("%f, %f",d, den);
//   if (scale )
//   {
//     return 0.4;
//   }
//   if(!scale)
//   { 
//      return 0.2; 
//   }else {
//      return 0.3;
//   }
  
// }

// void dlio::OdomNode::adaptiveVoxelFilter(pcl::PointCloud<PointType>::ConstPtr &cloud_, float& v)
// {
//   pcl::PointCloud<PointType>::Ptr temp_ (new pcl::PointCloud<PointType> ());
//   this->voxel_2.setLeafSize(v,v,v);
//   this->voxel_2.setInputCloud(cloud_);
//   this->voxel_2.filter(*temp_);
//   this->current_scan = temp_;
// }

// void dlio::OdomNode::AdaptiveVoxelFilter_mean( pcl::PointCloud<PointType>::ConstPtr &pcl_in, pcl::PointCloud<PointType>::Ptr & pcl_out)
// {
//     // pcl_out->clear();
//     int valid_num = 0;
//     // 当前点和上一点的间隔足够大
//     for (size_t i = 0; i < pcl_in->size(); ++i)
//     {
//         valid_num++;
//         // downsampleRate
//         if (valid_num %  this->downsamplerate == 0){
         
//             if(((abs(pcl_in->points[i].x - pcl_in->points[i-1].x) > 1e-7) 
//                 || (abs(pcl_in->points[i].y - pcl_in->points[i-1].y) > 1e-7)
//                 || (abs(pcl_in->points[i].z - pcl_in->points[i-1].z) > 1e-7))
//                 && (pcl_in->points[i].x * pcl_in->points[i].x + pcl_in->points[i].y * pcl_in->points[i].y + pcl_in->points[i].z * pcl_in->points[i].z > (0.01 * 0.01))) {
//             pcl_out->push_back(pcl_in->points[i]);
            
//             }   
//         }
//         else
//             continue;
        
//     }
    
//     this->adaptiveVoxelFilter_size = pcl_out->size();
// }


// pcl::PointCloud<PointType>::Ptr dlio::OdomNode::transformPointCloud(pcl::PointCloud<PointType>::ConstPtr cloudIn, PointTypePose *transformIn)
// {
//         pcl::PointCloud<PointType>::Ptr cloudOut = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

//         int cloudsize = cloudIn->size();
//         cloudOut->resize(cloudsize);

//         Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

// #pragma omp parallel for num_threads(12) //8
//         for (int i = 0; i < cloudsize; ++i)
//         {
//             const auto &pointFrom = cloudIn->points[i];
//             cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
//             cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
//             cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
//             cloudOut->points[i].intensity = pointFrom.intensity;
//             if (this->sensor == dlio::SensorType::OUSTER) {
//               cloudOut->points[i].t = pointFrom.t;
//               continue;
//             } else if (this->sensor == dlio::SensorType::VELODYNE) {
//               cloudOut->points[i].time = pointFrom.time;
//               continue;
//             } else if (this->sensor == dlio::SensorType::LIVOX) {
//               cloudOut->points[i].offset_time = pointFrom.offset_time;
//               continue;
//             } else if (this->sensor == dlio::SensorType::HESAI) {
//               cloudOut->points[i].timestamp = pointFrom.timestamp;
//               continue;
//             }
//         }
//         return cloudOut;
// }

// float dlio::OdomNode::constraintTransformation(float value, float limit)
// {
//   if (value < -limit)
//       value = -limit;
//   if (value > limit)
//       value = limit;

//   return value;
// }

// void dlio::OdomNode::updateState_kf() {

//   // Lock thread to prevent state from being accessed by PropagateState 与imu数据呼应
//   std::lock_guard<std::mutex> lock( this->geo.mtx );

//   Eigen::Vector3f pin = this->lidarPose.p;
//   Eigen::Quaternionf qin = this->lidarPose.q;
//   double dt = this->scan_stamp - this->prev_scan_stamp;

//   Eigen::Quaternionf qe, qhat, qcorr;
//   qhat = this->state.q;

//   // Constuct error quaternion 共轭
//   qe = qhat.conjugate()*qin;

//   double sgn = 1.;
//   if (qe.w() < 0) {
//     sgn = -1;
//   }

//   // Construct quaternion correction 旋转校正
//   qcorr.w() = 1 - abs(qe.w());
//   qcorr.vec() = sgn*qe.vec();
//   qcorr = qhat * qcorr;

//   Eigen::Vector3f err = pin - this->state.p;
//   Eigen::Vector3f err_body;

//   err_body = qhat.conjugate()._transformVector(err);

//   double abias_max = this->geo_abias_max_;
//   double gbias_max = this->geo_gbias_max_;

//   // Update accel bias
//   this->state.b.accel -= dt * this->geo_Kab_ * err_body;
//   this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);
//   // Update gyro bias
//   this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
//   this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
//   this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
//   this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);

//   this->state.p = this->lidarPose.p;
//   this->state.v.lin.w += dt * this->geo_Kv_ * err;
//   this->state.q = this->lidarPose.q;
//   // this->state.p += dt * this->geo_Kp_ * err;
//   // this->state.v.lin.w += dt * this->geo_Kv_ * err;
//   // this->state.q.w() += dt * this->geo_Kq_ * qcorr.w();
//   // this->state.q.x() += dt * this->geo_Kq_ * qcorr.x();
//   // this->state.q.y() += dt * this->geo_Kq_ * qcorr.y();
//   // this->state.q.z() += dt * this->geo_Kq_ * qcorr.z();
//   // this->state.q.normalize();

//   // store previous pose, orientation, and velocity
//   this->geo.prev_p = this->state.p;
//   this->geo.prev_q = this->state.q;
//   this->geo.prev_vel = this->state.v.lin.w;


// }