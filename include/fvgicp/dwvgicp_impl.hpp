#ifndef FAST_GICP_FAST_VGICP_IMPL_HPP
#define PCL_NO_PRECOMPILE
#define FAST_GICP_FAST_VGICP_IMPL_HPP

#include <atomic>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>

#include "fvgicp/so3.hpp"
#include "fvgicp/dwvgicp.hpp"

namespace fast_gicp {

template <typename PointSource, typename PointTarget>
FastVGICP<PointSource, PointTarget>::FastVGICP() : FastGICP<PointSource, PointTarget>() {
  this->reg_name_ = "FastVGICP";

  voxel_resolution_ = 1.0;
  search_method_ = NeighborSearchMethod::DIRECT1; //默认是DIRECT1
  voxel_mode_ = VoxelAccumulationMode::ADDITIVE;
}

template <typename PointSource, typename PointTarget>
FastVGICP<PointSource, PointTarget>::~FastVGICP() {}

template <typename PointSource, typename PointTarget>
void FastVGICP<PointSource, PointTarget>::setResolution(double resolution) {
  voxel_resolution_ = resolution;
}

template <typename PointSource, typename PointTarget>
void FastVGICP<PointSource, PointTarget>::setNeighborSearchMethod(NeighborSearchMethod method) {
  search_method_ = method;
}

template <typename PointSource, typename PointTarget>
void FastVGICP<PointSource, PointTarget>::setVoxelAccumulationMode(VoxelAccumulationMode mode) {
  voxel_mode_ = mode;
}

// 交换源点云和目标点云
template <typename PointSource, typename PointTarget>
void FastVGICP<PointSource, PointTarget>::swapSourceAndTarget() {
  input_.swap(target_);
  source_kdtree_.swap(target_kdtree_);
  source_covs_.swap(target_covs_);
  voxelmap_.reset(); 
  voxel_correspondences_.clear();
  voxel_mahalanobis_.clear();
}


template <typename PointSource, typename PointTarget>
void FastVGICP<PointSource, PointTarget>::setInputTarget(const PointCloudTargetConstPtr& cloud) {
  if (target_ == cloud) {
    
    return;
  }
  FastGICP<PointSource, PointTarget>::setInputTarget(cloud);
  if(reuse_voxelmap_)
    voxelmap_.reset();
}



//在align()中调用,提供初值计算变换后的点云
template <typename PointSource, typename PointTarget>
void FastVGICP<PointSource, PointTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  // target_,target_kdtree_kdtree_,target_covs_没有改变时，voxelmap_不需要重建
  if (reuse_voxelmap_)
      voxelmap_.reset();
  // LsqRegistration<PointSource, PointTarget>::computeTransformation(output, guess);
  FastGICP<PointSource, PointTarget>::computeTransformation(output, guess); // FastGICP内部实现calculate_covariances()
}

/*
  @brief 更新点云体素的对应关系
*/
template <typename PointSource, typename PointTarget>
void FastVGICP<PointSource, PointTarget>::update_correspondences(const Eigen::Isometry3d& trans) {
 // 清除之前的体素对应关系
  voxel_correspondences_.clear();
  // 获取邻居偏移量
  auto offsets = neighbor_offsets(search_method_);

  // 创建用于存储对应关系的容器
  std::vector<std::vector<std::pair<int, GaussianVoxel::Ptr>>> corrs(num_threads_);
  // 遍历每个线程，平均每个线程的任务
  for (auto& c : corrs) {
    c.reserve((input_->size() * offsets.size()) / num_threads_);
  }
  
// 并行计算点云之间的对应关系
#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    // 获取输入点云中的点
    PointTarget pt;
    pt.getVector4fMap() = input_->at(i).getVector4fMap();

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    // 应用给定的位置变换
    Eigen::Vector4d transed_mean_A = trans * mean_A;

      // distance weight && range uncerity
    double dist = pt.getVector3fMap().template cast<double>().norm(); // norm()计算三维向量的范数，即向量的长度或模，也即R
    double s_x = dist * distance_variance_; // 0.002R m
    double s_y = dist * sin(azimuth_variance_ / 180 * M_PI); // 0.015° -> rad
    double s_z = dist * sin(elevation_variance_ / 180 * M_PI); // 0.015° 
    double elevation = atan2(sqrt(pt.x * pt.x + pt.y * pt.y), pt.z);
    double azimuth = atan2(pt.y, pt.x);
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(elevation, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R; // Rotation matrix
    R = yawAngle * pitchAngle;
    Eigen::Matrix3d S; // Scaling matix
    S << s_x, 0.0, 0.0,   0.0, s_y, 0.0,   0.0, 0.0, s_z;
    Eigen::Matrix3d A = R * S;
    Eigen::Matrix3d cov_r = A * A.transpose();

    cov_dw = Eigen::Matrix4d::Zero();
    cov_dw.block<3, 3>(0, 0) = cov_r;

    // 获取转换后的点的体素坐标 (整数)
    Eigen::Vector3i coord = voxelmap_->voxel_coord(transed_mean_A);
    // 依据近邻搜索的方式不同，目前有4种DIRECT1 (一对一),DIRECT(1对7)，DIRECT27, 基于CUDA加速的半径近邻搜索(在cuda版本中实现)
    for (const auto& offset : offsets) {
      // 查找体素地图(目标点云)中的体素坐标，对voxel_操作
      auto voxel = voxelmap_->lookup_voxel(coord + offset);
      if (voxel != nullptr) {
        // 按线程号累计存储输入点云点和体素坐标之间的对应关系
        corrs[omp_get_thread_num()].push_back(std::make_pair(i, voxel));
      }
    }
  }
  // 预留内存以存储体素对应关系
  voxel_correspondences_.reserve(input_->size() * offsets.size());
  // 将对应关系添加到体素对应关系容器中
  for (const auto& c : corrs) {
    voxel_correspondences_.insert(voxel_correspondences_.end(), c.begin(), c.end());
  }
  // 并行计算体素之间的马氏距离
voxel_mahalanobis_.resize(voxel_correspondences_.size());
#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    const auto& cov_A = source_covs_[corr.first];//取出源点云协方差矩阵
    const auto& cov_B = corr.second->cov;// 取出目标点云协方差矩阵

    // 计算组合协方差矩阵，[C (j,B)  + TC(i ,A)T.transpose()]
    Eigen::Matrix4d RCR = (cov_B + cov_dw) + trans.matrix() * (cov_A + cov_dw) * trans.matrix().transpose();
    RCR(3, 3) = 1.0;// 齐次化

    voxel_mahalanobis_[i] = RCR.inverse();
    voxel_mahalanobis_[i](3, 3) = 0.0;
  }
}

// 线性化操作 ;
template <typename PointSource, typename PointTarget>
double FastVGICP<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  // target_改变后，初始化体素地图
  if (reuse_voxelmap_ ||  voxelmap_ == nullptr) {
    // std::cout << source_covs_.size() << std::endl;
    // std::cout << target_covs_.size() << std::endl;
    voxelmap_.reset(new GaussianVoxelMap<PointTarget>(voxel_resolution_, voxel_mode_)); // 设置体素分辨率，体素聚集方式
    voxelmap_->create_voxelmap(*target_, target_covs_);// 体素化，同时将点云分配到Voexl中，对每个存在点云的Voxel计算分布的均值和协方差
    
  }

  // ***对输入点云操作，更新点云之间的对应关系
  update_correspondences(trans);

  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[corr.first];

    const Eigen::Vector4d mean_B = corr.second->mean;
    const auto& cov_B = corr.second->cov;

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    double w = std::sqrt(target_voxel->num_points);
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error;

    if (H == nullptr || b == nullptr) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0;

    Eigen::Matrix<double, 6, 6> Hi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * jlossexp;
    Eigen::Matrix<double, 6, 1> bi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * error;

    int thread_num = omp_get_thread_num();
    Hs[thread_num] += Hi;
    bs[thread_num] += bi;
  }
  //更新 H ，b
  if (H && b) {
    H->setZero();
    b->setZero();
    for (int i = 0; i < num_threads_; i++) {
      (*H) += Hs[i];
      (*b) += bs[i];
    }
  }

  return sum_errors;
}

template <typename PointSource, typename PointTarget>
double FastVGICP<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  double sum_errors = 0.0;
#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[corr.first];

    const Eigen::Vector4d mean_B = corr.second->mean;
    const auto& cov_B = corr.second->cov;

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    double w = std::sqrt(target_voxel->num_points);
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error;
  }

  return sum_errors;
}



}  // namespace fast_gicp

#endif