/*
 * @Author: piluohong 1912694135@qq.com
 * @Date: 2024-02-26 16:01:11
 * @LastEditors: piluohong 1912694135@qq.com
 * @LastEditTime: 2024-04-23 20:20:54
 * @FilePath: /slam/hhh_ws/src/GLIOM/include/fvgicp/fast_vgicp.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef FAST_GICP_FAST_VGICP_HPP
#define PCL_NO_PRECOMPILE
#define FAST_GICP_FAST_VGICP_HPP

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>

#include "gicp_settings.hpp"
#include "fast_gicp.hpp"
#include "dwvgicp_voxel.hpp"



namespace fast_gicp {

/**
       * @brief Fast Voxelized GICP algorithm boosted with OpenMP
       * @details 继承自FGICP ,VGICP 的协方差矩阵需要在输入source和target时候重新计算，由于VGICP中体素化特殊的ditribution对应方式
       * @addindex 增加部分函数，方便具体工程使用
       * @def 'override'是一个关键字，用来表示一个派生类(子类)的方法覆盖了父类(基类)中的同名方法。意味着子类提供一个新的实现，替代了从父类继承的方法。
*/
template<typename PointSource, typename PointTarget>
class FastVGICP : public FastGICP<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
  using Ptr = pcl::shared_ptr<FastVGICP<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const FastVGICP<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<FastVGICP<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const FastVGICP<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;

  using FastGICP<PointSource, PointTarget>::num_threads_;

  using FastGICP<PointSource, PointTarget>::source_covs_;
  using FastGICP<PointSource, PointTarget>::target_covs_;
public:
  using FastGICP<PointSource, PointTarget>::source_kdtree_; 
  using FastGICP<PointSource, PointTarget>::target_kdtree_;
public:
  FastVGICP();
  virtual ~FastVGICP() override;

  void setResolution(double resolution);
  void setVoxelAccumulationMode(VoxelAccumulationMode mode);
  void setNeighborSearchMethod(NeighborSearchMethod method);

  void setAzimuthVar(double var){azimuth_variance_ = var;};
  void setElevationVar(double var){elevation_variance_ = var;};
  void setDistVar(double var){distance_variance_ = var;};

  virtual void swapSourceAndTarget() override;
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;
  
  

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;
  virtual void update_correspondences(const Eigen::Isometry3d& trans) override;
  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr) override;
  virtual double compute_error(const Eigen::Isometry3d& trans) override;
public:
  bool reuse_voxelmap_ = true;
protected:
  double voxel_resolution_;
  NeighborSearchMethod search_method_;
  VoxelAccumulationMode voxel_mode_;// 体素内分布聚集方式
  
// 智能指针
  std::unique_ptr<GaussianVoxelMap<PointTarget>> voxelmap_;

  std::vector<std::pair<int, GaussianVoxel::Ptr>> voxel_correspondences_;
  CovarianceList voxel_mahalanobis_;

  double azimuth_variance_ = 0.0; 
  double elevation_variance_ = 0.0;
  double distance_variance_ = 0.0;
  Eigen::Matrix4d cov_dw;
};
}  // namespace fast_gicp

#endif
