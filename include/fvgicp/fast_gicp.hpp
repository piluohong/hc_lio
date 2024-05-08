#ifndef FAST_GICP_FAST_GICP_HPP
#define FAST_GICP_FAST_GICP_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>
#include <fvgicp/lsq_registration.hpp>
#include <fvgicp/gicp_settings.hpp>

#include "nano_gicp/lsq_registration.h"
#include "nano_gicp/nanoflann_adaptor.h"

namespace fast_gicp {

/**
 * @brief Fast GICP algorithm optimized for multi threading with OpenMP
 */
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> CovarianceList;
template<typename PointSource, typename PointTarget>
class FastGICP : public LsqRegistration<PointSource, PointTarget> {
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
  using Ptr = pcl::shared_ptr<FastGICP<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const FastGICP<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<FastGICP<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const FastGICP<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;

public:
  FastGICP();
  virtual ~FastGICP() override;

  float source_density_;
  float target_density_;

  void setNumThreads(int n);
  void setCorrespondenceRandomness(int k); // 每个点协方差矩阵计算的邻居点个数
  void setRegularizationMethod(RegularizationMethod method);



  virtual void swapSourceAndTarget() override;
  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  virtual void setSourceCovariances(const CovarianceList& covs);
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;
  virtual void setTargetCovariances(const CovarianceList& covs);

  // 计算源点云的协方差
  virtual bool calculateSourceCovariances();
  // 计算目标点云的协方差
  virtual bool calculateTargetCovariances();

  const CovarianceList& getSourceCovariances() const {
    return source_covs_;
  }

  const CovarianceList& getTargetCovariances() const {
    return target_covs_;
  }

  std::shared_ptr<nanoflann::KdTreeFLANN<PointSource>> source_kdtree_;
  std::shared_ptr<nanoflann::KdTreeFLANN<PointTarget>> target_kdtree_;
protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  virtual void update_correspondences(const Eigen::Isometry3d& trans);

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) override;

  virtual double compute_error(const Eigen::Isometry3d& trans) override;

  template<typename PointT>
  bool calculate_covariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,nanoflann::KdTreeFLANN<PointT>& kdtree, CovarianceList& covariances, float& density);
  

protected:
  int num_threads_;
  int k_correspondences_;

  RegularizationMethod regularization_method_;

  CovarianceList source_covs_;
  CovarianceList target_covs_;

  CovarianceList mahalanobis_;

  std::vector<int> correspondences_;
  std::vector<float> sq_distances_;

   
  

};

}  // namespace fast_gicp

#endif