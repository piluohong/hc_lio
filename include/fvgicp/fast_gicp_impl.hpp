#ifndef FAST_GICP_FAST_GICP_IMPL_HPP
#define FAST_GICP_FAST_GICP_IMPL_HPP

#include <fvgicp/so3.hpp>
#include <fvgicp/fast_gicp.hpp>

namespace fast_gicp {

template <typename PointSource, typename PointTarget>
FastGICP<PointSource, PointTarget>::FastGICP() {
#ifdef _OPENMP
  num_threads_ = omp_get_max_threads();
#else
  num_threads_ = 1;
#endif

  k_correspondences_ = 20;// 默认 每个点以最近的20个点计算协方差
  reg_name_ = "FastGICP";
  corr_dist_threshold_ = std::numeric_limits<float>::max();//  返回该数据类型的最大有限值

  regularization_method_ = RegularizationMethod::PLANE;// 默认权重固定
  source_kdtree_.reset(new nanoflann::KdTreeFLANN<PointSource>);
  target_kdtree_.reset(new nanoflann::KdTreeFLANN<PointTarget>);
}

template <typename PointSource, typename PointTarget>
FastGICP<PointSource, PointTarget>::~FastGICP() {}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setNumThreads(int n) {
  num_threads_ = n;

#ifdef _OPENMP
  if (n == 0) {
    num_threads_ = omp_get_max_threads();
  }
#endif
}

// 设置相应的近邻搜索范围 defalut = 20
template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setCorrespondenceRandomness(int k) {
  k_correspondences_ = k;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setRegularizationMethod(RegularizationMethod method) {
  regularization_method_ = method;
}

// template <typename PointSource, typename PointTarget>
// void FastGICP<PointSource, PointTarget>::setAzimuthVar(double var) {
//   azimuth_variance_ = var;
// }
// template <typename PointSource, typename PointTarget>
// void FastGICP<PointSource, PointTarget>::setElevationVar(double var) {
//   elevation_variance_ = var;
// }
// template <typename PointSource, typename PointTarget>
// void FastGICP<PointSource, PointTarget>::setDistVar(double var) {
// //   distance_variance_ = var;
// }

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::swapSourceAndTarget() {
  input_.swap(target_);
  source_kdtree_.swap(target_kdtree_);
  source_covs_.swap(target_covs_);

  correspondences_.clear();
  sq_distances_.clear();
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::clearSource() {
  input_.reset();
  source_covs_.clear();
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::clearTarget() {
  target_.reset();
  target_covs_.clear();
}

// fast_gicp中的源点云输入的函数模板
template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setInputSource(const PointCloudSourceConstPtr& cloud) {
  if (input_ == cloud) {
    return;
  }

  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
  source_kdtree_->setInputCloud(cloud);// 建立源点云的源点云的kd树
  source_covs_.clear();
}

// fast_gicp中的目标点云输入的函数模板
template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setInputTarget(const PointCloudTargetConstPtr& cloud) {
  if (target_ == cloud) {
    return;
  }
  
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
  if(target_covs_.size() != cloud->size())
  {
    target_kdtree_->setInputCloud(cloud);
    target_covs_.clear();
  }
  
}

// 计算源点云的协方差矩阵
template <typename PointSource, typename PointTarget>
bool FastGICP<PointSource, PointTarget>::calculateSourceCovariances() {
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> source_covs;
  std::shared_ptr<float> source_density = std::make_shared<float>();
  bool ret = calculate_covariances(input_, *source_kdtree_, source_covs,*source_density);
  source_covs_ = source_covs;
  source_density_ = *source_density;
  return ret;
}

// 计算目标点云的协方差矩阵
template <typename PointSource, typename PointTarget>
bool FastGICP<PointSource, PointTarget>::calculateTargetCovariances() {
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> target_covs;
  std::shared_ptr<float> target_density = std::make_shared<float>();
  bool ret = calculate_covariances(target_, *target_kdtree_, target_covs,*target_density);
  target_covs_ = target_covs;
  target_density_ = *target_density;
  return ret;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setSourceCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {

  source_covs_ = covs;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setTargetCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
  
  target_covs_ = covs;
}

// align()中调用
template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  if (source_covs_.empty() || source_covs_.size() != input_->size()) {
    std::shared_ptr<float> density_temp = std::make_shared<float>();
    calculate_covariances(input_, *source_kdtree_, source_covs_,*density_temp);
    source_density_ = *density_temp;
  }
  if (target_covs_.empty() || target_covs_.size() != target_->size()) {
    std::shared_ptr<float> density_temp = std::make_shared<float>();
    calculate_covariances(target_, *target_kdtree_, target_covs_,*density_temp);
    target_density_ = *density_temp;
  }

  LsqRegistration<PointSource, PointTarget>::computeTransformation(output, guess);

}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::update_correspondences(const Eigen::Isometry3d& trans) {
  assert(source_covs_.size() == input_->size());
  assert(target_covs_.size() == target_->size());

  Eigen::Isometry3f trans_f = trans.cast<float>();

  correspondences_.resize(input_->size());
  sq_distances_.resize(input_->size());
  mahalanobis_.resize(input_->size());

  std::vector<int> k_indices(1);
  std::vector<float> k_sq_dists(1);

#pragma omp parallel for num_threads(num_threads_) firstprivate(k_indices, k_sq_dists) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    PointTarget pt;

    pt.getVector4fMap() = trans_f * input_->at(i).getVector4fMap();//

    target_kdtree_->nearestKSearch(pt, 1, k_indices, k_sq_dists);

    sq_distances_[i] = k_sq_dists[0];
    correspondences_[i] = k_sq_dists[0] < corr_dist_threshold_ * corr_dist_threshold_ ? k_indices[0] : -1; // 求点对的对应阈值

    if (correspondences_[i] < 0) {
      continue;
    }

    const int target_index = correspondences_[i];
    const auto& cov_A = source_covs_[i];
    const auto& cov_B = target_covs_[target_index];

    //

    Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
    RCR(3, 3) = 1.0;

    // 计算马氏距离: CiB + T*CiA*T(-1)
    mahalanobis_[i] = RCR.inverse();
    mahalanobis_[i](3, 3) = 0.0f;
  }
}

template <typename PointSource, typename PointTarget>
double FastGICP<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
 
  update_correspondences(trans);

  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8) 
  for (int i = 0; i < input_->size(); i++) {
    int target_index = correspondences_[i];
    if (target_index < 0) {
      continue;
    }

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[i];

    const Eigen::Vector4d mean_B = target_->at(target_index).getVector4fMap().template cast<double>();
    const auto& cov_B = target_covs_[target_index];

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    sum_errors += error.transpose() * mahalanobis_[i] * error;

    if (H == nullptr || b == nullptr) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(trans.linear()*mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0;

    Eigen::Matrix<double, 6, 6> Hi = jlossexp.transpose() * mahalanobis_[i] * jlossexp;
    Eigen::Matrix<double, 6, 1> bi = jlossexp.transpose() * mahalanobis_[i] * error;

    Hs[omp_get_thread_num()] += Hi;
    bs[omp_get_thread_num()] += bi;
  }

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
double FastGICP<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  double sum_errors = 0.0;

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8) 
  for (int i = 0; i < input_->size(); i++) {
    int target_index = correspondences_[i];
    if (target_index < 0) {
      continue;
    }

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[i];

    const Eigen::Vector4d mean_B = target_->at(target_index).getVector4fMap().template cast<double>();
    const auto& cov_B = target_covs_[target_index];

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    sum_errors += error.transpose() * mahalanobis_[i] * error;
  }

  return sum_errors;
}

// 计算每个点的协方差矩阵
template <typename PointSource, typename PointTarget>
template <typename PointT>
bool FastGICP<PointSource, PointTarget>::calculate_covariances(
  const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
  nanoflann::KdTreeFLANN<PointT>& kdtree,
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances,
  float& density) {
  if (kdtree.getInputCloud() != cloud) {
    kdtree.setInputCloud(cloud);
  }
  covariances.resize(cloud->size());
  float sum_k_sq_distances = 0.0;

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8) reduction(+:sum_k_sq_distances) //  schedule(guided, 8)指定了任务调度的方式和块大小。guided表示采用导向调度策略，8表示每个线程开始时获取的连续任务块大小
  for (int i = 0; i < cloud->size(); i++) {
    std::vector<int> k_indices;
    std::vector<float> k_sq_distances;
    PointTarget pt;
    pt.getVector4fMap() = cloud->at(i).getVector4fMap();
    kdtree.nearestKSearch(cloud->at(i), k_correspondences_, k_indices, k_sq_distances);// 近邻搜索

    //density
    const int normalization = ((k_correspondences_ - 1) * (k_correspondences_+2)) / 2;
    sum_k_sq_distances += std::accumulate(k_sq_distances.begin()+1,k_sq_distances.end(),0.0) / normalization;

    Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
    for (int j = 0; j < k_indices.size(); j++) {
      neighbors.col(j) = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
    }

    neighbors.colwise() -= neighbors.rowwise().mean().eval();
    Eigen::Matrix4d cov = neighbors * neighbors.transpose()/ k_correspondences_;

    if (regularization_method_ == RegularizationMethod::NONE) {
      covariances[i] = cov;
    } 
    else if (regularization_method_ == RegularizationMethod::FROBENIUS)
    {
      double lambda = 1e-3;
      Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() + lambda * Eigen::Matrix3d::Identity();
      Eigen::Matrix3d C_inv = C.inverse();
      covariances[i].setZero();
      covariances[i].template block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
    } 
    else {
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Vector3d values;

      // 不同归一化化的values计算方式不同 (svd分解求解特征值) -> 设计协方差矩阵
      switch (regularization_method_) {
        default:
          std::cerr << "here must not be reached" << std::endl;
          abort();
        case RegularizationMethod::PLANE:
          values = Eigen::Vector3d(1, 1, 1e-3);
          break;
        case RegularizationMethod::MIN_EIG:
          values = svd.singularValues().array().max(1e-3);
          break;
        case RegularizationMethod::NORMALIZED_MIN_EIG:
          values = svd.singularValues() / svd.singularValues().maxCoeff();
          values = values.array().max(1e-3);
          break;
      }

      covariances[i].setZero();
      covariances[i].template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
    }
  }

  density = sum_k_sq_distances / cloud->size();
  // density = 0.f;
  return true;
}

}  // namespace fast_gicp

#endif
