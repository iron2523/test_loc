#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

#include "nano_gicp/lsq_registration.h"
#include "nano_gicp/nanoflann_adaptor.h"

namespace nano_gicp {

typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> CovarianceList;

enum class RegularizationMethod { NONE, MIN_EIG, NORMALIZED_MIN_EIG, PLANE, FROBENIUS };

template<typename PointSource, typename PointTarget>
class NanoGICP : public LsqRegistration<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;

public:
  NanoGICP();
  virtual ~NanoGICP() override;

  void setNumThreads(int n);
  void setCorrespondenceRandomness(int k);
  void setMaxCorrespondenceDistance(double corr);
  void setRegularizationMethod(RegularizationMethod method);

  virtual void swapSourceAndTarget() override;
  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  virtual void setSourceCovariances(const std::shared_ptr<const CovarianceList>& covs);
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;
  virtual void setTargetCovariances(const std::shared_ptr<const CovarianceList>& covs);

  virtual void registerInputSource(const PointCloudSourceConstPtr& cloud);
  virtual void registerInputTarget(const PointCloudTargetConstPtr& cloud);

  virtual bool calculateSourceCovariances();
  virtual bool calculateTargetCovariances();

  std::shared_ptr<const CovarianceList> getSourceCovariances() const {
    return source_covs_;
  }

  std::shared_ptr<const CovarianceList> getTargetCovariances() const {
    return target_covs_;
  }

  virtual void update_correspondences(const Eigen::Isometry3d& trans);

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) override;

  virtual double compute_error(const Eigen::Isometry3d& trans) override;

  template<typename PointT>
  bool calculate_covariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const nanoflann::KdTreeFLANN<PointT>& kdtree, CovarianceList& covariances, float& density);

public:
  std::shared_ptr<const nanoflann::KdTreeFLANN<PointSource>> source_kdtree_;
  std::shared_ptr<const nanoflann::KdTreeFLANN<PointTarget>> target_kdtree_;

  std::shared_ptr<const CovarianceList> source_covs_;
  std::shared_ptr<const CovarianceList> target_covs_;

  float source_density_;
  float target_density_;

  int num_correspondences;

protected:
  int num_threads_;
  int k_correspondences_;
  double corr_dist_threshold_;

  RegularizationMethod regularization_method_;

  CovarianceList mahalanobis_;

  std::vector<int> correspondences_;
  std::vector<float> sq_distances_;
};
}  // namespace nano_gicp