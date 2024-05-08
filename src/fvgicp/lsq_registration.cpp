#include "fvgicp/lsq_registration.hpp"
#include "fvgicp/lsq_registration_impl.hpp"
#include "dlio/dlio.h"
template class fast_gicp::LsqRegistration<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::LsqRegistration<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::LsqRegistration<pcl::PointNormal, pcl::PointNormal>;
template class fast_gicp::LsqRegistration<PointType,PointType>;
