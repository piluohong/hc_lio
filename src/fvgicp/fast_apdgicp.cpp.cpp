#include "fvgicp/fast_apdgicp.hpp"
#include "fvgicp/fast_apdgicp_impl.hpp"
#include "dlio/dlio.h"

template class fast_gicp::FastAPDGICPSingleThread<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastAPDGICPSingleThread<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::FastAPDGICPSingleThread<pcl::PointNormal, pcl::PointNormal>;
template class fast_gicp::FastAPDGICP<PointType,PointType>;