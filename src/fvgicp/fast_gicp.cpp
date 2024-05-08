#include "fvgicp/fast_gicp.hpp"
#include "fvgicp/fast_gicp_impl.hpp"
#include "dlio/dlio.h"

template class fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::FastGICP<pcl::PointNormal, pcl::PointNormal>;
template class fast_gicp::FastGICP<PointType,PointType>;
