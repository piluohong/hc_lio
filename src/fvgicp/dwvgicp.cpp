#include "fvgicp/dwvgicp.hpp"
#include "fvgicp/dwvgicp_impl.hpp"
#include "dlio/dlio.h"


template class fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::FastVGICP<pcl::PointNormal, pcl::PointNormal>;
template class fast_gicp::FastVGICP<PointType,PointType>;

