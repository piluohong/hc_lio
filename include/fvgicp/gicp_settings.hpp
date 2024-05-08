#ifndef FAST_GICP_GICP_SETTINGS_HPP
#define PCL_NO_PRECOMPILE
#define FAST_GICP_GICP_SETTINGS_HPP

namespace fast_gicp {

enum class RegularizationMethod { NONE, MIN_EIG, NORMALIZED_MIN_EIG, PLANE, FROBENIUS }; //  归一化方法

enum class NeighborSearchMethod { DIRECT27, DIRECT7, DIRECT1, /* supported on only VGICP_CUDA */ DIRECT_RADIUS };
// 体素分布的求法： 加法 -添加(点的类型)权重的加法-乘法
enum class VoxelAccumulationMode { ADDITIVE, ADDITIVE_WEIGHTED, MULTIPLICATIVE };
}

#endif