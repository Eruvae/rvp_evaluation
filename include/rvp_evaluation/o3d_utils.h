#pragma once

#include <vector>
#include <memory>
#include <open3d/geometry/VoxelGrid.h>

namespace rvp_evaluation
{

/**
 * @brief Read and cluster model, convert to voxel grid
 * 
 * @param filepath path to model
 * @param resolution voxel grid resolution
 * @return a vector with voxel grids for each cluster
 */
std::vector<std::shared_ptr<open3d::geometry::VoxelGrid>> readAndClusterModelO3D(const std::string &filepath, double resolution);

/**
 * @brief Read model, convert to voxel grid
 * 
 * @param filepath path to model
 * @param resolution voxel grid resolution
 * @return the voxel grid for the model
 */
std::shared_ptr<open3d::geometry::VoxelGrid> readModelO3D(const std::string &filepath, double resolution);

} // namespace rvp_evaluation