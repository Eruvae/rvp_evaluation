#include "rvp_evaluation/o3d_utils.h"

#include <ros/ros.h>
#include <open3d/Open3D.h>

namespace rvp_evaluation
{

std::vector<std::shared_ptr<open3d::geometry::VoxelGrid>> readAndClusterModelO3D(const std::string &filepath, double resolution)
{
  std::vector<std::shared_ptr<open3d::geometry::VoxelGrid>> vx;

  std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  bool success = open3d::io::ReadTriangleMeshUsingASSIMP(filepath, *mesh, open3d::io::ReadTriangleMeshOptions{true, false, nullptr});
  if (!success) return vx;

  mesh->RemoveDuplicatedVertices();
  mesh->RemoveDuplicatedTriangles();
  mesh->RemoveDegenerateTriangles();

  auto [cluster_indices, triangle_nums, surface_areas] = mesh->ClusterConnectedTriangles();

  std::cout << std::endl;
  for (const auto &i : triangle_nums)
      std::cout << i << ", ";

  std::vector<std::vector<size_t>> indices_to_remove_vector(triangle_nums.size());
  for (size_t i = 0; i < cluster_indices.size(); i++)
  {
    for (size_t j = 0; j < triangle_nums.size(); j++)
    {
      if (j != static_cast<size_t>(cluster_indices[i]))
        indices_to_remove_vector[j].push_back(i);
    }
  }

  ROS_INFO_STREAM("Model loaded, converting to voxel grid...");

  for (const auto &indices : indices_to_remove_vector)
  {
    std::shared_ptr<open3d::geometry::TriangleMesh> fruit_mesh = std::make_shared<open3d::geometry::TriangleMesh>(*mesh);
    fruit_mesh->RemoveTrianglesByIndex(indices);
    fruit_mesh->RemoveUnreferencedVertices();
    vx.push_back(open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*fruit_mesh, resolution));
  }
  ROS_INFO_STREAM("Converting to voxel grid successful");
  return vx;
}

std::shared_ptr<open3d::geometry::VoxelGrid> readModelO3D(const std::string &filepath, double resolution)
{
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  bool success = open3d::io::ReadTriangleMeshUsingASSIMP(filepath, *mesh, open3d::io::ReadTriangleMeshOptions{true, false, nullptr});
  if (!success) return nullptr;

  mesh->RemoveDuplicatedVertices();
  mesh->RemoveDuplicatedTriangles();
  mesh->RemoveDegenerateTriangles();

  return open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*mesh, resolution);
}

} // namespace rvp_evaluation