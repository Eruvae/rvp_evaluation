#include <ros/ros.h>
#include <open3d/Open3D.h>

std::vector<std::shared_ptr<open3d::geometry::VoxelGrid>> readModel(const std::string &filepath, double resolution)
{
  std::vector<std::shared_ptr<open3d::geometry::VoxelGrid>> vx;

  std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  bool success = open3d::io::ReadTriangleMeshUsingASSIMP(filepath, *mesh, open3d::io::ReadTriangleMeshOptions{true, false, nullptr});
  if (!success) return vx;
  open3d::visualization::DrawGeometries({mesh});

  mesh->RemoveDuplicatedVertices();
  mesh->RemoveDuplicatedTriangles();

  /// \return A vector that contains the cluster index per
  /// triangle, a second vector contains the number of triangles per
  /// cluster, and a third vector contains the surface area per cluster.
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
    //open3d::visualization::DrawGeometries({fruit_mesh});
    vx.push_back(open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*fruit_mesh, resolution));
  }
  ROS_INFO_STREAM("Converting to voxel grid successful");
  return vx;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open3d_loader_test");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  auto vx = readModel(std::string(getenv("HOME")) + "/.gazebo/models/capsicum_plant_6/meshes/VG07_6_fruitonly.dae", 0.01);

  //open3d::visualization::DrawGeometries(vx);

  ros::waitForShutdown();
}
