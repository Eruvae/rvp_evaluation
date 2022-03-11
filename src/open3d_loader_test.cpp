#include <ros/ros.h>
#include <open3d/Open3D.h>

std::shared_ptr<open3d::geometry::VoxelGrid> readModel(const std::string &filepath, double resolution)
{
  open3d::geometry::TriangleMesh mesh;
  bool success = open3d::io::ReadTriangleMeshUsingASSIMP(filepath, mesh, open3d::io::ReadTriangleMeshOptions{true, false, nullptr});
  if (!success) return nullptr;
  ROS_INFO_STREAM("Model loaded, converting to voxel grid...");
  std::shared_ptr<open3d::geometry::VoxelGrid> vx = open3d::geometry::VoxelGrid::CreateFromTriangleMesh(mesh, resolution);
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

  auto vx = readModel(std::string(getenv("HOME")) + "/.gazebo/models/capsicum_plant_6/meshes/VG07_6.dae", 0.02);
  open3d::visualization::DrawGeometries({vx});

  ros::waitForShutdown();
}
