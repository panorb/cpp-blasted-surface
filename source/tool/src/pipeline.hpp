#pragma once

#include <string>

#include <spdlog/spdlog.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/impl/point_types.hpp>


std::vector<Eigen::Vector3f> run_pipeline(const std::string base_pointcloud_file);
bool load_pcl_pointcloud(const std::string base_pointcloud_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void voxelgrid_downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
void estimate_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr output_normals);
void poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, pcl::PolygonMesh& out_mesh);
bool skeletonize(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, std::vector<Eigen::Vector3f>& out_skeleton_vertices);
