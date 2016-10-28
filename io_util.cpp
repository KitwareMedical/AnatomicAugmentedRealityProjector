#include "io_util.hpp"
#include <iostream>
#include <fstream>

bool io_util::write_ply(const std::string & filename, cv::Mat const& pointcloud, unsigned flags)
{
  if (!pointcloud.data)
  {
    return false;
  }

  bool binary = (flags&PlyBinary);
  std::vector<int> points_index;
  points_index.reserve(pointcloud.total());
  std::cout << "nb_points total : " << pointcloud.total() << std::endl;

  const cv::Vec3f * points_data = pointcloud.ptr<cv::Vec3f>(0);

  int total = static_cast<int>(pointcloud.total());
  for (int i = 0; i<total; i++)
  {
    //if (!sl::INVALID(points_data[i]) && (!normals_data || !sl::INVALID(normals_data[i])))
    {
      points_index.push_back(i);
    }
  }

  std::ofstream outfile;
  std::ios::openmode mode = std::ios::out | std::ios::trunc | (binary ? std::ios::binary : static_cast<std::ios::openmode>(0));
  outfile.open(filename.c_str(), mode);
  if (!outfile.is_open())
  {
    return false;
  }

  int j = 0;
  for (std::vector<int>::const_iterator iter = points_index.begin(); iter != points_index.end(); iter++)
  {
    cv::Vec3f const& p = points_data[*iter];
    if (p[2] > 0)
    {
      j++;
    }
  }
  const char * format_header = (binary ? "binary_little_endian 1.0" : "ascii 1.0");
  outfile << "ply" << std::endl
    << "format " << format_header << std::endl
    << "comment scan3d-capture generated" << std::endl
    << "element vertex " << j << std::endl
    << "property float x" << std::endl
    << "property float y" << std::endl
    << "property float z" << std::endl;
  
  outfile << "element face 0" << std::endl
    << "property list uchar int vertex_indices" << std::endl
    << "end_header" << std::endl;

  for (std::vector<int>::const_iterator iter = points_index.begin(); iter != points_index.end(); iter++)
  {
    cv::Vec3f const& p = points_data[*iter];
    // We only keep the points corresponding to the line we are reconstructing 
    if (p[2] > 0)
    {
      if (binary)
      {
        outfile.write(reinterpret_cast<const char *>(&(p[0])), sizeof(float));
        outfile.write(reinterpret_cast<const char *>(&(p[1])), sizeof(float));
        outfile.write(reinterpret_cast<const char *>(&(p[2])), sizeof(float));
      }
      else
      {
        outfile << p[0] << " " << p[1] << " " << p[2];
        outfile << std::endl;
      }
    }
  }

  outfile.close();
  std::cerr << "[write_ply] Saved " << points_index.size() << " points (" << filename << ")" << std::endl;
  return true;
}