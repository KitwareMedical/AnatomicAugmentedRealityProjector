#ifndef __IO_UTIL_HPP__
#define __IO_UTIL_HPP__

#include <opencv2/core/core.hpp>

namespace io_util
{
  enum PlyFlags { PlyPoints = 0x00, PlyColors = 0x01, PlyNormals = 0x02, PlyBinary = 0x04, PlyPlane = 0x08, PlyFaces = 0x10, PlyTexture = 0x20 };

  bool write_ply( const std::string & filename, cv::Mat const& pointcloud_points, cv::Mat const& pointcloud_colors );
};

#endif  /* __IO_UTIL_HPP__ */