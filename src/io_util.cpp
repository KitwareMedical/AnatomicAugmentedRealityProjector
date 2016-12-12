/*=========================================================================

Library:   AnatomicAugmentedRealityProjector

Author: Maeliss Jallais

Copyright 2010 Kitware Inc. 28 Corporate Drive,
Clifton Park, NY, 12065, USA.

All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

=========================================================================*/

#include "io_util.hpp"

#include <iostream>
#include <fstream>

bool io_util::write_ply(const std::string & filename, cv::Mat const& pointcloud_points, cv::Mat const& pointcloud_colors)
{
  if (!pointcloud_points.data
    || ( pointcloud_colors.data && pointcloud_colors.rows != pointcloud_points.rows && pointcloud_colors.cols != pointcloud_points.cols ))
  {
    return false;
  }

  bool binary = false;
  bool colors = false;
  if( pointcloud_colors.data )
    {
    colors = true;
    }

  std::vector<int> points_index;
  points_index.reserve( pointcloud_points.total());
  std::cout << "nb_points total : " << pointcloud_points.total() << std::endl;

  const cv::Vec3f * points_data = pointcloud_points.ptr<cv::Vec3f>(0);
  const cv::Vec3b * colors_data = ( colors ? pointcloud_colors.ptr<cv::Vec3b>( 0 ) : NULL );

  int total = static_cast<int>( pointcloud_points.total());
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
  if( colors )
    {
    outfile << "property uchar red" << std::endl
            << "property uchar green" << std::endl
            << "property uchar blue" << std::endl
            << "property uchar alpha" << std::endl;
    }

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

        if( colors )
          {
          cv::Vec3b const& c = colors_data[ *iter ];
          const unsigned char a = 255U;
          outfile.write( reinterpret_cast<const char *>( &( c[ 2 ] ) ), sizeof( unsigned char ) );
          outfile.write( reinterpret_cast<const char *>( &( c[ 1 ] ) ), sizeof( unsigned char ) );
          outfile.write( reinterpret_cast<const char *>( &( c[ 0 ] ) ), sizeof( unsigned char ) );
          outfile.write( reinterpret_cast<const char *>( &a ), sizeof( unsigned char ) );
          }
      }
      else
      {
        outfile << p[0] << " " << p[1] << " " << p[2];
        if( colors )
          {
          cv::Vec3b const& c = colors_data[ *iter ];
          outfile << " " << static_cast<int>( c[ 2 ] ) << " " << static_cast<int>( c[ 1 ] ) << " " << static_cast<int>( c[ 0 ] ) << " 255";
          }
        outfile << std::endl;
      }
    }
  }

  outfile.close();
  std::cerr << "[write_ply] Saved " << points_index.size() << " points (" << filename << ")" << std::endl;
  return true;
}