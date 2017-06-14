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

#ifndef __IO_UTIL_HPP__
#define __IO_UTIL_HPP__

#include <opencv2/core/core.hpp>


namespace io_util
{
  enum PlyFlags { PlyPoints = 0x00, PlyColors = 0x01, PlyNormals = 0x02, PlyBinary = 0x04, PlyPlane = 0x08, PlyFaces = 0x10, PlyTexture = 0x20 };

  bool write_ply( const std::string & filename, cv::Mat const& pointcloud_points, cv::Mat const& pointcloud_colors );
};

#endif  /* __IO_UTIL_HPP__ */
