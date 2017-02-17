##############################################################################
#
# Library:   AnatomicAugmentedRealityProjector
#
# Author: Maeliss Jallais
#
# Copyright 2010 Kitware Inc. 28 Corporate Drive,
# Clifton Park, NY, 12065, USA.
#
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
##############################################################################

cmake_minimum_required( VERSION 3.5 )

project( AnatomicAugmentedRealityProjector CXX )

list(INSERT CMAKE_MODULE_PATH 0 ${AnatomicAugmentedRealityProjector_SOURCE_DIR}/cmake)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(OpenCV REQUIRED)
find_package(FlyCapture REQUIRED)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

# if(ITKVtkGlue_LOADED)
  # find_package(VTK REQUIRED)
  # include(${VTK_USE_FILE})
# else()
  # find_package(ItkVtkGlue REQUIRED)
  # include(${ItkVtkGlue_USE_FILE})
  # set(Glue ItkVtkGlue)
# endif()

set( source_files
  src/CalibrationData.cpp
  src/CameraInput.cpp
  src/io_util.cpp
  src/Main.cpp
  src/MainWindow.cpp
  src/ProjectorWidget.cpp
  )

set( include_files
  include/CalibrationData.hpp
  include/CameraInput.hpp
  include/io_util.hpp
  include/MainWindow.hpp
  include/ProjectorWidget.hpp
  )

qt5_wrap_ui( ui_files ${AnatomicAugmentedRealityProjector_SOURCE_DIR}/form/MainWindow.ui )
qt5_wrap_cpp( moc_files
  ${AnatomicAugmentedRealityProjector_SOURCE_DIR}/include/MainWindow.hpp
  ${AnatomicAugmentedRealityProjector_SOURCE_DIR}/include/ProjectorWidget.hpp
  )

include_directories( ${CMAKE_CURRENT_BINARY_DIR} ${FLYCAPTURE_INCLUDE_DIR} include)

add_executable( AnatomicAugmentedRealityProjector
  ${source_files}
  ${include_files}
  ${ui_files}
  ${moc_files}
  )


target_link_libraries( AnatomicAugmentedRealityProjector
  Qt5::Widgets Qt5::Concurrent 
  ${OpenCV_LIBS} 
  ${FLYCAPTURE2_LIB}
  ${ITK_LIBRARIES}
  #${VTK_LIBRARIES}
  #${Glue}
  )
