cmake_minimum_required( VERSION 3.5 )

project( AnatomicAugmentedRealityProjector CXX )

list(INSERT CMAKE_MODULE_PATH 0 ${AnatomicAugmentedRealityProjector_SOURCE_DIR}/cmake)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(opencv REQUIRED)
find_package(FlyCapture REQUIRED)

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
qt5_wrap_cpp( moc_files ${AnatomicAugmentedRealityProjector_SOURCE_DIR}/include/MainWindow.hpp
  ${AnatomicAugmentedRealityProjector_SOURCE_DIR}/include/ProjectorWidget.hpp )

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
  )
