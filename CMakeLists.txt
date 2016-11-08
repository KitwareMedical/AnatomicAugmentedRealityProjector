
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(opencv REQUIRED)
find_package(FlyCapture REQUIRED)

set( source_files
  CalibrationData.cpp
  CalibrationData.hpp
  CameraInput.cpp
  CameraInput.hpp
  io_util.cpp
  io_util.hpp
  Main.cpp
  MainWindow.cpp
  MainWindow.hpp
  ProjectorWidget.cpp
  ProjectorWidget.hpp
  )
  
include_directories( ${CMAKE_CURRENT_BINARY_DIR} ${FLYCAPTURE_INCLUDE_DIR})

add_executable( CameraProjector 
  ${source_files}
  )

target_link_libraries( CameraProjector 
  Qt5::Widgets Qt5::Concurrent 
  ${OpenCV_LIBS} 
  ${FLYCAPTURE2_LIB}
  )
