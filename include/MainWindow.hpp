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

#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "ProjectorWidget.hpp"
#include "CameraInput.hpp"
#include "CalibrationData.hpp"
#include "PointCloudInput.hpp"
#include <qgraphicsscene.h>
#include <QMainWindow>
#include <QLabel>


namespace Ui {
  class MainWindow;
}

class Vec2iHash
  {
  public:
    std::size_t operator()( const cv::Vec2i &vec ) const
      {
      std::size_t s = std::hash<int>()( ( vec( 1 ) << 8 ) | ( vec( 0 ) ) );
      //std::cout << "Hash code : " << s << std::endl;
      return s;
      }
  };

class MainWindow : public QMainWindow
  {
  Q_OBJECT

public:
  explicit MainWindow( QWidget *parent = 0 );
  ~MainWindow();
  cv::Mat NNClassifyColors(cv::Mat colors);
  void NNDensityProbabilityReplacement(cv::Mat pointcloud, cv::Mat pointcloud_BGR, std::vector<cv::Vec3f> *points_B, std::vector<cv::Vec3f> *points_G, std::vector<cv::Vec3f> *points_R, double threshold);
  
  cv::Point3d approximate_ray_plane_intersection(const cv::Mat & T, const cv::Point3d & vc, const cv::Point3d & vp);

  bool ComputePointCloud( cv::Mat *pointcloud, cv::Mat *pointcloud_colors, cv::Mat mat_color_ref, cv::Mat mat_color, cv::Mat imageTest, cv::Mat color_image, double delay );
  cv::Mat GetCurrentMat() const { return this->CurrentMat; };
  void SetCurrentMat( cv::Mat currentMat ) { this->CurrentMat = currentMat; };
  std::vector<cv::Vec3f> ransac( std::vector<cv::Vec3f> points, int min, int iter, float thres, int min_inliers, const cv::Vec3f normal_B = cv::Vec3f( 0, 0, 0 ), const cv::Vec3f normal_R = cv::Vec3f( 0, 0, 0 ) );
  void density_probability( cv::Mat pointcloud, cv::Mat pointcloud_BGR, std::vector<cv::Vec3f> *points_B, std::vector<cv::Vec3f> *points_G, std::vector<cv::Vec3f> *points_R, double threshold );
  cv::Vec3f three_planes_intersection( cv::Vec3f n1, cv::Vec3f n2, cv::Vec3f n3, cv::Vec3f x1, cv::Vec3f x2, cv::Vec3f x3 );
  float compute_maximum( std::vector<cv::Vec3f> points, int axis, float min, float max, float variance, float interval_min = -9999, float interval_max = 9999 );
  void save_pointcloud_plane_intersection( cv::Mat pointcloud, cv::Mat pointcloud_colors, cv::Vec3f normal_B, cv::Vec3f normal_G, cv::Vec3f normal_R, cv::Vec3f A_B, cv::Vec3f A_G, cv::Vec3f A_R, cv::Vec3f intersection, float size_circles, QString name );
  void save_pointcloud_centers( cv::Mat pointcloud, cv::Mat pointcloud_colors, cv::Vec3f center_B, cv::Vec3f center_G, cv::Vec3f center_R, float size_circles, QString name );
  void save_pointcloud( cv::Mat pointcloud, cv::Mat pointcloud_colors, QString name );
  void get_true_colors( cv::Mat *pointcloud_colors );

protected slots:
  void on_proj_display_clicked();
  void on_proj_displayColor_clicked();
  void on_detect_colors_clicked();
  void on_cam_display_clicked();
  void on_cam_record_clicked();
  void on_analyze_clicked();
  void _on_new_projector_image(QPixmap image);

  void DisplayCamera();

  void SetProjectorHeight();
  void SetProjectorWidth();
  void SetProjectorLineThickness();
  void SetProjectorLineRow();
  void SetCameraTriggerDelay();
  void SetCameraFrameRate();
  void SetCameraNbImages();
  void SetDelayParameter1();
  void SetDelayParameter2();
  void SetProjectorBlueColor();
  void SetProjectorGreenColor();
  void SetProjectorRedColor();

private:
  Ui::MainWindow *ui;
  ProjectorWidget Projector;
  CameraInput CamInput;
  PointCloudInput PCInput = PointCloudInput(&CamInput, &Projector, &Calib);
  QTimer *timer;
  QTimer *AnalyzeTimer;
  CalibrationData Calib;
  
  cv::Mat CurrentMat;
  int TimerShots;
  float max_x, max_y, max_z, min_x, min_y, min_z;
};

#endif // MAINWINDOW_H
