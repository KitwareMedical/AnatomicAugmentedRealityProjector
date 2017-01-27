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
  void triangulate_stereo( const cv::Mat & K1, const cv::Mat & kc1, const cv::Mat & K2, const cv::Mat & kc2,
    const cv::Mat & Rt, const cv::Mat & T, const cv::Point2i & p1, const cv::Point2i & p2,
    cv::Point3d & p3d, double * distance );
  cv::Point3d approximate_ray_intersection( const cv::Point3d & v1, const cv::Point3d & q1,
    const cv::Point3d & v2, const cv::Point3d & q2, double * distance );
  cv::Point3d approximate_ray_plane_intersection( const cv::Mat & Rt, const cv::Mat & T,
    const cv::Point3d & vc, const cv::Point3d & qc, const cv::Point3d & vp, const cv::Point3d & qp );
  /*Take an image with RGB components*/
  int DecodeColor( cv::Mat mat );
  void ComputePointCloud( cv::Mat *pointcloud, cv::Mat *pointcloud_colors, cv::Mat mat_color_ref, cv::Mat mat_color, cv::Mat imageTest );
  cv::Mat GetCurrentMat() const { return this->CurrentMat; };
  void SetCurrentMat( cv::Mat currentMat ) { this->CurrentMat = currentMat; };
  int GetTimerShots() const { return this->TimerShots; };
  void SetTimerShots( int timerShots ) { this->TimerShots = timerShots; };
  std::vector<cv::Vec3f> get_normal_of_a_plane( std::vector<cv::Vec3f> points );
  std::vector<cv::Vec3f> ransac( std::vector<cv::Vec3f> points, int min, int iter, float thres, int min_inliers );
  std::vector<cv::Vec3f> ransac_green_plane( std::vector<cv::Vec3f> points, int min, int iter, float thres, int min_inliers, const cv::Vec3f normal_B, const cv::Vec3f normal_R );
  void density_probability( cv::Mat pointcloud, cv::Mat pointcloud_BGR, std::vector<cv::Vec3f> *points_B, std::vector<cv::Vec3f> *points_G, std::vector<cv::Vec3f> *points_R );
  cv::Vec3f three_planes_intersection( cv::Vec3f n1, cv::Vec3f n2, cv::Vec3f n3, cv::Vec3f x1, cv::Vec3f x2, cv::Vec3f x3 );

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
  void SetCameraFrameRate();
  void SetCameraNbImages();
  void SetProjectorBlueColor();
  void SetProjectorGreenColor();
  void SetProjectorRedColor();

private:
  Ui::MainWindow *ui;
  ProjectorWidget Projector;
  CameraInput CamInput;
  QTimer *timer;
  QTimer *AnalyzeTimer;
  CalibrationData Calib;
  cv::Mat CurrentMat;
  int TimerShots;
};

#endif // MAINWINDOW_H
