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

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
  void triangulate_stereo(const cv::Mat & K1, const cv::Mat & kc1, const cv::Mat & K2, const cv::Mat & kc2,
    const cv::Mat & Rt, const cv::Mat & T, const cv::Point2i & p1, const cv::Point2i & p2,
    cv::Point3d & p3d, double * distance);
  cv::Point3d approximate_ray_intersection(const cv::Point3d & v1, const cv::Point3d & q1,
    const cv::Point3d & v2, const cv::Point3d & q2, double * distance);

protected slots:
  void on_proj_display_clicked();
  void on_cam_display_clicked();
  void on_cam_record_clicked();
  void on_analyze_clicked();
  void _on_new_projector_image(QPixmap image);
  
  void SetProjectorHeight();
  void SetProjectorWidth();
  void SetProjectorLineThickness();
  void SetProjectorLineRow();
  void SetCameraFrameRate();
  void SetCameraNbImages();

  void DisplayCamera();

private:
  Ui::MainWindow *ui;
  ProjectorWidget Projector;
  CameraInput CamInput;
  QTimer *timer;
  CalibrationData Calib;
};

#endif // MAINWINDOW_H

