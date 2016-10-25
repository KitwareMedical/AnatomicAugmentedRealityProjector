#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "ProjectorWidget.hpp"
#include "CameraInput.hpp"

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

protected slots:
  void on_proj_display_clicked();
  void on_cam_display_clicked();
  void on_cam_record_clicked();

  void SetProjectorHeight();
  void SetProjectorWidth();
  void SetProjectorLineThickness();
  void SetProjectorLineRow();
  void SetCameraFrameRate();
  void SetCameraNbImages();

  void DisplayCamera();

  void _on_new_projector_image(QPixmap image);

private:
  Ui::MainWindow *ui;
  ProjectorWidget Projector;
  CameraInput CamInput;
  QTimer *timer;
  //QGraphicsScene *Scene;
  //QPixmap *PixMap;
};

#endif // MAINWINDOW_H

