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
#include "MainWindow.hpp"
#include "ui_MainWindow.h"

#include "FileConfiguration.h"

#include "FlyCapture2.h"

#include "itkImage.h"
#include "itkVector.h"
#include "itkGaussianMembershipFunction.h"
#include "itkDiscreteGaussianImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <QtConcurrent>
#include <qtconcurrentrun.h>
#include <QDesktopWidget>
#include <QtGui>
#include <QThread>
#include <QGraphicsPixmapItem>
#include <QFileDialog>

#include <fstream>
#include <iostream>
#include <map>
#include <time.h>
#include <ctime>
#include <chrono>


MainWindow::MainWindow( QWidget *parent ) :
  QMainWindow( parent ),
  ui( new Ui::MainWindow ),
  Projector(),
  CamInput(),
  max_x( -9999 ),
  max_y( -9999 ),
  max_z( -9999 ),
  min_x( 9999 ),
  min_y( 9999 ),
  min_z( 9999 )
{
  ui->setupUi( this );
  this->setWindowTitle( "Camera Projector" );

  connect( ui->proj_height, SIGNAL( valueChanged( int ) ), this, SLOT( SetProjectorHeight() ) );
  connect( ui->proj_width, SIGNAL( valueChanged( int ) ), this, SLOT( SetProjectorWidth() ) );
  connect( ui->proj_thickness, SIGNAL( valueChanged( int ) ), this, SLOT( SetProjectorLineThickness() ) );
  connect( ui->proj_row, SIGNAL( valueChanged( int ) ), this, SLOT( SetProjectorLineRow() ) );
  connect( ui->cam_framerate, SIGNAL( valueChanged( double ) ), this, SLOT( SetCameraFrameRate() ) );
  connect( ui->trigger_delay, SIGNAL( valueChanged( double ) ), this, SLOT( SetCameraTriggerDelay() ) );
  connect( ui->delayParameter1, SIGNAL( valueChanged( double ) ), this, SLOT( SetDelayParameter1() ) );
  connect( ui->delayParameter2, SIGNAL( valueChanged( double ) ), this, SLOT( SetDelayParameter2() ) );

  connect( ui->cam_nbimages, SIGNAL( valueChanged( int ) ), this, SLOT( SetCameraNbImages() ) );
  connect( ui->proj_blue, SIGNAL( valueChanged( int ) ), this, SLOT( SetProjectorBlueColor() ) );
  connect( ui->proj_green, SIGNAL( valueChanged( int ) ), this, SLOT( SetProjectorGreenColor() ) );
  connect( ui->proj_red, SIGNAL( valueChanged( int ) ), this, SLOT( SetProjectorRedColor() ) );

  this->SetDelayParameter1();
  this->SetDelayParameter2();

  // Timer
  this->timer = new QTimer( this );
  this->timer->setSingleShot( false );
  this->timer->setInterval( 5 );
  this->connect( timer, SIGNAL( timeout() ), SLOT( DisplayCamera() ) );

  // AnalyzeTimer
  this->AnalyzeTimer = new QTimer( this );
  this->AnalyzeTimer->setSingleShot( false );
  this->AnalyzeTimer->setInterval( 200 );
  this->connect( AnalyzeTimer, SIGNAL( timeout() ), SLOT( Analyze() ) );

  CalibrationData calib;
  QString calibrationFile = CAMERA_CALIB_FILE_LOCATION;

  bool error = this->Calib.LoadCalibration( calibrationFile );
  if( error == false )
    {
    std::cout << "Impossible to read the calibration file" << std::endl;
    }
  else
    {
    this->Calib.Display();
    }
  this->on_proj_displayColor_clicked();

  bool success = this->CamInput.Run();
  if( success == false )
    {
    std::cout << "Impossible to start the camera." << std::endl;
    }

  hires = PCInput.ComputePointCloud( 300 );
  save_pointcloud( hires.points, hires.colors, "hires" );
}


void MainWindow::SetDelayParameter1()
{
  PCInput.delayParam1 = this->ui->delayParameter1->value();
}


void MainWindow::SetDelayParameter2()
{
  PCInput.delayParam2 = this->ui->delayParameter2->value();
}


cv::Mat MainWindow::NNClassifyColors( cv::Mat colors )
{
  static cv::Mat layer1M = ( cv::Mat_<double>( 3, 20 ) << -0.46561, -0.464753, -0.441701, 0.265619, -0.0794222, -0.336208, -0.407372, -0.476262, -0.358929, 0.11973, -0.144362, 0.011893, 0.00369619, -0.58382, -0.460448, -0.4883, 0.257962, 0.111014, 0.221085, -0.115245, -0.468333, -0.331791, 0.327327, -0.721701, 0.0308189, -0.0109495, -0.591048, -0.465264, -0.124108, -0.17753, -0.240377, -0.474372, -0.186954, 0.194256, 0.498416, -0.153055, -0.152493, -0.118814, -0.273497, -0.314121, -0.422223, -0.47281, 0.11764, 0.184481, 0.164925, 0.391668, 0.548537, 0.136319, 0.275656, 0.229943, -0.0160464, -0.450993, -0.608574, -0.048987, -0.0741594, -0.494928, 0.135861, 0.0284544, -0.190202, 0.365849 );
  static cv::Mat layer1b = ( cv::Mat_<double>( 1, 20 ) << 0.0, 0.0, 1.02417, -0.101617, 1.10532, -0.354024, -0.865269, -0.1142, 0.451612, -1.19046, 0.0, -0.173002, -0.102641, -0.592558, -0.929955, 0.0, 0.130743, 0.997855, -0.536268, 1.17299 );

  static cv::Mat layer2M = ( cv::Mat_<double>( 20, 7 ) << 0.0486266, 0.387843, 0.325573, 0.464626, -0.0581379, -0.33381, 0.353558, 0.0419606, -0.228845, -0.209768, 0.436064, -0.312049, 0.217268, -0.0905344, -0.431897, -0.0411522, -0.282755, -0.0519514, -0.0037479, 0.123086, 0.0705703, -0.209886, 0.0900581, 0.0664114, 0.354451, -0.287335, 0.134728, 0.00584876, 0.165506, -0.404287, 0.236688, 0.439842, -0.429936, -0.143274, -0.00450467, -0.484109, 0.127069, 0.0745968, 0.0902402, -0.353209, 0.400499, 0.123299, 0.372309, -0.0251773, 0.371902, 0.132656, -0.461655, -0.0749941, 0.105539, 0.231794, 0.372855, 0.346352, 0.616517, 0.450901, 0.187632, -0.567534, -0.343429, -0.0100041, 0.217494, -0.295606, 0.365753, -0.692834, 0.00629612, -0.307328, 0.406752, -0.243178, -0.283883, 0.10712, -0.28927, -0.105234, 0.119902, 0.38432, 0.0743947, 0.134401, -0.0409988, 0.0459979, 0.367451, -0.313109, 0.626782, 0.0327785, 0.224117, 0.403504, 0.814649, 0.0936099, 0.211686, -0.267912, 0.467233, 0.184689, 0.222669, 0.438642, -0.136827, 0.493749, 0.103821, -0.502788, 0.183925, -0.248316, -0.0516033, 0.433942, 0.183481, 0.205047, -0.193198, 0.392753, -0.0955889, -0.424168, -0.192003, -0.0868697, 0.150881, -0.324374, -0.22052, 0.206335, -0.233846, 0.00296679, 0.039072, -0.119868, -0.619125, 0.0602632, 0.000689579, 0.424598, -0.0495005, 0.159581, -0.441435, 0.136871, 0.166651, -0.271205, 0.236366, 0.274756, -0.028359, 0.100489, 0.474152, -0.444237, 0.0476164, 0.496671, -0.145268, 0.330884, 0.0574391, -0.350685, -0.302294, -0.151138, -0.416798, 0.0846137 );
  static cv::Mat layer2b = ( cv::Mat_<double>( 1, 7 ) << 1.04831, -1.60268, -0.6065, 0.171145, -0.447067, -0.251562, 1.17075 );

  static cv::Mat layer3M = ( cv::Mat_<double>( 7, 4 ) << -1.19894, -0.941196, -2.3716, 0.343039, -0.926799, -0.675361, -0.884165, 0.285122, -0.42241, -1.2389, -0.397014, 0.214381, -0.764641, 0.145982, -0.473203, 0.0182096, -0.324915, 0.161614, 0.287208, 0.357671, -0.211177, -0.825902, -0.638148, -0.360811, -1.20959, -0.731103, 0.35537, -0.118167 );
  static cv::Mat layer3b = ( cv::Mat_<double>( 1, 4 ) << -1.78275, 0.550166, 0.344412, -0.0907723 );

  return cv::max( 0, ( cv::max( 0, ( colors* layer1M + layer1b ) ) * layer2M + layer2b ) )*layer3M + layer3b;
}


MainWindow::~MainWindow()
{
  delete ui;
}


inline QImage cvMatToQImage( const cv::Mat &mat )
{
  switch( mat.type() )
    {
      // 8-bit, 3 channel
      case CV_8UC3:
      {
      QImage image( mat.data, mat.cols, mat.rows, static_cast< int >( mat.step ), QImage::Format_RGB888 );
      return image.rgbSwapped();
      }
      // 8-bit, 1 channel
      case CV_8UC1:
      {
        // creating a color table only the first time
      static QVector<QRgb> sColorTable;

      if( sColorTable.isEmpty() )
        {
        for( int i = 0; i < 256; i++ )
          {
          sColorTable.append( qRgb( i, i, i ) );
          //NOTE : /!\ takes time
          }
        }
      QImage image( mat.data, mat.cols, mat.rows, static_cast< int >( mat.step ), QImage::Format_Indexed8 );
      image.setColorTable( sColorTable );
      return image;
      }
      default:
        qWarning() << "Type not handled : " << mat.type();
        break;
    }
  return QImage();
}


// Note : If we know that the lifetime of the cv::Mat is shorter than the QImage, then pass false for the inCloneImageData argument. This will share the QImage data.
inline cv::Mat QImageToCvMat( const QImage& image, bool inCloneImageData = true )
{
  switch( image.format() )
    {
      case QImage::Format_Indexed8:
      {
        //8-bit, 1 channel
      cv::Mat mat( image.height(), image.width(), CV_8UC1, const_cast< uchar* >( image.bits() ), static_cast< size_t >( image.bytesPerLine() ) );
      return ( inCloneImageData ? mat.clone() : mat );
      }
      case QImage::Format_RGB888:
      {
      if( !inCloneImageData )
        {
        qWarning() << "ASM::QImageToCvMat() - Conversion requires cloning because we use a temporary QImage";
        }

      QImage   swapped;
      swapped = image.rgbSwapped();

      return cv::Mat( swapped.height(), swapped.width(), CV_8UC3, const_cast< uchar* >( swapped.bits() ), static_cast< size_t >( swapped.bytesPerLine() ) ).clone();
      }
      default:
        qWarning() << "Type not handled : " << image.format();
        break;
    }
  return cv::Mat();
}


void MainWindow::on_proj_display_clicked()
{
  /*cv::Mat mat = this->Projector.CreatePattern();
  if (!mat.data)
  {
    std::cout << "Could not open or find the image" << std::endl;
    return;
  }
  QPixmap pixmap = QPixmap::fromImage(cvMatToQImage(mat));
  this->Projector.SetPixmap(pixmap);

  connect(&(this->Projector), SIGNAL(new_image(QPixmap)), this, SLOT(_on_new_projector_image(QPixmap)));

  this->Projector.start();

  //disconnect projector display signal
  disconnect(&(this->Projector), SIGNAL(new_image(QPixmap)), this, SLOT(_on_new_projector_image(QPixmap)));
  */

  QString imagename = "C:\\Camera_Projector_Calibration\\Color-line\\Test-colors\\red_cube_crop.png";
  cv::Mat mat = cv::imread( qPrintable( imagename ) );
  if( !mat.data || mat.type() != CV_8UC3 )
    {
    qCritical() << "ERROR invalid cv::Mat data\n";
    return;
    }

  std::vector<cv::Vec3b> blue;
  int line = 0;
  cv::Vec3b crt;
  for( int i = 0; i < mat.rows; i++ )
    {
    for( int j = 0; j < mat.cols; j++ )
      {
      crt = mat.at<cv::Vec3b>( i, j );
      if( crt[ 2 ] > crt[ 1 ] && crt[ 2 ] > crt[ 0 ] && crt[ 2 ] > 20 )
        {
        blue.push_back( crt );
        line++;
        }
      }
    }

  cv::Mat G = cv::Mat::zeros( line, 3, CV_8UC1 );
  line = 0;
  for( int i = 0; i < mat.rows; i++ )
    {
    for( int j = 0; j < mat.cols; j++ )
      {
      crt = mat.at<cv::Vec3b>( i, j );
      if( crt[ 2 ] > crt[ 1 ] && crt[ 2 ] > crt[ 0 ] && crt[ 2 ] > 20 )
        {
        std::cout << crt << std::endl;
        blue.push_back( crt );
        G.at<unsigned char>( line, 0 ) = crt[ 0 ];
        G.at<unsigned char>( line, 1 ) = crt[ 1 ];
        G.at<unsigned char>( line, 2 ) = crt[ 2 ];
        line++;
        }
      }
    }
  std::cout << "line : " << line << std::endl;

  cv::Scalar mean, stddev, cov;
  cv::meanStdDev( blue, mean, stddev );
  std::cout << "mean : " << mean << std::endl;
  std::cout << "standard deviation : " << stddev << std::endl;

  cv::Mat covG, meanG;
  cv::calcCovarMatrix( G, covG, meanG, CV_COVAR_NORMAL | CV_COVAR_ROWS | CV_COVAR_SCALE );
  std::cout << "mean : " << meanG << std::endl;
  std::cout << "cov : " << covG << std::endl;

  std::fstream outputFile;
  outputFile.open( "C:\\Camera_Projector_Calibration\\Tests_publication\\red_mean_cov.txt", std::ios::out );
  outputFile << "Mean : " << mean << std::endl;
  outputFile << "Std dev : " << stddev << std::endl;
  outputFile << "Mean : " << meanG << std::endl;
  outputFile << "Cov : " << covG << std::endl;
  outputFile.close();
}


void MainWindow::on_proj_displayColor_clicked()
{
  cv::Mat mat = this->Projector.CreateColoredImage( this->Projector.GetBlueColor(), this->Projector.GetGreenColor(), this->Projector.GetRedColor() );
  if( !mat.data )
    {
    std::cout << "Could not open or find the image" << std::endl;
    return;
    }

  QPixmap pixmap = QPixmap::fromImage( cvMatToQImage( mat ) );
  this->Projector.SetPixmap( pixmap );

  connect( &( this->Projector ), SIGNAL( new_image( QPixmap ) ), this, SLOT( _on_new_projector_image( QPixmap ) ) );

  this->Projector.start();

  //disconnect projector display signal
  disconnect( &( this->Projector ), SIGNAL( new_image( QPixmap ) ), this, SLOT( _on_new_projector_image( QPixmap ) ) );
}


void MainWindow::PutImageOnPointCloud( PointCloud p, cv::Mat Image, cv::Vec3f origin, cv::Vec3f u1, cv::Vec3f u2, cv::Vec3f u3 )
{
  for( int line = 0; line < p.points.rows; line++ )
    {
    for( int column = 0; column < p.points.cols; column++ )
      {

      cv::Vec3f point = p.points.at<cv::Vec3f>( line, column );
      cv::Vec3f displacement = point - origin;

      float x = displacement.dot( u3 );
      float y = displacement.dot( u2 );

      x += 8.5;
      y -= 4.5;
      x *= 10;
      x += Image.rows / 2;
      y *= 10;

      if( ( x >= 0 && y >= 0 && x < Image.cols &&  y < Image.rows ) )
        {
        p.colors.at<cv::Vec3b>( line, column ) = cv::Vec3b( x, y, 255 );//Image.at<cv::Vec3b>(x, y);
        }
      else
        {
        p.colors.at<cv::Vec3b>( line, column ) = cv::Vec3b( 255, 255, 255 );
        }
      }
    }
}


void MainWindow::ProjectPointCloud( PointCloud p )
{
  cv::Mat result = this->Projector.CreateColoredImage( this->Projector.GetBlueColor(), this->Projector.GetGreenColor(), this->Projector.GetRedColor() );

  for( int line = 0; line < p.points.rows; line++ )
    {
    for( int column = 0; column < p.points.cols; column++ )
      {
      int thickness = -1;
      int lineType = 8;
      float normalizedLine = line / ( float )p.points.rows;
      cv::Point3d point = p.points.at<cv::Point3f>( line, column );
      //cv::Vec3b color = cv::Vec3b( 255 * (fmod(point.x + 100, 2) < .5), 255 * (fmod(point.y + 100, 2) < .5),255 * ( fmod(point.z + 100, 2) < .5 ));
      cv::circle( result,
                  cv::Point2d( column * ( 2.55 - normalizedLine / 3 ) - ( 1 - normalizedLine ) * 120, normalizedLine * result.rows ),
                  2,
                  p.colors.at<cv::Vec3b>( line, column ),
                  thickness,
                  lineType );
      }
    }

  QPixmap pixmap = QPixmap::fromImage( cvMatToQImage( result ) );
  this->Projector.SetPixmap( pixmap );

  connect( &( this->Projector ), SIGNAL( new_image( QPixmap ) ), this, SLOT( _on_new_projector_image( QPixmap ) ) );

  //this->Projector.start();
  this->Projector.update();
  //disconnect projector display signal
  disconnect( &( this->Projector ), SIGNAL( new_image( QPixmap ) ), this, SLOT( _on_new_projector_image( QPixmap ) ) );
}


void MainWindow::on_detect_colors_clicked()
{
  hires = PCInput.ComputePointCloud( 300 );
  save_pointcloud( hires.points, hires.colors, "hires" );
  return;
}


void MainWindow::on_cam_display_clicked()
{
  this->timer->start();
}


void MainWindow::on_cam_record_clicked()
{
  this->CamInput.RecordImages();
}


void MainWindow::DisplayCamera()
{
  QGraphicsScene *scene = new QGraphicsScene( this );
  ui->cam_image->setScene( scene );
  this->CurrentMat = this->CamInput.GetImageFromBuffer();
  QPixmap PixMap = QPixmap::fromImage( cvMatToQImage( this->CurrentMat ) );
  scene->clear();
  ui->cam_image->scene()->addItem( new QGraphicsPixmapItem( PixMap ) );
  scene->setSceneRect( 0, 0, PixMap.width(), PixMap.height() );
  ui->cam_image->fitInView( scene->sceneRect(), Qt::KeepAspectRatio );
}


void MainWindow::_on_new_projector_image( QPixmap pixmap )
{
  this->Projector.SetPixmap( pixmap );
}


void MainWindow::SetProjectorHeight()
{
  this->Projector.SetHeight( ui->proj_height->value() );
}


void MainWindow::SetProjectorWidth()
{
  this->Projector.SetWidth( ui->proj_width->value() );
}


void MainWindow::SetProjectorLineThickness()
{
  this->Projector.SetLineThickness( ui->proj_thickness->value() );
}


void MainWindow::SetProjectorLineRow()
{
  this->Projector.SetRow( ui->proj_row->value() );
}


void MainWindow::SetCameraTriggerDelay()
{
  this->CamInput.SetCameraTriggerDelay( ui->trigger_delay->value() / 1000 );
}


void MainWindow::SetCameraFrameRate()
{
  this->CamInput.SetCameraFrameRate( ui->cam_framerate->value() );
}


void MainWindow::SetCameraNbImages()
{
  this->CamInput.SetNbImages( ui->cam_nbimages->value() );
}


void MainWindow::SetProjectorBlueColor()
{
  this->Projector.SetBlueColor( ui->proj_blue->value() );
}


void MainWindow::SetProjectorGreenColor()
{
  this->Projector.SetGreenColor( ui->proj_green->value() );
}


void MainWindow::SetProjectorRedColor()
{
  this->Projector.SetRedColor( ui->proj_red->value() );
}


void MainWindow::on_analyze_clicked()
{
  ui->analyze->setText( "Stop" );
  disconnect( ui->analyze, SIGNAL( clicked() ), this, SLOT( on_analyze_clicked() ) );
  connect( ui->analyze, SIGNAL( clicked() ), this, SLOT( StopAnalyze() ) );

  CamInput.SetCameraTriggerDelay( 0 );
  this->DisplayCamera();

  this->AnalyzeTimer->start();

  return;
}


void MainWindow::Analyze()
{
  QCoreApplication::processEvents();
  PointCloud pointcloud = this->PCInput.ComputePointCloud( 45 );

  if( !pointcloud.points.data )
    {
    qCritical() << "ERROR, reconstruction failed\n";
    }

  if( this->ui->savePointClouds->isChecked() )
    {
    save_pointcloud( pointcloud.points, pointcloud.colors, "pointcloud_BGR_original" );
    }
  
  /***************************Finding the blue, red and green planes*****************************/
  std::vector<cv::Vec3f> points_B, points_G, points_R;
  points_B.clear();
  points_G.clear();
  points_R.clear();
  //density_probability (pointcloud.points, pointcloud.colors, &points_B, &points_G, &points_R, 2e-13 );

  NNDensityProbabilityReplacement( pointcloud.points, pointcloud.colors, &points_B, &points_G, &points_R, 6 );

  //density_probability( pointcloud.points, pointcloud.colors, &points_B, &points_G, &points_R , 2e-33 );

  //std::cout << "Number of blue points found : " << points_B.size() << std::endl;
  //std::cout << "Number of red points found : " << points_R.size() << std::endl;
  //std::cout << "Number of green points found : " << points_G.size() << std::endl;

  // Find the gravity centers of the blue, red and green points
  /*
  cv::Vec3f center_B = cv::Vec3f( 0, 0, 0 );
  cv::Vec3f center_R = cv::Vec3f( 0, 0, 0 );
  cv::Vec3f center_G = cv::Vec3f( 0, 0, 0 );
  cv::Vec3f center_total = cv::Vec3f( 0, 0, 0 );
  int nb_total = 0;
  int nb = 0;
  */

  //find the point that has the most points of every color nearby, scored by the worst of red, blue, or green
  float distance_B = 0, distance_R = 0, distance_G = 0;

  cv::Vec3f center = cv::Vec3f( 0, 0, 0 );
  int best_number_of_rarest_color = 0;
  for( int iteration = 0; iteration < 200; iteration++ )
    {
    //select a random point
    cv::Vec3f random_point;
    int idx;
    switch( rand() % 3 )
      {
        case 0:
          idx = rand() % points_B.size();
          random_point = points_B[ idx ];
          break;
        case 1:
          idx = rand() % points_G.size();
          random_point = points_G[ idx ];
          break;
        case 2:
          idx = rand() % points_R.size();
          random_point = points_R[ idx ];
          break;
      }

    //count how many of each color are near that point
    int r_count = 0, b_count = 0, g_count = 0;

    for( auto iter = points_R.cbegin(); iter != points_R.cend(); ++iter )
      {
      float distance = std::sqrt( pow( random_point[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( random_point[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( random_point[ 2 ] - ( *iter )[ 2 ], 2 ) );

      if( distance < 5.0 )
        {
        r_count++;
        }
      }
    for( auto iter = points_G.cbegin(); iter != points_G.cend(); ++iter )
      {
      float distance = std::sqrt( pow( random_point[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( random_point[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( random_point[ 2 ] - ( *iter )[ 2 ], 2 ) );

      if( distance < 5.0 )
        {
        g_count++;
        }
      }
    for( auto iter = points_B.cbegin(); iter != points_B.cend(); ++iter )
      {
      float distance = std::sqrt( pow( random_point[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( random_point[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( random_point[ 2 ] - ( *iter )[ 2 ], 2 ) );

      if( distance < 5.0 )
        {
        b_count++;
        }
      }

    int score = std::min( std::min( r_count, b_count ), g_count );
    if( score > best_number_of_rarest_color )
      {
      center = random_point;
      best_number_of_rarest_color = score;
      }
    }
    /*
    float distB = 0, distG = 0, distR = 0;
    float dist_circles = 1.3f;
    float variance = 3;
    std::cout << "max_x = " << max_x << std::endl;
    std::cout << "max_y = " << max_y << std::endl;
    std::cout << "max_z = " << max_z << std::endl;
    std::cout << "min_x = " << min_x << std::endl;
    std::cout << "min_y = " << min_y << std::endl;
    std::cout << "min_z = " << min_z << std::endl;

    float max_x_B = compute_maximum( points_B, 0, this->min_x, this->max_x, variance );
    if( max_x_B == 0 )
      {
      std::cout << "Error in the computation of max_x_B" << std::endl;
      }
    std::cout << "max_x_B = " << max_x_B << std::endl;
    float max_x_R = compute_maximum( points_R, 0, this->min_x, this->max_x, variance );
    if( max_x_R == 0 )
      {
      std::cout << "Error in the computation of max_x_R" << std::endl;
      }
    std::cout << "max_x_R = " << max_x_R << std::endl;
    float max_x_G = compute_maximum( points_G, 0, this->min_x, this->max_x, variance );
    if( max_x_G == 0 )
      {
      std::cout << "Error in the computation of max_x_G" << std::endl;
      }
    std::cout << "max_x_G = " << max_x_G << std::endl;

    float max_y_B = compute_maximum( points_B, 1, this->min_y, this->max_y, variance, max_x_B - variance / 100, max_x_B + variance / 100 );
    if( max_y_B == 0 )
      {
      std::cout << "Error in the computation of max_y_B" << std::endl;
      }
    std::cout << "max_y_B = " << max_y_B << std::endl;
    float max_y_R = compute_maximum( points_R, 1, this->min_y, this->max_y, variance, max_x_R - variance / 100, max_x_R + variance / 100 );
    if( max_y_R == 0 )
      {
      std::cout << "Error in the computation of max_y_R" << std::endl;
      }
    std::cout << "max_y_R = " << max_y_R << std::endl;
    float max_y_G = compute_maximum( points_G, 1, this->min_y, this->max_y, variance, max_x_G - variance / 100, max_x_G + variance / 100 );
    if( max_y_G == 0 )
      {
      std::cout << "Error in the computation of max_y_G" << std::endl;
      }
    std::cout << "max_y_G = " << max_y_G << std::endl;

    float max_z_B = compute_maximum( points_B, 2, this->min_z, this->max_z, variance, max_x_B - variance / 100, max_x_B + variance / 100 );
    if( max_z_B == 0 )
      {
      std::cout << "Error in the computation of max_z_B" << std::endl;
      }
    std::cout << "max_z_B = " << max_z_B << std::endl;
    float max_z_R = compute_maximum( points_R, 2, this->min_z, this->max_z, variance, max_x_R - variance / 100, max_x_R + variance / 100 );
    if( max_z_R == 0 )
      {
      std::cout << "Error in the computation of max_z_R" << std::endl;
      }
    std::cout << "max_z_R = " << max_z_R << std::endl;
    float max_z_G = compute_maximum( points_G, 2, this->min_z, this->max_z, variance, max_x_G - variance / 100, max_x_G + variance / 100 );
    if( max_z_G == 0 )
      {
      std::cout << "Error in the computation of max_z_G" << std::endl;
      }
    std::cout << "max_z_G = " << max_z_G << std::endl;

    center_B = cv::Vec3f{ max_x_B, max_y_B, max_z_B };
    center_R = cv::Vec3f{ max_x_R, max_y_R, max_z_R };
    center_G = cv::Vec3f{ max_x_G, max_y_G, max_z_G };

    std::cout << "histogram centers" << center_B << center_R << center_G << std::endl;
  #ifdef DEBUG_POINTCLOUDS
    save_pointcloud_centers( pointcloud.points, pointcloud.colors, center_B, center_G, center_R, 1, "pointcloud_BGR_centers_histo" );
  #endif
    for( float dist = 15.f; dist > 5.0f; dist -= 1 )
      {
      nb = 0; center_G = cv::Vec3b( 0, 0, 0 );
      for( auto iter = points_G.cbegin(); iter != points_G.cend(); ++iter )
        {
        distance_B = std::sqrt( pow( center_B[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center_B[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center_B[ 2 ] - ( *iter )[ 2 ], 2 ) );
        distance_R = std::sqrt( pow( center_R[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center_R[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center_R[ 2 ] - ( *iter )[ 2 ], 2 ) );
        if( distance_B < dist && distance_R < dist )
          {
          center_G += ( *iter );
          nb++;
          }
        }
      center_G = center_G / nb;

      nb = 0; center_B = cv::Vec3b( 0, 0, 0 );
      for( auto iter = points_B.cbegin(); iter != points_B.cend(); ++iter )
        {
        distance_G = std::sqrt( pow( center_G[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center_G[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center_G[ 2 ] - ( *iter )[ 2 ], 2 ) );
        distance_R = std::sqrt( pow( center_R[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center_R[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center_R[ 2 ] - ( *iter )[ 2 ], 2 ) );
        if( distance_G < dist && distance_R < dist )
          {
          center_B += ( *iter );
          nb++;
          }
        }
      center_B = center_B / nb;

      nb = 0; center_R = cv::Vec3b( 0, 0, 0 );
      for( auto iter = points_R.cbegin(); iter != points_R.cend(); ++iter )
        {
        distance_B = std::sqrt( pow( center_B[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center_B[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center_B[ 2 ] - ( *iter )[ 2 ], 2 ) );
        distance_G = std::sqrt( pow( center_G[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center_G[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center_G[ 2 ] - ( *iter )[ 2 ], 2 ) );
        if( distance_B < dist && distance_G < dist )
          {
          center_R += ( *iter );
          nb++;
          }
        }
      center_R = center_R / nb;
      }
    std::cout << "Center_B : " << center_B << std::endl;
    std::cout << "Center_R : " << center_R << std::endl;
    std::cout << "Center_G : " << center_G << std::endl;
  #ifdef DEBUG_POINTCLOUDS
    save_pointcloud_centers( pointcloud.points, pointcloud.colors, center_B, center_G, center_R, dist_circles, "pointcloud_BGR_centers" );
  #endif
  /**************    M1    ***************/
  
  // Redefine the colored vectors
  double radius_good = 5;
  std::vector<cv::Vec3f> good_B;
  for( auto iter = points_B.cbegin(); iter != points_B.cend(); ++iter )
    {
    distance_B = std::sqrt( pow( center[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center[ 2 ] - ( *iter )[ 2 ], 2 ) );
    if( distance_B < radius_good )
      {
      good_B.push_back( *iter );
      }
    }
  std::vector<cv::Vec3f> good_R;
  for( auto iter = points_R.cbegin(); iter != points_R.cend(); ++iter )
    {
    distance_R = std::sqrt( pow( center[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center[ 2 ] - ( *iter )[ 2 ], 2 ) );
    if( distance_R < radius_good )
      {
      good_R.push_back( *iter );
      }
    }
  std::vector<cv::Vec3f> good_G;
  for( auto iter = points_G.cbegin(); iter != points_G.cend(); ++iter )
    {
    distance_G = std::sqrt( pow( center[ 0 ] - ( *iter )[ 0 ], 2 ) + pow( center[ 1 ] - ( *iter )[ 1 ], 2 ) + pow( center[ 2 ] - ( *iter )[ 2 ], 2 ) );
    if( distance_G < radius_good )
      {
      good_G.push_back( *iter );
      }
    }
  if( this->ui->savePointClouds->isChecked() )
    {
    save_pointcloud_centers( pointcloud.points, pointcloud.colors, center, center, center, 5.f, "pointcloud_BGR_selected_points_M1" );
    }
  //std::cout << "Size of blue vector : " << good_B.size() << std::endl;
  //std::cout << "Size of red vector : " << good_R.size() << std::endl;
  //std::cout << "Size of green vector : " << good_G.size() << std::endl;
  /*
  // Compute the 3 planes
  std::vector<cv::Vec3f> res_B = ransac( good_B, 3, 100, 0.01f, 10 );
  if( res_B.size() != 2 )
    {
    std::cout << "Error in the RANSAC algorithm" << std::endl;
    return;
    }
  cv::Vec3f normal_B = res_B[ 0 ];
  cv::Vec3f A_B = res_B[ 1 ];

  std::vector<cv::Vec3f> res_R = ransac( good_R, 3, 100, 0.01f, std::min( 10, int( good_R.size() ) - 2 ), normal_B );
  if( res_R.size() != 2 )
    {
    std::cout << "Error in the RANSAC algorithm" << std::endl;
    return;
    }
  cv::Vec3f normal_R = res_R[ 0 ];
  cv::Vec3f A_R = res_R[ 1 ];

  std::vector<cv::Vec3f> res_G = ransac( good_G, 3, 100, 0.01f, std::min( 10, int( good_G.size() ) - 2 ), normal_B, normal_R );
  if( res_G.size() != 2 )
    {
    std::cout << "Error in the RANSAC algorithm" << std::endl;
    return;
    }
  cv::Vec3f normal_G = res_G[ 0 ];
  cv::Vec3f A_G = res_G[ 1 ];

  //std::cout << "Blue plane : normal : " << normal_B << " point A : " << A_B << std::endl;
  //std::cout << "Green plane : normal : " << normal_G << " point A : " << A_G << std::endl;
  //std::cout << "Red plane : normal : " << normal_R << " point A : " << A_R << std::endl;

  //std::cout << "Orhtogonal BR ? " << normal_B.dot( normal_R ) << std::endl;
  //std::cout << "Orhtogonal BG ? " << normal_B.dot( normal_G ) << std::endl;
  //std::cout << "Orhtogonal RG ? " << normal_R.dot( normal_G ) << std::endl;

  cv::Vec3f intersection;
  intersection = three_planes_intersection( normal_B, normal_G, normal_R, A_B, A_G, A_R );
  std::cout << "Intersection : " << intersection << std::endl;

  save_pointcloud_plane_intersection( pointcloud, pointcloud_colors, normal_B, normal_G, normal_R, A_B, A_G, A_R, intersection, 0.001f, "pointcloud_BGR_plane" );
  std::fstream outputFile;
  outputFile.open( "C:\\Camera_Projector_Calibration\\Tests_publication\\800-between-395-780\\intersection_point_circle.txt", std::ios::out );
  outputFile << "Intersection : " << intersection << std::endl;
  */

  /**************    M2 = circles    ***************/
  /*
  std::vector<cv::Vec3f> blue, green, red;
  float dist_B, dist_G, dist_R;
  for( int row = 0; row < pointcloud.points.rows; row++ )
    {
    for( int col = 0; col < pointcloud.points.cols; col++ )
      {
      cv::Vec3f crt = pointcloud.points.at<cv::Vec3f>( row, col );
      if( crt[ 2 ] > 0 )
        {
        dist_B = std::sqrt( pow( center_B[ 0 ] - crt[ 0 ], 2 ) + pow( center_B[ 1 ] - crt[ 1 ], 2 ) + pow( center_B[ 2 ] - crt[ 2 ], 2 ) );
        dist_R = std::sqrt( pow( center_R[ 0 ] - crt[ 0 ], 2 ) + pow( center_R[ 1 ] - crt[ 1 ], 2 ) + pow( center_R[ 2 ] - crt[ 2 ], 2 ) );
        dist_G = std::sqrt( pow( center_G[ 0 ] - crt[ 0 ], 2 ) + pow( center_G[ 1 ] - crt[ 1 ], 2 ) + pow( center_G[ 2 ] - crt[ 2 ], 2 ) );
        pointcloud.colors.at<cv::Vec3b>( row, col ) = cv::Vec3b( 0, 0, 0 );
        if( dist_B < dist_circles )
          {
          blue.push_back( crt );
          }
        if( dist_R < dist_circles )
          {
          red.push_back( crt );
          }
        if( dist_G < dist_circles )
          {
          green.push_back( crt );
          }
        }
      }
    }
    */

  std::vector<cv::Vec3f> res_red = ransac( good_R, 3, 300, 0.2f, std::min( 10, 10 - 2 ) );
  if( res_red.size() != 2 )
    {
    std::cout << "Error in the RANSAC algorithm" << std::endl;
    return;
    }
  cv::Vec3f normal_red = res_red[ 0 ];
  cv::Vec3f A_red = res_red[ 1 ];

  std::vector<cv::Vec3f> res_green = ransac( good_G, 3, 300, 0.2f, std::min( 10, 10 - 2 ) );
  if( res_green.size() != 2 )
    {
    std::cout << "Error in the RANSAC algorithm" << std::endl;
    return;
    }
  cv::Vec3f normal_green = res_green[ 0 ];
  cv::Vec3f A_green = res_green[ 1 ];

  std::vector<cv::Vec3f> res_blue = ransac( good_B, 3, 300, 0.2f, 10 );//, normal_red, normal_green);
  if( res_blue.size() != 2 )
    {
    std::cout << "Error in the RANSAC algorithm" << std::endl;
    return;
    }
  cv::Vec3f normal_blue = res_blue[ 0 ];
  cv::Vec3f A_blue = res_blue[ 1 ];


  cv::Vec3f intersection_circle;
  intersection_circle = three_planes_intersection( normal_blue, normal_green, normal_red, A_blue, A_green, A_red );
  std::cout << "Intersection_circle : " << intersection_circle << std::endl;

  //save_pointcloud_plane_intersection( hires.points, hires.colors, normal_blue, normal_green, normal_red, A_blue, A_green, A_red, intersection_circle, 0.15f, "pointcloud_BGR_plane_circles" );

  std::fstream outputFile;
  outputFile.open( TRACKING_OUT_FILE, std::ios_base::app );
  outputFile << "Intersection_circle : " << intersection_circle << "Normals (RGB)" << normal_red << normal_green << normal_blue << "Time: " << std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() ) << std::endl;
  outputFile.close();

  if( this->ui->drawBox->isChecked() )
    {
    PutImageOnPointCloud( hires, cv::Mat::zeros( 100, 100, CV_8UC3 ), intersection_circle, normal_red, normal_green, normal_blue );
    }

  if( this->ui->drawArt->isChecked() )
    {
    for( int line = 0; line < hires.points.rows; line++ )
      {
      for( int column = 0; column < hires.points.cols; column++ )
        {
        cv::Vec3f point = hires.points.at<cv::Vec3f>( line, column );
        cv::Vec3f displacement = point - intersection_circle;

        float x = displacement.dot( normal_blue );
        float y = displacement.dot( normal_green );
        float z = displacement.dot( normal_red );

        x += 8.5;
        y -= 4.5;
        z += 2;

        if( ( x*x + y*y + z*z ) < 14 )
          {
          hires.colors.at<cv::Vec3b>( line, column ) = cv::Vec3b( 0, 0, 255 );//Image.at<cv::Vec3b>(x, y);
          }
        }
      }
    }

  ProjectPointCloud( hires );
  QCoreApplication::processEvents();
  if( this->ui->loop->isChecked() )
    {
    this->AnalyzeTimer->start();
    }
  else
    {
    this->StopAnalyze();
    }
  return;
}


void MainWindow::StopAnalyze()
{
  this->AnalyzeTimer->stop();

  ui->analyze->setText( "Analyze" );
  disconnect( ui->analyze, SIGNAL( clicked() ), this, SLOT( StopAnalyze() ) );
  connect( ui->analyze, SIGNAL( clicked() ), this, SLOT( on_analyze_clicked() ) );

  return;
}


cv::Point3d MainWindow::approximate_ray_plane_intersection( const cv::Mat & T, const cv::Point3d & vc, const cv::Point3d & vp )
{
  cv::Mat vcMat = cv::Mat( vc );
  cv::Mat vpMat = cv::Mat( vp );

  cv::Mat num = vpMat.t() * ( T );
  cv::Mat denum = vpMat.t()*vcMat;
  double lambda = num.at<double>( 0, 0 ) / denum.at<double>( 0, 0 );

  cv::Point3d p = lambda*vc;

  return p;
}


bool MainWindow::ComputePointCloud( cv::Mat *pointcloud, cv::Mat *pointcloud_colors, cv::Mat mat_color_ref, cv::Mat mat_color, cv::Mat imageTest, cv::Mat color_image, double delay )
{
  cv::Mat mat_BGR;
  cv::Mat mat_gray;
  std::vector<cv::Point2d> cam_points;
  std::vector<cv::Point2d>::iterator it_cam_points;

  int row = 0;
  cv::Point3d p;
  cv::Mat inp1( 1, 1, CV_64FC2 );
  cv::Mat outp1;
  cv::Point3d u1;
  cv::Point3d projectorNormal, cameraVector;
  cv::Mat distortedProjectorPoints( 1, 2, CV_64FC2 );
  cv::Mat undistortedProjectorPoints;
  cv::Point3d projectorVector1;
  cv::Point3d projectorVector2;

  cv::Point3d w2, v2;
  unsigned char sat_max;
  int sum;
  double average;
  cv::Point2i point_max;

  if( !mat_color_ref.data || mat_color_ref.type() != CV_8UC3 || !mat_color.data || mat_color.type() != CV_8UC3 )
    {
    qCritical() << "ERROR invalid cv::Mat data\n";
    return false;
    }

  cv::subtract( mat_color, mat_color_ref, mat_BGR );
  if( !mat_BGR.data || mat_BGR.type() != CV_8UC3 )
    {
    qCritical() << "ERROR invalid cv::Mat data\n";
    return false;
    }

  //Convert the captured frame from BGR to gray
  cv::cvtColor( mat_BGR, mat_gray, cv::COLOR_BGR2GRAY );

  // Looking for the point with th maximum intensity for each column
  for( int j = 0; j < mat_gray.cols; j++ )  //for( int j = mat_gray.cols / 7; j < mat_gray.cols - mat_gray.cols / 7; j++ )
    {
    sum = mat_gray.at< unsigned char >( 0, j ) + mat_gray.at< unsigned char >( 1, j ) + mat_gray.at< unsigned char >( 2, j );
    sat_max = sum;
    point_max = cv::Point2i( 0, 0 );
    for( int i = 2; i < mat_gray.rows - 1; ++i )    //for( int i = 2; i < mat_gray.rows - 1; i++ )
      {
      sum = sum - mat_gray.at< unsigned char >( i - 2, j ) + mat_gray.at< unsigned char >( i + 1, j );
      average = sum / 3;
      if( average > sat_max && average > 60 )
        {
        point_max = cv::Point2i( j, i );
        sat_max = average;
        }
      }
    if( point_max != cv::Point2i( 0, 0 ) )
      {
      double average = 0;
      double COM = 0;
      for( int y = std::max( 0, point_max.y - 3 ); y <= std::min( point_max.y + 3, mat_gray.rows - 1 ); y++ )
        {
        average += mat_gray.at<unsigned char>( y, point_max.x );
        COM += y * mat_gray.at<unsigned char>( y, point_max.x );
        }

      double y = COM / average;
      cam_points.push_back( cv::Point2d( point_max.x, y ) );
      imageTest.at<cv::Vec3b>( point_max ) = { 255, 0, 0 };
      }
    }

  row = ( 1 - delay / .019 ) *this->Projector.GetHeight() ;
  if( row <= 0 || row > this->Projector.GetHeight() )
    {
    std::cout << "The computed row is not valid. The line is skipped. Computed row = " << row << std::endl;
    return false; // We skip the line
    }

  // Computation of the point used to define the plane of the projector
  // to image camera coordinates

  distortedProjectorPoints.at<cv::Vec2d>( 0, 0 ) = cv::Vec2d( this->Projector.GetWidth(), row );
  distortedProjectorPoints.at<cv::Vec2d>( 0, 1 ) = cv::Vec2d( 0, row );

  cv::undistortPoints( distortedProjectorPoints, undistortedProjectorPoints, this->Calib.Proj_K, cv::Mat() );
  assert( undistortedProjectorPoints.type() == CV_64FC2 && undistortedProjectorPoints.rows == 1 && undistortedProjectorPoints.cols == 2 );
  const cv::Vec2d & outvec2 = undistortedProjectorPoints.at<cv::Vec2d>( 0, 0 );
  projectorVector1 = cv::Point3d( outvec2[ 0 ], outvec2[ 1 ], 1.0 );

  const cv::Vec2d & outvec3 = undistortedProjectorPoints.at<cv::Vec2d>( 0, 1 );
  projectorVector2 = cv::Point3d( outvec3[ 0 ], outvec3[ 1 ], 1.0 );

  //find normal of laser plane via cross product of two vectors in plane
  projectorNormal = cv::Point3d( cv::Mat( this->Calib.R*( cv::Mat( projectorVector1 ).cross( cv::Mat( projectorVector2 ) ) ) ) );

  //std::cout << "p1:" << projectorVector1 << "\np2:" << projectorVector2 << "\n pn:" << projectorNormal << std::endl;

  it_cam_points = cam_points.begin();
  for( it_cam_points; it_cam_points != cam_points.end(); ++it_cam_points )
    {
    //to image camera coordinates
    inp1.at<cv::Vec2d>( 0, 0 ) = cv::Vec2d( it_cam_points->x, it_cam_points->y );
    cv::undistortPoints( inp1, outp1, this->Calib.Cam_K, this->Calib.Cam_kc );
    assert( outp1.type() == CV_64FC2 && outp1.rows == 1 && outp1.cols == 1 );
    const cv::Vec2d & outvec1 = outp1.at<cv::Vec2d>( 0, 0 );
    cameraVector = cv::Point3d( outvec1[ 0 ], -outvec1[ 1 ], 1 );
    //to world coordinates

    p = approximate_ray_plane_intersection( this->Calib.T, cameraVector, projectorNormal );

    cv::Vec3f & cloud_point = ( *pointcloud ).at<cv::Vec3f>( ( *it_cam_points ).y, ( *it_cam_points ).x );
    cloud_point[ 0 ] = p.x;
    cloud_point[ 1 ] = -p.y;
    cloud_point[ 2 ] = p.z;

    //std::cout << cloud_point << std::endl;
    double B = mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y - 1, ( *it_cam_points ).x )[ 0 ] + mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y, ( *it_cam_points ).x )[ 0 ] + mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y + 1, ( *it_cam_points ).x )[ 0 ];
    double G = mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y - 1, ( *it_cam_points ).x )[ 1 ] + mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y, ( *it_cam_points ).x )[ 1 ] + mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y + 1, ( *it_cam_points ).x )[ 1 ];
    double R = mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y - 1, ( *it_cam_points ).x )[ 2 ] + mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y, ( *it_cam_points ).x )[ 2 ] + mat_BGR.at<cv::Vec3b>( ( *it_cam_points ).y + 1, ( *it_cam_points ).x )[ 2 ];
    unsigned char vec_B = ( B ) / 3;
    unsigned char vec_G = ( G ) / 3;
    unsigned char vec_R = ( R ) / 3;

    cv::Vec3b & cloud_color = ( *pointcloud_colors ).at<cv::Vec3b>( ( *it_cam_points ).y, ( *it_cam_points ).x );
    cloud_color[ 0 ] = vec_B;
    cloud_color[ 1 ] = vec_G;
    cloud_color[ 2 ] = vec_R;
    color_image.at<cv::Vec3b>( ( *it_cam_points ).y, ( *it_cam_points ).x ) = cv::Vec3b{ vec_B, vec_G, vec_R };

    if( row < 780 && row > 395 )
      {
      imageTest.at<cv::Vec3b>( ( *it_cam_points ).y, ( *it_cam_points ).x ) = { 0, 255, 0 };
      }
    else
      {
      imageTest.at<cv::Vec3b>( ( *it_cam_points ).y, ( *it_cam_points ).x ) = { 255, 255, 255 };
      }
    }

  return true;
}


std::vector<cv::Vec3f> MainWindow::ransac( std::vector<cv::Vec3f> points, int min, int iter, float thres, int min_inliers, const cv::Vec3f normal_B, const cv::Vec3f normal_R )
/*  min  the minimum number of data values required to fit the model
    iter  the maximum number of iterations allowed in the algorithm
    thres  a threshold value for determining when a data point fits a model
    min_inliers  the number of close data values required to assert that a model fits well to data
    normal_B, normal_R  if specified, normals to which the computed plan must be orthogonal
    Returns a vector of 2 elements : the normal and a point of the computed plane */
{
  std::vector<cv::Vec3f> res;
  int n = static_cast< int >( points.size() );
  if( n < 3 )
    {
    std::cerr << "At least 3 points required" << std::endl;
    return res;
    }
  cv::Vec3f normal = cv::Vec3f( 0, 0, 0 );
  float orthogonal = 0.001f;

  int idx1, idx2, idx3;
  std::vector<cv::Vec3f> sample;
  cv::Vec3f A, B, C, AB, AC;
  cv::Vec3f crt_vec, crt_normal = ( 0, 0, 0 ), best_normal = ( 0, 0, 0 );
  float distance;
  float distFromA;
  int inliers, best_inliers = 0;
  cv::Vec3f best_A = ( 0, 0, 0 );
  // initialize random seed :
  srand( time( 0 ) );

  double cube_diagonal = 3; //cm
  for( int i = 0; i < iter; i++ )
    {
    // Select 3 points randomly
    sample.clear();
    idx1 = rand() % n;
    sample.push_back( points[ idx1 ] );
    A = points[ idx1 ];
    do
      {
      idx2 = rand() % n;
      } while( idx2 == idx1 );
      sample.push_back( points[ idx2 ] );
      B = points[ idx2 ];
      do
        {
        idx3 = rand() % n;
        } while( idx3 == idx1 || idx3 == idx2 );
        sample.push_back( points[ idx3 ] );
        C = points[ idx3 ];

        AB = B - A;
        AC = C - A;
        crt_normal = AB.cross( AC );

        inliers = 0;
        if( sqrt( ( B - A ).dot( B - A ) ) < cube_diagonal && sqrt( ( C - A ).dot( C - A ) ) < cube_diagonal && sqrt( ( B - C ).dot( B - C ) ) < cube_diagonal )
          {
          for( auto crt_point = points.begin(); crt_point != points.end(); crt_point++ )
            {
            crt_vec = *crt_point - A;
            distance = std::abs( crt_normal.dot( crt_vec ) );
            distance = distance / sqrt( crt_normal.dot( crt_normal ) );

            distFromA = sqrt( crt_vec.dot( crt_vec ) );
            if( distance < thres && distFromA < cube_diagonal )
              {
              inliers++;
              }
            }
          }
        if( inliers >= min_inliers && inliers > best_inliers && abs( crt_normal.dot( normal_B ) ) < orthogonal && abs( crt_normal.dot( normal_R ) ) < orthogonal ) //We may have found a good model
          {
          best_inliers = inliers;
          best_normal = crt_normal;
          best_A = A;
          }
    }
  //std::cout << "Ransac : best_inliers : " << best_inliers << std::endl;

  best_normal /= sqrt( best_normal.dot( best_normal ) );

  if( best_normal[ 2 ] > 0 )
    {
    best_normal *= -1;
    }
  res.push_back( best_normal );
  res.push_back( best_A );
  return res;
}


void MainWindow::NNDensityProbabilityReplacement( cv::Mat pointcloud, cv::Mat pointcloud_BGR, std::vector<cv::Vec3f> *points_B, std::vector<cv::Vec3f> *points_G, std::vector<cv::Vec3f> *points_R, double threshold_unused )
{
  cv::Mat pt_BGR = pointcloud_BGR.clone();

  double sum_B = 0, sum_G = 0, sum_R = 0;
  int nb_B = 0, nb_G = 0, nb_R = 0;

  float max_x_R = -9999, min_x_R = 9999;
  float max_y_R = -9999, min_y_R = 9999;

  // we don't take into account the 2 pixels on the borders
  //for( int row = this->CamInput.GetTopLine(); row < this->CamInput.GetBottomLine(); row++ )

  int prevclass = 3;
  for( int row = 2; row < pointcloud_BGR.rows - 2; row++ )
    {
    for( int col = 2; col < pointcloud_BGR.cols - 2; col++ )
      {
      cv::Vec3f crt = pointcloud.at<cv::Vec3f>( row, col );
      cv::Vec3b crt_BGR = pointcloud_BGR.at<cv::Vec3b>( row, col );
      if( crt[ 2 ] > -9999 ) // valid points in the point cloud
        {
        if( crt[ 0 ] > this->max_x )
          {
          this->max_x = crt[ 0 ];
          }
        if( crt[ 0 ] < this->min_x )
          {
          this->min_x = crt[ 0 ];
          }
        if( crt[ 1 ] > this->max_y )
          {
          this->max_y = crt[ 1 ];
          }
        if( crt[ 1 ] < this->min_y )
          {
          this->min_y = crt[ 1 ];
          }
        if( crt[ 2 ] > this->max_z )
          {
          this->max_z = crt[ 2 ];
          }
        if( crt[ 2 ] < this->min_z )
          {
          this->min_z = crt[ 2 ];
          }
          //cv::Mat res = (cv::Mat_<double>(1, 4) << crt_BGR[2], crt_BGR[1], crt_BGR[0], 120);
        cv::Mat res = NNClassifyColors( ( cv::Mat_<double>( 1, 3 ) << crt_BGR[ 2 ], crt_BGR[ 1 ], crt_BGR[ 0 ] ) );
        res.at<double>( 3 ) += .3;
        res.at<double>( 2 ) -= .1;
        int mostLikely = 0;
        double maxval = -999999;
        for( int i = 0; i < 4; i++ )
          {
          if( res.at<double>( i ) > maxval )
            {
            mostLikely = i;
            maxval = res.at<double>( i );
            }
          }
          //0 = red, 1 = green, 2 = blue, 3 = background

        if( mostLikely != 3 )// && mostLikely == prevclass)
          {
          if( mostLikely == 1 )
            {
            pt_BGR.at<cv::Vec3b>( row, col ) = cv::Vec3b( 0, 255, 0 );
            ( *points_G ).push_back( crt );

            nb_G++;
            }
          else if( mostLikely == 2 )
            {
            pt_BGR.at<cv::Vec3b>( row, col ) = cv::Vec3b( 255, 0, 0 );
            ( *points_B ).push_back( crt );

            nb_B++;
            if( crt[ 0 ] > max_x_R )
              {
              max_x_R = crt[ 0 ];
              }
            if( crt[ 0 ] < min_x_R )
              {
              min_x_R = crt[ 0 ];
              }
            if( crt[ 1 ] > max_y_R )
              {
              max_y_R = crt[ 1 ];
              }
            if( crt[ 1 ] < min_y_R )
              {
              min_y_R = crt[ 1 ];
              }
            }
          else if( mostLikely == 0 )
            {
            pt_BGR.at<cv::Vec3b>( row, col ) = cv::Vec3b( 0, 0, 255 );
            ( *points_R ).push_back( crt );
            nb_R++;
            }
          }
        else
          {
          pt_BGR.at<cv::Vec3b>( row, col ) = cv::Vec3b( 255, 255, 255 );
          }
        prevclass = mostLikely;
        }
      }
    }
  if( this->ui->savePointClouds->isChecked() )
    {
    save_pointcloud( pointcloud, pt_BGR, "pointcloud_BGR_BGR" );
    }
  sum_B = sum_B / nb_B;
  sum_G = sum_G / nb_G;
  sum_R = sum_R / nb_R;
  std::cout << "blue sum = " << sum_B << std::endl;
  std::cout << "green sum = " << sum_G << std::endl;
  std::cout << "red sum = " << sum_R << std::endl;

  std::cout << "min_x_R = " << min_x_R << std::endl;
  std::cout << "max_x_R = " << max_x_R << std::endl;
  std::cout << "min_y_R = " << min_y_R << std::endl;
  std::cout << "max_y_R = " << max_y_R << std::endl;
}


void MainWindow::density_probability( cv::Mat pointcloud, cv::Mat pointcloud_BGR, std::vector<cv::Vec3f> *points_B, std::vector<cv::Vec3f> *points_G, std::vector<cv::Vec3f> *points_R, double threshold )
{
  cv::Mat pt_BGR = pointcloud_BGR.clone();
  typedef itk::Vector< unsigned char, 3 > MeasurementVectorType;
  typedef itk::Statistics::GaussianMembershipFunction< MeasurementVectorType >
    DensityFunctionType;

  // BGR - Green - curved
  DensityFunctionType::Pointer densityFunction_G_BGR = DensityFunctionType::New();
  densityFunction_G_BGR->SetMeasurementVectorSize( 3 );
  DensityFunctionType::MeanVectorType mean_G_BGR( 3 );
  mean_G_BGR[ 0 ] = 89.98476454293629;// 53.91532061885764;
  mean_G_BGR[ 1 ] = 113.5203139427516;// 65.79425537608252;
  mean_G_BGR[ 2 ] = 69.0803324099723;// 44.53785151308747;
  DensityFunctionType::CovarianceMatrixType cov_G_BGR;
  cov_G_BGR.SetSize( 3, 3 );
  cov_G_BGR[ 0 ][ 0 ] = 159.8986598476079;// 32.87146616410637;
  cov_G_BGR[ 0 ][ 1 ] = 120.4950001662561;// 26.84658224589317;
  cov_G_BGR[ 0 ][ 2 ] = 89.770845322959;// 15.02046465692957;
  cov_G_BGR[ 1 ][ 0 ] = 120.4950001662561;// 26.84658224589317;
  cov_G_BGR[ 1 ][ 1 ] = 166.0926159679223;// 37.15121420178843;
  cov_G_BGR[ 1 ][ 2 ] = 111.4628187322072;// 18.63856216879524;
  cov_G_BGR[ 2 ][ 0 ] = 89.770845322959;// 15.02046465692957;
  cov_G_BGR[ 2 ][ 1 ] = 111.4628187322072;// 18.63856216879524;
  cov_G_BGR[ 2 ][ 2 ] = 109.2779419024306;// 17.51177637067411;
  densityFunction_G_BGR->SetMean( mean_G_BGR );
  densityFunction_G_BGR->SetCovariance( cov_G_BGR );
  //std::cout << "Green mean BGR : " << mean_G_BGR << std::endl;
  //std::cout << "Green covariance BGR : " << cov_G_BGR << std::endl;

  // BGR - Blue - curved
  DensityFunctionType::Pointer densityFunction_B_BGR = DensityFunctionType::New();
  densityFunction_B_BGR->SetMeasurementVectorSize( 3 );
  DensityFunctionType::MeanVectorType mean_B_BGR( 3 );
  mean_B_BGR[ 0 ] = 162.790273556231;// 81.12688848920864;
  mean_B_BGR[ 1 ] = 69.31408308004053;// 46.22345623501199;
  mean_B_BGR[ 2 ] = 59.89260385005066;// 32.8949340527578;
  DensityFunctionType::CovarianceMatrixType cov_B_BGR;
  cov_B_BGR.SetSize( 3, 3 );
  cov_B_BGR[ 0 ][ 0 ] = 247.0512529140221;// 51.13153120578004;
  cov_B_BGR[ 0 ][ 1 ] = 23.33132238862042;// 18.96356743876536;
  cov_B_BGR[ 0 ][ 2 ] = 9.271295842918425;// 11.71003429720219;
  cov_B_BGR[ 1 ][ 0 ] = 23.33132238862042;// 18.96356743876536;
  cov_B_BGR[ 1 ][ 1 ] = 18.81523226462756;// 15.15898517674382;
  cov_B_BGR[ 1 ][ 2 ] = 5.455210543550453;// 12.24514280886434;
  cov_B_BGR[ 2 ][ 0 ] = 9.271295842918425;// 11.71003429720219;
  cov_B_BGR[ 2 ][ 1 ] = 5.455210543550453;// 12.24514280886434;
  cov_B_BGR[ 2 ][ 2 ] = 26.2255481338454;// 15.71471054720592;
  densityFunction_B_BGR->SetMean( mean_B_BGR );
  densityFunction_B_BGR->SetCovariance( cov_B_BGR );
  //std::cout << "Blue mean BGR : " << mean_B_BGR << std::endl;
  //std::cout << "Blue covariance BGR : " << cov_B_BGR << std::endl;

  // BGR - Red - curved
  DensityFunctionType::Pointer densityFunction_R_BGR = DensityFunctionType::New();
  densityFunction_R_BGR->SetMeasurementVectorSize( 3 );
  DensityFunctionType::MeanVectorType mean_R_BGR( 3 );
  mean_R_BGR[ 0 ] = 55.29753265602322;// 37.69092824226465;
  mean_R_BGR[ 1 ] = 65.80188679245283;// 46.39889400921659;
  mean_R_BGR[ 2 ] = 210.0304789550073;// 116.4342857142857;
  DensityFunctionType::CovarianceMatrixType cov_R_BGR;
  cov_R_BGR.SetSize( 3, 3 );
  cov_R_BGR[ 0 ][ 0 ] = 88.49347722135754;// 35.28504081051287;
  cov_R_BGR[ 0 ][ 1 ] = 27.61482323301476;// 29.05505777448908;
  cov_R_BGR[ 0 ][ 2 ] = 44.47569203806028;// 43.80816883288441;
  cov_R_BGR[ 1 ][ 0 ] = 27.61482323301476;// 29.05505777448908;
  cov_R_BGR[ 1 ][ 1 ] = 41.77134622230733;// 28.58625552464882;
  cov_R_BGR[ 1 ][ 2 ] = 70.2651094011009;// 42.04286214615217;
  cov_R_BGR[ 2 ][ 0 ] = 44.47569203806028;// 43.80816883288441;
  cov_R_BGR[ 2 ][ 1 ] = 70.2651094011009;// 42.04286214615217;
  cov_R_BGR[ 2 ][ 2 ] = 343.3067633409943;// 93.23570138241101;
  densityFunction_R_BGR->SetMean( mean_R_BGR );
  densityFunction_R_BGR->SetCovariance( cov_R_BGR );
  //std::cout << "Red mean BGR : " << mean_R_BGR << std::endl;
  //std::cout << "Red covariance BGR : " << cov_R_BGR << std::endl;

  MeasurementVectorType mv_BGR;
  mv_BGR.Fill( 0 );

  double res_BGR = 0;
  double res_BGR_G = 0;
  double res_BGR_B = 0;
  double res_BGR_R = 0;

  double sum_B = 0, sum_G = 0, sum_R = 0;
  int nb_B = 0, nb_G = 0, nb_R = 0;

  float max_x_R = -9999, min_x_R = 9999;
  float max_y_R = -9999, min_y_R = 9999;

  // we don't take into account the 2 pixels on the borders
  //for( int row = this->CamInput.GetTopLine(); row < this->CamInput.GetBottomLine(); row++ )
  for( int row = 2; row < pointcloud_BGR.rows - 2; row++ )
    {
    for( int col = 2; col < pointcloud_BGR.cols - 2; col++ )
      {
      cv::Vec3f crt = pointcloud.at<cv::Vec3f>( row, col );
      cv::Vec3b crt_BGR = pointcloud_BGR.at<cv::Vec3b>( row, col );
      if( crt[ 2 ] > 0 ) // valid points in the point cloud
        {
        if( crt[ 0 ] > this->max_x )
          {
          this->max_x = crt[ 0 ];
          }
        if( crt[ 0 ] < this->min_x )
          {
          this->min_x = crt[ 0 ];
          }
        if( crt[ 1 ] > this->max_y )
          {
          this->max_y = crt[ 1 ];
          }
        if( crt[ 1 ] < this->min_y )
          {
          this->min_y = crt[ 1 ];
          }
        if( crt[ 2 ] > this->max_z )
          {
          this->max_z = crt[ 2 ];
          }
        if( crt[ 2 ] < this->min_z )
          {
          this->min_z = crt[ 2 ];
          }

        mv_BGR[ 0 ] = crt_BGR[ 0 ];
        mv_BGR[ 1 ] = crt_BGR[ 1 ];
        mv_BGR[ 2 ] = crt_BGR[ 2 ];

        res_BGR_G = densityFunction_G_BGR->Evaluate( mv_BGR );
        res_BGR_B = densityFunction_B_BGR->Evaluate( mv_BGR );
        res_BGR_R = densityFunction_R_BGR->Evaluate( mv_BGR );

        res_BGR = std::max( { res_BGR_G, res_BGR_B, res_BGR_R } );

        //if( res_BGR > 5e-94 )
        if( res_BGR > threshold )
          {
          if( res_BGR == res_BGR_G )
            {
            pt_BGR.at<cv::Vec3b>( row, col ) = cv::Vec3b( 0, 255, 0 );
            ( *points_G ).push_back( crt );
            sum_G += res_BGR;
            nb_G++;
            }
          else if( res_BGR == res_BGR_B )
            {
            pt_BGR.at<cv::Vec3b>( row, col ) = cv::Vec3b( 255, 0, 0 );
            ( *points_B ).push_back( crt );
            sum_B += res_BGR;
            nb_B++;
            if( crt[ 0 ] > max_x_R )
              {
              max_x_R = crt[ 0 ];
              }
            if( crt[ 0 ] < min_x_R )
              {
              min_x_R = crt[ 0 ];
              }
            if( crt[ 1 ] > max_y_R )
              {
              max_y_R = crt[ 1 ];
              }
            if( crt[ 1 ] < min_y_R )
              {
              min_y_R = crt[ 1 ];
              }
            }
          else if( res_BGR == res_BGR_R )
            {
            pt_BGR.at<cv::Vec3b>( row, col ) = cv::Vec3b( 0, 0, 255 );
            ( *points_R ).push_back( crt );
            sum_R += res_BGR;
            nb_R++;
            }
          }
        else
          {
          pt_BGR.at<cv::Vec3b>( row, col ) = cv::Vec3b( 255, 255, 255 );
          }
        }
      }
    }

  save_pointcloud( pointcloud, pt_BGR, "pointcloud_BGR_BGR" );

  sum_B = sum_B / nb_B;
  sum_G = sum_G / nb_G;
  sum_R = sum_R / nb_R;
  std::cout << "blue sum = " << sum_B << std::endl;
  std::cout << "green sum = " << sum_G << std::endl;
  std::cout << "red sum = " << sum_R << std::endl;

  std::cout << "min_x_R = " << min_x_R << std::endl;
  std::cout << "max_x_R = " << max_x_R << std::endl;
  std::cout << "min_y_R = " << min_y_R << std::endl;
  std::cout << "max_y_R = " << max_y_R << std::endl;
}


cv::Vec3f MainWindow::three_planes_intersection( cv::Vec3f n1, cv::Vec3f n2, cv::Vec3f n3, cv::Vec3f x1, cv::Vec3f x2, cv::Vec3f x3 )
// Input : 3 planes defined by their nrmal n and a point x
{
  cv::Mat mat;
  cv::hconcat( n1, n2, mat );
  cv::hconcat( mat, n3, mat );

  float det = cv::determinant( mat );
  std::cout << "det : " << det << std::endl;
  if( std::abs( det ) < 1e-20 )
    {
    std::cout << "2 planes are parallel" << std::endl;
    return cv::Vec3f( 0, 0, 0 );
    }
  cv::Vec3f a, b, c;
  a = ( x1.dot( n1 ) )*( n2.cross( n3 ) );
  b = ( x2.dot( n2 ) )*( n3.cross( n1 ) );
  c = ( x3.dot( n3 ) )*( n1.cross( n2 ) );

  return ( 1 / det )*( a + b + c );
}


float MainWindow::compute_maximum( std::vector<cv::Vec3f> points, int axis, float min, float max, float variance, float interval_min, float interval_max )
{
  if( axis != 0 && axis != 1 && axis != 2 )
    {
    std::cout << "Error in the dimension chosen to compute the maximum" << std::endl;
    return 0;
    }

  int scale = 100;

  min *= scale;
  max *= scale;

  typedef itk::Image< float, 1 > FloatHistogramType;
  typedef itk::DiscreteGaussianImageFilter<FloatHistogramType, FloatHistogramType> FilterType;


  FloatHistogramType::RegionType region;
  FloatHistogramType::IndexType start;
  start[ 0 ] = 0;
  FloatHistogramType::SizeType size;
  size[ 0 ] = std::abs( max - min ) + 1;
  region.SetSize( size );
  region.SetIndex( start );

  FloatHistogramType::Pointer histogram = FloatHistogramType::New();
  histogram->SetRegions( region );
  histogram->Allocate();
  histogram->FillBuffer( 0 );

  FloatHistogramType::IndexType pixelIndex;

  for( auto iter = points.cbegin(); iter != points.cend(); ++iter )
    {
    if( ( *iter )[ 0 ] >= interval_min && ( *iter )[ 0 ] <= interval_max )
      {
      pixelIndex[ 0 ] = floor( ( *iter )[ axis ] * scale - min );
      histogram->SetPixel( pixelIndex, histogram->GetPixel( pixelIndex ) + 1 );

      }
    }

  FilterType::Pointer gaussianFilter = FilterType::New();
  gaussianFilter->SetInput( histogram );
  gaussianFilter->SetVariance( variance );
  gaussianFilter->Update();

  FloatHistogramType::Pointer result = FloatHistogramType::New();
  result = gaussianFilter->GetOutput();

  typedef itk::MinimumMaximumImageCalculator <FloatHistogramType> ImageCalculatorFilterType;
  ImageCalculatorFilterType::Pointer imageCalculatorFilter = ImageCalculatorFilterType::New();
  imageCalculatorFilter->SetImage( gaussianFilter->GetOutput() );
  imageCalculatorFilter->ComputeMaximum();

  FloatHistogramType::IndexType maximum = imageCalculatorFilter->GetIndexOfMaximum();
  return ( maximum[ 0 ] + min ) / 100;
}


void MainWindow::save_pointcloud_plane_intersection( cv::Mat pointcloud, cv::Mat pointcloud_colors, cv::Vec3f normal_B, cv::Vec3f normal_G, cv::Vec3f normal_R, cv::Vec3f A_B, cv::Vec3f A_G, cv::Vec3f A_R, cv::Vec3f intersection, float size_circles, QString name )
{
  // Display the 3 planes and the intersection point
  float dist_B, dist_G, dist_R, dist_intersection;
  for( int row = 0; row < pointcloud.rows; row++ )
    {
    for( int col = 0; col < pointcloud.cols; col++ )
      {
      cv::Vec3f crt = pointcloud.at<cv::Vec3f>( row, col );
      if( crt[ 2 ] > 0 )
        {
        cv::Vec3f vec_B = pointcloud.at<cv::Vec3f>( row, col ) - A_B;
        cv::Vec3f vec_G = pointcloud.at<cv::Vec3f>( row, col ) - A_G;
        cv::Vec3f vec_R = pointcloud.at<cv::Vec3f>( row, col ) - A_R;

        dist_B = std::abs( normal_B.dot( vec_B ) ) / sqrt( normal_B.dot( normal_B ) );
        dist_G = std::abs( normal_G.dot( vec_G ) ) / sqrt( normal_G.dot( normal_G ) );
        dist_R = std::abs( normal_R.dot( vec_R ) ) / sqrt( normal_R.dot( normal_R ) );
        dist_intersection = std::sqrt( pow( intersection[ 0 ] - crt[ 0 ], 2 ) + pow( intersection[ 1 ] - crt[ 1 ], 2 ) + pow( intersection[ 2 ] - crt[ 2 ], 2 ) );
        if( dist_intersection < size_circles * 5 )
          {
          pointcloud_colors.at<cv::Vec3b>( row, col ) = cv::Vec3f( 0, 255, 255 );
          }
        else if( dist_B < size_circles )
          {
          pointcloud_colors.at<cv::Vec3b>( row, col ) = cv::Vec3f( 255, 0, 0 );
          }
        else if( dist_G < size_circles )
          {
          pointcloud_colors.at<cv::Vec3b>( row, col ) = cv::Vec3f( 0, 255, 0 );
          }
        else if( dist_R < size_circles )
          {
          pointcloud_colors.at<cv::Vec3b>( row, col ) = cv::Vec3f( 0, 0, 255 );
          }
        else
          {
          pointcloud_colors.at<cv::Vec3b>( row, col ) = cv::Vec3f( 255, 255, 255 );
          }
        }
      }
    }
  //save_pointcloud( pointcloud, pointcloud_colors, name );
}


void MainWindow::save_pointcloud_centers( cv::Mat pointcloud, cv::Mat pointcloud_colors, cv::Vec3f center_B, cv::Vec3f center_G, cv::Vec3f center_R, float size_circles, QString name )
{
  // Display the zones where the colored points are taken
  float dist_B, dist_G, dist_R;
  for( int row = 0; row < pointcloud.rows; row++ )
    {
    for( int col = 0; col < pointcloud.cols; col++ )
      {
      cv::Vec3f crt = pointcloud.at<cv::Vec3f>( row, col );
      if( crt[ 2 ] > -9999 )
        {
        dist_B = std::sqrt( pow( center_B[ 0 ] - crt[ 0 ], 2 ) + pow( center_B[ 1 ] - crt[ 1 ], 2 ) + pow( center_B[ 2 ] - crt[ 2 ], 2 ) );
        dist_R = std::sqrt( pow( center_R[ 0 ] - crt[ 0 ], 2 ) + pow( center_R[ 1 ] - crt[ 1 ], 2 ) + pow( center_R[ 2 ] - crt[ 2 ], 2 ) );
        dist_G = std::sqrt( pow( center_G[ 0 ] - crt[ 0 ], 2 ) + pow( center_G[ 1 ] - crt[ 1 ], 2 ) + pow( center_G[ 2 ] - crt[ 2 ], 2 ) );
        pointcloud_colors.at<cv::Vec3b>( row, col ) = cv::Vec3b( 0, 0, 0 );
        if( dist_B < size_circles )
          {
          pointcloud_colors.at<cv::Vec3b>( row, col ) += cv::Vec3b( 255, 0, 0 );
          }
        if( dist_R < size_circles )
          {
          pointcloud_colors.at<cv::Vec3b>( row, col ) += cv::Vec3b( 0, 0, 255 );
          }
        if( dist_G < size_circles )
          {
          pointcloud_colors.at<cv::Vec3b>( row, col ) += cv::Vec3b( 0, 255, 0 );
          }
        }
      }
    }
  save_pointcloud( pointcloud, pointcloud_colors, name );
}


void MainWindow::save_pointcloud( cv::Mat pointcloud, cv::Mat pointcloud_colors, QString name )
{
  QString namefile = POINTCLOUD_FOLDER + name;
  //QString filename = QFileDialog::getSaveFileName( this, "Save pointcloud", namefile + ".ply", "Pointclouds (*.ply)" );
  QString filename = namefile + ".ply";
  std::cout << filename.toStdString() << std::endl;
  if( !filename.isEmpty() )
    {
    std::cout << "Saving the pointcloud" << std::endl;
    bool binary = false;
    bool success = io_util::write_ply( filename.toStdString(), pointcloud, pointcloud_colors );
    if( success == false )
      {
      qCritical() << "ERROR, saving the pointcloud failed\n";
      return;
      }
    }
}


void MainWindow::get_true_colors( cv::Mat *pointcloud_colors )
{
  cv::Mat color_image = cv::Mat::zeros( ( *pointcloud_colors ).rows, ( *pointcloud_colors ).cols, CV_8UC3 );

  unsigned char blue_proj = this->Projector.GetBlueColor();
  unsigned char green_proj = this->Projector.GetGreenColor();
  unsigned char red_proj = this->Projector.GetRedColor();
  std::cout << "blue, green, red = " << int( blue_proj ) << " " << int( green_proj ) << " " << int( red_proj ) << std::endl;

  unsigned char min = std::min( { blue_proj, green_proj, red_proj } );
  unsigned char max = std::max( { blue_proj, green_proj, red_proj } );
  std::cout << "min = " << int( min ) << " max = " << int( max ) << std::endl;

  float coef = float( max - min ) / float( max + min );

  std::cout << 1 - float( coef*blue_proj / 800 ) << std::endl;
  std::cout << ( 1 / float( blue_proj ) ) << std::endl;
  std::cout << ( 1 / float( blue_proj ) ) * 1500 << std::endl;

  for( int row = 0; row < ( *pointcloud_colors ).rows; row++ )
    {
    for( int col = 0; col < ( *pointcloud_colors ).cols; col++ )
      {
      cv::Vec3b & crt = ( *pointcloud_colors ).at<cv::Vec3b>( row, col );

      if( crt != cv::Vec3b( 0, 0, 0 ) )
        {
        //std::cout << "coef = " << coef << std::endl;
        //std::cout << "crt before = " << int( crt[ 0 ] ) << " " << int( crt[ 1 ] ) << " " << int( crt[ 2 ] ) << std::endl;
        crt[ 0 ] = ( 1 - float( coef*blue_proj / 800 ) ) * crt[ 0 ] + ( 1 / float( blue_proj ) ) * 1500;
        crt[ 1 ] = ( 1 - float( coef*green_proj / 800 ) ) * crt[ 1 ] + ( 1 / float( green_proj ) ) * 1500;
        crt[ 2 ] = ( 1 - float( coef*red_proj / 800 ) ) * crt[ 2 ] + ( 1 / float( red_proj ) ) * 1500;
        //std::cout << "crt after = " << int( crt[ 0 ] ) << " " << int( crt[ 1 ] ) << " " << int( crt[ 2 ] ) << std::endl;
        color_image.at<cv::Vec3b>( row, col ) = crt;
        }
      }
    }
  QString imagename = QString( "C:\\Camera_Projector_Calibration\\Tests_publication\\color_image_true_colors.png" );
  cv::imwrite( qPrintable( imagename ), color_image );
}
