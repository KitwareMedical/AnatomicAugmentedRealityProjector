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

#include "ProjectorWidget.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QApplication>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QPainter>

#include <iostream>

ProjectorWidget::ProjectorWidget( QWidget * parent, Qt::WindowFlags flags ) :
  QWidget( parent, flags ),
  Height( 1080 ),
  Width( 1920 ),
  LineThickness( 1 ),
  Row( 100 ),
  BlueColor( 255 ),
  GreenColor( 255 ),
  RedColor( 255 )
{
}


ProjectorWidget::~ProjectorWidget()
{
}


// /!\ Use of usigned char to code the color between 0 (black) and 255 (white)
cv::Mat ProjectorWidget::CreateLineImage()
{
  cv::Mat image = cv::Mat::zeros( this->Height, this->Width, CV_8UC1 ); // use CV_32S for int
  int lmax = 5 ;
  for( int j = 0; j < image.rows; j++ )
    {
    for( int i = 0; i < image.cols; i = i + 2 * lmax )
      {
      for( int l = 0; l < lmax; l++ )
        {
        image.at<unsigned char>( j, i + l ) = 255;
        }
      }
    }
  return image;
}


cv::Mat ProjectorWidget::CreatePattern() // Create a rainbow
{
  cv::Mat image = cv::Mat::zeros( this->Height, this->Width, CV_8UC3 );
  for( int j = 0; j < image.rows; j++ )
    {
    float color = ( j + 1 ) * 180 / this->GetHeight();
    float row = color*this->GetHeight() / 180;
    std::cout << "color : " << color << " row : " << row << std::endl;
    for( int i = 0; i < image.cols; i++ )
      {
      image.at<cv::Vec3b>( j, i ) = { unsigned char( j * 180 / this->GetHeight() ), 255, 255 };
      }
    }
  cv::cvtColor( image, image, cv::COLOR_HSV2BGR );
  return image;
}


cv::Mat ProjectorWidget::CreateColoredImage( int blue, int green, int red )
{
  cv::Mat image = cv::Mat( this->Height, this->Width, CV_8UC3, { double( blue ), double( green ), double( red ) } );
  return image;
}


std::vector<cv::Point2i> ProjectorWidget::GetCoordLine( cv::Mat image )
{
    // TODO: condition on type of matrix
  std::vector<cv::Point2i> coord;
  for( int i = 0; i < image.rows; i++ )
    {
    unsigned char *row = image.ptr<unsigned char>( i );
    for( int j = 0; j < image.cols; j++ )
      {
      if( ( int )row[ j ] != 0 ) // or =255
        {
        coord.push_back( cv::Point2i( j, i ) );
        }
      }
    }

  return coord;
}


void ProjectorWidget::paintEvent( QPaintEvent * )
{
  QPainter painter( this );

  if( !this->Pixmap.isNull() )
    {
    QRectF rect = QRectF( QPointF( 0, 0 ), QPointF( width(), height() ) );
    painter.drawPixmap( rect, this->Pixmap, rect );
    emit new_image( this->Pixmap );
    }
  else
    {
    QRectF rect = QRectF( QPointF( 0, 0 ), QPointF( width(), height() ) );
    painter.drawText( rect, Qt::AlignCenter, "No image" );
    }
}


void ProjectorWidget::start()
{
  QDesktopWidget * desktop = QApplication::desktop();
  int screen = desktop->screenCount();
  if( screen == 1 )
    {
    QMessageBox msgBox;
    msgBox.setWindowTitle( "Warning" );
    msgBox.setText( "Only one screen was detected. Are you sure you want to continue ?" );
    msgBox.setWindowFlags( Qt::WindowStaysOnTopHint );
    msgBox.setStandardButtons( QMessageBox::Yes );
    msgBox.addButton( QMessageBox::No );
    msgBox.setDefaultButton( QMessageBox::No );
    if( msgBox.exec() == QMessageBox::No )
      {
      std::cout << "Pattern not displayed. Connect a projector and try again." << std::endl;
      return;
      }
    }
  // We choose the last screen added (highest number) = the projector
  //display
  QRect screen_resolution = desktop->screenGeometry( screen - 1 );
  move( QPoint( screen_resolution.x(), screen_resolution.y() ) );
  showFullScreen();
  QApplication::processEvents();
}
