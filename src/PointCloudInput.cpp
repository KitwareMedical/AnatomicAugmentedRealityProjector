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
#include "PointCloudInput.hpp"
#include "ProjectorWidget.hpp"

#include "FlyCapture2.h"

#include "itkImage.h"
#include "itkVector.h"
#include "itkGaussianMembershipFunction.h"
#include "itkDiscreteGaussianImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <time.h>


PointCloudInput::PointCloudInput( CameraInput *CamInput, ProjectorWidget *Projector, CalibrationData *Calib ) :
  CamInput( CamInput ), Projector( Projector ), Calib( Calib )
{
}


PointCloudInput::~PointCloudInput()
{
}


PointCloud PointCloudInput::ComputePointCloud( int numrows )
{
  CamInput->SetCameraTriggerDelay( .014 );
  cv::Mat mat_color_ref = this->CamInput->GetImageFromBuffer();

  double max_delay = .0119;
  double min_delay = 0.0;
  double delta = ( max_delay - min_delay ) / numrows;


  cv::Mat pointcloud = cv::Mat::zeros( numrows, mat_color_ref.cols, CV_32FC3 );
  cv::Mat pointcloud_colors = cv::Mat( numrows, mat_color_ref.cols, CV_8UC3 );

  /***********************3D Reconstruction of every lines****************************/
  std::cout << "Start : 3D reconstruction of every line" << std::endl;
  // imageTest is used to control which points have been used on the projector for the reconstruction
  cv::Mat imageTest = cv::Mat::zeros( mat_color_ref.rows, mat_color_ref.cols, CV_8UC3 );
  cv::Mat crt_mat;
  cv::Mat color_image = cv::Mat::zeros( mat_color_ref.rows, mat_color_ref.cols, CV_8UC3 );

  this->CamInput->SetCameraTriggerDelay( 0 );
  for( int depth_map_row = 0; depth_map_row < numrows; depth_map_row++ )
    {
    crt_mat = this->CamInput->GetImageFromBuffer();
    double nextDelay = ( depth_map_row + 1 ) * delta + min_delay;
    // Begin changing the delay for the next image before processing the current image. There are silent errors
    // If you take a picture too soon after changing the delay
    this->CamInput->SetCameraTriggerDelay( nextDelay );
    ComputePointCloudRow( &pointcloud, &pointcloud_colors, mat_color_ref, crt_mat, imageTest, color_image, nextDelay - delta, depth_map_row );
    }

  //QString imagename;
  //imagename = QString( "C:\\Camera_Projector_Calibration\\Tests_publication\\color_image.png" );
  //cv::imwrite( qPrintable( imagename ), color_image );

  std::cout << "End : 3D reconstruction of every line" << std::endl;
  return{ pointcloud, pointcloud_colors };
}


cv::Point3d PointCloudInput::approximate_ray_plane_intersection( const cv::Mat & T, const cv::Point3d & vc, const cv::Point3d & vp )
{
  cv::Mat vcMat = cv::Mat( vc );
  cv::Mat vpMat = cv::Mat( vp );

  cv::Mat num = vpMat.t() * ( T );
  cv::Mat denum = vpMat.t()*vcMat;
  double lambda = num.at<double>( 0, 0 ) / denum.at<double>( 0, 0 );

  cv::Point3d p = lambda*vc;

  return p;
}


bool PointCloudInput::ComputePointCloudRow( cv::Mat *pointcloud, cv::Mat *pointcloud_colors,
                                            cv::Mat mat_color_ref, cv::Mat mat_color, cv::Mat imageTest, cv::Mat color_image, double delay, int storage_row )
{
  cv::Mat mat_BGR;
  cv::Mat mat_gray;
  std::vector<cv::Point2d> cam_points;
  std::vector<cv::Point2d>::iterator it_cam_points;
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
    std::cerr << "ERROR invalid cv::Mat data\n" << std::endl;
    return false;
    }

  cv::subtract( mat_color, mat_color_ref, mat_BGR );
  if( !mat_BGR.data || mat_BGR.type() != CV_8UC3 )
    {
    std::cerr << "ERROR invalid cv::Mat data\n" << std::endl;
    return false;
    }

  //Convert the captured frame from BGR to gray
  cv::cvtColor( mat_BGR, mat_gray, cv::COLOR_BGR2GRAY );

  // Looking for the point with th maximum intensity for each column
  for( int j = 0; j < mat_gray.cols; j++ )
    {
    sum = mat_gray.at< unsigned char >( 0, j ) + mat_gray.at< unsigned char >( 1, j ) + mat_gray.at< unsigned char >( 2, j );
    sat_max = sum;
    point_max = cv::Point2i( 0, 0 );
    for( int i = 2; i < mat_gray.rows - 1; ++i )
      {
      sum = sum - mat_gray.at< unsigned char >( i - 2, j ) + mat_gray.at< unsigned char >( i + 1, j );
      average = sum / 3;
      if( average > sat_max && average > 27 )
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

  double normalizedDelay = delay / .0119;
  double row = delayParam1 * normalizedDelay + delayParam2 * ( 1 - normalizedDelay );
  
  //double row = (delayParam1 - delay / delayParam2) *this->Projector->GetHeight();
  
  // TODO : check wether the computed row is consistent or not
  /*if( row <= 0 || row > this->Projector->GetHeight() )
      {
      std::cout << "The computed row is not valid. The line is skipped. Computed row = " << row << std::endl;
      return false; // We skip the line
      }*/

  // Computation of the point used to define the plane of the projector
  // to image camera coordinates

  distortedProjectorPoints.at<cv::Vec2d>( 0, 0 ) = cv::Vec2d( this->Projector->GetWidth(), row );
  distortedProjectorPoints.at<cv::Vec2d>( 0, 1 ) = cv::Vec2d( 0, row );

  cv::undistortPoints( distortedProjectorPoints, undistortedProjectorPoints, this->Calib->Proj_K, cv::Mat() );
  assert( undistortedProjectorPoints.type() == CV_64FC2 && undistortedProjectorPoints.rows == 1 && undistortedProjectorPoints.cols == 2 );
  const cv::Vec2d & outvec2 = undistortedProjectorPoints.at<cv::Vec2d>( 0, 0 );
  projectorVector1 = cv::Point3d( outvec2[ 0 ], outvec2[ 1 ], 1.0 );

  const cv::Vec2d & outvec3 = undistortedProjectorPoints.at<cv::Vec2d>( 0, 1 );
  projectorVector2 = cv::Point3d( outvec3[ 0 ], outvec3[ 1 ], 1.0 );

  //find normal of laser plane via cross product of two vectors in plane
  projectorNormal = cv::Point3d( cv::Mat( this->Calib->R*( cv::Mat( projectorVector1 ).cross( cv::Mat( projectorVector2 ) ) ) ) );

  it_cam_points = cam_points.begin();
  for( it_cam_points; it_cam_points != cam_points.end(); ++it_cam_points )
    {
    //to image camera coordinates
    inp1.at<cv::Vec2d>( 0, 0 ) = cv::Vec2d( it_cam_points->x, it_cam_points->y );
    cv::undistortPoints( inp1, outp1, this->Calib->Cam_K, this->Calib->Cam_kc );
    assert( outp1.type() == CV_64FC2 && outp1.rows == 1 && outp1.cols == 1 );
    const cv::Vec2d & outvec1 = outp1.at<cv::Vec2d>( 0, 0 );
    cameraVector = cv::Point3d( outvec1[ 0 ], -outvec1[ 1 ], 1 );

    p = approximate_ray_plane_intersection( this->Calib->T, cameraVector, projectorNormal );

    if( sqrt( p.x*p.x + p.y * p.y + p.z * p.z ) < 400 )
      {
      cv::Vec3f & cloud_point = ( *pointcloud ).at<cv::Vec3f>( storage_row, ( *it_cam_points ).x );
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

      cv::Vec3b & cloud_color = ( *pointcloud_colors ).at<cv::Vec3b>( storage_row, ( *it_cam_points ).x );
      cloud_color[ 0 ] = vec_B;
      cloud_color[ 1 ] = vec_G;
      cloud_color[ 2 ] = vec_R;
      color_image.at<cv::Vec3b>( ( *it_cam_points ).y, ( *it_cam_points ).x ) = cv::Vec3b{ vec_B, vec_G, vec_R };

      }
    }

  return true;
}
