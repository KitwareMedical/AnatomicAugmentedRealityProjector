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
#ifndef CAMERAINPUT_HPP
#define CAMERAINPUT_HPP

#include "FlyCapture2.h"

#include <QThread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraInput
{
public :
  CameraInput();
  ~CameraInput();

  bool Run(); // return true if a camera was found and successfully started
  
  void SetCameraTriggerDelay(double delay);
  void IncrementTriggerDelay();
  
  void SetCameraFrameRate(double framerate);
  double GetCameraFrameRate();
  void FindTopBottomLines( cv::Mat mat_color_ref, cv::Mat mat_color );

  //void SetFrameRate(double frameRate) { this->FrameRate = frameRate; };
  void SetNbImages(int nbImages) { this->NbImages = nbImages; };
  void SetTopLine( int topLine ) { this->TopLine = topLine; };
  void SetBottomLine( int bottomLine ) { this->BottomLine = bottomLine; };
  void SetBufferSize( int size ) { this->BufferSize = size; };

  //double GetFrameRate() const { return this->FrameRate; };
  int GetNbImages() const { return this->NbImages; };
  int GetTopLine() const { return this->TopLine; };
  int GetBottomLine() const { return this->BottomLine; };
  int GetBufferSize() const { return this->BufferSize; };

  void RecordImages();
  cv::Mat GetImageFromBuffer();

  void PutFrameInBuffer( cv::Mat &f, int index );
  cv::Mat ConvertImageToMat( FlyCapture2::Image rgbImage );

  FlyCapture2::Camera Camera;

private :
  //double FrameRate;
  double delay = 0;
	
  int NbImages;
  int TopLine;
  int BottomLine;
  std::vector<cv::Mat> FrameBuffer;
  int BufferSize;
};


#endif