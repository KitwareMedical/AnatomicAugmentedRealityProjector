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

#ifndef __CALIBRATIONDATA_HPP__
#define __CALIBRATIONDATA_HPP__

#include <QString>

#include <opencv2/core/core.hpp>

#include <iostream>

class CalibrationData
{
public:
  static const int CALIBRATION_FILE_VERSION = 1;

  CalibrationData();
  ~CalibrationData();

  void Clear(void);

  bool IsValid(void) const;

  bool LoadCalibration(QString const& filename);
  bool SaveCalibration(QString const& filename);

  bool LoadCalibrationYML(QString const& filename);
  bool SaveCalibrationYML(QString const& filename);

  bool SaveCalibrationMatlab(QString const& filename);

  void Display(std::ostream & stream = std::cout) const;

  //data
  cv::Mat Cam_K;
  cv::Mat Cam_kc;
  cv::Mat Proj_K;
  cv::Mat Proj_kc;
  cv::Mat R;
  cv::Mat T;

  double CamError;
  double ProjError;
  double StereoError;

  QString Filename;
};

#endif //__CALIBRATIONDATA_HPP__