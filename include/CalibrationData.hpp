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