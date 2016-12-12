#include "CalibrationData.hpp"

#include <opencv2/calib3d/calib3d.hpp>

#include <QFileInfo>

#include <iostream>


CalibrationData::CalibrationData() :
  Cam_K(), Cam_kc(),
  Proj_K(), Proj_kc(),
  R(), T(),
  CamError(0.0), ProjError(0.0), StereoError(0.0),
  Filename()
{
}

CalibrationData::~CalibrationData()
{
}

void CalibrationData::Clear(void)
{
  Cam_K = cv::Mat();
  Cam_kc = cv::Mat();
  Proj_K = cv::Mat();
  Proj_kc = cv::Mat();
  R = cv::Mat();
  T = cv::Mat();
  Filename = QString();
}

bool CalibrationData::IsValid(void) const
{
  return (Cam_K.data && Cam_kc.data && Proj_K.data && Proj_kc.data && R.data && T.data);
}

bool CalibrationData::LoadCalibration(QString const& filename)
{
  QFileInfo info(filename);
  QString type = info.suffix();

  if (type == "yml") { return LoadCalibrationYML(filename); }

  return false;
}

bool CalibrationData::SaveCalibration(QString const& filename)
{
  QFileInfo info(filename);
  QString type = info.suffix();

  if (type == "yml") { return SaveCalibrationYML(filename); }
  if (type == "m") { return SaveCalibrationMatlab(filename); }

  return false;
}

bool CalibrationData::LoadCalibrationYML(QString const& filename)
{
  cv::FileStorage fs(filename.toStdString(), cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    return false;
  }

  fs["cam_K"] >> Cam_K;
  fs["cam_kc"] >> Cam_kc;
  fs["proj_K"] >> Proj_K;
  fs["proj_Kc"] >> Proj_kc;
  fs["R"] >> R;
  fs["T"] >> T;

  fs["cam_error"] >> CamError;
  fs["proj_error"] >> ProjError;
  fs["stereo_error"] >> StereoError;

  fs.release();

  this->Filename = filename;

  return true;
}

bool CalibrationData::SaveCalibrationYML(QString const& filename)
{
  cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
  if (!fs.isOpened())
  {
    return false;
  }

  fs << "Cam_K" << Cam_K << "Cam_kc" << Cam_kc
    << "Proj_K" << Proj_K << "Proj_Kc" << Proj_kc
    << "R" << R << "T" << T
    << "cam_error" << CamError
    << "proj_error" << ProjError
    << "stereo_error" << StereoError
    ;
  fs.release();

  this->Filename = filename;

  return true;
}

bool CalibrationData::SaveCalibrationMatlab(QString const& filename)
{
  FILE * fp = fopen(qPrintable(filename), "w");
  if (!fp)
  {
    return false;
  }

  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  fprintf(fp,
    "%% Projector-Camera Stereo calibration parameters:\n"
    "\n"
    "%% Intrinsic parameters of camera:\n"
    "fc_left = [ %lf %lf ]; %% Focal Length\n"
    "cc_left = [ %lf %lf ]; %% Principal point\n"
    "alpha_c_left = [ %lf ]; %% Skew\n"
    "kc_left = [ %lf %lf %lf %lf %lf ]; %% Distortion\n"
    "\n"
    "%% Intrinsic parameters of projector:\n"
    "fc_right = [ %lf %lf ]; %% Focal Length\n"
    "cc_right = [ %lf %lf ]; %% Principal point\n"
    "alpha_c_right = [ %lf ]; %% Skew\n"
    "kc_right = [ %lf %lf %lf %lf %lf ]; %% Distortion\n"
    "\n"
    "%% Extrinsic parameters (position of projector wrt camera):\n"
    "om = [ %lf %lf %lf ]; %% Rotation vector\n"
    "T = [ %lf %lf %lf ]; %% Translation vector\n",
    Cam_K.at<double>(0, 0), Cam_K.at<double>(1, 1), Cam_K.at<double>(0, 2), Cam_K.at<double>(1, 2), Cam_K.at<double>(0, 1),
    Cam_kc.at<double>(0, 0), Cam_kc.at<double>(0, 1), Cam_kc.at<double>(0, 2), Cam_kc.at<double>(0, 3), Cam_kc.at<double>(0, 4),
    Proj_K.at<double>(0, 0), Proj_K.at<double>(1, 1), Proj_K.at<double>(0, 2), Proj_K.at<double>(1, 2), Proj_K.at<double>(0, 1),
    Proj_kc.at<double>(0, 0), Proj_kc.at<double>(0, 1), Proj_kc.at<double>(0, 2), Proj_kc.at<double>(0, 3), Proj_kc.at<double>(0, 4),
    rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0),
    T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0)
  );
  fclose(fp);

  return true;
}

void CalibrationData::Display(std::ostream & stream) const
{
  stream << "Camera Calib: " << std::endl
    << " - reprojection error: " << CamError << std::endl
    << " - K:\n" << Cam_K << std::endl
    << " - kc: " << Cam_kc << std::endl
    ;
  stream << std::endl;
  stream << "Projector Calib: " << std::endl
    << " - reprojection error: " << ProjError << std::endl
    << " - K:\n" << Proj_K << std::endl
    << " - kc: " << Proj_kc << std::endl
    ;
  stream << std::endl;
  stream << "Stereo Calib: " << std::endl
    << " - reprojection error: " << StereoError << std::endl
    << " - R:\n" << R << std::endl
    << " - T:\n" << T << std::endl
    ;
}