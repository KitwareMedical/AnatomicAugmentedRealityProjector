#include "CameraInput.hpp"

#include "FlyCapture2.h"

#include <QTime>

#include <iomanip>
#include <iostream>
#include <stdio.h>
//#include <ostream>

using namespace FlyCapture2;

CameraInput::CameraInput() : NbImages(1), Camera()
{}

CameraInput::~CameraInput()
{
  Error error;
  // Stop capturing images
  error = this->Camera.StopCapture();
  if (error != PGRERROR_OK)
  {
    error.PrintErrorTrace();
  }

  // Disconnect the camera
  error = this->Camera.Disconnect();
  if (error != PGRERROR_OK)
  {
    error.PrintErrorTrace();
  }
}

void CameraInput::Run()
{
  Error error;
  BusManager busMgr;
  //sleep(5);
  cv::waitKey(2);
  PGRGuid guid;
  unsigned int numCameras;

  error = busMgr.GetNumOfCameras(&numCameras);
  if (error != PGRERROR_OK)
  {
    error.PrintErrorTrace();
    return;
  }
  if (numCameras < 1)
  {
    std::cout << "No camera detected." << std::endl;
    return;
  }
  else
  {
    std::cout << "Number of cameras detected: " << numCameras << std::endl;
  }

  error = busMgr.GetCameraFromIndex(0, &guid);
  if (error != PGRERROR_OK)
  {
    error.PrintErrorTrace();
    return;
  }

  error = Camera.Connect(&guid);
  if (error != PGRERROR_OK)
  {
    error.PrintErrorTrace();
    return;
  }

  //this->SetCameraFrameRate(this->FrameRate);

  error = Camera.StartCapture();
  if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
  {
    std::cout << "Bandwidth exceeded" << std::endl;
    return;
  }
  else if (error != PGRERROR_OK)
  {
    std::cout << "Failed to start image capture" << std::endl;
    return;
  }
}

void CameraInput::SetCameraFrameRate(double frameRate)
{
  Error error;

  // Check if the camera supports the FRAME_RATE property
  std::cout << "Detecting frame rate from camera... " << std::endl;
  PropertyInfo propInfo;
  propInfo.type = FRAME_RATE;
  error = this->Camera.GetPropertyInfo(&propInfo);
  if (error != PGRERROR_OK)
  {
    error.PrintErrorTrace();
    return;
  }
  if (propInfo.present == true)
  {
    // Get the frame rate
    Property prop;
    prop.type = FRAME_RATE;
    error = this->Camera.GetProperty(&prop);
    if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
    }
    else
    {
      prop.autoManualMode = false;
      // Set the frame rate.
      // Note that the actual recording frame rate may be slower,
      // depending on the bus speed and disk writing speed.
      prop.absValue = frameRate;
      error = this->Camera.SetProperty(&prop);
      if (error != PGRERROR_OK)
      {
        error.PrintErrorTrace();
        return;
      }
    }
  }
  std::cout << "Asking frame rate of " << std::fixed << std::setprecision(1) << frameRate << std::endl;
  this->GetCameraFrameRate();
}

double CameraInput::GetCameraFrameRate()
{
  Error error;

  // Check if the camera supports the FRAME_RATE property
  //std::cout << "Detecting frame rate from camera... " << std::endl;
  PropertyInfo propInfo;
  propInfo.type = FRAME_RATE;
  error = this->Camera.GetPropertyInfo(&propInfo);
  if (error != PGRERROR_OK)
  {
    error.PrintErrorTrace();
    return 0;
  }
  if (propInfo.present == true)
  {
    // Get the frame rate
    Property prop;
    prop.type = FRAME_RATE;
    error = this->Camera.GetProperty(&prop);
    if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
    }
    else
    {
      // Set the frame rate.
      // Note that the actual recording frame rate may be slower,
      // depending on the bus speed and disk writing speed.
      std::cout << "Using frame rate of " << std::fixed << std::setprecision(1) << prop.absValue << std::endl;
      return prop.absValue;
    }
  }
  return 0;
}
// Note : Check the returned value when calling the function

void CameraInput::RecordImages()
{
  std::cout << "Grabbing " << this->NbImages << " images" << std::endl;

  Error error;
  Image rawImage;
  for (int imageCount = 0; imageCount < this->NbImages; imageCount++)
  {
    // Retrieve an image
    error = this->Camera.RetrieveBuffer(&rawImage);
    if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      continue;
    }

    std::cout << ".";

    // Get the raw image dimensions
    PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    rawImage.GetDimensions(&rows, &cols, &stride, &pixFormat);

    // Create a converted image
    Image convertedImage;

    // Convert the raw image
    error = rawImage.Convert(PIXEL_FORMAT_BGRU, &convertedImage);
    if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      return;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = this->Camera.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      return;
    }
    // Create a unique filename
    std::ostringstream filename;
    filename << "Results\\" << camInfo.serialNumber << "-" << imageCount << ".bmp";

    // Save the image. If a file format is not passed in, then the file
    // extension is parsed to attempt to determine the file format.
    error = convertedImage.Save(filename.str().c_str());
    if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      return;
    }
  }
  std::cout << std::endl;
  std::cout << "Finished grabbing images" << std::endl;
}
// note : return a value to detect an error ?

cv::Mat CameraInput::DisplayImages()
{
  FlyCapture2::Error error;
  static bool flag = false;
  FlyCapture2::Image rawImage;
  error = this->Camera.RetrieveBuffer(&rawImage);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    error.PrintErrorTrace();
    //if (timer.elapsed() > warmup) { error_frame++; }
    //return;
  }
  //error_frame = 0;
  // convert to rgb
  FlyCapture2::Image rgbImage;
  rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

  // convert to OpenCV Mat
  unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
  cv::Mat mat = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);


  cv::transpose(mat, mat);
  cv::flip(mat, mat, 0);
  cv::transpose(mat, mat);
  cv::flip(mat, mat, 0);
  return mat;
}
