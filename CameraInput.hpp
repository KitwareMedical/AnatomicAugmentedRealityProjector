#include "FlyCapture2.h"

#include <QThread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraInput
{
public :
  CameraInput();
  ~CameraInput();

  void Run();
  void SetCameraFrameRate(double framerate);
  double GetCameraFrameRate();
  
  //void SetFrameRate(double frameRate) { this->FrameRate = frameRate; };
  void SetNbImages(int nbImages) { this->NbImages = nbImages; };
  //double GetFrameRate() const { return this->FrameRate; };
  int GetNbImages() const { return this->NbImages; };

  void RecordImages();
  cv::Mat DisplayImages();


  FlyCapture2::Camera Camera;

private :
  //double FrameRate;
  int NbImages;
  
};
