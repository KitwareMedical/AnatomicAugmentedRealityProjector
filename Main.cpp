#include "MainWindow.hpp"
#include "CalibrationData.hpp"

#include <opencv2/highgui/highgui.hpp>

#include <QApplication>
#include <QFileDialog>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  MainWindow window;
  std::cout<<"Draw the window"<<std::endl;
  window.show();

  CalibrationData calib;
  QString calibrationFile = "../calibration-small-stick-rotation.yml";

  bool error = calib.LoadCalibration(calibrationFile);
  if (error == false)
  {
    std::cout << "Impossible to read the calibration file" << std::endl;
  }
  //calib.display(std::cout);

  std::cout<<"Start the main application"<<std::endl;
  return app.exec();

}
