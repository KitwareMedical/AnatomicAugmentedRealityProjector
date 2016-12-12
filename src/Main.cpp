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

  /*CalibrationData calib;
  QString calibrationFile = "../calibration-small-stick-rotation.yml";

  bool error = calib.LoadCalibration(calibrationFile);
  if (error == false)
  {
    std::cout << "Impossible to read the calibration file" << std::endl;
  }
  //calib.display(std::cout);
  */
  std::cout<<"Start the main application"<<std::endl;
  return app.exec();

}
