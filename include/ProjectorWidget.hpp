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

#ifndef __PROJECTOR_HPP__
#define __PROJECTOR_HPP__

#include <opencv2/core/core.hpp>

#include <QWidget>

#include <stdio.h>
#include <stdlib.h>
#include <vector>


class ProjectorWidget : public QWidget
{
  Q_OBJECT

public:
  ProjectorWidget(QWidget * parent = 0, Qt::WindowFlags flags = 0);
  ~ProjectorWidget();

  cv::Mat CreateLineImage();
  cv::Mat CreatePattern();
  cv::Mat CreateColoredImage( int blue, int green, int red );
  std::vector<cv::Point2i> GetCoordLine(cv::Mat image);

  QPixmap GetPixmap() const { return this->Pixmap; };
  int GetWidth() const { return this->Width; };
  int GetHeight() const { return this->Height; };
  int GetLineThickness() const { return this->LineThickness; };
  int GetRow() const { return this->Row; };
  unsigned char GetBlueColor() { return this->BlueColor; };
  unsigned char GetGreenColor() { return this->GreenColor; };
  unsigned char GetRedColor() { return this->RedColor; };
  void SetPixmap(QPixmap image) { this->Pixmap = image; };
  void SetWidth(int x) { this->Width = x; };
  void SetHeight(int y) { this->Height = y; };
  void SetLineThickness(int thickness) { this->LineThickness = thickness; };
  void SetRow(int r) { this->Row = r; };
  void SetBlueColor( unsigned char blue ) { this->BlueColor = blue; };
  void SetGreenColor( unsigned char green ) { this->GreenColor = green; };
  void SetRedColor( unsigned char red ) { this->RedColor = red; };

  void start();

signals:
  void new_image(QPixmap pixmap);

protected:
  virtual void paintEvent(QPaintEvent *);

private:
  QPixmap Pixmap;
  int Height;
  int Width;
  int LineThickness;
  int Row;
  unsigned char BlueColor;
  unsigned char GreenColor;
  unsigned char RedColor;
};

#endif  /* __PROJECTOR_HPP__ */