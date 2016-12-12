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
  int GetBlueColor() { return this->BlueColor; };
  int GetGreenColor() { return this->GreenColor; };
  int GetRedColor() { return this->RedColor; };
  void SetPixmap(QPixmap image) { this->Pixmap = image; };
  void SetWidth(int x) { this->Width = x; };
  void SetHeight(int y) { this->Height = y; };
  void SetLineThickness(int thickness) { this->LineThickness = thickness; };
  void SetRow(int r) { this->Row = r; };
  void SetBlueColor( int blue ) { this->BlueColor = blue; };
  void SetGreenColor( int green ) { this->GreenColor = green; };
  void SetRedColor( int red ) { this->RedColor = red; };

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
  int BlueColor;
  int GreenColor;
  int RedColor;
};

#endif  /* __PROJECTOR_HPP__ */