#include "ProjectorWidget.hpp"

#include <QApplication>
#include <QDesktopWidget>
#include <QPainter>

#include <iostream>

ProjectorWidget::ProjectorWidget(QWidget * parent, Qt::WindowFlags flags) :
  QWidget(parent, flags),
  Height(720),
  Width(1920),
  LineThickness(1), 
  Row(100)
{}

ProjectorWidget::~ProjectorWidget()
{
}

// /!\ Use of usigned char to code the color between 0 (black) and 255 (white)
cv::Mat ProjectorWidget::CreateLineImage()
{
  cv::Mat image = cv::Mat::zeros(this->Height, this->Width, CV_8UC1); // use CV_32S for int
  for (int j = 0; j < image.cols; j++)
  {
    for (int t = 0; t < this->LineThickness; t++)
    {
      image.at<unsigned char>(this->Row + t, j) = 255;
    }
  }
  return image;
}

std::vector<cv::Point2i> ProjectorWidget::GetCoordLine(cv::Mat image)
{
  // TODO: condition on type of matrix 
  std::vector<cv::Point2i> coord;
  for (int i = 0; i < image.rows; i++)
  {
    unsigned char *row = image.ptr<unsigned char>(i);
    //std::cout << "ligne : " << i << std::endl;
    //std::cout << "row :" << *row << std::endl;
    for (int j = 0; j < image.cols; j++)
    {
      if ((int)row[j] != 0) // or =255
      {
        coord.push_back(cv::Point2i(i, j));
      }
    }
  }

  return coord;
}

void ProjectorWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);

  if (!this->Pixmap.isNull())
  {
    //QPixmap scale_pixmap = Pixmap.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    //QRectF rect = QRectF(QPointF(0, 0), QPointF(scale_pixmap.width(), scale_pixmap.height()));
    QRectF rect = QRectF(QPointF(0, 0), QPointF(width(), height()));
    painter.drawPixmap(rect, this->Pixmap, rect);
    emit new_image(this->Pixmap);
  }
  else
  {
    QRectF rect = QRectF(QPointF(0, 0), QPointF(width(), height()));
    painter.drawText(rect, Qt::AlignCenter, "No image");
  }
}

void ProjectorWidget::start()
{
  QDesktopWidget * desktop = QApplication::desktop();
  int screen = desktop->screenCount();
  std::cout << screen << std::endl;
  // We choose the last screen added (highest number) = the projector
  //display
  QRect screen_resolution = desktop->screenGeometry(screen - 1);
  move(QPoint(screen_resolution.x(), screen_resolution.y()));
  showFullScreen();
  QApplication::processEvents();
}