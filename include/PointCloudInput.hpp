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

#ifndef POINTCLOUDINPUT_HPP
#define POINTCLOUDINPUT_HPP

#include "ProjectorWidget.hpp"
#include "CameraInput.hpp"
#include "CalibrationData.hpp"

struct PointCloud {
     cv::Mat points;
	 cv::Mat colors;
};

class PointCloudInput {

	public:
		PointCloudInput(CameraInput* CamInput, ProjectorWidget* Projector, CalibrationData* Calib);
		~PointCloudInput();
		cv::Point3d approximate_ray_plane_intersection(const cv::Mat & T, const cv::Point3d & vc, const cv::Point3d & vp);
		PointCloud ComputePointCloud();
		bool ComputePointCloudRow(cv::Mat *pointcloud, cv::Mat *pointcloud_colors, cv::Mat mat_color_ref, cv::Mat mat_color, cv::Mat imageTest, cv::Mat color_image, double delay, int row);
		cv::Mat GetCurrentMat() const { return this->CurrentMat; };
		void SetCurrentMat(cv::Mat currentMat) { this->CurrentMat = currentMat; };


	private:
		
		CameraInput* CamInput;
		ProjectorWidget* Projector;
		
		CalibrationData* Calib;

		cv::Mat CurrentMat;

};



#endif