////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MAP_H
#define _MAP_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

#include <ros/ros.h>
using namespace std;

class ImageMap
{
    public:
        ImageMap();
        ~ImageMap();

        bool loadMapImage(const string& file_name, double image_ref_x, double image_ref_y, double grid_size, double max_height);

        cv::Mat image_;
        double min_x_,min_y_,max_x_,max_y_;
        double grid_size_;
        double image_frame_x_;
        double image_frame_y_;
        double max_height_;
        

};

#endif