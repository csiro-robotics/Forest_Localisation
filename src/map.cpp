////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "map.h"

ImageMap::ImageMap()
{

}

ImageMap::~ImageMap()
{

}

bool ImageMap::loadMapImage(const string& file_name, double image_ref_x, double image_ref_y, double grid_size, double max_height)
{
    //Load the ACM image map and the respective global reference frame (x,y) coordinates
    image_ = cv::imread(file_name,cv::IMREAD_GRAYSCALE);
    if(image_.empty())
    {
        std::cout << "Could not read the image: " << file_name << std::endl;
        return false;
    }

    image_frame_x_ = image_ref_x;
    image_frame_y_ = image_ref_y;
    grid_size_ = grid_size;
    max_height_ = max_height;
    
    ROS_INFO("ACM Map data loaded");
    
    return true;
}

