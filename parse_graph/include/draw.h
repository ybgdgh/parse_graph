#ifndef DRAW_H
#define DRAW_H

#include<stdio.h>  
#include <iostream>  
#include <map>
#include <string.h>

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

namespace Draw_PG
{
    
    void draw_node(cv::Mat img_pg, int x, int y, std::string node_name);
    void draw_attribute_node(cv::Mat img_pg, int x, int y, std::string node_name);
    void draw_arrow(cv::Mat img_pg, int x1, int y1, int x2, int y2);
    void draw_attribute_arrow(cv::Mat img_pg, int x1, int y1, int x2, int y2);
    void draw_triangle(cv::Mat img_pg, int x, int y);
    void draw_Arc(Mat img_pg, Point StartPoint, Point meddlePoint, Point EndPoint, int Fill);
}








#endif