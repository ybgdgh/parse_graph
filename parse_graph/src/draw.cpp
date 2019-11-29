#include "draw.h"


namespace Draw_PG
{
    void draw_node(cv::Mat img_pg, int x, int y, std::string node_name)
    {
        cv::circle(img_pg,cv::Point(x,y),16,cv::Scalar(255, 160, 0),2,8);
        cv::circle(img_pg,cv::Point(x,y),15,cv::Scalar(0, 160, 255),-1,8);
        cv::putText(img_pg, node_name, cv::Point(x+20,y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, 8);
    
    }

    void draw_attribute_node(cv::Mat img_pg, int x, int y, std::string node_name)
    {
        cv::circle(img_pg,cv::Point(x,y),10,cv::Scalar(0, 160, 160),4,8);
        // cv::putText(img_pg, node_name, cv::Point(x+15,y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, 8);
    
    }

    void draw_arrow(cv::Mat img_pg, int x1, int y1, int x2, int y2)
    {  
        y1=y1+15;
        y2=y2-15;
        double par = 10.0;//箭头部分三角形的腰长
        double slopy = atan2((y2 - y1), (x2 - x1));
        double cosy = cos(slopy);
        double siny = sin(slopy);
        cv::Point point1 = cv::Point(x2 + int(-par*cosy - (par / 2.0*siny)), y2 + int(-par*siny + (par / 2.0*cosy)));
        cv::Point point2 = cv::Point(x2 + int(-par*cosy + (par / 2.0*siny)), y2 - int(par / 2.0*cosy + par*siny));
        std::vector<cv::Point> contour;
        contour.push_back(point1);
        contour.push_back(point2);
        contour.push_back(cv::Point(x2,y2));

        std::vector<std::vector<cv::Point >> contours;
	    contours.push_back(contour);

        cv::polylines(img_pg, contours, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);//第2个参数可以采用contour或者contours，均可
	
        cv::fillPoly(img_pg, contours, cv::Scalar(0, 255, 0));//fillPoly函数的第二个参数是二维数组！！

        //绘制箭身
        cv::line(img_pg,Point(x1,y1),Point(x2,y2),Scalar(0,255,0),1,8);

        
    }

    void draw_attribute_arrow(cv::Mat img_pg, int x1, int y1, int x2, int y2)
    {  
        double par = 10.0;//箭头部分三角形的腰长
        double slopy = atan2((y2 - y1), (x2 - x1));
        double cosy = cos(slopy);
        double siny = sin(slopy);
        cv::Point point1 = cv::Point(x2 + int(-par*cosy - (par / 2.0*siny)), y2 + int(-par*siny + (par / 2.0*cosy)));
        cv::Point point2 = cv::Point(x2 + int(-par*cosy + (par / 2.0*siny)), y2 - int(par / 2.0*cosy + par*siny));
        std::vector<cv::Point> contour;
        contour.push_back(point1);
        contour.push_back(point2);
        contour.push_back(cv::Point(x2,y2));

        std::vector<std::vector<cv::Point >> contours;
	    contours.push_back(contour);

        cv::polylines(img_pg, contours, true, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);//第2个参数可以采用contour或者contours，均可
	
        cv::fillPoly(img_pg, contours, cv::Scalar(0, 0, 255));//fillPoly函数的第二个参数是二维数组！！

        //绘制箭身
        cv::line(img_pg,Point(x1,y1),Point(x2,y2),Scalar(0,0,255),1,8);

        
    }

    void draw_triangle(cv::Mat img_pg, int x, int y)
    {  
        double par = 20.0;//箭头部分三角形的腰长
        double slopy = atan2(-10, 0);
        double cosy = cos(slopy);
        double siny = sin(slopy);
        cv::Point point1 = cv::Point(x + int(-par*cosy - (par / 2.0*siny)), y + int(-par*siny + (par / 2.0*cosy)));
        cv::Point point2 = cv::Point(x + int(-par*cosy + (par / 2.0*siny)), y - int(par / 2.0*cosy + par*siny));
        std::vector<cv::Point> contour;
        contour.push_back(point1);
        contour.push_back(point2);
        contour.push_back(cv::Point(x,y));

        std::vector<std::vector<cv::Point >> contours;
	    contours.push_back(contour);

        cv::polylines(img_pg, contours, true, cv::Scalar(0, 160, 255), 4, cv::LINE_AA);//第2个参数可以采用contour或者contours，均可
	
        cv::fillPoly(img_pg, contours, cv::Scalar(0, 160, 255));//fillPoly函数的第二个参数是二维数组！！
        
    }


    // 圆弧箭头
    // 图像、圆心、开始点、结束点、线宽
    void draw_Arc(Mat img_pg, Point StartPoint, Point meddlePoint, Point EndPoint, int Fill)
    {
        if (Fill <= 0) return;

        StartPoint.y = StartPoint.y+5;
        EndPoint.y = EndPoint.y+5;
        double par = 10.0;//箭头部分三角形的腰长
        double slopy = atan2((EndPoint.y - meddlePoint.y), (EndPoint.x - meddlePoint.x));
        double cosy = cos(slopy);
        double siny = sin(slopy);
        cv::Point point1 = cv::Point(EndPoint.x + int(-par*cosy - (par / 2.0*siny)), EndPoint.y + int(-par*siny + (par / 2.0*cosy)));
        cv::Point point2 = cv::Point(EndPoint.x + int(-par*cosy + (par / 2.0*siny)), EndPoint.y - int(par / 2.0*cosy + par*siny));
        std::vector<cv::Point> contour;
        contour.push_back(point1);
        contour.push_back(point2);
        contour.push_back(EndPoint);

        std::vector<std::vector<cv::Point >> contours;
	    contours.push_back(contour);

        cv::polylines(img_pg, contours, true, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);//第2个参数可以采用contour或者contours，均可
	
        cv::fillPoly(img_pg, contours, cv::Scalar(255, 0, 0));//fillPoly函数的第二个参数是二维数组！！

        int offsetX = int(par*siny / 2);
        int offsetY = int(par*cosy / 2);
        cv::Point point3, point4;
        point3 = cv::Point(EndPoint.x + int(-par*cosy - (par / 2.0*siny)) + offsetX, EndPoint.y + int(-par*siny + (par / 2.0*cosy)) - offsetY);
        point4 = cv::Point(EndPoint.x + int(-par*cosy + (par / 2.0*siny) - offsetX), EndPoint.y - int(par / 2.0*cosy + par*siny) + offsetY);
            
        double x1 = StartPoint.x;
        double y1 = StartPoint.y;
        double x2 = meddlePoint.x;
        double y2 = meddlePoint.y;
        double x3 = (point3.x+point4.x)/2;
        double y3 = (point3.y+point4.y)/2;


        double a, b, e, r;
        double x_, y_;


        a = (x1 + x2) * (x1 - x2) + (y1 + y2) * (y1 - y2);
        b = (x3 + x2) * (x3 - x2) + (y3 + y2) * (y3 - y2);
        e = (x1 - x2) * (y3 - y2) - (x2 - x3) * (y2 - y1);


        x_ = (a * (y3 - y2) + b * (y2 - y1)) / (2 * e);
        y_ = (a * (x2 - x3) + b * (x1 - x2)) / (2 * e);
        // r = sqrt((x2 - x_) * (x2 - x_) + (y2 - y_) * (y2 - y_));

        cv::Point ArcCenter(x_,y_);
        vector<Point> Dots;
        double Angle1 = atan2((StartPoint.y - ArcCenter.y), (StartPoint.x - ArcCenter.x));
        double Angle2 = atan2((y3 - ArcCenter.y), (x3 - ArcCenter.x));
        double Angle = Angle1 - Angle2;
        Angle = Angle * 180.0 / CV_PI;
    
        // if (Angle < 0) Angle = 360 + Angle;
        if (Angle == 0) Angle = 360;
        int  brim = floor(abs(Angle) / 10); // 向下取整
    
        Dots.push_back(StartPoint);
        if(Angle < 0 )
        {
            for (int i = 0; i < brim; i++)
            {
                double dSinRot = sin((10 * (i + 1)) * CV_PI / 180);
                double dCosRot = cos((10 * (i + 1)) * CV_PI / 180);
                int x = ArcCenter.x + dCosRot * (StartPoint.x - ArcCenter.x) - dSinRot * (StartPoint.y - ArcCenter.y);
                int y = ArcCenter.y + dSinRot * (StartPoint.x - ArcCenter.x) + dCosRot * (StartPoint.y - ArcCenter.y);
                Dots.push_back(Point(x, y));
            }
        }
        else 
        {
            for (int i = 0; i < brim; i++)
            {
                double dSinRot = sin(-(10 * (i + 1)) * CV_PI / 180);
                double dCosRot = cos(-(10 * (i + 1)) * CV_PI / 180);
                int x = ArcCenter.x + dCosRot * (StartPoint.x - ArcCenter.x) - dSinRot * (StartPoint.y - ArcCenter.y);
                int y = ArcCenter.y + dSinRot * (StartPoint.x - ArcCenter.x) + dCosRot * (StartPoint.y - ArcCenter.y);
                Dots.push_back(Point(x, y));
            }
        }
        
        Dots.push_back(EndPoint);
        RNG &rng = theRNG();
        Scalar color = Scalar(255, 0, 0);
        for (int i = 0; i < Dots.size() - 1; i++) {
            line(img_pg, Dots[i], Dots[i + 1], color, Fill);
        }
        Dots.clear();
    }
    

}