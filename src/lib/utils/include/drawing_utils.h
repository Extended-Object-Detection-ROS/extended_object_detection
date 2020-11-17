#ifndef _DRAWING_UTILS_
#define _DRAWING_UTILS_

#include <opencv2/opencv.hpp>

namespace eod{
    
    void drawTransparentRectangle(cv::Mat image, cv::Rect rectange, cv::Scalar col, double alpha);
    
    cv::Point drawFilledRectangleWithText(cv::Mat image, cv::Point topleft, std::string text, cv::Scalar col);
    
}

#endif
