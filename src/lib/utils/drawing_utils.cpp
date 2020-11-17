#include "drawing_utils.h"

using namespace std;
using namespace cv;

namespace eod{

    Point drawFilledRectangleWithText(Mat image, Point topleft, string text, Scalar col){
        int fontCoeff = 12;
        Point brRect = Point(topleft.x + text.length() * fontCoeff / 1.6, topleft.y + fontCoeff);            
        Point textCorner = Point(topleft.x +1 , topleft.y + fontCoeff * 0.8);
            
        //rectangle(image, topleft, brRect, col, -1);
        drawTransparentRectangle(image, Rect(topleft, brRect), col, 0.5);
        
        // check textcorner!
        putText(image, text, textCorner, FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0),1);
        return brRect;
    }
    
    void drawTransparentRectangle(Mat image, Rect rectange, Scalar col, double alpha){
        Rect rect = Rect(0, 0, image.size().width, image.size().height);
        Mat roi = image(rectange & rect);
        addWeighted(col, alpha, roi, 1.0 - alpha , 0.0, roi); 
    }
    
}
