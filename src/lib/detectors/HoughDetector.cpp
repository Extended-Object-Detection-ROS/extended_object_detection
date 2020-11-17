#include "HoughDetector.h"

using namespace std;
using namespace cv;

namespace eod{

    HoughAttribute::HoughAttribute(){
        Type = HOUGH_A;
        initialized = false;
    }

    HoughAttribute::HoughAttribute(int TypeH_){
        Type = HOUGH_A;
        TypeH = TypeH_;

        initialized = false;
    }

    HoughAttribute::HoughAttribute(double d, double md, double p1, double p2, int mr, int Mr){
        Type = HOUGH_A;
        TypeH = CIRCLE;
        initialized = true;
        dp = d;
        min_dist = md;
        param1 = p1;
        param2 = p2;
        minradius = mr;
        maxradius = Mr;
    }

    HoughAttribute::HoughAttribute(int rho_, float theta_, int threshold_, int minLinLength_, int maxLineGap_){
        Type = HOUGH_A;
        TypeH = LINE;
        initialized = true;
        rho = rho_;
        theta = theta_;
        threshold = threshold_;
        minLinLength = minLinLength_;
        maxLineGap = maxLineGap_;
    }
    
    bool HoughAttribute::Check2(const Mat& image,ExtendedObjectInfo& rect){
      return false;
    }
    
    void HoughAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }

    void HoughAttribute::SetParamsCircle(double d, double md, double p1, double p2, int mr, int Mr){
        Type = HOUGH_A;
        TypeH = CIRCLE;
        initialized = true;
        dp = d;
        min_dist = md;
        param1 = p1;
        param2 = p2;
        minradius = mr;
        maxradius = Mr;
    }    

    vector<ExtendedObjectInfo> HoughAttribute::Detect2(const Mat& image, int seq){      

        vector<ExtendedObjectInfo> res;
        switch( TypeH ){
        case CIRCLE:
            res = findCircles(image);
            break;
        case LINE:
            res = findLines(image);
            break;
        }      
      return res;
    }
    
    vector<ExtendedObjectInfo> HoughAttribute::findLines(const Mat& image){
      vector<ExtendedObjectInfo> boundingRects;
      Mat canny_img, gray_img;
      Canny(image, canny_img, 50, 200, 3);
      vector<Vec4i> lines;
      if(!initialized) HoughLinesP(canny_img, lines, 1, CV_PI/180, 50, 50, 10 );
      else HoughLinesP(canny_img, lines, rho, theta, threshold, minLinLength, maxLineGap );

      for( size_t i = 0; i < lines.size(); i++ )
      {
        Vec4i l = lines[i];
        boundingRects.push_back(ExtendedObjectInfo( l[0], l[1], l[2]-l[0], l[3]-l[1]) );
	//line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
      }
      return boundingRects;
    }
    
    vector<ExtendedObjectInfo> HoughAttribute::findCircles(const Mat& image){
        Mat frame_gray, src_gray;
#if (CV_MAJOR_VERSION > 3)
        cvtColor(image, src_gray, COLOR_BGR2GRAY);
#else
        cvtColor(image, src_gray, CV_BGR2GRAY);
#endif        
        GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
        vector<Vec3f> circles;
        vector<ExtendedObjectInfo> shapes;

        if (!initialized){
#if (CV_MAJOR_VERSION > 3)            
            //HoughCircles( src_gray, circles, HOUGH_GRADIENT, 1, 10, 100, 30, 1, 0 );
            HoughCircles( src_gray, circles, HOUGH_GRADIENT, 2, image.rows/8, 30, 150, 10, 100 );
#else            
            HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, 10, 100, 30, 1, 0 );
#endif        
            
        }
        else{
#if (CV_MAJOR_VERSION > 3)      
            HoughCircles( src_gray, circles, HOUGH_GRADIENT, dp, min_dist, param1, param2, minradius, maxradius );
#else            
            HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, dp, min_dist, param1, param2, minradius, maxradius );
#endif
        }  

        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

            shapes.push_back( ExtendedObjectInfo(
                center.x - radius,
                center.y - radius,
                radius*2,
                radius*2
                ));
            shapes[i].setScoreWeight(1, Weight);
        }
        return shapes;
    }

}
