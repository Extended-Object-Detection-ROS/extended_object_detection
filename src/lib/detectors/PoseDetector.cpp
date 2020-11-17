#include "PoseDetector.h"

using namespace std;
using namespace cv;

namespace eod{

	PoseAttribute::PoseAttribute(){
        Type = POSE_A;

	}

    PoseAttribute::PoseAttribute(double x_min, double x_max, double y_min, double y_max ){
        Type = POSE_A;
        this->x_min = x_min;
        this->x_max = x_max;
        this->y_min = y_min;
        this->y_max = y_max;
    }	    
    
    vector<ExtendedObjectInfo> PoseAttribute::Detect2(const Mat& image, int seq){
        vector<ExtendedObjectInfo> rects(0);
        return rects;
    }
    
    void PoseAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
    
    bool PoseAttribute::Check2(const Mat& image,ExtendedObjectInfo& rect){      
        Point center = rect.getCenter();
        
        double y_lim_u = image.size().height * y_max;
        double y_lim_d = image.size().height * y_min;
        
        double x_lim_u = image.size().width * x_max;
        double x_lim_d = image.size().width * x_min;
        
        //printf("%i %i | %f %f %f %f\n",center.x, center.y, x_lim_d, x_lim_u, y_lim_d, y_lim_u);
        
        if( y_lim_u >= center.y && y_lim_d <= center.y && x_lim_u >= center.x && x_lim_d <= center.x ){
            rect.setScoreWeight(1, Weight);
            return true;                
        }
        return false;
    }    

}
