#include "HsvColorDetector.h"
//#include "geometry_utils.h"

using namespace std;
using namespace cv;

namespace eod{

	HsvColorAttribute::HsvColorAttribute(){
        Type = HSV_COLOR_A;
        Hmin = Smin = Vmin = 0;
        Hmax = 180;
        Smax = Vmax = 255;
        kernel = Mat::ones(3,3, CV_32F);
	}

    HsvColorAttribute::HsvColorAttribute(int hm, int hM, int sm, int sM, int vm, int vM){
        Type = HSV_COLOR_A;
        Hmin = hm;
        Hmax = hM;
        Smin = sm;
        Smax = sM;
        Vmin = vm;
        Vmax = vM;
        kernel = Mat::ones(3,3, CV_32F);
	}
	
	HsvColorAttribute::~HsvColorAttribute(){
        kernel.release();
    }
	
	bool HsvColorAttribute::Check2(const Mat& image, ExtendedObjectInfo& rect){
        Rect original(0,0,image.cols,image.rows);
        Mat cropped;
        cropped = image(rect.getRect() & original);
        double start_area = cropped.cols * cropped.rows;
        
        Mat hsv;
        cvtColor(cropped, hsv, COLOR_BGR2HSV);
            
        Mat mask;            
        inRange(hsv, Scalar(Hmin, Smin, Vmin), Scalar(Hmax, Smax, Vmax), mask);
                
        morphologyEx(mask, mask, MORPH_OPEN, kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);
            
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours( mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
        
        // maybe clear insiders
        
        double detected_area = 0;
        for( size_t i = 0 ; i < contours.size() ; i++ ){
            detected_area += contourArea(contours[i]);            
        }
        
        double percent = detected_area / start_area;
        
        hsv.release();
        mask.release();
        kernel.release();
        rect.cnt++;
                
        rect.setScoreWeight(percent, Weight);            
        if( percent >= Probability ){                    
            return true;             
        }        
        return false;	  
	}
    
    vector<ExtendedObjectInfo> HsvColorAttribute::Detect2(const Mat& image , int seq)
    {        
        vector<ExtendedObjectInfo> rects;
        
        Mat hsv;
        cvtColor(image, hsv, COLOR_BGR2HSV);
        
        Mat mask;            
        inRange(hsv, Scalar(Hmin, Smin, Vmin), Scalar(Hmax, Smax, Vmax), mask);
        
        Mat kernel = Mat::ones(3,3, CV_32F);
        morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours( mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
        
        for( size_t i = 0 ; i < contours.size() ; i++ ){
            ExtendedObjectInfo tmp = boundingRect( contours[i] );    
            if( returnContours )
                tmp.contour.push_back(contours[i]);
            //double percent = contourArea(contours[i]);// was idea to devide area contour on rect contour, but it is non sence in rotated objects
            tmp.setScoreWeight(1, Weight);            
            rects.push_back(tmp);
        }
            
                    
        return rects;
    }
    
    void HsvColorAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }

	
}
