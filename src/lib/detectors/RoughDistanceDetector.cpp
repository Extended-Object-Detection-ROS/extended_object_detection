#include "RoughDistanceDetector.h"
#include "ObjectBase.h"

using namespace std;
using namespace cv;

namespace eod{
    
    RoughDistAttribute::RoughDistAttribute(){
        Type = ROUGH_DIST_A;        
        realHeight = 0;
        realWidth = 0;
        inited = false;
    }
    
    RoughDistAttribute::RoughDistAttribute(double realW, double realH){
        Type = ROUGH_DIST_A;                
        realHeight = realH;
        realWidth = realW;
        inited = true;
    }
            
    
    vector<ExtendedObjectInfo> RoughDistAttribute::Detect2(const Mat& image, int seq){
        return vector<ExtendedObjectInfo>();
    }
                
    bool RoughDistAttribute::Check2(const Mat& image, ExtendedObjectInfo& rect){
        return false;
    }
        
    void RoughDistAttribute::Extract2(const Mat& image, ExtendedObjectInfo& rect){
        Mat camMat = parent_base->getCameraMatrix(); 
        Mat distCoef = parent_base->getDistortionCoeff();
        
        if( !camMat.empty() && !distCoef.empty() ){
                 
            vector<Point2f> image_corners = rect.getCorners();            
            vector<Point3f> original_corners;
            
            if( realHeight != 0 ){
                double width = realHeight * rect.width / rect.height;
                original_corners = getOriginalCorners(width, realHeight);                
            }
            else if( realWidth != 0){
                double height = realWidth * rect.height / rect.width;
                original_corners = getOriginalCorners(realWidth, height);                
            }   
            else{
                printf("Error: realHeight and realWidth in RoughtDistance attribute are not inited!\n");
                return;
            }            
            Vec3d rvec, tvec;                          
            if( solvePnP(original_corners, image_corners, camMat, distCoef, rvec, tvec)){
                rect.tvec.push_back(tvec);
                rect.rvec.push_back(rvec);
            }                
            else{
                printf("Error! Unable to solve PnP in RoughDistAttribute.\n");
            }
        }                
    }
    
    
    vector<Point3f> RoughDistAttribute::getOriginalCorners(double width, double height){
        vector<Point3f> corners;
        double w2 = width/2;
        double h2 = height/2;
        corners.push_back(Point3f(-w2, -h2, 0));
        corners.push_back(Point3f(w2, -h2, 0));
        corners.push_back(Point3f(w2, h2, 0));
        corners.push_back(Point3f(-w2, h2, 0));
        return corners;
    }
}

