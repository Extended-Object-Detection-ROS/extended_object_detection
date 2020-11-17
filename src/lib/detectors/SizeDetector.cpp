#include "SizeDetector.h"

using namespace std;
using namespace cv;

namespace eod{

	SizeAttribute::SizeAttribute(){
        Type = SIZE_A;
	}

    SizeAttribute::SizeAttribute(double mSpc, double MSpc){
        Type = SIZE_A;
        minSizePc = (double)mSpc * 0.01;
        maxSizePc = (double)MSpc * 0.01;
    }    
    
    vector<ExtendedObjectInfo> SizeAttribute::Detect2(const Mat& image, int seq){
        vector<ExtendedObjectInfo> rects(0);
        return rects;        
    }
    
    bool SizeAttribute::Check2(const Mat& image,ExtendedObjectInfo& rect){
        int rectArea = rect.width * rect.height;
        int imgArea = image.size().height * image.size().width;
        
        double ratio = (double)rectArea / imgArea;
        
        if ( ratio >= minSizePc && ratio <= maxSizePc){
            rect.setScoreWeight(1, Weight);
            return true;            
        }
        return false;
    }
    
    void SizeAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
    

}
