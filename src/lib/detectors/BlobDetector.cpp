#include "BlobDetector.h"

using namespace std;
using namespace cv;

namespace eod{
    
    BlobAttribute::BlobAttribute(){
        Type = BLOB_A;
        inited = false;
    }
    
    void BlobAttribute::SetParams(int minThreshold, int maxThreshold, int blobColor, int minArea, double minCircularity, double minConvexity, double minInertiaRatio){
        params.minThreshold = minThreshold;
        params.maxThreshold = maxThreshold;
        
        if( blobColor == 0)
            params.filterByColor = false;
        else{
            params.filterByColor = true;
            params.blobColor = blobColor;
        }
            
        if( minArea == 0 )
            params.filterByArea = false;
        else{
            params.filterByArea = true;
            params.minArea = minArea;
        }
        
        if( minCircularity == 0 )
            params.filterByCircularity = false;
        else{
            params.filterByCircularity = true;
            params.minCircularity = minCircularity;
        }
        
        if( minConvexity == 0 )
            params.filterByConvexity = false;
        else{
            params.filterByConvexity = true;
            params.minConvexity = minConvexity;
        }
        
        if( minInertiaRatio == 0 )
            params.filterByInertia = false;
        else{
            params.filterByInertia = true;
            params.minInertiaRatio = minInertiaRatio;
        }
            
        detector = SimpleBlobDetector::create(params);   
        
        inited = true;
    }
    
    vector<ExtendedObjectInfo> BlobAttribute::Detect2(const cv::Mat& image, int seq){
        vector<ExtendedObjectInfo> res;
        if( !inited )
            return res;
        
        vector<KeyPoint> keypoints;
        detector->detect( image, keypoints);
        
        for( size_t i = 0 ; i < keypoints.size() ; i++ ){
            int x = (int)keypoints[i].pt.x;
            int y = (int)keypoints[i].pt.y;
            int r = (int)keypoints[i].size;
            ExtendedObjectInfo tmp = ExtendedObjectInfo(x-r, y-r, 2*r, 2*r);
            tmp.setScoreWeight(1, Weight);
            res.push_back(tmp);                    
        }
        return res;
    }
    
    bool BlobAttribute::Check2(const cv::Mat& image, ExtendedObjectInfo &rect){
        return false;
    }
    
    void BlobAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
    
}
