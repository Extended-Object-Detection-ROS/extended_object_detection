/*
Project: Extended Object Detection Library
Author: Moscowsky Anton
File: OpenCV blobdetector interface
*/

#ifndef _BLOB_DETECTOR_
#define _BLOB_DETECTOR_

#include "Attribute.h"

namespace eod{
    
    class BlobAttribute : public Attribute {
    public:
        
        BlobAttribute();
        
        void SetParams(int minThreshold, int maxThreshold, int blobColor, int minArea, double minCircularity, double minConvexity, double minInertiaRatio);
        
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
	
        bool Check2(const cv::Mat& image, ExtendedObjectInfo &rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:
        cv::SimpleBlobDetector::Params params;
        cv::Ptr<cv::SimpleBlobDetector> detector;   
        
    };
}

#endif // _BLOB_DETECTOR_
