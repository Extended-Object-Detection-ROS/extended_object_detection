#ifndef _TRACKER_
#define _TRACKER_

#include "SimpleObject.h"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

namespace eod{
    
    enum TrackStatus{
        DETECTED=0,
        TRACKED,
        LOST
    };
    
    class Track {
    public:
        int id;
        ExtendedObjectInfo current_object;
        TrackStatus status;
        std::vector<cv::Point> history_image;        
        
        Track(int, ExtendedObjectInfo, std::string);        
        void addObject(ExtendedObjectInfo, TrackStatus);
                
                
        void initTracker(const cv::Mat&);
        void updateTracker(const cv::Mat&);
    private:        
        
        cv::Ptr<cv::Tracker> tracker;
        
        int track_counter;        
                
    };
    
    class eodTracker : public SimpleObject{
    public:
        double iou_threshold;
        double decay;  
        
        eodTracker();
        eodTracker(std::string name, std::string trackerType);
        std::vector <ExtendedObjectInfo> Identify(const cv::Mat& frame, const cv::Mat& depth, int seq = 0);
        
        void draw(cv::Mat& image, cv::Scalar col = cv::Scalar(0,255,0) );
        
    private:        
        int id_cnt;
        std::vector<Track*> tracks;
        cv::Mat_<double> closenessMap;
        cv::Mat previous_frame;
        std::string tracker_type;
        
        void createClosenessMap();
        //double closestLoc(int* row, int* col);
        void offElementsMap(int row, int col);
        
        std::vector<ExtendedObjectInfo> setTrackingResult();
    };
    
}

#endif // _TRACKER_
