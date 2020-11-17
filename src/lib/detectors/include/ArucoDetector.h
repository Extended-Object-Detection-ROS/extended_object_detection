#ifndef _ARUCO_DETECTOR_
#define _ARUCO_DETECTOR_

#include <opencv2/aruco.hpp>
#include <map>
#include "ObjectIdentifier.h"


namespace eod{        
    
    class GlobalArucoDetector{
    public:
        GlobalArucoDetector(cv::aruco::PREDEFINED_DICTIONARY_NAME dict);
        
        std::vector<ExtendedObjectInfo> detect(const cv::Mat& image, int ID, double seq, double markerLen = 0, double Weight = 1, bool returnContours = true);
        
        std::vector< int > markerIds; 
        std::vector< std::vector<cv::Point2f> > markerCorners;
        
        void setCamParams(cv::Mat camMat, cv::Mat distCoef);        
        bool hasCamParams();
        
    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary;    
        double prev_seq;
        cv::Mat camMat;
        cv::Mat distCoef;
        
    };
    
    extern std::map <cv::aruco::PREDEFINED_DICTIONARY_NAME, GlobalArucoDetector*> GADS;
    
    class ArucoAttribute : public Attribute{
    public:	    
        /// <summary>
        /// Default constructor
        /// </summary>
        ArucoAttribute();
        
        /// <summary>
        /// Constructor with ID
        /// </summary>
        ArucoAttribute(int dictionary, int IDmarker, double markerLen = 0);
        
        /// <summary>
        /// Detects aruco markers with 
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);

        /*
        /// <summary>
        /// 
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of appropriate objects</returns>
        vector<ExtendedObjectInfo> Check(Mat image, vector<ExtendedObjectInfo> rects);
        */
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
    private:
        
        // ID = -1 means detector search all aruco markers
        int IDmarker;
        
        cv::aruco::PREDEFINED_DICTIONARY_NAME dict;
        GlobalArucoDetector *GAD;                           
        double markerLen;
    };

}

#endif // _ARUCO_DETECTOR_
