/*
Project: Object detection Library
Descr: feature detector (SIFT, SURF and so on)
Author: Moscowsky Anton
*/

#ifndef _FEATURE_DETECTOR_
#define _FEATURE_DETECTOR_

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "ObjectIdentifier.h"

using namespace cv;
using namespace cv::xfeatures2d;

namespace eod{

    enum FEATURE_DETECTOR {FD_SIFT, FD_SURF, FD_ORB};
    
    class FeatureAttribute : public Attribute{
    public:	    
        /// <summary>
        /// Default constructor
        /// </summary>
        FeatureAttribute();
        
        /// <summary>
        /// Constructor from image
        /// </summary>
        FeatureAttribute(FEATURE_DETECTOR featureExtractorType, std::string image, int min_matches, double height = 0);
    
        /// <summary>
        /// Detects aruco markers with 
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);

        bool Check2(const cv::Mat& image,ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:
        FEATURE_DETECTOR featureExtractorType;
        int min_matches_to_detect;
        
        std::vector<KeyPoint> original_keypoints;
        cv::Mat original_descriptors;                        
        std::vector<Point2f> original_corners;
        std::vector<Point3f> original_corners_shifted;
        double height;
        double scale;
        cv::Mat image_or;
        
  };
  
  
  class GlobalSimpleFeatureDetector{
  public:
      
      GlobalSimpleFeatureDetector();
      
      //GlobalSimpleFeatureDetector();
      
      bool get_descriptors_and_keypoints(const cv::Mat& image, std::vector<KeyPoint> & keypoints, cv::Mat & descriptors, FEATURE_DETECTOR detector_type);
      
      std::vector<ExtendedObjectInfo> Detect(const cv::Mat& image, int seq, std::vector<KeyPoint> original_keypoints, cv::Mat original_descriptors, std::vector<cv::Point2f> original_corners, int min_matches_to_detect, FEATURE_DETECTOR detector_type, double scale, std::vector<cv::Point3f> original_corners_shifted, double Weight, bool returnContours = true);
      
      void setCamParams(cv::Mat camMat, cv::Mat distCoef);        
      bool hasCamParams();
      
  private:
      Ptr<SIFT> sift_detector;
      Ptr<SURF> surf_detector;
      Ptr<ORB> orb_detector;
      
      std::vector<KeyPoint> sift_current_keypoints;
      Mat sift_current_descriptors;
      
      std::vector<KeyPoint> surf_current_keypoints;
      Mat surf_current_descriptors;
      
      std::vector<KeyPoint> orb_current_keypoints;
      Mat orb_current_descriptors;
      
      BFMatcher bf_matcher;
      FlannBasedMatcher flann_matcher;
      
      int sift_prev_seq;
      int surf_prev_seq;
      int orb_prev_seq;
      
      cv::Mat camMat;
      cv::Mat distCoef;
      
  };
  
  extern GlobalSimpleFeatureDetector* GSFD;
    
}

#endif // _FEATURE_DETECTOR_
