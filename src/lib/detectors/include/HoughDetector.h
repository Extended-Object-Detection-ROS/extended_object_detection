/*
Project: Object detection Library
Author: Moscowsky Anton
File: Header file describes classes of Hough (form) detection
*/

#ifndef _HOUGH_DETECTOR_
#define _HOUGH_DETECTOR_

#include "Attribute.h"

namespace eod{    

    enum HOUGH_TYPE {CIRCLE = 0, LINE = 1};

    class HoughAttribute : public Attribute {
      
    public:
        /// <summary>
        /// Empty constructor.
        /// </summary>
        HoughAttribute();
	
        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="Type">Type of detecting objects, must be from HOUGHT_TYPE</param>
        HoughAttribute(int Type);
	
        /// <summary>
        /// Constructor for Circle.
        /// </summary>
        /// <param name="d">The inverse ratio of resolution</param>
        /// <param name="md">Minimum distance between detected centers</param>
        /// <param name="p1">upper threshold for the interval Canny ende detector</param>
        /// <param name="p2">threshold for center detection</param>
        /// <param name="mr">minimum radio to be detected</param>
        /// <param name="Mr">maximun radius to be detected</param>
        HoughAttribute(double d, double md, double p1, double p2, int mr, int Mr);
	
        /// <summary>
        /// Constructor for Line.
        /// </summary> 
        /// <param name="rho">The resolution of the parameter r in pixels.</param>
        /// <param name="theta">The resolution of the parameter \theta in radians.</param>
        /// <param name="threshold">The minimum number of intersections to “detect” a line</param>
        /// <param name="minLinLength">The minimum number of points that can form a line. Lines with less than this number of points are disregarded.</param>
        /// <param name="maxLineGap">The maximum gap between two points to be considered in the same line.</param>
        HoughAttribute(int rho, float theta, int threshold, int minLinLength, int maxLineGap);

        /// <summary>
        /// Functon detects objects in image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
	
        /*
        /// <summary>
        /// Functon checks objects params on image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <param name="rects">List of rects to be checked</param>
        /// <returns>Vector of rects of appropriate objects</returns>
        vector<ExtendedObjectInfo> Check(Mat image, vector<ExtendedObjectInfo> rects);
        */
        
        bool Check2(const cv::Mat& image,ExtendedObjectInfo& rect);

        void SetParamsCircle(double d, double md, double p1, double p2, int mr, int Mr);

    private:
        int TypeH;
        bool initialized;
	
        // CIRCLE PARAMS
        double dp;
        double min_dist;
        double param1;
        double param2;
        int minradius;
        int maxradius;

        // LINE PARAMS
        int rho;
        float theta;
        int threshold;
        int minLinLength;
        int maxLineGap;
	
        std::vector<ExtendedObjectInfo> findCircles(const cv::Mat& image);

        std::vector<ExtendedObjectInfo> findLines(const cv::Mat& image);

    };

}

#endif
