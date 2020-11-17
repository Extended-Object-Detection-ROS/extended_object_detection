#ifndef _ROUGH_DIST_DETECTOR_
#define _ROUGH_DIST_DETECTOR_

#include "Attribute.h"


namespace eod{

   class RoughDistAttribute : public Attribute{
    public:	
        /// <summary>
        /// Empty constructor 
        /// </summary>	  
        RoughDistAttribute();
        
        /// <summary>
        /// Сonstructor
        /// </summary>	  
        /// <param name="realWidth">Real width of object in meters</param>
        /// <param name="realHeight">Real height of object in meters</param>	        	
        RoughDistAttribute(double realWidth, double realHeight);

        double maxSizePc;
        double minSizePc;

        /// <summary>
        /// Functon detects objects in image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>  
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
                
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);

    private:
        double realWidth, realHeight;
        std::vector<cv::Point3f> getOriginalCorners(double width, double height);
	};

}


#endif //_ROUGH_DIST_DETECTOR_
