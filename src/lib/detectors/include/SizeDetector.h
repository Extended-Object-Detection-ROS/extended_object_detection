#ifndef _SIZE_DETECTOR_
#define _SIZE_DETECTOR_

#include "ObjectIdentifier.h"


namespace eod{

   class SizeAttribute : public Attribute{
    public:	
        /// <summary>
        /// Empty constructor 
        /// </summary>	  
        SizeAttribute();
        
        /// <summary>
        /// Сonstructor, inits Percent detection
        /// </summary>	  
        /// <param name="mSpc">Minimal value of area in %</param>
        /// <param name="MSpc">Maximal value of area in %</param>	        	
        SizeAttribute(double mSpc, double MSpc );

        double maxSizePc;
        double minSizePc;

        /// <summary>
        /// Functon detects objects in image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>  
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        /*
        /// <summary>
        /// Functon checks objects params on image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <param name="rects">List of rects to be checked</param>
        /// <returns>Vector of rects of appropriate objects</returns>  
        vector<ExtendedObjectInfo> Check(Mat image,vector<ExtendedObjectInfo> rects);
        */
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);

    private:
	};

}


#endif
