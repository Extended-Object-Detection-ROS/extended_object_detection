/*
Project: Extended Object Detection Library
Author: Moscowsky Anton
File: OpenCV cascade classifier interface
*/

#ifndef _SHAPE_DETECTOR_
#define _SHAPE_DETECTOR_

#include "ObjectIdentifier.h"
//#include <opencv\cvwimage.h>

namespace eod{

	class HaarCascadeAttribute : public Attribute {
	public:
	
        std::string cascade_name;

        /// <summary>
        /// Empty constructor.
        /// /// </summary>
        HaarCascadeAttribute();

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="name">Path to cascade</param>
        /// <returns>Vector of rects of found objects</returns>
        HaarCascadeAttribute(std::string name);
        
        ~HaarCascadeAttribute();

        /// <summary>
        /// Functon detects objects in image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
	
        bool Check2(const cv::Mat& image, ExtendedObjectInfo &rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
	
	private:
        cv::CascadeClassifier shapeClassifier;

	};	

}

#endif 
