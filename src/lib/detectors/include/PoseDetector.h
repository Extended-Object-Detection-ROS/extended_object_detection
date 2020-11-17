/*
Project: Object detection Library
Author: Moscowsky Anton
File: Header file describes class for Pose detection
*/

#ifndef _POSE_DETECTOR_
#define _POSE_DETECTOR_

#include "ObjectIdentifier.h"


// verify that center in given ranges
namespace eod{

   class PoseAttribute : public Attribute{
    public:
	
	/// <summary>
	/// Empty constructor 
	/// </summary>	  
	PoseAttribute();
	
	/// <summary>
	/// Сonstructor, inits Percent detection
	/// </summary>	  	
        PoseAttribute(double x_min, double x_max, double y_min, double y_max );        

        double x_min, x_max, y_min, y_max;

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
    
	bool Check2(const cv::Mat& image, ExtendedObjectInfo &rect);
    
    void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);

    private:
	};

}


#endif
