/*
Project: Object detection Library
Descr: header describes color attributes and detection
Author: Moscowsky Anton
*/

#ifndef _HSV_COLOR_DETECTOR_
#define _HSV_COLOR_DETECTOR_

//#include "ClusterForel.h"
#include "ObjectIdentifier.h"


namespace eod{

//#define SCREEN_OUTPUT_COLOR
#define WHITE 0,255,0,15,240,255

  class HsvColorAttribute : public Attribute{
    public:	    
        /// <summary>
        /// Default constructor, inits checkProbs at 0.75 and all parameters to max range 0~255
        /// </summary>
        HsvColorAttribute();

        /// <summary>
        /// Constructor, sets color ranges in HSV space
        /// </summary>
        /// <param name="hm">Minmal value of hue parameter</param>
        /// <param name="hM">Maximal value of hue parameter</param>
        /// <param name="sm">Minmal value of saturation parameter</param>
        /// <param name="sM">Maximal value of saturation parameter</param>
        /// <param name="vm">Minmal value of 'value' parameter</param>
        /// <param name="vM">Maximal value of 'value' parameter</param>
        HsvColorAttribute(int hm, int hM, int sm, int sM, int vm, int vM);
        
        ~HsvColorAttribute();

        int Hmin, Hmax;
        int Smin, Smax;
        int Vmin, Vmax;

        int typeDetect;

        /// <summary>
        /// Detects objects with given color on image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);        
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:      
        cv::Mat kernel;        
        
  };

}

#endif //_HSV_COLOR_DETECTOR_
