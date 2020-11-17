
#ifndef _BASIC_MOTION_DETECTOR_
#define _BASIC_MOTION_DETECTOR_

#include "Attribute.h"


#define BLUR_KER 21

namespace eod{

//#define SCREEN_OUTPUT_BASIC

  class BasicMotionAttribute : public Attribute{
    public:

        /// <summary>
        /// Default constructor, inits checkProbs at 0.75 and all parameters to max range 0~255
        /// </summary>
        BasicMotionAttribute();

        /// <summary>
        /// Detects objects with given color on image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);

        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
    
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:

    cv::Mat firstFrame;
  };

}

#endif // _BASIC_MOTION_DETECTOR_
