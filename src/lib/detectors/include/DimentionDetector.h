#ifndef _DIMENTION_DETECT_
#define _DIMENTION_DETECT_

#include "Attribute.h"

namespace eod{ 
  
  class DimentionAttribute : public Attribute{
    
  public:
    
    
    /// <summary>
    /// Default constructor
    /// </summary>    
    DimentionAttribute();   
    
    /// <summary>
    /// Detects OK areas with given restriction on linear dimention
    /// </summary>
    /// <param name="ratioMin">min Width / Height, if it 0 tehre is no limit</param>    
    /// <param name="ratioMax">max Width / Heigh, if it 0 tehre is no limit</param>   
    DimentionAttribute(double ratioMin, double ratioMax);   
    
    /// <summary>
    /// Detects OK areas with given restriction on linear dimention
    /// </summary>
    /// <param name="image">Destination image</param>
    /// <returns>Vector of rects of found objects</returns>
    std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
    
    /// <summary>
    /// Сhecks one object with given restriction on linear dimention
    /// </summary>
    /// <param name="image">Destination image</param>
    /// <param name="rect">Rect to be checked</param>
    /// <returns>True if object is presented, false overwice</returns>  	
    bool Check2(const cv::Mat& image,ExtendedObjectInfo& rect);
    
    void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
    
  private:
    double ratioMin, ratioMax;
        
  };

  
}

#endif
