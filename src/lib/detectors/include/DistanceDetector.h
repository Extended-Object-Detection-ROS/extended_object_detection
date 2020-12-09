#ifndef _DISTANCE_DETECT_
#define _DISTANCE_DETECT_

#include "Attribute.h"

namespace eod{ 
  
  class DistanceAttribute : public Attribute{
    
  public:
        
    /// <summary>
    /// Default constructor
    /// </summary>    
    DistanceAttribute();   
    
    /// <summary>
    /// Detects OK areas with given restriction on linear dimention
    /// </summary>
    /// <param name="distMin">min distance to object in meters</param>    
    /// <param name="distMin">max distance to object in meters</param>   
    DistanceAttribute(double distMin, double distMax);   
    
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
    double distMin, distMax;
        
  };

  
}

#endif // _DISTANCE_DETECT_
