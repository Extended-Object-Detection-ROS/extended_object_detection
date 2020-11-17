#ifndef _HIST_COLOR_DETECT_
#define _HIST_COLOR_DETECT_

#include "Attribute.h"


namespace eod{  

  /// <summary>
  /// Loads histogram from given YAML file
  /// </summary>
  /// <param name="filename">Path to YAML file with histogramm</param>
  /// <param name="hist">Histogram multi dimentional matrix</param>
  /// <returns>True if loaded successfuly</returns>
  bool LoadHistFromFile(std::string filename, cv::MatND* hist);
  
  /// <summary>
  /// Savess histogram in YAML file
  /// </summary>
  /// <param name="hist">Histogram multi dimentional matrix</param>
  /// <param name="filename">Path to YAML file with histogramm</param>  
  /// <returns>True if saved successfuly</returns>
  bool SaveHistToFile(cv::MatND hist, std::string filename);
  
  
  class HistColorAttribute : public Attribute
  {
  public:
    
    /// <summary>
    /// Default constructor
    /// </summary>
    HistColorAttribute();    
    
    /// <summary>
    /// Constructor
    /// </summary>
    /// <param name="hist">Target historgam</param>
    HistColorAttribute(cv::MatND hist);
    
    ~HistColorAttribute();
    
    /// <summary>
    /// Constructor
    /// </summary>
    /// <param name="filename">Path to historgam YAML file</param>
    HistColorAttribute(std::string filename);
    
    /// <summary>
    /// Detects OK areas with given Historgam
    /// </summary>
    /// <param name="image">Destination image</param>
    /// <returns>Vector of rects of found objects</returns>
    std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
    
    bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
    
    void setHist(cv::MatND hist);
    
    void removeHist();
    
  private:
    //ColorAttribute whiteDetector;
    cv::MatND histogram;
    cv::Mat kernel;
    
  };
}

#endif
