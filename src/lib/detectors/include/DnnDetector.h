#ifndef _DNN_DETECTOR_H_
#define _DNN_DETECTOR_H_

#include "Attribute.h"
#include <opencv2/dnn.hpp>

namespace eod{       
    
    enum DNN_FRMWRK{
        TF_DNN_FW,// tensorflow
        DN_DNN_FW,// darknet
    };
    
    // GLOBAL DNN DETECTOR
    
    class GlobalDnnDetector{
    public:
        GlobalDnnDetector();
        GlobalDnnDetector(std::string framework_name, std::string weights_file, std::string config_file, int inpWidth, int inpHeight, std::string labelMap, bool forceCuda);                        
        std::vector<ExtendedObjectInfo> detect(const cv::Mat& image, int seq, double Weight);
        
        bool inited;
        std::string framework_name;
        std::string weights_file;
        std::string config_file;
        std::string label_file;
        int inpWidth;
        int inpHeight;
        
    private:
        DNN_FRMWRK framework;
        cv::dnn::Net net;
        
        std::vector<ExtendedObjectInfo> saved_answer;
        int prev_seq;
        std::map<int, std::string> labelMap;
        bool isLabelMap;
        //bool forceCuda;
    };
    
    extern std::vector <GlobalDnnDetector*> GDNNDS;
    // EXTENDED OBJECT DETECTION LIB INTERFACE
    
    class DnnAttribute : public Attribute{
    public:	    
        /// <summary>
        /// Default constructor
        /// </summary>
        DnnAttribute();
        
        DnnAttribute(int object_id, std::string framework_name, std::string weights_file, std::string config_file, int inpWidth, int inpHeight, std::string labelMap, bool forceCuda =false);
        
        /// <summary>
        /// Detects aruco markers with 
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        /// <summary>
        /// 
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of appropriate objects</returns>        
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:
        GlobalDnnDetector* GDNND;
        int object_id;
        
    };
    
    
}
#endif //_DNN_DETECTOR_H_
