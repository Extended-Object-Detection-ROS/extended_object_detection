//
// Used as reference https://github.com/lysukhin/tensorflow-object-detection-cpp

#ifndef _TF_DETECT_
#define _TF_DETECT_

#include "Attribute.h"

#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"

namespace eod{   
    
    class GlobalTFDetector{
    public:        
        
        GlobalTFDetector();
        
        GlobalTFDetector(std::string graphPath, std::string labelsPath);
        
        std::vector<ExtendedObjectInfo> detect(const cv::Mat& frame, int seq, int id_obj, double prob_score, double Weight);
        
        std::unique_ptr<tensorflow::Session> session;
        std::map<int, std::string> labelsMap;        
        
        tensorflow::Tensor tensor;
        std::vector<tensorflow::Tensor> outputs;
        //double thresholdScore = 0.5;
        
        bool inited;
        int prev_seq;
        
        std::string inputLayer;
        std::vector<std::string> outputLayer;
        
        std::string graphPath;
        std::string labelsPath;
    
    };
    
    class TensorFlowAttribute : public Attribute{    
    public:
    
        /// <summary>
        /// Default constructor
        /// </summary>    
        TensorFlowAttribute();   
        
        /// <summary>
        /// Main constructor
        /// </summary>    
        TensorFlowAttribute(std::string graphPath, std::string labelsPath, int id_obj);       
        
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
        bool Check2(const cv::Mat& image, ExtendedObjectInfo &rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:    
        int id_obj;
        GlobalTFDetector* GTFD;
            
    };
    //extern GlobalTFDetector* GTFD;
    
    extern std::vector< GlobalTFDetector* > GTFDS;
    
}

#endif //_TF_DETECT_
