/*
Project: Object detection Library
Author: Moscowsky Anton
File: Attribute description
*/
#ifndef _ATTRIBUTE_
#define _ATTRIBUTE_

#include "ExtendedObjectInfo.h"
#include "Filtering.h"
#include "Clusterization.h"

namespace eod{
    
    class ObjectBase;
    
    enum AttributeMode {
        DETECT,
        CHECK,
        EXTRACT
    };
            
    enum AttributeChannel {
        RGB,
        DEPTH
    };
    
    enum AttributeTypes {
        UNK_A = 0,
        HSV_COLOR_A,
        HAAR_CASCADE_A,
        SIZE_A,        
        HIST_COLOR_A,
        HOUGH_A,
        DIMEN_A,
        BASIC_MOTION_A,
        ARUCO_A,
        FEATURE_A,
        POSE_A,
        TF_A,
        DNN_A,
        QR_A,
        QR_ZBAR_A,
        LOG_AND_A,
        LOG_NOT_A,
        LOG_OR_A,
        LOG_XOR_A,
        BLOB_A,
        DEPTH_A,
        ROUGH_DIST_A,
        DIST_A,
        FACE_DLIB_A,
    };
    
    AttributeTypes getAttributeTypeFromName(std::string name);
    
    class Attribute{
        public:
            Attribute();

            int Type;
            int ID;
            std::string Name;     
            double Weight;       
            bool returnContours;
            bool return3D;
            
            std::vector<EoiFilter*> filters;     
            ClusterizationMethod* clusterization_method;
            
            /// <summary>
            /// Functon detects objects in image
            /// </summary>
            /// <param name="image">Destination image</param>
            /// <returns>Vector of rects of found objects</returns>  
            std::vector<ExtendedObjectInfo> Detect(const cv::Mat& image, int seq);
            
            virtual std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq) = 0;
            
            /// <summary>
            /// Functon checks objects params on image
            /// </summary>
            /// <param name="image">Destination image</param>
            /// <param name="rects">List of rects to be checked</param>
            /// <returns>Vector of rects of appropriate objects</returns>  	
            void Check(const cv::Mat& image, std::vector<ExtendedObjectInfo>* rects);
            
            /// <summary>
            /// Functon checks rect params on image
            /// </summary>
            /// <param name="image">Destination image</param>
            /// <param name="rect">Rect to be checked</param>
            /// <returns>True if object is presented, false overwice</returns>  	
            virtual bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect) = 0;
            
            void Extract(const cv::Mat& image, std::vector<ExtendedObjectInfo>* rects);
            
            virtual void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect) = 0;
    
            ObjectBase* parent_base;
            
            void setProbability(double prob);
                                
        protected:    
            int prev_seq;
            std::vector<ExtendedObjectInfo> saved_answer;
            
            bool inited;            
            double Probability;            
                        
        private:
            

    };
}

#endif // _ATTRIBUTE_
