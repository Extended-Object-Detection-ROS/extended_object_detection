#ifndef _LOGIC_ATTRIBUTE_
#define _LOGIC_ATTRIBUTE_

#include "Attribute.h"

namespace eod{
    
    // ------------------------
    // AND
    // ------------------------
    class AndAttribute : public Attribute{
    public:
        
        AndAttribute();
        AndAttribute(Attribute*, Attribute*, double);
        
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:
        Attribute* attributeA;
        Attribute* attributeB;
        double iou_threshold;
        
    };
    
    // ------------------------
    // NOT
    // ------------------------
    class NotAttribute : public Attribute{
    public:
        
        NotAttribute();
        NotAttribute(Attribute*);
        
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:
        Attribute* attribute;        
    };
    
    
    // ------------------------
    // OR
    // ------------------------
    class OrAttribute : public Attribute{
    public:
        
        OrAttribute();
        OrAttribute(Attribute*, Attribute*, double);
        
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
    private:
        Attribute* attributeA;
        Attribute* attributeB;
        double iou_threshold;    
    };
    
    
    // ------------------------
    // XOR
    // ------------------------
    
}

#endif 
