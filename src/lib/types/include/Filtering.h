#ifndef _FILTERING_H_
#define _FILTERING_H_

#include "ExtendedObjectInfo.h"

namespace eod{
    
    enum FilterTypes{
        UNK_F,
        INSIDER_F,
        IOU_F,
        ROI_F,
    };
    
    FilterTypes getFilterTypeFromString(std::string str);
    
    
    //-----------
    // Base class
    //-----------
    class EoiFilter{
    public: 
        EoiFilter();
        
        virtual void Filter(std::vector<ExtendedObjectInfo>* src) = 0;
        
    protected:
        FilterTypes type;
        
    };
    
    //-----------
    // Insider
    //-----------
    class InsiderFilter : public EoiFilter{
    public:
        InsiderFilter();
        
        void Filter(std::vector<ExtendedObjectInfo>* src);
    };
    
    //-----------
    // IOU
    //-----------
    class IOUFilter : public EoiFilter{
    public:
        IOUFilter(double iou_threshold);
        
        void Filter(std::vector<ExtendedObjectInfo>* src);
    private:
        double iou_threshold;
        
    };
    
    //-----------
    // ROI
    // ----------
    class ROIFilter : public EoiFilter{
    public:
        ROIFilter(cv::Rect ROI);
        
        void Filter(std::vector<ExtendedObjectInfo>* src);
    private:
        cv::Rect ROI;
    };
    
}

#endif
