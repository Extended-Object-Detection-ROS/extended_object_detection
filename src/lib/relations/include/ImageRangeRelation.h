#ifndef IMAGE_RANGE_RELATION_H
#define IMAGE_RANGE_RELATION_H

#include "Relationship.h"

namespace eod{
    
    enum IM_RANGE_REL_SUBTYPE{
        PURE_PIXEL,
        IM_DIAG_RELATIVE,
        OBJ_DIAG_RELATIVE,
    };
    
    class ImageRangeRelation : public RelationShip{
    public:        
        ImageRangeRelation();
        ImageRangeRelation(TiXmlElement* relation_tag);
        ImageRangeRelation(int px_dist_low, int px_dist_high);
        ImageRangeRelation(double dist, double prob);
        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
    private:
        int sub_type;
        
        int px_dist_low, px_dist_high;
        double dist, prob;
    };
}

#endif //IMAGE_RANGE_RELATION_H
