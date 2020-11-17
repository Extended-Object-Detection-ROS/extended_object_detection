#ifndef TD_RANGE_RELATION_H
#define TD_RANGE_RELATION_H

#include "Relationship.h"

namespace eod{
    
    enum THREE_DIM_RANGE_REL_SUBTYPE{
        PURE_METERS,
        //OBJ_DIAG_RELATIVE, //rename
    };
    
    class ThreeDimRangeRelation : public RelationShip{
    public:
        ThreeDimRangeRelation();
        ThreeDimRangeRelation(TiXmlElement* relation_tag);
        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
    private:
        int sub_type;
        double dist_low, dist_high;
        double dist, prob;
        
    };
    
}

#endif //TD_RANGE_RELATION_H
