#include "SizeRelations.h"

namespace eod{

    //====================================
    // SAME SIZE
    //====================================
    SizeSameRelation::SizeSameRelation(){
        Type = SIZE_SAME_R;
        error = 0;
        inited = true;
    }
    
    SizeSameRelation::SizeSameRelation(double err){
        Type = SIZE_SAME_R;
        error = err;
        inited = true;
    }

    bool SizeSameRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        double areaA = A->width * A->height;
        double areaB = B->width * B->height;        
        return areaA * (1+error) > areaB && areaA * (1-error) < areaB;
    }

    
    

}
