#include "LogicRelations.h"

namespace eod {

    //=========================================
    // AND
    //=========================================

    AndRelation::AndRelation(){
        Type = LOG_AND_R;
        inited = false;
    }

    AndRelation::AndRelation(RelationShip *r1, RelationShip *r2){
        R1 = r1;
        R2 = r2;
        Type = LOG_AND_R;
        inited = true;
    }
    

    bool AndRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return (R1->checkRelation(image, A, B) && R2->checkRelation(image, A, B));
    }
    
    //=========================================
    // OR
    //=========================================

    OrRelation::OrRelation(){
        Type = LOG_OR_R;
        inited = false;
    }

    OrRelation::OrRelation(RelationShip *r1, RelationShip *r2){
        R1 = r1;
        R2 = r2;
        Type = LOG_OR_R;
        inited = true;
    }
    

    bool OrRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return (R1->checkRelation(image, A, B) || R2->checkRelation(image, A,B));
    }    

    //=========================================
    // NOT
    //=========================================

    NotRelation::NotRelation(){
        Type = LOG_NOT_R;
        inited = false;
    }

    NotRelation::NotRelation(RelationShip *r){
        R = r;
        Type = LOG_NOT_R;
        inited = true;
    }

    bool NotRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return !(R->checkRelation(image, A, B));
    }


}
