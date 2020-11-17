#include "ThreeDimRangeRelation.h"
#include "geometry_utils.h"

namespace eod{
    
    ThreeDimRangeRelation::ThreeDimRangeRelation(){
        inited = false;
        Type = TD_RANGE_R;
    }
    
    ThreeDimRangeRelation::ThreeDimRangeRelation(TiXmlElement* relation_tag){
        Type = TD_RANGE_R;
        if( relation_tag->Attribute("distLow", &dist_low) && relation_tag->Attribute("distHigh", &dist_high) ){
            inited = true;
            sub_type = PURE_METERS;
        }
//         else if( relation_tag->Attribute("distDiag", &dist) && relation_tag->Attribute("probDiag", &prob) ){
//             inited = true;
//             sub_type = OBJ_DIAG_RELATIVE;
//         }
        else{
            printf("ThreeDimRangeRelation has not initied propelly!");
            inited = false;
        }   
    }
    
    bool ThreeDimRangeRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        if(inited){
            
            if( A->tvec.size() == 0 || B->tvec.size() == 0){
                printf("Object on input to ThreeDimRangeRelation has not 3d translation yet!");
                return false;
            }
            if( A->tvec[0][3] == 1 || B->tvec[0][3] == 1){
                printf("Object on input to ThreeDimRangeRelation has unit 3d translation!");
                return false;
            }
            double distance = range_v3d(A->tvec[0], B->tvec[0]);            
            if(sub_type == PURE_METERS){
                if( distance > dist_high )            
                    return false;
                if( distance < dist_low )
                    return false;
                return true;
            }
//             if(sub_type == OBJ_DIAG_RELATIVE){
//                 
//             }
            else{
                printf("ThreeDimRangeRelation has unknown sub type!\n");
                return false;
            }
        }
        else{
            printf("ThreeDimRangeRelation isn't inited!\n");
            return false;
        }
    }
}
