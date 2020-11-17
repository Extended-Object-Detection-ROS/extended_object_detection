#include "ImageRangeRelation.h"
#include "geometry_utils.h"
#include <math.h>

namespace eod{
    ImageRangeRelation::ImageRangeRelation(){
        inited = false;
        Type = IM_RANGE_R;
    }
    
    ImageRangeRelation::ImageRangeRelation(int px_dist_low_, int px_dist_high_){
        inited = true;
        Type = IM_RANGE_R;
        sub_type = PURE_PIXEL;
        px_dist_high = px_dist_high_;
        px_dist_low = px_dist_low_;
    }
    
    ImageRangeRelation::ImageRangeRelation(double dist_, double prob_){
        inited = true;
        Type = IM_RANGE_R;
        sub_type = IM_DIAG_RELATIVE;
    }
    
    ImageRangeRelation::ImageRangeRelation(TiXmlElement* relation_tag){
        Type = IM_RANGE_R;
        if( relation_tag->Attribute("pxDistLow", &px_dist_low) && relation_tag->Attribute("pxDistHigh", &px_dist_high) ){
            sub_type = PURE_PIXEL;
            inited = true;
        }
        else if( relation_tag->Attribute("distDiagObj", &dist) && relation_tag->Attribute("probDiagObj", &prob) ){
            sub_type = OBJ_DIAG_RELATIVE;            
            inited = true;
        }
        else if( relation_tag->Attribute("distDiagIm", &dist) && relation_tag->Attribute("probDiagIm", &prob) ){
            sub_type = IM_DIAG_RELATIVE;            
            inited = true;
        }
        else{
            printf("ImageRangeRelation has not initied propelly!");
            inited = false;
        }
    }
    
    bool ImageRangeRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        if( inited ){
            if( sub_type == PURE_PIXEL ){
                float range_ = range(A->getCenter(), B->getCenter()); 
                if( range_ > px_dist_high)
                    return false;
                if( range_ < px_dist_low)
                    return false;                
                return true;                
            }
            if( sub_type == IM_DIAG_RELATIVE ){
                double diag = sqrt( image.size().width * image.size().width + image.size().height * image.size().height);
                double range_ = range(A->getCenter(), B->getCenter()); 
                if( range_ > diag * dist * (1 + prob) )
                    return false;
                if( range_ < diag * dist * (1 - prob) )
                    return false;
                return true;                
            }
            if( sub_type == OBJ_DIAG_RELATIVE ){
                double diag = sqrt( A->width * A->width + A->height * A->height);
                double range_ = range(A->getCenter(), B->getCenter()); 
                if( range_ > diag * dist * (1 + prob) )
                    return false;
                if( range_ < diag * dist * (1 - prob) )
                    return false;
                return true;                
            }
            printf("ImageRangeRelation has unknown sub type!\n");
            return false;
        }
        else{
            printf("ImageRangeRelation isn't inited!\n");
            return false;
        }
    }
}
