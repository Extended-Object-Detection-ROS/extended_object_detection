#include "SpaceRelations.h"
#include "geometry_utils.h"

namespace eod{

    // ------------------------------------------------
    // IN
    // ------------------------------------------------
    SpaceInRelation::SpaceInRelation(){
        Type = SPACE_IN_R;
        inited = true;
    }
    
    bool SpaceInRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return rect_inside(A->getRect(), B->getRect());
    }
    
    // ------------------------------------------------
    // OUT
    // ------------------------------------------------
    SpaceOutRelation::SpaceOutRelation(){
        Type = SPACE_OUT_R;
        inited = true;
    }
    
    bool SpaceOutRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return intersectionOverUnion(A, B) == 0;
    }
    
    // ------------------------------------------------
    // UP
    // ------------------------------------------------
    SpaceUpRelation::SpaceUpRelation(){
        Type = SPACE_UP_R;
        inited = true;
    }
    
    bool SpaceUpRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return A->getCenter().y < B->getCenter().y;
    }
    
    // ------------------------------------------------
    // DOWN
    // ------------------------------------------------
    SpaceDownRelation::SpaceDownRelation(){
        Type = SPACE_DOWN_R;
        inited = true;
    }
    
    bool SpaceDownRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return A->getCenter().y > B->getCenter().y;
    }
    
    // ------------------------------------------------
    // LEFT
    // ------------------------------------------------
    SpaceLeftRelation::SpaceLeftRelation(){
        Type = SPACE_LEFT_R;
        inited = true;
    }
    
    bool SpaceLeftRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return A->getCenter().x < B->getCenter().x;
    }
    
    // ------------------------------------------------
    // RIGHT
    // ------------------------------------------------
    SpaceRightRelation::SpaceRightRelation(){
        Type = SPACE_RIGHT_R;
        inited = true;
    }
    
    bool SpaceRightRelation::checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B){
        return A->getCenter().x > B->getCenter().x;
    }

 


}
