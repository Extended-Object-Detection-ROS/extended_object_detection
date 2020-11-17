#ifndef SPACERELATIONS_H
#define SPACERELATIONS_H

#include "Relationship.h"

namespace eod{

    // In
    class SpaceInRelation : public RelationShip{
    public:
        SpaceInRelation();        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
    };

    // Out
    class SpaceOutRelation : public RelationShip{
    public:
        SpaceOutRelation();        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
    };
    
    // Up
    class SpaceUpRelation : public RelationShip{
    public:
        SpaceUpRelation();        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
    };
    
    // Down
    class SpaceDownRelation : public RelationShip{
    public:
        SpaceDownRelation();        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
    };
    
    // Left
    class SpaceLeftRelation : public RelationShip{
    public:
        SpaceLeftRelation();        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
    };
    
    // Right
    class SpaceRightRelation : public RelationShip{
    public:
        SpaceRightRelation();        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
    };

}


#endif // SPACERELATIONS_H
