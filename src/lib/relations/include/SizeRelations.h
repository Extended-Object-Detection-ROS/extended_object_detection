#ifndef SIZERELATIONS_H
#define SIZERELATIONS_H

#include "Relationship.h"

namespace eod{

    // Same
    class SizeSameRelation : public RelationShip{
    public:
        SizeSameRelation();
        SizeSameRelation(double error);
        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);

    private:
        double error;
    };
    
    // Bigger
    class SizeBiggerRelation : public RelationShip{
    public:
        SizeBiggerRelation();
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);        
    };
    
    // Smaller
    class SizeSmallerRelation : public RelationShip{
    public:
        SizeSmallerRelation();
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);        
    };    
    
    // Percent
    class SizePercentRelation : public RelationShip{
    public:
        SizePercentRelation();
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);        
    };

}


#endif // SIZERELATIONS_H
