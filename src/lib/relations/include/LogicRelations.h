#ifndef LOGICRELATIONS_H
#define LOGICRELATIONS_H

#include "Relationship.h"

namespace eod{

    class AndRelation : public RelationShip{
    public:
        AndRelation();
        AndRelation(RelationShip* r1, RelationShip* r2);
        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
        
    private:
        RelationShip* R1;
        RelationShip* R2;

    };
    
    class OrRelation : public RelationShip{
    public:
        OrRelation();
        OrRelation(RelationShip* r1, RelationShip* r2);
        
        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
        
    private:
        RelationShip* R1;
        RelationShip* R2;

    };

    class NotRelation : public RelationShip{
    public:
        NotRelation();
        NotRelation(RelationShip * r);

        bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B);
        
    private:
        RelationShip* R;
    };

}

#endif // LOGICRELATIONS_H
