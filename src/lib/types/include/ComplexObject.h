/*
Project: Object Detection Lib
Author: Moscowsky Anton
File: Scene detection mechanism using theory of Subdefenite Models.
*/

#ifndef COMPLEX_OBJECT_H_
#define COMPLEX_OBJECT_H_

//#include "ObjectIdentifier.h"
#include "SimpleObject.h"
#include "Relationship.h"
#include <chrono>

namespace eod {
    
    class ComplexObject;

    class elementValue{
    public:
        elementValue();

        double inline  getTotal(){return total;}

        double endCycle();

        void addValue(double val);

    private:
        double total;
        std::vector<double> values;

    };

    // SubDef variable!
    struct objectExample{
        SimpleObject* pointerToObj;
        std::string nameInScene;
        std::vector<std::vector<ExtendedObjectInfo> > elements;
        std::vector<elementValue> value;
    };

    struct relationExample{
        RelationShip * relation;
        struct objectExample *upObj;
        struct objectExample *downObj;
        int upNo, downNo;
    };
                    
    class ComplexObject{
    public:
        std::string name;
        int ID;

        std::chrono::duration<double> allowed_time;
        
        std::vector<ExtendedObjectInfo> objects;
        
        ComplexObject();

        std::vector<struct objectExample> objectArray; // return to private

        void AddObject(SimpleObject* obj2add, std::string nameInScene);
        void AddRelation(RelationShip* rel2add, std::string nameInScene1, std::string nameInScene2);

        std::vector<std::vector<ExtendedObjectInfo> > getFullAnswer();
        
        std::vector<ExtendedObjectInfo> Identify(const cv::Mat& frame, const cv::Mat& depth, int seq, bool& full_answer );
        
        std::vector<ExtendedObjectInfo> Identify(const cv::Mat& frame, bool& full_answer);

        std::vector<ExtendedObjectInfo> IdentifyWeak(const cv::Mat& frame, double cnt = 1);        

        void draw(cv::Mat& frameTd, cv::Scalar colorIn = cv::Scalar(255,255,255), cv::Scalar colorOut = cv::Scalar(255,255,255) );

        void drawAll(cv::Mat& frameTd, cv::Scalar color = cv::Scalar(255,255,255), int tickness = 1);

        void drawOne(cv::Mat& frameTd, ExtendedObjectInfo* scene, cv::Scalar color = cv::Scalar(255,255,255) );

        void drawOne(cv::Mat& frameTd, int no, cv::Scalar color = cv::Scalar(255,255,255), int tickness = 1 );

        void prepareObjects(const cv::Mat& frame, const cv::Mat& depth, int seq = 0);
        

    private:

        int nRelations, nObjects;
        
        std::chrono::steady_clock::time_point start_ticking;
        bool finding_aborted;
        

        std::vector<struct relationExample> solvingTable;
        //vector<struct objectExample> objectArray; // return!

        std::vector<std::vector<ExtendedObjectInfo> > answerArray;

        int solveOneRelation(const cv::Mat& frame, struct relationExample* rEx);

        double solveOneRelationWeak(const cv::Mat& frame, struct relationExample* rEx);

        void SplitObj(const cv::Mat& frame, int objNum);
        bool Check();

        int deleteRepeats();

        void printSolvingTable(bool weak = false);
    };

}

#endif //COMPLEX_OBJECT_H_
