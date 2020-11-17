#include "ComplexObject.h"
#include "drawing_utils.h"

using namespace cv;
using namespace std;


namespace eod{

    // ~~~~~~~~~~~~~~~~~~~~~~~~~
    // ELEMENTS VALUES LOGIC
    // ~~~~~~~~~~~~~~~~~~~~~~~~~

    elementValue::elementValue(){
        total = 1;
    }

    double elementValue::endCycle(){
        double prevTotal = total;
        total = 0;
        for( size_t i = 0 ; i < values.size() ; i++){
            total += values.at(i);
        }
        total /= values.size();
        values.clear();
        return prevTotal - total;
    }

    void elementValue::addValue(double val){
        values.push_back(val);
    }

    
    ComplexObject::ComplexObject() {
        nObjects = 0;
        nRelations = 0;
        finding_aborted = false;
        allowed_time = chrono::duration<double>(0.25);
    }
    
    vector<vector<ExtendedObjectInfo> > ComplexObject::getFullAnswer(){
        return answerArray;
    }

    // ================
    // ADD OBJECT
    // ================
    void ComplexObject::AddObject(SimpleObject* obj2add, string nameInScene){
        nObjects++;
        struct objectExample objEx;
        objEx.nameInScene = nameInScene;
        objEx.pointerToObj = obj2add;

        objectArray.push_back(objEx);
    }

    // ================
    // ADD RELATION
    // ================
    void ComplexObject::AddRelation(RelationShip* rel2add, string nameInScene1, string nameInScene2){
        struct relationExample relEx;
        nRelations++;
        relEx.relation = rel2add;

        for( size_t i = 0 ; i < objectArray.size(); i++){
            if(nameInScene1 == objectArray.at(i).nameInScene ){
                relEx.upObj = &(objectArray.at(i));
                relEx.upNo = i;
            }
            if(nameInScene2 == objectArray.at(i).nameInScene ){
                relEx.downObj = &(objectArray.at(i));
                relEx.downNo = i;
            }
        }

        solvingTable.push_back(relEx);
    }

    // ================
    // One function iface
    // ================
    
    std::vector<ExtendedObjectInfo> ComplexObject::Identify(const cv::Mat& frame, const cv::Mat& depth, int seq, bool& full_answer ){
        prepareObjects(frame, depth, seq);
        return Identify(frame, full_answer);        
    }
    
    // ================
    // PREPARE
    // ================
    void ComplexObject::prepareObjects(const cv::Mat& frame, const cv::Mat& depth, int seq ){
        answerArray.clear();
        // get rects
        for( size_t i = 0 ; i < objectArray.size(); i++){
            if( !objectArray.at(i).pointerToObj->identified ){                  
                objectArray.at(i).pointerToObj->Identify(frame, depth, seq);
                objectArray.at(i).pointerToObj->identified = true;
            }
            objectArray.at(i).elements.clear();            
            objectArray.at(i).elements.push_back(objectArray.at(i).pointerToObj->objects );            
            objectArray.at(i).value = vector<elementValue>(objectArray.at(i).elements.back().size());

        }
        for( size_t i = 0 ; i < objectArray.size(); i++){
            objectArray.at(i).pointerToObj->identified = false;
        }
    }

    // ====================
    // IDENTIFY WEAK
    // ====================
    vector<ExtendedObjectInfo> ComplexObject::IdentifyWeak(const cv::Mat& frame, double cnt){
        double changes;
        do{
            printSolvingTable(true);
            changes = 0;
            for( size_t i = 0 ; i < solvingTable.size() ; i ++){
                //printSolvingTable(); // debug output
                solveOneRelationWeak(frame, &solvingTable.at(i) );
            }
            for(size_t k = 0 ; k < objectArray.size() ; k++){
                for(size_t j = 0 ; j < objectArray.at(k).value.size();j++){
                    changes += objectArray.at(k).value.at(j).endCycle();
                }

            }


        }while(changes > 0);

        // split here must be here

        vector<ExtendedObjectInfo> tmp;
        for(size_t k = 0 ; k < objectArray.size() ; k++){
            for(size_t j = 0 ; j < objectArray.at(k).elements.back().size();j++){
                if(objectArray.at(k).value.at(j).getTotal() >= cnt){
                    tmp.push_back( objectArray.at(k).elements.back().at(j) );
                }
            }
        }
        answerArray.push_back(tmp);

        vector<ExtendedObjectInfo> vec_res;
        for( size_t i = 0 ; i < answerArray.size(); i ++){
            ExtendedObjectInfo result;
            for( size_t j = 0 ; j < answerArray.at(i).size(); j++ ){
                result = result | answerArray.at(i).at(j).getRect();
            }
            vec_res.push_back(result);
        }

        return vec_res;

    }

    // ================
    // IDENTIFY
    // ================
    vector<ExtendedObjectInfo> ComplexObject::Identify(const cv::Mat& frame, bool& full_answer){        
        // solving proc
        //printSolvingTable();
        int changes;
        do{
            changes = 0;
            for( size_t i = 0 ; i < solvingTable.size() ; i ++){                               
                //printSolvingTable(); // debug output
                changes += solveOneRelation(frame, &solvingTable.at(i) );
            }
        }while(changes > 0);
        
        /*
        printf("First part is done!\n");
        for( size_t i = 0 ; i < objectArray.size() ; i++){
            printf("Objects at %i candidate %i\n",i,objectArray.at(i).elements.back().size());
        }
        */
#ifdef DEBUG_PRINT_NM
        printSolvingTable(); // debug output
#endif
        if( ! Check() ){
            finding_aborted = false;
            start_ticking = chrono::steady_clock::now();
            // here should be dat thing [two years later: I forgot what thing]
            for( size_t i = 0 ; i < objectArray.size() ; i++){
                int numAns = objectArray.at(i).elements.back().size();
                if( numAns > 1){
                    SplitObj(frame, i);
                }
                if( finding_aborted ){
                    printf("Complex Objects Searching Aborted!\n");
                    // add all of them
                    vector<ExtendedObjectInfo> vec_res;                
                    ExtendedObjectInfo summ_of_all = objectArray.at(0).elements.back().at(0);
                    for( size_t i = 0 ; i < objectArray.size() ; i++){
                        for( size_t j = 0 ; j < objectArray.at(i).elements.back().size(); j++ ){
                            summ_of_all = summ_of_all | objectArray.at(i).elements.back().at(j);
                        }
                    }
                    vec_res.push_back(summ_of_all);
                    summ_of_all.print("Summ is: ");                    
                    
                    objects = vec_res;
                    full_answer = false;
                    return vec_res;
                }
            }                        
        }
        else{
            vector<ExtendedObjectInfo> tmp;
            for(size_t k = 0 ; k < objectArray.size() ; k++){
                tmp.push_back( objectArray.at(k).elements.back().at(0) );
            }
            answerArray.push_back(tmp);
        }
        //printf("Second part is done!\n");
        
        int repa = deleteRepeats();
        
        // add all of them
        vector<ExtendedObjectInfo> vec_res;        
        
        for( size_t i = 0 ; i < answerArray.size(); i ++){
            if( answerArray.at(i).size() > 0 ){
                ExtendedObjectInfo result = answerArray.at(i).at(0).getRect();
                for( size_t j = 1 ; j < answerArray.at(i).size(); j++ ){
                    result = result | answerArray.at(i).at(j).getRect();                
                }                
                vec_res.push_back(result);
            }                        
        }
        objects = vec_res;
        full_answer = true;
        return vec_res;
    }

    // ================
    // SPLIT
    // ================
    void ComplexObject::SplitObj(const cv::Mat& frame, int objNum){
        if( finding_aborted )
            return;
        // check ellapsed time        
        chrono::steady_clock::time_point tik = chrono::steady_clock::now();
        chrono::duration<double> elapsed_time = tik - start_ticking;
        if(elapsed_time > allowed_time){
            finding_aborted = true;
            return;
        }
        
        /*
        for( size_t i = 0 ; i < objectArray.size() ; i++){
            printf("%i ",objectArray.at(i).elements.back().size());
        }
        printf("\n");
        */
        vector<vector<ExtendedObjectInfo> > temp(objectArray.size());
        //temp.swap(objectArray.at(objNum).elements.back());
        for(size_t i = 0 ; i < temp.size() ; i ++ ){
            temp.at(i).assign(objectArray.at(i).elements.back().begin(),objectArray.at(i).elements.back().end());
        }

        for( size_t i = 0 ; i < temp.at(objNum).size() ; i ++ ){
            //printf("\n size %i",temp.at(objNum).size());
            objectArray.at(objNum).elements.back().clear();
            objectArray.at(objNum).elements.back().push_back(temp.at(objNum).at(i));

            //printSolvingTable(); // debug output
            int changes;
            do{
                changes   = 0;
                for( size_t k = 0 ; k < solvingTable.size() ; k ++){
                    changes += solveOneRelation(frame, &solvingTable.at(k) );
#ifdef DEBUG_PRINT_NM
                    printf("\n%i %i %i",objNum,i,k);
                    printSolvingTable(); // debug output
#endif
                }
            }while(changes > 0);

            if(Check()){
                vector<ExtendedObjectInfo> tmp;
                for(size_t k = 0 ; k < objectArray.size() ; k++){
                    tmp.push_back( objectArray.at(k).elements.back().at(0) );
                }
                answerArray.push_back(tmp);

            }
            else{
                for(size_t k = objNum+1 ; k < objectArray.size() ; k++){
                    int numAns = objectArray.at(k).elements.back().size();
                    if( numAns > 1){
                        SplitObj(frame, k);
                    }
                }
            }
            for(size_t i = 0 ; i < temp.size() ; i ++ ){
                //temp.at(i).assign(objectArray.at(objNum).elements.back().begin(),objectArray.at(objNum).elements.back().end());
                objectArray.at(i).elements.back().assign(temp.at(i).begin(),temp.at(i).end());
            }
        }
#ifdef DEBUG_PRINT_NM
        printf("\n%i end",objNum);
        printSolvingTable(); // debug output
#endif
    }

    // ================
    // CHECK
    // ================
    bool ComplexObject::Check(){
        for( size_t i = 0 ; i < objectArray.size() ; i++){
            if(objectArray.at(i).elements.back().size() != 1)
                return false;
        }
        return true;
    }


    // ================
    // SOLVE ONE WEAK
    // ================
    double ComplexObject::solveOneRelationWeak(const cv::Mat& frame, struct relationExample* rEx){
        // UP ROW
        for( size_t i = 0 ; i < rEx->upObj->elements.back().size(); i++){
            double max_value = 0;
            for( size_t j = 0 ; j < rEx->downObj->elements.back().size() ; j ++){
                // RELATION OK
                if( rEx->relation->checkRelation(frame, &(rEx->upObj->elements.back().at(i)),&(rEx->downObj->elements.back().at(j))) ){
                        double val = rEx->downObj->value.at(j).getTotal();
                        if(max_value < val) max_value = val;
                }
            }
            rEx->upObj->value.at(i).addValue(max_value);
        }

        // DOWN ROW
        for( size_t i = 0 ; i < rEx->downObj->elements.back().size(); i++){
            double max_value = 0;
            for( size_t j = 0 ; j < rEx->upObj->elements.back().size() ; j ++){
                // RELATION OK
                if( rEx->relation->checkRelation(frame, &(rEx->upObj->elements.back().at(j)),&(rEx->downObj->elements.back().at(i))) ){
                        double val = rEx->upObj->value.at(j).getTotal();
                        if(max_value < val) max_value = val;
                }
            }
            rEx->downObj->value.at(i).addValue(max_value);
        }

        /*
        double changes = 0;
        for(size_t i = 0 ; i < rEx->upObj->value.size() ; i++){
            changes += fabs(rEx->upObj->value.at(i).endCycle());
        }

        for(size_t i = 0 ; i < rEx->downObj->value.size() ; i++){
            changes += fabs(rEx->downObj->value.at(i).endCycle());
        }
        return changes;
        */
    }

    // ================
    // SOLVE ONE
    // ================
    int ComplexObject::solveOneRelation(const cv::Mat& frame, struct relationExample* rEx){
        // up row
        vector<ExtendedObjectInfo> newUpRow;
        for( size_t i = 0 ; i < rEx->upObj->elements.back().size(); i++){
            for( size_t j = 0 ; j < rEx->downObj->elements.back().size() ; j ++){
                if( rEx->relation->checkRelation(frame, &(rEx->upObj->elements.back().at(i)),&(rEx->downObj->elements.back().at(j))) ){
                //if( rEx->relation->checkRelation(frame, rEx->upObj->elements.end()->at(i),rEx->downObj->elements.end()->at(j)) ){
                    newUpRow.push_back(rEx->upObj->elements.back().at(i));
                    break;
                }
            }
        }
        // down row
        vector<ExtendedObjectInfo> newDownRow;
        for( size_t i = 0 ; i < rEx->downObj->elements.back().size(); i++){
            for( size_t j = 0 ; j < rEx->upObj->elements.back().size() ; j ++){
                if( rEx->relation->checkRelation(frame, &(rEx->upObj->elements.back().at(j)),&(rEx->downObj->elements.back().at(i))) ){
                    newDownRow.push_back(rEx->downObj->elements.back().at(i));
                    break;
                }
            }
        }
        //debug
        /*
        int newD = newDownRow.size();
        int newU = newUpRow.size();
        int oldD = rEx->downObj->elements.back().size();
        int oldU = rEx->upObj->elements.back().size();
        */
        int changes = rEx->upObj->elements.back().size() - newUpRow.size() + rEx->downObj->elements.back().size() - newDownRow.size();

        rEx->upObj->elements.push_back(newUpRow);
        rEx->downObj->elements.push_back(newDownRow);

        return changes;
    }

    // ================
    // DELETE REPEATS
    // ================
    int ComplexObject::deleteRepeats(){
        int reps = 0;
        for(size_t i = 0 ; i < answerArray.size(); i++){
            for(size_t j = i+1 ; j < answerArray.size();){
                if( answerArray.at(i) == answerArray.at(j) ){
                    answerArray.erase(answerArray.begin() + j);
                    reps++;
                }
                else{
                    j++;
                }
            }
        }
    }
    // ================
    // DRAW
    // ================
    void ComplexObject::draw(Mat& frameTd, Scalar colorIn, Scalar colorOut){
/*
        ExtendedObjectInfo result;
        for( size_t i = 0 ; i < objectArray.size(); i ++){
            for( size_t j = 0 ; j < objectArray.at(i).elements.back().size(); j++ ){
                result = result | objectArray.at(i).elements.back().at(j).getRect();

                rectangle(frameTd, objectArray.at(i).elements.back().at(j).getRect(), colorIn );
            }
        }
        rectangle(frameTd, result.getRect(), colorOut );
        */
        for( size_t i = 0 ; i < answerArray.size(); i ++){
            ExtendedObjectInfo result;
            for( size_t j = 0 ; j < answerArray.at(i).size(); j++ ){
                result = result | answerArray.at(i).at(j).getRect();
                rectangle(frameTd, answerArray.at(i).at(j).getRect(), colorIn );
            }
            rectangle(frameTd, result.getRect(), colorOut );
        }
    }

    void ComplexObject::drawAll(Mat& frameTd, Scalar color, int tickness){
        for( size_t j = 0 ; j < objects.size(); j++ ){
            drawOne(frameTd,j,color,tickness);
        }
    }

    // ================
    // DRAW ONE
    // ================
    void ComplexObject::drawOne(Mat& frameTd, int no, Scalar color, int tickness ){
        if( no < objects.size() ){
            objects[no].draw(frameTd, color);
            
            string objectInfo = to_string(ID)+": "+name;
            Point prevBr = drawFilledRectangleWithText(frameTd, Point(objects[no].x,objects[no].y -12)  , objectInfo, color);     
            
//             for( size_t j = 0 ; j < answerArray.at(no).size(); j++ ){
//                 rectangle(frameTd, answerArray.at(no).at(j).getRect(), color );                
//             }
            
            for( size_t i = 0 ; i < objectArray.size() ; i++ ){
                string simObjName = " @ " + objectArray[i].nameInScene+": "+objectArray[i].pointerToObj->name;
                
                //rectangle(frameTd, answerArray.at(no).at(i).getRect(), color );                
                answerArray.at(no).at(i).draw(frameTd, color);
                
                //prevBr = drawFilledRectangleWithText(frameTd, Point(objects[no].tl().x, prevBr.y), simObjName, color);
                drawFilledRectangleWithText(frameTd, answerArray.at(no).at(i).tl(), simObjName, color);
            }
        }
    }
    
    /*
    void ComplexObject::drawOne(Mat& frameTd, int no, Scalar color, int tickness ){
        if( no >= answerArray.size() ) return;
        ExtendedObjectInfo result;
        for( size_t j = 0 ; j < answerArray.at(no).size(); j++ ){
            if( j == 0 )
                result = answerArray.at(no).at(0).getRect();
            else
                result = result | answerArray.at(no).at(j).getRect();
            rectangle(frameTd, answerArray.at(no).at(j).getRect(), color );
            for(size_t i = j ; i < answerArray.at(no).size(); i++){
                for( size_t k = 0 ; k < solvingTable.size() ; k ++){
                    if(solvingTable.at(k).downNo == j && solvingTable.at(k).upNo == i ){
                        line(frameTd,answerArray.at(no).at(j).getCenter(),answerArray.at(no).at(i).getCenter(),color,tickness);
                        break;
                    }
                    if(solvingTable.at(k).downNo == i && solvingTable.at(k).upNo == j ){
                        line(frameTd,answerArray.at(no).at(j).getCenter(),answerArray.at(no).at(i).getCenter(),color,tickness);
                        break;
                    }
                }
            }
        }
        rectangle(frameTd, result.getRect(), color, tickness );
        putText(frameTd, name, Point(result.x, result.y-10), FONT_HERSHEY_SIMPLEX, 0.5, color, tickness);

    }*/

    void ComplexObject::drawOne(Mat& frameTd, ExtendedObjectInfo* scene, Scalar color ){
        putText(frameTd, name, Point(scene->x, scene->y-10), FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        rectangle(frameTd, scene->getRect(), color );
    }

    // ================
    // PRINT
    // ================
    void ComplexObject::printSolvingTable(bool weak){
        printf("\nSolving table slice");
        for( size_t i = 0 ; i < solvingTable.size() ; i ++){
            printf("\nR: %s \n",solvingTable.at(i).relation->Name.c_str());
            printf("\tO: %s, ",solvingTable.at(i).upObj->nameInScene.c_str());
            for( size_t j = 0 ; j < solvingTable.at(i).upObj->elements.back().size(); j++){
                Rect r = solvingTable.at(i).upObj->elements.back().at(j).getRect();
                if( weak ) printf("[%i;%i;%i;%i](%f) ",r.x,r.y,r.width,r.height,solvingTable.at(i).upObj->value.at(j).getTotal());
                else printf("[%i;%i;%i;%i] ",r.x,r.y,r.width,r.height);
            }
            printf("\n\tO: %s, ",solvingTable.at(i).downObj->nameInScene.c_str());
            for( size_t j = 0 ; j < solvingTable.at(i).downObj->elements.back().size(); j++){
                Rect r = solvingTable.at(i).downObj->elements.back().at(j).getRect();
                if( weak ) printf("[%i;%i;%i;%i](%f) ",r.x,r.y,r.width,r.height,solvingTable.at(i).downObj->value.at(j).getTotal());
                else printf("[%i;%i;%i;%i] ",r.x,r.y,r.width,r.height);
            }
        }
    }
    
}
