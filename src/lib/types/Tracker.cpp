#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION > 3
#include "Tracker.h"
#include "geometry_utils.h"
#include "drawing_utils.h"
#include <algorithm>

#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;

namespace eod{
    // ------------------
    // Track
    // ------------------
    Track::Track(int id_, ExtendedObjectInfo first, string trackerType){
        id = id_;
        current_object = first;
        current_object.track_id = id;
        //current_object.track_history_image = &history_image;
        track_counter = 0;
        status = DETECTED;
                
                    
        //tracker = cv::Tracker::create(trackerType);
        if (trackerType == "BOOSTING")
            tracker = cv::TrackerBoosting::create();
        else if (trackerType == "MIL")
            tracker = cv::TrackerMIL::create();
        else if (trackerType == "KCF")
            tracker = cv::TrackerKCF::create();
        else if (trackerType == "TLD")
            tracker = cv::TrackerTLD::create();
        else if (trackerType == "MEDIANFLOW")
            tracker = cv::TrackerMedianFlow::create();
        else if (trackerType == "GOTURN")
            tracker = cv::TrackerGOTURN::create();
        else if (trackerType == "MOSSE")
            tracker = cv::TrackerMOSSE::create();
        else if (trackerType == "CSRT")
            tracker = cv::TrackerCSRT::create();
        else{
            printf("Unknow tracker type %s!",trackerType.c_str());
        }


        if( ! tracker )
            printf("Failed tracker %s init!",trackerType.c_str());
    }
    
    void Track::addObject(ExtendedObjectInfo new_one, TrackStatus status_update){
        history_image.push_back(current_object.getCenter());
        
        if( status_update == DETECTED ){
        }
        else if( status_update == TRACKED ){            
            new_one.total_score = current_object.total_score;   
        }
        current_object = new_one;
        current_object.track_id = id;    
        status = status_update;
    }
    
    void Track::initTracker(const Mat& frame){        
        if( !frame.empty() ){
            if( tracker ){
                Rect2d base_rect = current_object.getRect2d();
                tracker->init(frame, base_rect);
            }            
        }
        else
            printf("Frame for tracker init is empty!\n");
    }
    
    void Track::updateTracker(const Mat& frame){
        Rect2d box = current_object.getRect2d();        
        if( tracker ){
            if( tracker->update(frame, box)){
                ExtendedObjectInfo tracked(box);            
                addObject(box, TRACKED);            
            }
            // what?
            else{
                ExtendedObjectInfo tracked(box);            
                status = LOST;
            }
        }
        else{            
            status = LOST;
        }
    }
    
    // ------------------
    // Tracker
    // ------------------
    eodTracker::eodTracker(){
        id_cnt = 0;
    }    
    
    eodTracker::eodTracker(string name, string tracker_type_) : SimpleObject(name){
        id_cnt = 0;
        tracker_type = tracker_type_;
    }
    
    vector <ExtendedObjectInfo> eodTracker::Identify(const Mat& frame, const Mat& depth, int seq ){
        // do regular things        
        SimpleObject::Identify(frame, depth, seq);
        
        // remove lost tracks
        auto it = tracks.begin();
        while( it != tracks.end() ){
            if ((*it)->status == LOST )
                it = tracks.erase(it);
            else
                ++it;
        }            
        if( objects.size() == 0 && tracks.size() == 0){
            frame.copyTo(previous_frame);
            return objects;
        }
        // first time 
        if( tracks.size() == 0 ){            
            for( size_t i = 0 ; i < objects.size() ; i++ ){
                tracks.push_back(new Track(id_cnt++, objects[i], tracker_type));
            }
            setTrackingResult();
            frame.copyTo(previous_frame);
            return objects;
        }                
        // track rects         
        if( objects.size() ){
            createClosenessMap();            
            Mat mask(closenessMap.size(), CV_8UC1, Scalar(255,255,255));            
            vector<int> takenTracks, takenObjects;
            
            while(true){                
                double min, max = 0;
                Point min_loc, max_loc;
                minMaxLoc(closenessMap, &min, &max, &min_loc, &max_loc, mask);
                if( max == 0 ) break;
                tracks[max_loc.y]->addObject(objects[max_loc.x], DETECTED);
                mask.row(max_loc.y).setTo(Scalar(0,0,0));
                takenTracks.push_back(max_loc.y);
                takenObjects.push_back(max_loc.x);
                mask.col(max_loc.x).setTo(Scalar(0,0,0));                                
            }          
            mask.release();
            for( size_t i = 0 ; i < tracks.size() ; i++ ){
                // for extra tracks do open cv tracking
                if( find(takenTracks.begin(), takenTracks.end(), i) == takenTracks.end() ){
                    if( tracks[i]->status != TRACKED ){
                        tracks[i]->initTracker(previous_frame);
                    }                    
                    tracks[i]->updateTracker(frame);                                    
                }
            }
            for( size_t i = 0 ; i < objects.size() ; i++ ){
                // for extras objects create new tracks
                if( find(takenObjects.begin(), takenObjects.end(), i) == takenObjects.end() ){
                    tracks.push_back(new Track(id_cnt++, objects[i], tracker_type));                    
                }
            }            
        }
        else{
            for( size_t i = 0 ; i < tracks.size() ; i++ ){
                if( tracks[i]->status != TRACKED ){
                    tracks[i]->initTracker(previous_frame);                    
                }                                    
                tracks[i]->updateTracker(frame);                                    
            }
        }        
        closenessMap.release();
        //previous_frame = frame; //NOTE maybe here should be deep copy
        //previous_frame = frame.clone();
        frame.copyTo(previous_frame);
        // change SimpleObject's objects
        vector<ExtendedObjectInfo> tracked_objects = setTrackingResult();                
        // check and extract from tracking results
        for (size_t i = 0; i < mode_attributes.size(); i++){            
            if(mode_attributes[i].first.first == CHECK){              
                if( mode_attributes[i].first.second == RGB)
                    mode_attributes[i].second->Check(frame, &tracked_objects);
                else if( mode_attributes[i].first.second == DEPTH)
                    mode_attributes[i].second->Check(depth, &tracked_objects);                        
            }           
            else if(mode_attributes[i].first.first == EXTRACT){                
                if( mode_attributes[i].first.second == RGB)
                    mode_attributes[i].second->Extract(frame, &tracked_objects);
                else if( mode_attributes[i].first.second == DEPTH)
                    mode_attributes[i].second->Extract(depth, &tracked_objects);
            }            
        }
        for( size_t i = 0 ; i < tracked_objects.size(); i++)
            //tracked_objects[i].calcTotalScore();
            tracked_objects[i].mergeAllData(merging_policy);
        objects.insert( objects.end(), tracked_objects.begin(), tracked_objects.end() );
                
        return objects;
    }
    
    
    void eodTracker::createClosenessMap(){     
        closenessMap = Mat_<double>(tracks.size(), objects.size());        
        for(int i = 0; i < tracks.size(); i++){            
            for(int j = 0; j < objects.size(); j++){
                double closeness = intersectionOverUnion(&(tracks[i]->current_object), &(objects[j]));                
                closenessMap[i][j] = closeness >= iou_threshold ? closeness : 0;
            }
        }            
    }
    
    vector<ExtendedObjectInfo> eodTracker::setTrackingResult(){
        vector<ExtendedObjectInfo> tracked_objects;
        objects.clear();
        for( size_t i = 0 ; i < tracks.size() ; i ++ ){            
            tracks[i]->current_object.track_id = tracks[i]->id;
            tracks[i]->current_object.track_status = tracks[i]->status;
            if( tracks[i]->status == TRACKED ){                
                double score = tracks[i]->current_object.total_score *= (1.0 - decay);
                tracks[i]->current_object.setScoreWeight(score, 1);
                if( score < Probability )
                    tracks[i]->status = LOST;                                          
                tracked_objects.push_back(tracks[i]->current_object);
            }
            else{
                objects.push_back(tracks[i]->current_object);            
            }
        }
        return tracked_objects;
    }
    
    void eodTracker::draw(Mat& image, Scalar col  ){        
        for (size_t i = 0; i < objects.size(); i++){
            if(tracks[i]->status == TRACKED){
                col = Scalar(0,255,255);//yellow
                objects[i].draw(image, col);
                
                string objectInfo = to_string(ID) +": "+ name + " ["+to_string(roundf(objects[i].total_score * 100) / 100).substr(0,4)+"]"+(objects[i].track_id == -1 ? "" : "("+to_string(objects[i].track_id)+")");                                    
                Point prevBr = drawFilledRectangleWithText(image, objects[i].tl(), objectInfo, col);
                
                prevBr = drawFilledRectangleWithText(image, Point(objects[i].tl().x, prevBr.y), " * Tracked with "+tracker_type + "["+to_string(roundf(objects[i].scores_with_weights[0].first * 100) / 100).substr(0,4)+"]", col);
                
                int real_indx = 1;                
                for(size_t j = 0 ; j < mode_attributes.size() ; j++ ){
                    if( mode_attributes[j].first.first == DETECT )
                        continue;
                    if( mode_attributes[j].first.first == EXTRACT ){
                        continue;
                    }
                    string attributeName = (mode_attributes[j].first.first == DETECT ? " + " : " - ") + mode_attributes[j].second->Name + " ["+(real_indx < objects[i].scores_with_weights.size() ? to_string(roundf(objects[i].scores_with_weights[real_indx].first * 100) / 100).substr(0,4) : "-1" )+"]" + (objects[i].sub_id[real_indx] == -1 ? "" : "{"+to_string(objects[i].sub_id[real_indx])+"}") + (objects[i].extracted_info[real_indx] == "" ? "" : "("+objects[i].extracted_info[real_indx]+")");
                    
                    prevBr = drawFilledRectangleWithText(image, Point(objects[i].tl().x, prevBr.y), attributeName, col);
                    real_indx++;
                }        
                
            }
            else if(tracks[i]->status == DETECTED){                
                objects[i].draw(image, col);
                drawOne(image, &objects[i], col);
            }
            else{
                col = Scalar(0,0,255);
                
                string objectInfo = name + " ["+to_string(roundf(objects[i].total_score * 100) / 100).substr(0,4)+"]"+(objects[i].track_id == -1 ? "" : "("+to_string(objects[i].track_id)+")");                                    
                Point prevBr = drawFilledRectangleWithText(image, objects[i].tl(), objectInfo, col);
                
                drawFilledRectangleWithText(image, Point(objects[i].tl().x, prevBr.y), " # Lost by "+tracker_type, col);
            }
        }
        for(size_t i = 0 ; i < tracks.size() ; i ++ ){            
            for( int j = 0 ; j < ((int)tracks[i]->history_image.size() - 1); j++ ){                    
                line(image, tracks[i]->history_image[j], tracks[i]->history_image[j+1], col);
            }    
        }
    }
}
#endif
