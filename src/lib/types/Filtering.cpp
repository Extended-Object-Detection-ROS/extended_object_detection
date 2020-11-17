#include "Filtering.h"
#include "geometry_utils.h"

using namespace std;
using namespace cv;

namespace eod{
    
    FilterTypes getFilterTypeFromString(std::string str){
        transform(str.begin(), str.end(), str.begin(),[](unsigned char c){ return tolower(c); });
        
        if( str == "insider" )
            return INSIDER_F;
        else if( str == "nms" )
            return IOU_F;
        else if( str == "roi" )
            return ROI_F;
        
        return UNK_F;
    }
    
    
    EoiFilter::EoiFilter(){
        type = UNK_F;
    }
    
    //
    // Insider filter: clears rects of objects are complitely in other
    //
    
    InsiderFilter::InsiderFilter(){
        type = INSIDER_F;
    }
    
    void InsiderFilter::Filter(vector<ExtendedObjectInfo>* src){
        auto it = src->begin();
        while (it != src->end() ){            
            bool in = false;
            for(size_t i = 0 ; i < src->size() ; i++ ){
                if( rect_inside((*it).getRect(), src->at(i).getRect() ) ){
                    in = true;
                    break;
                }
            }
            if( in ){                
                it = src->erase(it);
            }
            else {
                ++it;
            }
        }                        
    }
    
    //
    // IOU (Nonmax supression) filter: deletes rects with high IOU with others
    //
    
    IOUFilter::IOUFilter(double thres){
        type = IOU_F;
        iou_threshold = thres;
    }
    
    void IOUFilter::Filter(vector<ExtendedObjectInfo>* src){
        vector<int> badIndexes;
        for( size_t i = 0 ; i < src->size(); i++ ){
            if( find(badIndexes.begin(), badIndexes.end(), i) != badIndexes.end() ){
                continue;
            }                
            for( size_t j = i + 1 ; j < src->size(); j++ ){
                if( find(badIndexes.begin(), badIndexes.end(), j) != badIndexes.end() ){
                    continue;
                }   
                double iou = intersectionOverUnion( &(src->at(i)), &(src->at(j)) );
                if( iou >= iou_threshold ){
                    // delete with lower score
                    if( src->at(i).scores_with_weights.back().first > src->at(j).scores_with_weights.back().first ){
                        badIndexes.push_back(j);
                    }
                    else{
                        badIndexes.push_back(i);
                        break;
                    }
                }
            }
        }        
        vector<ExtendedObjectInfo>* filtered = new vector<ExtendedObjectInfo>;
        for( size_t i = 0 ; i < src->size(); i++ ){
            if( find(badIndexes.begin(), badIndexes.end(), i) == badIndexes.end() ){
                filtered->push_back(src->at(i));
            }
        }            
        src->clear();
        src->assign(filtered->begin(), filtered->end()); 
    }
    
    // 
    // ROI filter: keeps object with rect whole in scpecifiec region (PoseDetector do the same but checks center)
    //
    ROIFilter::ROIFilter(Rect roi){
        type = ROI_F;
        ROI = roi;
    }
    
    void ROIFilter::Filter(vector<ExtendedObjectInfo>* src){
        auto it = src->begin();
        while (it != src->end() ){                                    
            if( !rect_inside((*it).getRect(), ROI ) ){
                it = src->erase(it);
            }                                        
            else {
                ++it;
            }
        }    
    }
    
}
