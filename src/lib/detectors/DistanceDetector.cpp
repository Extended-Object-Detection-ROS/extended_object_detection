#include "DistanceDetector.h"

using namespace std;
using namespace cv;

namespace eod{
    
  DistanceAttribute::DistanceAttribute(){
    Type = DIST_A;
    inited = false;
  }
  
  DistanceAttribute::DistanceAttribute(double distMin_, double distMax_){
    distMin = distMin_;
    distMax = distMax_;
    Type = DIST_A;
    inited = true;
  }
  
  vector<ExtendedObjectInfo> DistanceAttribute::Detect2(const Mat& image, int seq){
    vector <ExtendedObjectInfo> ans;
    return ans;
  }
  
  bool DistanceAttribute::Check2(const Mat& image, ExtendedObjectInfo &rect){        
     if( rect.tvec.size() < 1 ){
         printf("DimentionAttribute: object has no translation calculated!\n");
         return false;
     }
     if( rect.tvec[0][2] == 1 ){
         printf("DimentionAttribute: object has only unit translation, can't check it!\n");
         return false;
     }
     
     //TODO add flag to combine all tvecs
     
     if( distMin != -1)
         if( rect.tvec[0][2] < distMin )
             return false;
    
    if( distMax != -1)
        if( rect.tvec[0][2] > distMax )
            return false;
                 
     rect.setScoreWeight(1, Weight);
     return true;
  }
  
  void DistanceAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){} 
    
}
