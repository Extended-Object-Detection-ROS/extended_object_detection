#include "DimentionDetector.h"

using namespace std;
using namespace cv;

namespace eod{ 
  
  DimentionAttribute::DimentionAttribute(){
    Type = DIMEN_A;
  }
  
  DimentionAttribute::DimentionAttribute(double ratioMin_, double ratioMax_){
    ratioMax = ratioMax_;
    ratioMin = ratioMin_;
    Type = DIMEN_A;
  }
  
  vector<ExtendedObjectInfo> DimentionAttribute::Detect2(const Mat& image, int seq){
    vector <ExtendedObjectInfo> ans;
    return ans;
  }
  
  bool DimentionAttribute::Check2(const Mat& image, ExtendedObjectInfo &rect){    
    
    if( (double)rect.width / (double)rect.height < ratioMin )
        return false;
        
    if( (double)rect.width / (double)rect.height > ratioMax )
        return false;
    
    rect.setScoreWeight(1, Weight);
    
    return true;
  }
  
  void DimentionAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){}
  
}
