#include "HaarCascadeDetector.h"

using namespace std;
using namespace cv;

namespace eod{

    HaarCascadeAttribute::HaarCascadeAttribute(){
        Type = HAAR_CASCADE_A;
    }
    
    HaarCascadeAttribute::HaarCascadeAttribute(string name){
        Type = HAAR_CASCADE_A;
        cascade_name = name;
        if (!shapeClassifier.load(cascade_name)){
            printf("Error: Unable to load cacade! %s\n",cascade_name.c_str() );
            inited = false;            
        }
        inited = true;
    }

    HaarCascadeAttribute::~HaarCascadeAttribute(){
        //cvReleaseHaarClassifierCascade(shapeClassifier);
    }    
    
    bool HaarCascadeAttribute::Check2(const Mat& image,ExtendedObjectInfo &rect){
      return false;
    }
    
    void HaarCascadeAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }

    vector<ExtendedObjectInfo> HaarCascadeAttribute::Detect2(const Mat& image, int seq){
        
        vector<ExtendedObjectInfo> res;
        vector<Rect> shapes;            
        if( inited ){
            Mat frame_gray;
#if (CV_MAJOR_VERSION > 3)            
            cvtColor(image, frame_gray, COLOR_BGR2GRAY);
#else
            cvtColor(image, frame_gray, CV_BGR2GRAY);
#endif
            equalizeHist(frame_gray, frame_gray);

            shapeClassifier.detectMultiScale(frame_gray, shapes, 1.1, 1, 0 , Size(30, 30), image.size() );
            
            /*
            vector<int> rejectLevels;
            vector<double> levelWeights;
            // function that are not documented https://codeyarns.com/2014/10/30/how-to-get-detection-score-from-opencv-cascade-classifier/
            // returns detection score but not normalized on 0-1 and now I have no ide how to notmalize it
            shapeClassifier.detectMultiScale(frame_gray, shapes, rejectLevels, levelWeights, 1.1, 3, 0 , Size(30, 30), image.size(), true );                         
            */
            for( size_t i = 0 ; i < shapes.size(); i++){      
                /*
                if( levelWeights[i] >= Probability ){
                    ExtendedObjectInfo tmp = ExtendedObjectInfo(shapes[i]);
                    tmp.dcnt = levelWeights[i];
                    res.push_back(tmp);
                }
                */
                ExtendedObjectInfo tmp = ExtendedObjectInfo(shapes[i]);
                tmp.setScoreWeight(1, Weight);
                res.push_back(tmp);                    
            }
        }                                          
        return res;      
    }
}
