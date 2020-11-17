#include "BasicMotionDetector.h"

using namespace std;
using namespace cv;

namespace eod {

    BasicMotionAttribute::BasicMotionAttribute(){
        Type = BASIC_MOTION_A;
#ifdef SCREEN_OUTPUT_BASIC
        cv::namedWindow("debugBasicMotion");
#endif
    }

    vector<ExtendedObjectInfo> BasicMotionAttribute::Detect2(const Mat& image, int seq){        

        vector<ExtendedObjectInfo> result;

        if(firstFrame.empty()){
            cvtColor(image, firstFrame, COLOR_BGR2GRAY);
            GaussianBlur(firstFrame, firstFrame, Size(BLUR_KER, BLUR_KER), 0);
            return result;
        }

        Mat gray, frameDelta, thresh;
        vector<vector<Point> > cnts;

        cvtColor(image, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(BLUR_KER, BLUR_KER), 0);

        //compute difference between first frame and current frame
        absdiff(firstFrame, gray, frameDelta);
        threshold(frameDelta, thresh, 25, 255, THRESH_BINARY);

        dilate(thresh, thresh, Mat(), Point(-1,-1), 2);
        findContours(thresh, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for(int i = 0; i< cnts.size(); i++) {
            ExtendedObjectInfo temp = ExtendedObjectInfo(boundingRect(cnts[i]));            
            temp.contour.push_back(cnts[i]);
            result.push_back(temp);
        }
        firstFrame = gray.clone();

/*        ClusterForel cluster(result,60,10);
        cluster.run();
        result = cluster.get_clustrs();
#ifdef SCREEN_OUTPUT_BASIC
        for(size_t i = 0 ; i < result.size(); i++)
            rectangle(thresh,result.at(i).getRect(),Scalar(255,255,255));
        imshow("debugBasicMotion",thresh);
#endif        */    
      
        return result;
    }

    bool BasicMotionAttribute::Check2(const Mat& image, ExtendedObjectInfo& rect){
        return false;
    }
    
    void BasicMotionAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }

}
