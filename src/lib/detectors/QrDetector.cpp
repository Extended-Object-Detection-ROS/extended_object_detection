#include <opencv2/core/version.hpp>
#if (CV_MAJOR_VERSION > 3)
#include "QrDetector.h"
using namespace std;
using namespace cv;

namespace eod{
    
    QrAttribute::QrAttribute(){
        Type = QR_A;
        qrDecoder = new QRCodeDetector();   
        inited = true;
    }
    
    bool QrAttribute::Check2(const Mat& image, ExtendedObjectInfo& rect){
        return false;
    }
    
    void QrAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
    
    vector<ExtendedObjectInfo> QrAttribute::Detect2(const Mat& image, int seq){
        vector<ExtendedObjectInfo> rects;        
        Mat bbox, rectifiedImage;
        //string data = qrDecoder->detectAndDecode(image, bbox, rectifiedImage);
        qrDecoder->detect(image, bbox);            
    
        if( ! bbox.empty() ){
            vector<Point> contour;
            for(int i = 0 ; i < bbox.rows ; i++)
                contour.push_back(Point(bbox.at<float>(i,0), bbox.at<float>(i,1)));
            //contour.push_back() // should I push first point again?
            ExtendedObjectInfo tmp = boundingRect( contour );
            if( returnContours )
                tmp.contour.push_back(contour);
            tmp.setScoreWeight(1, Weight);
            
            
            string data = qrDecoder->decode(image, bbox);
            //printf("Decoded data is %s\n",data.c_str());            
            tmp.extracted_info[0] = data;
            
            rects.push_back(tmp);
        }        
        if( seq != 0)
            prev_seq = seq;
        return rects;        
    }
    
}
#endif
