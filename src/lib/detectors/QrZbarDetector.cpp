#ifdef USE_ZBAR
#include "QrZbarDetector.h"
#include <opencv2/aruco.hpp>
#include "ObjectBase.h"
#include "geometry_utils.h"

using namespace std;
using namespace cv;
using namespace zbar;

namespace eod{
    
    QrZbarAttribute::QrZbarAttribute(){
        Type = QR_ZBAR_A;
        // disable all
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
        // enable qr
        scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
        real_len = -1;
        info = "";
        inited = true;                
    }
    
    QrZbarAttribute::QrZbarAttribute(double real_len_, string info_){
        Type = QR_ZBAR_A;
        // disable all
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
        // enable qr
        scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
        real_len = real_len_;
        inited = true;
        info = info_;
    }
    
    bool QrZbarAttribute::Check2(const Mat& image, ExtendedObjectInfo& rect){
        return false;
    }
    
    void QrZbarAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
    
    vector<ExtendedObjectInfo> QrZbarAttribute::Detect2(const Mat& image_, int seq){
        vector<ExtendedObjectInfo> rects;
        
        // Convert image to grayscale
        Mat imGray;
        cvtColor(image_, imGray, COLOR_BGR2GRAY);            
        
        // Wrap image data in a zbar image
        Image image(image_.cols, image_.rows, "Y800", (uchar *)imGray.data, image_.cols * image_.rows);
        
        scanner.scan(image);
        
        for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
        {            
            if( info != ""){
                if( info != symbol->get_data() )
                    continue;
            }
            vector <Point> contour;
            
            for(int i = 0; i< symbol->get_location_size(); i++)
            {
                contour.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));                                     
            }   
            ExtendedObjectInfo tmp = boundingRect( contour );
            if( returnContours )
                tmp.contour.push_back(contour);
            tmp.setScoreWeight(1, Weight);
            tmp.extracted_info[0] = symbol->get_data();
            
            if( real_len > 0){
                if( symbol->get_location_size() != 4 ){
                    printf("QRcode points size error. Can't solve 3d-position.\n");
                }
                else{
                    Mat camMat = parent_base->getCameraMatrix(); 
                    Mat distCoef = parent_base->getDistortionCoeff();
                    if( !camMat.empty() && !distCoef.empty() ){
                        vector<vector<Point2f> > corners;
                        corners.push_back(int2floatPointVector(contour));
                        vector<Vec3d> rvecs, tvecs;   
                        cv::aruco::estimatePoseSingleMarkers(corners, real_len, camMat, distCoef, rvecs, tvecs);
                        tmp.tvec.push_back(tvecs[0]);
                        tmp.rvec.push_back(rvecs[0]);
                    }
                }                    
            }            
            rects.push_back(tmp);
        }                
        if( seq != 0)
            prev_seq = seq;
        return rects;        
    }
}
#endif // USE_ZBAR
