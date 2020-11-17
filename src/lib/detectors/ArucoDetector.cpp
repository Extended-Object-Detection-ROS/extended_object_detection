#include "ArucoDetector.h"
#include "ObjectBase.h"
#include "geometry_utils.h"

using namespace std;
using namespace cv;

namespace eod{
    
    map <cv::aruco::PREDEFINED_DICTIONARY_NAME, GlobalArucoDetector*> GADS;
        
    ArucoAttribute::ArucoAttribute(){
        this->IDmarker = -1;
        Type = ARUCO_A;        
        inited = false;
    }
    
    ArucoAttribute::ArucoAttribute(int dictionary, int IDmarker_, double markerLen_){
        if( dictionary < 0 ){
            dict = cv::aruco::DICT_4X4_50;
            printf("Aruco dictionary didin't suport negative values, dictionary is set to  default 4x4_50\n");
        }
        else if( dictionary >  20){
            dict = cv::aruco::DICT_4X4_50;
            printf("Aruco dictionary cant be more than 20, dictionary is set to  default 4x4_50\n");
        }
        else
            dict = cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary);
        
        markerLen = markerLen_;
        
        this->IDmarker = IDmarker_ >= 0 ? IDmarker_ : -1;
        Type = ARUCO_A;        
        
        map<cv::aruco::PREDEFINED_DICTIONARY_NAME,GlobalArucoDetector*>::iterator it;
        it = GADS.find(dict);
        if ( it == GADS.end() ) {
            GAD = new GlobalArucoDetector(dict);
            //printf("try to set this\n");
            //GAD->setCamParams(parent_base->getCameraMatrix(), parent_base->getDistortionCoeff() );
            //printf("set\n");
            GADS.insert(pair<cv::aruco::PREDEFINED_DICTIONARY_NAME,GlobalArucoDetector*>(dict,GAD));            
        }
        else{
            GAD = (*it).second;
        }
        inited = true;        
    }    
    
    vector<ExtendedObjectInfo> ArucoAttribute::Detect2(const Mat& image, int seq){
        if( !inited )
            return vector<ExtendedObjectInfo>(0);
        
        if( !GAD->hasCamParams() ){
            GAD->setCamParams(parent_base->getCameraMatrix(), parent_base->getDistortionCoeff() );
        }
                
        return GAD->detect(image, IDmarker, seq, markerLen, Weight, returnContours);
    }
          
        
    bool ArucoAttribute::Check2(const Mat& image,ExtendedObjectInfo& rect){
        return false;        
    }
    
    void ArucoAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
    
    /// GLOBAL DETECTORS
    GlobalArucoDetector::GlobalArucoDetector(cv::aruco::PREDEFINED_DICTIONARY_NAME dict){
        dictionary = cv::aruco::getPredefinedDictionary(dict);
    }
    
    vector<ExtendedObjectInfo> GlobalArucoDetector::detect(const Mat& image, int IDmarker, double seq, double markerLen, double Weight, bool returnContours){
        vector<ExtendedObjectInfo> rects;        
        if( seq == 0 || seq != prev_seq ){
            //markerIds.clear();
            //markerCorners.clear();
            cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
        }        
        for( size_t i = 0 ; i < markerIds.size(); i++ ){
            if( IDmarker == -1 || IDmarker == markerIds[i] ){
                ExtendedObjectInfo tmp = boundingRect( markerCorners[i] );                                                
                tmp.sub_id[0] = markerIds[i];
                tmp.extracted_info[0] = to_string(markerIds[i]);       
                if( returnContours )
                    tmp.contour.push_back(float2intPointVector(markerCorners[i]));
                if( hasCamParams() && markerLen > 0){
                    vector<cv::Vec3d> rvecs, tvecs;                    
                    cv::aruco::estimatePoseSingleMarkers(vector<vector<Point2f> >(markerCorners.begin()+i,markerCorners.begin()+i+1), markerLen, camMat, distCoef, rvecs, tvecs);
                    tmp.tvec.push_back(tvecs[0]);
                    tmp.rvec.push_back(rvecs[0]);
                }
                tmp.setScoreWeight(1, Weight);//TODO use reprojection to calc 
                rects.push_back(tmp);                
            }
        }
        if( seq != 0)
            prev_seq = seq;
        return rects;        
    }
    
    void GlobalArucoDetector::setCamParams(Mat camMat_, Mat distCoef_){
        camMat = camMat_;
        distCoef = distCoef_;
    }
    
    bool GlobalArucoDetector::hasCamParams(){
        return !(camMat.empty() & distCoef.empty());
    }
    
}
