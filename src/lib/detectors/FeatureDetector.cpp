#ifdef USE_OPENCV_CONTRIB
#include "FeatureDetector.h"
#include "ObjectBase.h"
#include "geometry_utils.h"

using namespace std;
using namespace cv;

namespace eod{
    
    FeatureAttribute::FeatureAttribute(){        
        Type = FEATURE_A;
        inited = false;
    }
    
    FeatureAttribute::FeatureAttribute(FEATURE_DETECTOR featureExtractorType_, string image, int min_matches, double height_){
        Type = FEATURE_A;                
        featureExtractorType = featureExtractorType_;
        
        
        Mat original = imread(image, 0);
        
        min_matches_to_detect = min_matches;
        if(original.data){
            image_or = original;
            inited = true;
            
            if( GSFD == NULL ){
                GSFD = new GlobalSimpleFeatureDetector();
            }            
            GSFD->get_descriptors_and_keypoints(image_or, original_keypoints, original_descriptors, featureExtractorType);            
            
            //printf("Extracted %i keypoints\n", original_keypoints.size());
            //printf("Descriptos size %i %i\n", original_descriptors.cols, original_descriptors.rows);
                        
            original_corners = vector<Point2f>(4);
            
            original_corners[0] = cvPoint(0,0);
            original_corners[1] = cvPoint(original.cols,0);
            original_corners[2] = cvPoint(original.cols,original.rows);
            original_corners[3] = cvPoint(0,original.rows);
            
            original_corners_shifted = vector<Point3f>(4);
            original_corners_shifted[0] = Point3f(-original.cols/2,-original.rows/2,0);
            original_corners_shifted[1] = Point3f(original.cols/2,-original.rows/2,0);
            original_corners_shifted[2] = Point3f(original.cols/2,original.rows/2,0);
            original_corners_shifted[3] = Point3f(-original.cols/2,original.rows/2,0);
            
            height = height_;
            scale = height / original.rows;
            
        }
        else
            printf("Can't find provided image %s in FeatureAttribute.\n", image.c_str());
            inited = false;
    }
                
    vector<ExtendedObjectInfo> FeatureAttribute::Detect2(const Mat& image_in, int seq){        
        vector<ExtendedObjectInfo> result;
        
        if(!GSFD->hasCamParams() ){
            GSFD->setCamParams(parent_base->getCameraMatrix(), parent_base->getDistortionCoeff() );
        }                                
        result = GSFD->Detect(image_in, seq,  original_keypoints, original_descriptors, original_corners, min_matches_to_detect, featureExtractorType, scale, original_corners_shifted, Weight, returnContours);            
        
        return result;
    }            

    
    bool FeatureAttribute::Check2(const Mat& image,ExtendedObjectInfo& rect){
        return false;        
    }
    
    void FeatureAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
    
    // -----------------------------------
    // -----------------------------------
    // Global Simple Feature Detector
    // -----------------------------------
    // -----------------------------------
    GlobalSimpleFeatureDetector* GSFD = NULL;
    
    GlobalSimpleFeatureDetector::GlobalSimpleFeatureDetector(){
        sift_detector = SIFT::create();
        surf_detector = SURF::create();
        orb_detector = ORB::create();
    }
    
    bool GlobalSimpleFeatureDetector::get_descriptors_and_keypoints(const Mat& image, std::vector<KeyPoint> & keypoints, Mat & descriptors, FEATURE_DETECTOR detector_type){
        
        switch( detector_type){
            case FD_SIFT:
                sift_detector->detect(image, keypoints);
                sift_detector->compute(image, keypoints, descriptors);
                return true;
            case FD_SURF:
                surf_detector->detect(image, keypoints);
                surf_detector->compute(image, keypoints, descriptors);
                return true;
            case FD_ORB:
                orb_detector->detect(image, keypoints);
                orb_detector->compute(image, keypoints, descriptors);
                return true;
            default:
                return false;
        }        
    }
    
    vector<ExtendedObjectInfo> GlobalSimpleFeatureDetector::Detect(const Mat& image, int seq, std::vector<KeyPoint> original_keypoints, Mat original_descriptors, vector<Point2f> original_corners, int min_matches_to_detect, FEATURE_DETECTOR detector_type, double scale, vector<Point3f> original_corners_shifted, double Weight, bool returnContours ){
        int prev_seq;
        switch(detector_type){
            case FD_SIFT:
                prev_seq = sift_prev_seq;
                break;
            case FD_SURF:
                prev_seq = surf_prev_seq;
                break;
            case FD_ORB:
                prev_seq = orb_prev_seq;
                break;
        }
        if( seq == 0 or seq != prev_seq ){
            // extract features if we haven't done it
            switch(detector_type){
                case FD_SIFT:
                    get_descriptors_and_keypoints(image, sift_current_keypoints, sift_current_descriptors,  detector_type);
                    sift_prev_seq = seq;
                    break;
                case FD_SURF:
                    get_descriptors_and_keypoints(image, surf_current_keypoints, surf_current_descriptors,  detector_type);
                    surf_prev_seq = seq;
                    break;    
                case FD_ORB:
                    get_descriptors_and_keypoints(image, orb_current_keypoints, orb_current_descriptors,  detector_type);
                    orb_prev_seq = seq;
                    break;    
            }
        }
        // detection
        vector<ExtendedObjectInfo> rects;
        
        vector<DMatch> matches;
        switch(detector_type){
            case FD_SIFT:
                bf_matcher.match( original_descriptors, sift_current_descriptors, matches);
                break;
            case FD_SURF:
                bf_matcher.match( original_descriptors, surf_current_descriptors, matches);
                break;    
            case FD_ORB:
                bf_matcher.match( original_descriptors, orb_current_descriptors, matches);
                break;
        }        
        
        //printf("[%i] Found %i matches\n",detector_type, matches.size());
        
        //double max_dist = 0;
        //double min_dist = 100;
        double mean_dist = 0;

        for (int i = 0 ; i < original_descriptors.rows; i++){
            double dist= matches[i].distance;
            //if( dist < min_dist) min_dist = dist;
            //if( dist > max_dist) max_dist = dist;
            mean_dist += dist;
        }
        mean_dist /= original_descriptors.rows;

        // find "good" matches
        vector <DMatch> good_matches;
        
        double good_dist = mean_dist;// / 2;
        for(int i = 0 ; i < original_descriptors.rows; i++){
            if( matches[i].distance < good_dist )
                good_matches.push_back(matches[i]);
        }
        
        if (good_matches.size() >= min_matches_to_detect){
            //printf("[%i] Found %i good matches\n",detector_type, good_matches.size());
            // localize object
            vector<Point2f> original_obj;
            vector<Point2f> obj;
            
            std::vector<KeyPoint> *keypoints;
            switch(detector_type){
                case FD_ORB:
                    keypoints = &orb_current_keypoints;
                    break;
                case FD_SIFT:
                    keypoints = &sift_current_keypoints;
                    break;
                case FD_SURF:
                    keypoints = &surf_current_keypoints;
                    break;
            }
            
            for(int i = 0; i < good_matches.size() ; i++){
                original_obj.push_back((original_keypoints)[good_matches[i].queryIdx].pt);
                obj.push_back((*keypoints)[good_matches[i].trainIdx].pt);

            }
            Mat H = findHomography(original_obj, obj, RANSAC);
            if (H.data){
                vector<Point2f> corners(4);
                perspectiveTransform(original_corners, corners, H);
                ExtendedObjectInfo tmp = boundingRect( Mat(corners) );                
                if( returnContours )
                    tmp.contour.push_back(float2intPointVector(vector<Point2f>(corners)));
                // tvec rvec
                if( hasCamParams() ){
                    Vec3d rvec, tvec;                                                            
                    if( solvePnP(original_corners_shifted, corners, camMat, distCoef, rvec, tvec)){                            
                        if( scale > 0 )
                            tmp.tvec.push_back(scaleVec3d(tvec,scale));
                        tmp.rvec.push_back(rvec);
                    }                        
                }
                tmp.setScoreWeight(1, Weight);//TODO try to solve feature matches %
                rects.push_back(tmp);
            }    
        }
        /*
        else{
            printf("[%i] Not enoght matches (%i)\n",detector_type, good_matches.size());
        }*/
        return rects;        
    }
    
    void GlobalSimpleFeatureDetector::setCamParams(Mat camMat_, Mat distCoef_){
        camMat = camMat_;
        distCoef = distCoef_;
    }
    
    bool GlobalSimpleFeatureDetector::hasCamParams(){
        return !(camMat.empty() & distCoef.empty());
    }
    
            
}
#endif //USE_OPENCV_CONTRIB
