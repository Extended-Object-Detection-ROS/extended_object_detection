#include "ExtendedObjectInfo.h"
#include <algorithm> 
#include <iterator> 
#include "geometry_utils.h"
#include "contour_utils.h"

using namespace std;
using namespace cv;

namespace eod{
    
    ExtendedObjectInfo::ExtendedObjectInfo() : Rect(){
        initVars();
    }
    
    ExtendedObjectInfo::ExtendedObjectInfo(int x, int y, int w, int h) : Rect(x,y,w,h){
        initVars();
    }

    ExtendedObjectInfo::ExtendedObjectInfo(Rect rect) : Rect(rect.x,rect.y,rect.width,rect.height){
        initVars();
    }
    
    ExtendedObjectInfo::ExtendedObjectInfo(Rect2d rect) : Rect(rect.x,rect.y,rect.width,rect.height){
        initVars();
    }
    
    void ExtendedObjectInfo::initVars(){                
        track_id = -1;
        total_score = 0;
        sub_id.push_back(-1);
        extracted_info.push_back("");
        track_status = 0;
    }    
    
    void ExtendedObjectInfo::inheritData(ExtendedObjectInfo* a, ExtendedObjectInfo* b){
        // sub_id
        sub_id.clear();
        sub_id.insert( sub_id.end(), a->sub_id.begin(), a->sub_id.end());
        sub_id.insert( sub_id.end(), b->sub_id.begin(), b->sub_id.end());
                
        // extracted info
        extracted_info.clear();
        extracted_info.insert( extracted_info.end(), a->extracted_info.begin(), a->extracted_info.end());
        extracted_info.insert( extracted_info.end(), b->extracted_info.begin(), b->extracted_info.end());
        
        // tvec
        tvec.insert( tvec.end(), a->tvec.begin(), a->tvec.end() );
        tvec.insert( tvec.end(), b->tvec.begin(), b->tvec.end() );
        
        // rvec
        rvec.insert( rvec.end(), a->rvec.begin(), a->rvec.end() );
        rvec.insert( rvec.end(), b->rvec.begin(), b->rvec.end() );
        
        // contours
        contour.insert( contour.end(), a->contour.begin(), a->contour.end() );
        contour.insert( contour.end(), b->contour.begin(), b->contour.end() );
        
        scores_with_weights.insert( scores_with_weights.end(), a->scores_with_weights.begin(), a->scores_with_weights.end() );
        scores_with_weights.insert( scores_with_weights.end(), b->scores_with_weights.begin(), b->scores_with_weights.end() );        
    }

    const ExtendedObjectInfo operator& ( ExtendedObjectInfo& left,  ExtendedObjectInfo& right){
        Rect a = left.getRect();
        Rect b = right.getRect();
        ExtendedObjectInfo ans = ExtendedObjectInfo(a & b);
        ans.inheritData(&left, &right);
        return ans;
    }

    const ExtendedObjectInfo operator| ( ExtendedObjectInfo& left,  ExtendedObjectInfo& right){
        Rect a = left.getRect();
        Rect b = right.getRect();
        ExtendedObjectInfo ans = ExtendedObjectInfo(a | b);
        ans.inheritData(&left, &right);
        return ans;
    }

    void ExtendedObjectInfo::print(string prefix){
        printf("%s (%i, %i, %i, %i)\n",prefix.c_str(),x,y,width,height);
    }
    
    string ExtendedObjectInfo::getInfo(){
        string info = to_string(x)+" "+to_string(y)+" "+to_string(width)+" "+to_string(height)+"\n";
        //TODO add additional
        return info;
    }
    
    Rect ExtendedObjectInfo::getRect(){
        return Rect(x,y,width,height);
    }
    
    Rect2d ExtendedObjectInfo::getRect2d(){
        return Rect2d(x,y,width,height);
    }

    Point ExtendedObjectInfo::getCenter(){
        return Point(x + width/2,y+height/2);
    }   
    
    vector<Point2f> ExtendedObjectInfo::getCorners(){
        vector<Point2f> corners;
        corners.push_back(Point2f(x,y));
        corners.push_back(Point2f(x+width,y));
        corners.push_back(Point2f(x+width,y+height));
        corners.push_back(Point2f(x,y+height));
        return corners;
    }
    
    void ExtendedObjectInfo::calcTotalScore(){        
        double score_ = 0, weight = 0;
        for( size_t i = 0 ; i < scores_with_weights.size() ; i++ ){
            score_ += scores_with_weights[i].first * scores_with_weights[i].second;
            weight += scores_with_weights[i].second;
        }
        total_score = score_ / weight;
    }
    
    void ExtendedObjectInfo::setScoreWeight(double score, double weight){
        scores_with_weights.push_back(make_pair(score, weight));        
    }
    
    void ExtendedObjectInfo::draw(const Mat& image, Scalar col){
#if (CV_MAJOR_VERSION > 3)        
        rectangle(image, getRect(), col, 2, 8);
#else
        Mat nimage;
        nimage = (Mat)image;
        rectangle(nimage, getRect(), col, 2, 8);
#endif
        if( contour.size() > 0)            
            drawContours( image, contour, 0, col, 1, 8);                
        
    }
    
    double getRange(ExtendedObjectInfo a, ExtendedObjectInfo b){
        Point a_c = a.getCenter();
        Point b_c = b.getCenter();
        return sqrt( pow(a_c.x - b_c.x,2) + pow(a_c.y - b_c.y,2));
    }            
    
    // cut rect to image size 
    void ExtendedObjectInfo::normalize(int im_width, int im_height){
        if( x < 0 )
            x = 0;
        if( x + width >= im_width)
            //x = im_width-1;
            width = im_width -x-1;
        if( y < 0 )
            y = 0;
        if( y + height >= im_height)
            height = im_height - y -1;
    }
    
    void ExtendedObjectInfo::mergeAllData(MergingPolicy mp){
        // score mergin'
        calcTotalScore();
        // mean of tvecs
        if( tvec.size() > 1 ){
            Vec3d total_tvec;
            for( size_t i = 0 ; i < tvec.size() ; i++ ){
                total_tvec[0] += tvec[i][0];
                total_tvec[1] += tvec[i][1];
                total_tvec[2] += tvec[i][2];
            }
            total_tvec[0] /= tvec.size();
            total_tvec[1] /= tvec.size();
            total_tvec[2] /= tvec.size();
            tvec.insert( tvec.begin(), total_tvec);
        }
        // contour merging
        if( contour.size() > 1 ){
            vector<Point> total_contour = contour[0];
            for( size_t i = 1 ; i < contour.size() ; i++){
                if( mp == INTERSECTION_MP)
                    total_contour = intersect_contours(total_contour, contour[i]);
                else
                    total_contour = unite_contours(total_contour, contour[i]);
                if( total_contour.size() == 0 )
                    total_contour = contour[i];                
            }   
            if( total_contour.size() == 0 )
                contour.clear();
            else
                contour.insert( contour.begin(), total_contour);
        }
    }
}

