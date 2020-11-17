#include "contour_utils.h"

using namespace std;
using namespace cv;

namespace eod{
    
    vector< Point> shift_contour(vector <Point> &contour, Point shift){
        vector<Point> shifted_contour;
        for( size_t i = 0 ; i < contour.size() ; i++){
            shifted_contour.push_back(contour[i] - shift);
        }
        return shifted_contour;
    }
    
    vector< Point> intersect_contours(vector <Point>& contour1, vector <Point>& contour2){
        return merge_contours(contour1, contour2, 0);
    }
    
    vector< Point> unite_contours(vector <Point>& contour1, vector <Point>& contour2){
        return merge_contours(contour1, contour2, 1);
    }
    
    vector< Point> merge_contours(vector <Point>& contour1, vector <Point>& contour2, int type){
        // get work area        
        Rect work_area = boundingRect( contour1 ) | boundingRect( contour2 );        
        
        Mat merged = Mat::zeros(work_area.size(), CV_8UC1);        
        Mat contour1_im = Mat::zeros(work_area.size(), CV_8UC1);
        Mat contour2_im = Mat::zeros(work_area.size(), CV_8UC1);
        
        //draw
        vector<vector<Point> > shifted1;
        shifted1.push_back(shift_contour(contour1, work_area.tl()));
        drawContours( contour1_im, shifted1, -1, 255, -1);
        vector<vector<Point> > shifted2;
        shifted2.push_back(shift_contour(contour2, work_area.tl()));
        drawContours( contour2_im, shifted2, -1, 255, -1);
        
        //imshow("contour1 debug", contour1_im);
        //imshow("contour2 debug", contour2_im);        
        
        if( type == 0 )
            // intersect
            bitwise_and( contour1_im, contour2_im, merged);
        else
            // unite
            bitwise_or( contour1_im, contour2_im, merged);        
        //imshow("merge contour debug", merged);
        
        // find
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(merged,contours,hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
        
        if(contours.size() > 1){
            //printf("Warn: merge_contours has output of more than one contours.\n");
        }
        if( contours.size() > 0 )
            return shift_contour(contours[0], work_area.tl() * -1);        
        else
            return vector<Point>() ;
    }
    
    
}
