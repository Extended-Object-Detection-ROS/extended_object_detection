#ifndef _EXTENDED_OBJECT_INFO_
#define _EXTENDED_OBJECT_INFO_

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

namespace eod{
    
    enum MergingPolicy {
        UNION_MP,
        INTERSECTION_MP,
    };
    
    class ExtendedObjectInfo : public cv::Rect {
    public:        
        // constructors stuff
        ExtendedObjectInfo();        
        ExtendedObjectInfo(int x, int y, int w, int h);
        ExtendedObjectInfo(cv::Rect rect);        
        ExtendedObjectInfo(cv::Rect2d rect);        
        ~ExtendedObjectInfo(){/*TODO add all pointer release*/}

        // basic rects
        cv::Rect getRect();
        cv::Rect2d getRect2d();
        cv::Point getCenter();
        
        std::vector<cv::Point2f> getCorners();
        
        // help functions
        void print(std::string prefix = ""); 
        std::string getInfo();        
        void draw(const cv::Mat& image, cv::Scalar col = cv::Scalar(0, 255, 0) );
        
        void normalize(int im_width, int im_height);
        
        // operators
        friend const ExtendedObjectInfo operator& ( ExtendedObjectInfo& a,  ExtendedObjectInfo& b);
        friend const ExtendedObjectInfo operator| ( ExtendedObjectInfo& a,  ExtendedObjectInfo& b);        
        
        // merge contours, tvecs, rvecs and so on
        void mergeAllData(MergingPolicy mp = INTERSECTION_MP);
        //
        // Additional Info                        
        //
        
        // Variable behind is common for entire object
        int track_id;
        int track_status;     
        
        // score\probability collecting
        double total_score;
        void setScoreWeight(double score, double weight);
        void calcTotalScore();        
        
        // weak detection stuff
        double dcnt; // TODO remove in future
        int cnt; // TODO remove in future
        std::vector<int> pattern; // TODO remove in future
        
        //std::vector<cv::Point>* track_history_image;        
        
        //
        // Variables behind is those, which could be individual for each attribute
        //        
        std::vector<int> sub_id;        
        std::vector<std::string> extracted_info;
        
        // for every one attribute
        std::vector<std::pair<double, double> > scores_with_weights;         
        
        // only for those attributes, which can obtain them
        std::vector<std::vector<cv::Point> > contour;
        
        // only for those attributes, which can solve them
        //translation vectors
        std::vector<cv::Vec3d> tvec;        
        // rotation vectors
        std::vector<cv::Vec3d> rvec;        
        
        void initVars();       
    private:            
        void inheritData(ExtendedObjectInfo* a, ExtendedObjectInfo* b);        
                 
    };
    
    double getRange(ExtendedObjectInfo a, ExtendedObjectInfo b);    
}

#endif // _EXTENDED_OBJECT_INFO_
