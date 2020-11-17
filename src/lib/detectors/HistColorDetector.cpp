#include "HistColorDetector.h" 

using namespace std;
using namespace cv;

namespace eod{
    
  bool SaveHistToFile(MatND hist, string filename){
    FileStorage fs(filename.c_str(), FileStorage::WRITE);
    if(!fs.isOpened()) {     
      printf("Error! Unable to open file %s!\n",filename.c_str());
      return false;      
    }    
    fs << "histogram" << hist;
    fs.release();
    return true;
  }
  
  bool LoadHistFromFile(string filename, MatND* hist){
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened()) {
      printf("Error! Unable to open file %s!\n",filename.c_str());
      return false;
      
    }
    fs["histogram"] >> *hist;
    fs.release();    
    return true;
  }  
  
  HistColorAttribute::HistColorAttribute(){
    inited = false;
    Type = HIST_COLOR_A;
    kernel = Mat::ones(3,3, CV_32F);
  }
  
  HistColorAttribute::HistColorAttribute(MatND hist){
    histogram = hist;
    inited = true;
    Type = HIST_COLOR_A;
    kernel = Mat::ones(3,3, CV_32F);
  }
  
  HistColorAttribute::HistColorAttribute(string file){
    if( LoadHistFromFile(file, &histogram) )
        inited = true;
    else inited = false;
    Type = HIST_COLOR_A;
    kernel = Mat::ones(3,3, CV_32F);
  }
  
  HistColorAttribute::~HistColorAttribute(){
      kernel.release();
      histogram.release();
  }
  
  vector<ExtendedObjectInfo> HistColorAttribute::Detect2(const Mat& image, int seq){
    vector<ExtendedObjectInfo>result;
    if(!inited){
        return result;
    }

    Mat hsv;
#if (CV_MAJOR_VERSION > 3)    
    cvtColor( image, hsv, COLOR_BGR2HSV );
#else
    cvtColor( image, hsv, CV_BGR2HSV );
#endif

    int channels[] = {0, 1};
    float h_range[] = {0, 179};
    float s_range[] = {0, 255};
    const float* ranges[] = {h_range, s_range};

    Mat backproj;
    calcBackProject( &hsv, 1, channels, histogram, backproj, ranges, 1, true);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat kernel = Mat::ones(3,3, CV_32F);
    morphologyEx(backproj, backproj, MORPH_OPEN, kernel);
    morphologyEx(backproj, backproj, MORPH_CLOSE, kernel);
    
#if (CV_MAJOR_VERSION > 3)
    findContours(backproj,contours,hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
#else    
    findContours(backproj,contours,hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
#endif
    
    for(int i = 0 ; i < contours.size(); i++){
        ExtendedObjectInfo temp = ExtendedObjectInfo(boundingRect(contours[i]));            
        temp.setScoreWeight(1, Weight);  
        if( returnContours)
            temp.contour.push_back(contours[i]);
        result.push_back(temp);
    }
    hsv.release();
    backproj.release();        
    
    return result;
  }
  
  bool HistColorAttribute::Check2(const Mat& image, ExtendedObjectInfo& rect){
        Rect original(0,0,image.cols,image.rows);
        Mat cropped;
        cropped = image(rect.getRect() & original);
        double start_area = cropped.cols * cropped.rows;
        Mat hsv;
#if (CV_MAJOR_VERSION > 3)    
        cvtColor( cropped, hsv, COLOR_BGR2HSV );
#else
        cvtColor( cropped, hsv, CV_BGR2HSV );
#endif
        
        int channels[] = {0, 1};
        float h_range[] = {0, 179};
        float s_range[] = {0, 255};
        const float* ranges[] = {h_range, s_range};
        
        Mat backproj;
        calcBackProject( &hsv, 1, channels, histogram, backproj, ranges, 1, true);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        Mat kernel = Mat::ones(3,3, CV_32F);
        morphologyEx(backproj, backproj, MORPH_OPEN, kernel);
        morphologyEx(backproj, backproj, MORPH_CLOSE, kernel);

#if (CV_MAJOR_VERSION > 3)
        findContours(backproj,contours,hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
#else    
        findContours(backproj,contours,hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
#endif
        
        double detected_area = 0;
        for( size_t i = 0 ; i < contours.size() ; i++ ){
            detected_area += contourArea(contours[i]);            
        }
        
        double percent = detected_area / start_area;
        
        hsv.release();
        backproj.release();        
            
        rect.setScoreWeight(percent, Weight);  
        if( percent >= Probability ){                              
            return true;             
        }              
        return false;
  }
  
  void HistColorAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
  
  void HistColorAttribute::setHist(MatND hist){
      histogram = hist;
      inited = true;      
  }
  
  void HistColorAttribute::removeHist(){
      inited=false;
      histogram.release();      
  }
  
}
