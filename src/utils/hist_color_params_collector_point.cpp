#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "SimpleObject.h"
#include "HistColorDetector.h"
#include "SizeDetector.h"

using namespace cv;
using namespace std;
static const std::string OUTPUT_WINDOW = "Histogram color params collector by point";

eod::SimpleObject coloredObject;
eod::HistColorAttribute* hca;
eod::SizeAttribute* sd;
int seq;

Mat mask;
Mat last_image;
Mat saved_image;
int lo = 20; 
int up = 20;
int size = 0;
MatND hist;
string out_filename;
int channels[] = { 0, 1 };
float h_range[] = { 0, 179 };
float s_range[] = { 0, 255 };
const float* ranges[] = { h_range, s_range };

static void on_trackbar_size( int, void*){
    sd->minSizePc = double(size) * 0.0001;
}

void pickPoint( int event, int x, int y, int, void *){
    if( event == EVENT_LBUTTONDOWN ){        
        hca->removeHist();
        Point seed = Point(x,y);
        int newMaskVal = 255;
        Scalar newVal = Scalar( 120, 120, 120 );

        int connectivity = 8;
        int flags = connectivity + (newMaskVal << 8 ) + FLOODFILL_FIXED_RANGE + FLOODFILL_MASK_ONLY;

        Mat mask2 = Mat::zeros( last_image.rows + 2, last_image.cols + 2, CV_8UC1 );
        floodFill( last_image, mask2, seed, newVal, 0, Scalar( lo, lo, lo ), Scalar( up, up, up), flags );
        mask = mask2( Range( 1, mask2.rows - 1 ), Range( 1, mask2.cols - 1 ) );
        saved_image = last_image.clone();
    }
    else if( event == EVENT_RBUTTONDOWN && !saved_image.empty() ){
        int h_bins = 30; int s_bins = 32;
        int histSize[] = { h_bins, s_bins };
        
        Mat hsv;
        cvtColor( saved_image, hsv, COLOR_BGR2HSV );
        /// Get the Histogram and normalize it
        calcHist( &hsv, 1, channels, mask, hist, 2, histSize, ranges, true, false );
        normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );
              
        hca->setHist(hist);
        mask.release();
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
      last_image = cv_bridge::toCvShare(msg, "bgr8")->image;  
      
      coloredObject.Identify(last_image, cv::Mat(), seq);
      seq++;
      
      Mat image2draw = last_image.clone();
      coloredObject.draw(image2draw,Scalar(0,255,0));
      
      if( !mask.empty() ){
          vector<vector<Point> > contours;
          vector<Vec4i> hierarchy;
          findContours( mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
          drawContours( image2draw, contours, -1, Scalar(0,0,255), 2, 8);
      }
      
      cv::imshow(OUTPUT_WINDOW, image2draw);
#if (CV_MAJOR_VERSION > 3)
      int k = waitKey(1);
#else        
      int k = cvWaitKey(1);
#endif
      if( k == 27 ){                    
          if( eod::SaveHistToFile(hist, out_filename) ){
                    
            printf("<Attribute Name=\"MyHistColorAttribute\" Type=\"HistColor\" Histogram=\"%s\"/>\n",out_filename.c_str());
            printf("<Attribute Name=\"MySizeAttribute\" Type=\"Size\" MinAreaPc=\"%.2f\" MaxAreaPc=\"100\"/>\n\n",double(size)* 0.0001);
          }
          
          ros::shutdown();
      }
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "hist_color_params_collector");      
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p("~");
    
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE); // Create Window
    
    coloredObject = eod::SimpleObject("ColoredObject");
    hca = new eod::HistColorAttribute();
    hca->Name = "Histogram";
    hca->filters.push_back(new eod::InsiderFilter());
    coloredObject.AddModeAttribute(eod::DETECT, eod::RGB, hca);
    sd = new eod::SizeAttribute(double(size), double(100));
    sd->Name = "Size";
    coloredObject.AddModeAttribute(eod::CHECK, eod::RGB, sd);
    
    image_transport::ImageTransport it(nh_);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("image_raw", 1, imageCallback);       
    seq = 0;
    
    if( !nh_p.getParam("out_filename",out_filename) ){
        out_filename = "/tmp/default.yaml";
        ROS_WARN("Histogram will be saved to %s",out_filename.c_str());
    }  
    
    setMouseCallback( OUTPUT_WINDOW, pickPoint, 0 );
    createTrackbar( "Low thresh", OUTPUT_WINDOW, &lo, 255, 0 );
    createTrackbar( "High thresh", OUTPUT_WINDOW, &up, 255, 0 );
    createTrackbar( "Size", OUTPUT_WINDOW, &size, 5000, on_trackbar_size );
    
    ros::spin();
    return 0;
}
