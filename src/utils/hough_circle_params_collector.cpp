#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "SimpleObject.h"
#include "HoughDetector.h"
#include "SizeDetector.h"

using namespace cv;
using namespace std;
const int dp_slider_max = 5;
int dp = 1;

const int md_slider_max = 300;
int md = 10;

const int p1_slider_max = 300;
int p1 = 100;

const int p2_slider_max = 300;
int p2 = 30;

const int mr_slider_max = 500;
int mr = 0;

const int Mr_slider_max = 500;
int Mr = 0;

int size = 0;
int seq;

Mat last_image;
static const std::string OUTPUT_WINDOW = "Hough circle params collector";

eod::SimpleObject houghObject;
eod::HoughAttribute* hca;
eod::SizeAttribute* sd;

static void on_trackbar_size( int, void*){
    sd->minSizePc = double(size)/100 * 0.01;
}

void set_params(){
    if( dp == 0 )
        dp = 1;
    hca->SetParamsCircle((double)dp, (double)md, (double)p1, (double)p2, mr, Mr);
}

static void on_trackbar_dp( int, void* ){set_params();}
static void on_trackbar_md( int, void* ){set_params();}
static void on_trackbar_p1( int, void* ){set_params();}
static void on_trackbar_p2( int, void* ){set_params();}
static void on_trackbar_mr( int, void* ){set_params();}
static void on_trackbar_Mr( int, void* ){set_params();}

// video image callback
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
      last_image = cv_bridge::toCvShare(msg, "bgr8")->image;  
      
      houghObject.Identify(last_image,  cv::Mat(), seq);
      seq++;
      
      Mat image2draw = last_image.clone();
      houghObject.draw(image2draw,Scalar(0,255,0));
      
      cv::imshow(OUTPUT_WINDOW, image2draw);
#if (CV_MAJOR_VERSION > 3)
      int k = waitKey(1);
#else        
      int k = cvWaitKey(1);
#endif
      if( k == 27 ){          
          printf("<Attribute Name=\"MyHoughCircleAttribute\" Type=\"Hough\" HoughType=\"0\" dp=\"%i\" md=\"%i\" p1=\"%i\" p2=\"%i\" mr=\"%i\" Mr=\"%i\"/>\n",dp, md, p1, p2, mr, Mr);
          printf("<Attribute Name=\"MySizeAttribute\" Type=\"Size\" MinAreaPc=\"%.2f\" MaxAreaPc=\"100\"/>\n\n",double(size)* 0.0001);
          ros::shutdown();
      }
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "hough_circle_params_collector");      
    ros::NodeHandle nh_;
    
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE); // Create Window
    
    houghObject = eod::SimpleObject("Circle");
    hca = new eod::HoughAttribute(dp, md, p1, p2, mr, Mr);
    houghObject.AddModeAttribute(eod::DETECT, eod::RGB, hca);
    hca->Name = "Hough";
    hca->filters.push_back(new eod::InsiderFilter());
    sd = new eod::SizeAttribute(double(size), double(100));
    sd->Name = "Size";
    houghObject.AddModeAttribute(eod::CHECK, eod::RGB, sd);
    
    createTrackbar( "dp", OUTPUT_WINDOW, &dp, dp_slider_max, on_trackbar_dp );
    //on_trackbar_dp( h_slider_min, 0);
    createTrackbar( "md", OUTPUT_WINDOW, &md, md_slider_max, on_trackbar_md );
    //on_trackbar_h_max( h_slider_max, 0);
    
    createTrackbar( "p1", OUTPUT_WINDOW, &p1, p1_slider_max, on_trackbar_p1 );
    //on_trackbar_s_min( s_slider_min, 0);
    createTrackbar( "p2", OUTPUT_WINDOW, &p2, p2_slider_max, on_trackbar_p2 );
    //on_trackbar_s_max( s_slider_max, 0);
    
    createTrackbar( "mr", OUTPUT_WINDOW, &mr, mr_slider_max, on_trackbar_mr );
    //on_trackbar_v_min( v_slider_min, 0);
    createTrackbar( "Mr", OUTPUT_WINDOW, &Mr, Mr_slider_max, on_trackbar_Mr );
    //on_trackbar_v_max( v_slider_max, 0);    
    
    createTrackbar( "Size", OUTPUT_WINDOW, &size, 1000, on_trackbar_size );    
    
    image_transport::ImageTransport it(nh_);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("image_raw", 1, imageCallback);       
    seq = 0;
    
    ros::spin();
    return 0;
}
