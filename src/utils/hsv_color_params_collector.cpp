#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "SimpleObject.h"
#include "HsvColorDetector.h"
#include "SizeDetector.h"

using namespace cv;
using namespace std;
const int h_slider_max_val = 179;
int h_slider_min = 0;
int h_slider_max = h_slider_max_val;
const int s_slider_max_val = 255;
int s_slider_min = 0;
int s_slider_max = s_slider_max_val;
const int v_slider_max_val = 255;
int v_slider_min = 0;
int v_slider_max = v_slider_max_val;
int size = 0;
int seq;

Mat last_image;
static const std::string OUTPUT_WINDOW = "HSV color params collector";

eod::SimpleObject coloredObject;
eod::HsvColorAttribute* hca;
eod::SizeAttribute* sd;

static void on_trackbar_size( int, void*){
    sd->minSizePc = double(size) * 0.0001;
}

void set_params(){
    hca->Hmin = h_slider_min;
    hca->Hmax = h_slider_max;
    hca->Smin = s_slider_min;
    hca->Smax = s_slider_max;
    hca->Vmin = v_slider_min;
    hca->Vmax = v_slider_max;
}

static void on_trackbar_h_min( int, void* ){set_params();}
static void on_trackbar_h_max( int, void* ){set_params();}
static void on_trackbar_s_min( int, void* ){set_params();}
static void on_trackbar_s_max( int, void* ){set_params();}
static void on_trackbar_v_min( int, void* ){set_params();}
static void on_trackbar_v_max( int, void* ){set_params();}

// video image callback
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
      last_image = cv_bridge::toCvShare(msg, "bgr8")->image;  
      
      coloredObject.Identify(last_image, cv::Mat(), seq);
      seq++;
      
      Mat image2draw = last_image.clone();
      coloredObject.draw(image2draw,Scalar(0,255,0));
      cv::imshow(OUTPUT_WINDOW, image2draw);
#if (CV_MAJOR_VERSION > 3)
      int k = waitKey(1);
#else        
      int k = cvWaitKey(1);
#endif
      if( k == 27 ){          
          ROS_WARN("YOUR ATTRIBUTES");
          printf("<Attribute Name=\"MyHSVColorAttribute\" Type=\"HSVColor\" Hmin=\"%i\" Hmax=\"%i\" Smin=\"%i\" Smax=\"%i\" Vmin=\"%i\" Vmax=\"%i\"/>\n",h_slider_min, h_slider_max, s_slider_min, s_slider_max, v_slider_min, v_slider_max);
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
    
    ros::init(argc, argv, "hsv_color_params_collector");      
    ros::NodeHandle nh_;
    
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE); // Create Window
    
    coloredObject = eod::SimpleObject("ColoredObject");
    hca = new eod::HsvColorAttribute(h_slider_min, h_slider_max, s_slider_min, s_slider_max, v_slider_min, v_slider_max);
    coloredObject.AddModeAttribute(eod::DETECT, eod::RGB, hca);
    hca->Name = "Histogram";
    hca->filters.push_back(new eod::InsiderFilter());
    sd = new eod::SizeAttribute(double(size), double(100));
    sd->Name = "Size";
    coloredObject.AddModeAttribute(eod::CHECK, eod::RGB, sd);
    
    createTrackbar( "Hmin", OUTPUT_WINDOW, &h_slider_min, h_slider_max_val, on_trackbar_h_min );
    on_trackbar_h_min( h_slider_min, 0);
    createTrackbar( "Hmax", OUTPUT_WINDOW, &h_slider_max, h_slider_max_val, on_trackbar_h_max );
    on_trackbar_h_max( h_slider_max, 0);
    
    createTrackbar( "Smin", OUTPUT_WINDOW, &s_slider_min, s_slider_max_val, on_trackbar_s_min );
    on_trackbar_s_min( s_slider_min, 0);
    createTrackbar( "Smax", OUTPUT_WINDOW, &s_slider_max, s_slider_max_val, on_trackbar_s_max );
    on_trackbar_s_max( s_slider_max, 0);
    
    createTrackbar( "Vmin", OUTPUT_WINDOW, &v_slider_min, v_slider_max_val, on_trackbar_v_min );
    on_trackbar_v_min( v_slider_min, 0);
    createTrackbar( "Vmax", OUTPUT_WINDOW, &v_slider_max, v_slider_max_val, on_trackbar_v_max );
    on_trackbar_v_max( v_slider_max, 0);    
    
    createTrackbar( "Size(/100)%", OUTPUT_WINDOW, &size, 1000, on_trackbar_size );    
    
    image_transport::ImageTransport it(nh_);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("image_raw", 1, imageCallback);       
    seq = 0;
    
    ros::spin();
    return 0;
}
