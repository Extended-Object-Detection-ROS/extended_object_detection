#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "SimpleObject.h"
#include "BlobDetector.h"

using namespace cv;
using namespace std;
const int th_slider_max = 300;
int mth = 10;
int Mth = 200;

const int bc_slider_max = 255;
int bc = 0;

const int ma_slider_max = 3000;
int ma = 1500;

const int mci_slider_max = 100;
int mci = 10;

const int mco_slider_max = 100;
int mco = 87;

const int mir_slider_max = 100;
int mir = 1;

int seq;

Mat last_image;
static const std::string OUTPUT_WINDOW = "Blob params collector";

eod::SimpleObject blobObject;
eod::BlobAttribute* ba;


void set_params(){
    ba->SetParams(mth, Mth, bc, ma, (double)mci / 100, (double)mco / 100, double(mir) / 100);
}

static void on_trackbar_mth( int, void* ){set_params();}
static void on_trackbar_Mth( int, void* ){set_params();}
static void on_trackbar_bc( int, void* ){set_params();}
static void on_trackbar_ma( int, void* ){set_params();}
static void on_trackbar_mci( int, void* ){set_params();}
static void on_trackbar_mco( int, void* ){set_params();}
static void on_trackbar_mir( int, void* ){set_params();}

// video image callback
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
      last_image = cv_bridge::toCvShare(msg, "bgr8")->image;  
      
      blobObject.Identify(last_image, cv::Mat(), seq);
      seq++;
      
      Mat image2draw = last_image.clone();
      blobObject.draw(image2draw,Scalar(0,255,0));
      
      cv::imshow(OUTPUT_WINDOW, image2draw);
#if (CV_MAJOR_VERSION > 3)
      int k = waitKey(1);
#else        
      int k = cvWaitKey(1);
#endif
      if( k == 27 ){
          //ROS_INFO("Params are minThreshold=%i maxThreshold=%i blobColor=%i minArea=%i minCircularity=%f minConvexity=%f minInertiaRatio=%f\n",mth , Mth, bc, ma, double(mci)/100, double(mco)/100, double(mir)/100);
          
          printf("<Attribute Name=\"MyBlobAttribute\" Type=\"Blob\" minThreshold=\"%i\" maxThreshold=\"%i\" blobColor=\"%i\" minArea=\"%i\" minCircularity=\"%.2f\" minConvexity=\"%.2f\" minInertiaRatio=\"%.2f\"/>\n",mth, Mth, bc, ma, double(mci)/100, double(mco)/100, double(mir)/100);
          
          ros::shutdown();
      }
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "blob_params_collector");      
    ros::NodeHandle nh_;
    
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE); // Create Window
    
    blobObject = eod::SimpleObject("Blob");
    ba = new eod::BlobAttribute();
    blobObject.AddModeAttribute(eod::DETECT, eod::RGB, ba);
    
    ba->Name = "Blob";
    //hca->filters.push_back(new eod::InsiderFilter());
    //sd = new eod::SizeAttribute(double(size), double(100));
    //sd->Name = "Size";
    //blobObject.AddModeAttribute(eod::CHECK, sd);
    
    createTrackbar( "minThreshold", OUTPUT_WINDOW, &mth, th_slider_max, on_trackbar_mth );
    //on_trackbar_dp( h_slider_min, 0);
    createTrackbar( "maxThreshold", OUTPUT_WINDOW, &Mth, th_slider_max, on_trackbar_Mth );
    //on_trackbar_h_max( h_slider_max, 0);
    
    createTrackbar( "blobColor", OUTPUT_WINDOW, &bc, bc_slider_max, on_trackbar_bc );
    
    createTrackbar( "minArea", OUTPUT_WINDOW, &ma, ma_slider_max, on_trackbar_ma );
    //on_trackbar_s_min( s_slider_min, 0);
    createTrackbar( "minCircularity*100", OUTPUT_WINDOW, &mci, mci_slider_max, on_trackbar_mci );
    //on_trackbar_s_max( s_slider_max, 0);
    
    createTrackbar( "minConvexity*100", OUTPUT_WINDOW, &mco, mco_slider_max, on_trackbar_mco );
    //on_trackbar_v_min( v_slider_min, 0);
    createTrackbar( "minInertiaRation*100", OUTPUT_WINDOW, &mir, mir_slider_max, on_trackbar_mir );
    //on_trackbar_v_max( v_slider_max, 0);    
        
    
    image_transport::ImageTransport it(nh_);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("image_raw", 1, imageCallback);       
    seq = 0;
    
    ros::spin();
    return 0;
}
