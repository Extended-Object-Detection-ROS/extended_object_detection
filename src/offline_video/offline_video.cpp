/*
Project: Extended Object Detection node for ROS
Program: Applyes detection result to video
Author: Moscowsky Anton
DATUM: 04/07/2020
*/
#include "ObjectBase.h"
#include "opencv2/opencv.hpp"
#include <unistd.h>

using namespace eod;
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    // preparations
    if( argc < 4 ){
        printf("Error! Usage: offline_video video_in objectbase video_out\n");
        return -1;
    }
    
    // Video load
    string video_path = argv[1];
    VideoCapture cap(video_path);     
    if( !cap.isOpened() ){
        printf("Error! Couldn't load video \'%s\'!\n", video_path.c_str());
        return -2;
    }
    
    // Object base load
    string objectBasePath = argv[2];    
    ObjectBase* objectBase = new ObjectBase();
    if( !objectBase->loadFromXML(objectBasePath) ){
        printf("Error! Couldn't load object base \'%s\'!\n", objectBasePath.c_str());
        return -3;
    }
    
    // video output
    string video_out_path = argv[3];
    int frame_width = cap.get(CAP_PROP_FRAME_WIDTH); 
    int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);
    VideoWriter video(video_out_path.c_str(), VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));
    
    char * dir = getcwd(NULL, 0); // Platform-dependent, see reference link below    
    printf("Current dir: %s", dir);
    
    // processing
    printf("Starting process...\n");
    int seq = 0;
    while(1){
        Mat frame;
        cap >> frame;
        
        if( frame.empty() )
            break;
        
        Mat image2draw = frame.clone();
        for( size_t i = 0 ; i < objectBase->simple_objects.size() ; i++ ){
            objectBase->simple_objects[i]->Identify(frame, Mat(), seq);            
            //objectBase->simple_objects[i]->draw(image2draw,Scalar(0,255,0));
        }
        vector<ExtendedObjectInfo> DetectedScenes;
        for( size_t i = 0 ; i <  objectBase->complex_objects.size(); i++){
            //objectBase->complex_objects.at(i)->prepareObjects(frame, seq);   
            bool fullans;
            DetectedScenes = objectBase->complex_objects.at(i)->Identify(frame, Mat(), seq, fullans);
            objectBase->complex_objects.at(i)->drawAll(image2draw, Scalar(255,255,0), 2);
        }
        if(!DetectedScenes.size()){
            for( size_t i = 0 ; i < objectBase->simple_objects.size() ; i++ )
                objectBase->simple_objects[i]->draw(image2draw,Scalar(0,255,0));
        }
        
        video.write(image2draw);
        imshow("Detection result", image2draw);
        
        char c=(char)waitKey(25);
        if(c==27){
            break;
            
        }
        seq++;
    }
    cap.release();
    video.release();
    destroyAllWindows();
    return 0;
    
}
