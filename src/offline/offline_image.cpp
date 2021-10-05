/*
Project: Extended Object Detection node for ROS
Program: Applyes detection result to image 
Author: Moscowsky Anton
DATUM: 30/09/2021
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
        printf("Error! Usage: offline_image image_in objectbase image_out\n");
        return -1;
    }
    
    // Image load
    string image_path = argv[1];        
    Mat frame = imread(image_path);
    
    if( frame.empty() ){
        printf("Error! Couldn't load image \'%s\'!\n", image_path.c_str());
        return -2;
    }
    
    // Object base load
    string objectBasePath = argv[2];    
    ObjectBase* objectBase = new ObjectBase();
    if( !objectBase->loadFromXML(objectBasePath) ){
        printf("Error! Couldn't load object base \'%s\'!\n", objectBasePath.c_str());
        return -3;
    }
    
    // image output
    string image_out_path = argv[3];    
    
    char * dir = getcwd(NULL, 0); // Platform-dependent, see reference link below    
    printf("Current dir: %s", dir);
    
    // processing
    printf("Starting process...\n");
    int seq = 0;
        
        Mat image2draw = frame.clone();
        for( size_t i = 0 ; i < objectBase->simple_objects.size() ; i++ ){
            objectBase->simple_objects[i]->Identify(frame, Mat(), seq);            
#ifndef USE_IGRAPH            
            objectBase->simple_objects[i]->draw(image2draw,Scalar(0,255,0));
#endif
        }
#ifdef USE_IGRAPH
        vector<ExtendedObjectInfo> DetectedScenes;
        for( size_t i = 0 ; i <  objectBase->complex_objects_graph.size(); i++){
                        
            DetectedScenes = objectBase->complex_objects_graph.at(i)->Identify(frame, Mat(), seq);
            
            objectBase->complex_objects_graph.at(i)->drawAll(image2draw, Scalar(255,255,0), 2);
        }
        if(!DetectedScenes.size()){
            for( size_t i = 0 ; i < objectBase->simple_objects.size() ; i++ )
                objectBase->simple_objects[i]->draw(image2draw,Scalar(0,255,0));
        }
#endif                        
        
        imshow("Detection result", image2draw);        
    
    char c=(char)waitKey(200);
        
    
    imwrite(image_out_path, image2draw);    
    
    destroyAllWindows();
    return 0;
    
}
