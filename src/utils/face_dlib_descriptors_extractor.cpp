#include "SimpleObject.h"
#include "FaceDlibDetector.h"

using namespace cv;
using namespace std;


int main(int argc, char **argv){
    
    if(argc < 5){
        //printf("Args %i\n",argc);
        printf("Usage: ./face_dlib_descriptors_extractor base_dir_path output_base_file_path shape_preditor_path recognition_model_path\n");
        return 1;
    }
    
    eod::FaceDlibAttribute* fdlib = new eod::FaceDlibAttribute(string(argv[1]), "", string(argv[3]), string(argv[4]));                
    fdlib->save_face_descriptors(string(argv[2]));            
    
    return 0;
}
