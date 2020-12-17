#if (USE_DLIB)
#include "FaceDlibDetector.h"
#include <dlib/image_processing.h>
#include <dlib/opencv.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>

using namespace std;
using namespace cv;

namespace eod{
    
    FaceDlibAttribute::FaceDlibAttribute(){
        face_detector = dlib::get_frontal_face_detector();
        inited = true;
        identification_inited = false;
    }
    
    FaceDlibAttribute::FaceDlibAttribute(string base_dir_path, string base_file_path, string shape_predictor_path, string recognition_model_path){
        face_detector = dlib::get_frontal_face_detector();
        inited = true;
        try
        {
            if( shape_predictor_path == "" || recognition_model_path == "" ){
                identification_inited = false;
                return;
            }
            
            dlib::deserialize(shape_predictor_path) >> sp;        
            dlib::deserialize(recognition_model_path) >> net;
            
            if(base_file_path == "")
                calc_face_descriptors(base_dir_path);
            else{                
                load_face_descriptors(base_file_path);
            }
        }
        catch (dlib::file::file_not_found& e){
            printf("[Face_dlib] error! Please check correctness of shape predictor and recognition net names!\n");
            identification_inited = false;
        }
        catch (dlib::directory::dir_not_found& e){        
            printf("[Face_dlib] error! Please check correctness of shape predictor and recognition net pathes!\n");
            identification_inited = false;
        }                    
        catch (dlib::serialization_error& e){
            printf("[Face_dlib] error! Please check correctness of shape predictor and recognition net pathes and names!\n");
            identification_inited = false;
        }
    }
    
    vector<ExtendedObjectInfo> FaceDlibAttribute::Detect2(const Mat& image, int seq){
        vector<ExtendedObjectInfo> answers;
        

        dlib::cv_image<dlib::bgr_pixel> img(image);
        
        vector<dlib::rectangle> dets = face_detector(img);
        
        for( size_t i = 0; i < dets.size() ; i++ ){
            ExtendedObjectInfo tmp( (int)dets[i].left(), (int)dets[i].top(), (int)dets[i].width(), (int)dets[i].height());
            if( identification_inited ){
                Extract2(image, tmp);
            }
            else{
                tmp.setScoreWeight(1, Weight);
            }
            answers.push_back(tmp);
        }
        
        return answers;
    }
    
    bool FaceDlibAttribute::Check2(const Mat& image, ExtendedObjectInfo& rect){
        return false;
    }
        
    void FaceDlibAttribute::Extract2(const Mat& image, ExtendedObjectInfo& rect){
        
        if( base_face_descriptors.size() == 0 )
            return;
        
        dlib::cv_image<dlib::bgr_pixel> img(image);        
        dlib::rectangle det(rect.tl().x, rect.tl().y, rect.br().x, rect.br().y);        
        auto shape = sp(img, det);        
        dlib::matrix<dlib::rgb_pixel> face_chip;
        dlib::extract_image_chip(img, dlib::get_face_chip_details(shape,150,0.25), face_chip);
        vector<dlib::matrix<dlib::rgb_pixel>> f_faces;
        f_faces.push_back(face_chip);
        vector<dlib::matrix<float,0,1>> f_descriptors = net(f_faces);
        
        string result = "Unknown";
        double min_value = 1-Probability;
        for (auto const& base_face_d : base_face_descriptors)
        {
            double l = dlib::length(base_face_d.second-f_descriptors[0]);
            if( l <= min_value ){
                min_value = l;
                result = base_face_d.first;
            }
            //printf("%s - %f\n",base_face_d.first.c_str(), l);
        }
        rect.extracted_info.back() = result;
        if( result == "Unknown")
            rect.setScoreWeight(0, Weight);
        else
            rect.setScoreWeight(1-min_value, Weight);
    }
        
    void FaceDlibAttribute::load_face_descriptors(string base_path){
        printf("[Face_dlib] loading face descriptors...\n");
        try
        {
            dlib::deserialize(base_path) >> base_face_descriptors;        
            identification_inited = true;
            printf("[Face_dlib] has loaded %lu face descriptors.\n", base_face_descriptors.size());
        }
        catch (dlib::serialization_error& e){
            printf("[Face_dlib] error! Please check correctness of base file path and name!\n");
            identification_inited = false;
        }
        
    }
    
    void FaceDlibAttribute::save_face_descriptors(string base_path){
        printf("[Face_dlib] saving face descriptors...\n");
        if( !identification_inited ){
            printf("[Face_dlib] can't save face descriptors, because of wrong attribute initialization.\n");
            return;
        }
        try
        {
            dlib::serialize(base_path) << base_face_descriptors;            
            printf("[Face_dlib] has saved %lu face descriptors.\n", base_face_descriptors.size());
        }
        catch (dlib::serialization_error& e){
            printf("[Face_dlib] error! Please check correctness of base file path and name!\n");            
        }        
    }
        
    void FaceDlibAttribute::calc_face_descriptors(string base_path){
        
        printf("[Face_dlib] calculating face descriptors...\n");
        try{
            vector<dlib::file> files = dlib::get_files_in_directory_tree(base_path.c_str(), dlib::match_endings(".jpeg .jpg .png"));
        
            dlib::matrix<dlib::rgb_pixel> input_image;
            
            vector<dlib::matrix<float,0,1>> person_descriptors;
            vector<string> person_names;
            vector<dlib::matrix<dlib::rgb_pixel>> person_faces;
            
            for (const dlib::file& file : files){
                dlib::load_image(input_image, file.full_name());
                
                vector<dlib::rectangle> dets = face_detector(input_image);
                if( dets.size() == 1 ){
                    printf("[Face_dlib] calculating face descriptors for %s...\n",file.name().c_str());
                    auto shape = sp(input_image, dets[0]);
                    dlib::matrix<dlib::rgb_pixel> face_chip;
                    dlib::extract_image_chip(input_image, dlib::get_face_chip_details(shape,150,0.25), face_chip);
                    person_faces.push_back(move(face_chip));      
                    
                    size_t lastindex = file.name().find_last_of("."); 
                    string rawname = file.name().substr(0, lastindex);                
                    person_names.push_back(rawname);
                }
                else{
                    printf("[Face_dlib] file %s has %s faces!\n", file.name().c_str(), dets.size() == 0 ? "no" : "more than one");
                }
                                        
            }
            person_descriptors = net(person_faces);
            for (size_t i = 0; i < person_names.size(); ++i)
                base_face_descriptors[person_names[i]] = person_descriptors[i];
        
            printf("[Face_dlib] has calculated %lu face descriptors.\n", person_names.size());
            identification_inited = true;
            //save_face_descriptors("/tmp/tmp.db");
                        
        }
        catch (dlib::directory::dir_not_found& e)
        {
            printf("[Face_dlib] Error! Can't found directory with base faces %s\n",base_path.c_str());
            identification_inited = false;
            return;
        }
    }
    
}
#endif
