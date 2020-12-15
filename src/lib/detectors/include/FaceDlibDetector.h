#ifndef _FACE_DLIB_ATTRIBUTE_
#define _FACE_DLIB_ATTRIBUTE_

#include "Attribute.h"

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/dnn.h>

template <template <int,template<typename>class,int,typename> class block, int N, template<typename>class BN, typename SUBNET>
using residual = dlib::add_prev1<block<N,BN,1,dlib::tag1<SUBNET>>>;

template <template <int,template<typename>class,int,typename> class block, int N, template<typename>class BN, typename SUBNET>
using residual_down = dlib::add_prev2<dlib::avg_pool<2,2,2,2,dlib::skip1<dlib::tag2<block<N,BN,2,dlib::tag1<SUBNET>>>>>>;

template <int N, template <typename> class BN, int stride, typename SUBNET> 
using block  = BN<dlib::con<N,3,3,1,1,dlib::relu<BN<dlib::con<N,3,3,stride,stride,SUBNET>>>>>;

template <int N, typename SUBNET> using ares      = dlib::relu<residual<block,N,dlib::affine,SUBNET>>;
template <int N, typename SUBNET> using ares_down = dlib::relu<residual_down<block,N,dlib::affine,SUBNET>>;

template <typename SUBNET> using alevel0 = ares_down<256,SUBNET>;
template <typename SUBNET> using alevel1 = ares<256,ares<256,ares_down<256,SUBNET>>>;
template <typename SUBNET> using alevel2 = ares<128,ares<128,ares_down<128,SUBNET>>>;
template <typename SUBNET> using alevel3 = ares<64,ares<64,ares<64,ares_down<64,SUBNET>>>>;
template <typename SUBNET> using alevel4 = ares<32,ares<32,ares<32,SUBNET>>>;

using anet_type = dlib::loss_metric<dlib::fc_no_bias<128,dlib::avg_pool_everything<
                            alevel0<
                            alevel1<
                            alevel2<
                            alevel3<
                            alevel4<
                            dlib::max_pool<3,3,2,2,dlib::relu<dlib::affine<dlib::con<32,7,7,2,2,
                            dlib::input_rgb_image_sized<150>
                            >>>>>>>>>>>>;

namespace eod{
    
    class FaceDlibAttribute : public Attribute{
    public:
        FaceDlibAttribute();
        
        FaceDlibAttribute(std::string base_path, std::string base_file_path, std::string shape_predictor_path, std::string recognition_model_path);
        
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
                        
        void save_face_descriptors(std::string base_path);
    private:
        // detector 
        dlib::frontal_face_detector face_detector;      
        
        // recognitor
        std::map<std::string, dlib::matrix<float,0,1>> base_face_descriptors;                        
        dlib::shape_predictor sp;
        anet_type net;
                
        bool identification_inited;

        void calc_face_descriptors(std::string base_path);
        
        void load_face_descriptors(std::string base_path);
    };
}

#endif // _FACE_DLIB_ATTRIBUTE_
