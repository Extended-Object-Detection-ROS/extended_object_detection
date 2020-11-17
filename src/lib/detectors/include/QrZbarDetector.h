#ifndef _QR_ZBAR_ATTRIBUTE_
#define _QR_ZBAR_ATTRIBUTE_

#include "Attribute.h"
#include <zbar.h>

namespace eod{
    
    class QrZbarAttribute : public Attribute{
    public:
        QrZbarAttribute();
        QrZbarAttribute(double real_len, std::string info);
        
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        ~QrZbarAttribute(){
            
        }
        
    private:
        zbar::ImageScanner scanner;
        double real_len;                        
        std::string info;

    };
}

#endif
