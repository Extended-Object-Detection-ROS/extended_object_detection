#ifndef _QR_ATTRIBUTE_
#define _QR_ATTRIBUTE_

#include "Attribute.h"

namespace eod{
    
    class QrAttribute : public Attribute{
    public:
        QrAttribute();
        
        std::vector<ExtendedObjectInfo> Detect2(const cv::Mat& image, int seq);
        
        bool Check2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);
        
        ~QrAttribute(){
            delete qrDecoder;
        }
        
    private:
        cv::QRCodeDetector *qrDecoder;

    };
}

#endif
