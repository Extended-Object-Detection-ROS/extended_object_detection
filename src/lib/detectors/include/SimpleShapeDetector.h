/*
Project: Object detection Library
Author: Moscowsky Anton
File: Header file describes classes of simple shape detection (squares and circles)
*/


#ifndef _SIMPLE_SHAPE_DETECTOR_H
#define _SIMPLE_SHAPE_DETECTOR_H

#include "ObjectIdentifier.h"

using namespace cv;

namespace eod{

    enum SIMPLE_SHAPES { RECT = 0, CIRCLE = 1};

    class SimpleShapeAttribute : public Attribute{
        public:

        /// <summary>
        /// Empty constructor.
        /// /// </summary>
        SimpleShapeAttribute();

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="name">Path to cascade</param>
        /// <returns>Vector of rects of found objects</returns>
        SimpleShapeAttribute(int type);

        /// <summary>
        /// Functon detects objects in image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <returns>Vector of rects of found objects</returns>
        vector<Rect> Detect2(Mat image, int seq);

        /*
        /// <summary>
        /// Functon checks objects params on image
        /// </summary>
        /// <param name="image">Destination image</param>
        /// <param name="rects">List of rects to be checked</param>
        /// <returns>Vector of rects of appropriate objects</returns>
        vector<Rect> Check(Mat image, vector<ExtendedObjectInfo> rects);
        */

        bool Check2(Mat image,ExtendedObjectInfo rect);
        
        void Extract2(const cv::Mat& image, ExtendedObjectInfo& rect);

    private:

    };
}

#endif
