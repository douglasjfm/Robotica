#ifndef IMAGEMSTR_H
#define IMAGEMSTR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace cv;

class ImagemStr
{
    public:
        Mat m;
        Mat des;
        char *nome;
        std::vector<KeyPoint> kps;
        ImagemStr(Mat mtx, char* nm, std::vector<KeyPoint> k, Mat d);
        virtual ~ImagemStr();

    protected:

    private:
};

#endif // IMAGEMSTR_H
